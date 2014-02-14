// Created: Aug/02/2013
// File: Jan/13/2014
// Author: José David Tascón Vidarte

#include "MatchesMap.hpp"

// ================================================================================================
// =============================== FUNCTIONS of CLASS MatchesMap ==================================
// ================================================================================================

std::vector<cv::DMatch> MatchesMap::match(int nmatch)
{
    return globalMatch[nmatch].matches;
}

/// TODO: LOOP CLOSURE HERE. CHECK LAST IMAGE with the first one or some of them
void MatchesMap::solveMatchesContinuous(std::vector< std::vector<float> > *descriptorsGPU)
{
    
//     std::cout << "Start solveMatches\n";
    SiftMatchGPU matcher(num_goodmatch);
    
    if(matcher.VerifyContextGL() == 0)
    {
        fprintf(stderr, "Can't open OpenGL context for Matcher in SiftGPU\n");
        return;
    }
        
    num_images = descriptorsGPU->size();
//     int combinations = num_images - 1; // Due to solve only continuous matches
    globalMatch.resize(num_images - 1);
    reliableMatch.resize(num_images - 1);
    DEBUG_1( std::cout << "\n================================ MATCH Features ==================================\n"; )
    
    for(register int cam = 0; cam < num_images - 1; ++cam)
    {
        int ft_per_im1 = int( (*descriptorsGPU)[cam].size()/128 );
        int ft_per_im2 = int( (*descriptorsGPU)[cam+1].size()/128 );
        matcher.SetDescriptors(0, ft_per_im1, &((*descriptorsGPU)[cam][0])); // prepare descriptor1
        matcher.SetDescriptors(1, ft_per_im2, &((*descriptorsGPU)[cam+1][0])); // prepare descritor2
        
        int match_buf[num_goodmatch][2];
        int nmatch = matcher.GetSiftMatch(num_goodmatch, match_buf); // match descriptors
        globalMatch[cam].matches.clear();
        
        if (!continuous) // check if all continuous must be stored
        {
	  if (nmatch < min_nmatch_reliable) // check reliability of match
	  {
	      reliableMatch[cam] = false;
	      continue;
	  }
        }
        
        reliableMatch[cam] = true;
        for (int i = 0; i < nmatch ; i++) // copy matches to internal match object
        {
	  globalMatch[cam].matches.push_back(cv::DMatch( match_buf[i][0], match_buf[i][1], i ));
        }
        globalMatch[cam].cam_id1 = cam;
        globalMatch[cam].cam_id2 = cam+1;
        DEBUG_1( printf("MATCH: %04i -> %04i:\t#matches = %i\n", cam, cam+1, nmatch); )
    }
    DEBUG_2( std::cout << "End solveMatches\n"; )
}

void MatchesMap::solveMatches(std::vector< std::vector<float> > *descriptorsGPU)
{
    /** Example gratia how matches are stored. If I have n = 5 images then: \n
     * The total number of possible matches is n*(n-1)/2 = 10 \n
     * The matches are organized in a vector with a MatchQuery structure \n
     * In a MatchQuery we have explicit camera number of matches: \n
     * globalMatch[X].cam1 or globalMatch[X].cam2 ; where X is the combination number and cam1 is the first image matched with second image cam2 \n
     * globalMatch[X].matches is a std::vector< cv::DMatch > \n
     * The matches are organized as follows: \n
     * Having,  globalMatch[0] => images 0-1 ;  From now it is defined the equivalent notation 0) => 0-1; \n
     * Therefore, the 10 combinations are: \n
     * 0) => 0-1; 1) => 1-2; 2) => 2-3; 3) => 3-4; 4) => 0-2; 5) => 1-3; 6) => 2-4; 7) => 0-3; 8) => 1-4; 9) => 0-4; \n
     */
//     std::cout << "Start solveMatches\n";
    SiftMatchGPU matcher(num_goodmatch);
    
    if(matcher.VerifyContextGL() == 0)
    {
        fprintf(stderr, "Can't open OpenGL context for Matcher in SiftGPU\n");
        return;
    }
        
    num_images = descriptorsGPU->size();
    register int stride = 1;
    register int cam = 0;
    register int it = 0;

//     int combinations = factorial(num_images)/(2*factorial(num_images-2)); // calculate combinations of matches
    int combinations = (num_images*(num_images-1))/2; // calculate combinations of matches
    
    globalMatch.resize(combinations);
    reliableMatch.resize(combinations);
    DEBUG_1( std::cout << "\n================================ MATCH Features ==================================\n"; )
    while(it < combinations)
    {
        for(cam = 0; cam + stride < num_images; cam++)
        {
	  int ft_per_im1 = int( (*descriptorsGPU)[cam].size()/128 );
	  int ft_per_im2 = int( (*descriptorsGPU)[cam+stride].size()/128 );
	  matcher.SetDescriptors(0, ft_per_im1, &((*descriptorsGPU)[cam][0])); // prepare descriptor1
	  matcher.SetDescriptors(1, ft_per_im2, &((*descriptorsGPU)[cam+stride][0])); // prepare descritor2

	  int match_buf[num_goodmatch][2];
	  int nmatch = matcher.GetSiftMatch(num_goodmatch, match_buf); // match descriptors
	  globalMatch[it].matches.clear();

	  if (nmatch < min_nmatch_reliable) // check reliability of match
	  {
	      if( !(continuous && (stride == 1)) )
	      {
		reliableMatch[it] = false;
		it++;
		continue;
	      }
	  }
	  
	  reliableMatch[it] = true;
	  for (int i = 0; i < nmatch ; i++) // copy matches to internal match object
	  {
	      globalMatch[it].matches.push_back(cv::DMatch( match_buf[i][0], match_buf[i][1], i ));
	  }
	  globalMatch[it].cam_id1 = cam;
	  globalMatch[it].cam_id2 = cam+stride;
	  DEBUG_1( printf("MATCH: %04i -> %04i:\t#matches = %i\n", cam, cam+stride, nmatch); )
	  it++;
        }
        stride++;
    }
    DEBUG_2( std::cout << "End solveMatches\n"; )
}

void MatchesMap::robustifyMatches(std::vector< std::vector< cv::KeyPoint > > *set_of_keypoints)
{
    DEBUG_1( std::cout << "\nRobustify matches with fundamental matrix:\n"; )
    for(register int it = 0; it < reliableMatch.size(); ++it)
    {
        if (reliableMatch[it])
        {
	  int cam1 = globalMatch[it].cam_id1;
	  int cam2 = globalMatch[it].cam_id2;
	  
	  std::vector<cv::KeyPoint> kps1 = (*set_of_keypoints)[cam1];
	  std::vector<cv::KeyPoint> kps2 = (*set_of_keypoints)[cam2];
	  std::vector< cv::Point2d > pts1, pts2;
	  extractPointsfromMatches( globalMatch[it].matches, kps1, kps2, pts1, pts2);
 	  DEBUG_3( std::cout << "matches size = " << globalMatch[it].matches.size() << "\n"; )
	  DEBUG_3( std::cout << "pts1 size = " << pts1.size() << "\n"; )
	  DEBUG_3( std::cout << "pts2 size = " << pts2.size() << "\n"; )
// 	  keyPointstoPoints(kps1, pts1);
// 	  keyPointstoPoints(kps2, pts2);
	  cv::Mat F_est;
	  int inlier = robustMatchesfromFundamental(pts1, pts2, globalMatch[it].matches, F_est, 5);
	  DEBUG_1( printf("MATCH: %04i -> %04i:\t#matches = %i\n", cam1, cam2, inlier); )
	  if( !((cam2 - cam1 == 1) && continuous) )
	  {
	      if (pts1.size() < min_nmatch_reliable) reliableMatch[it] = false;
	  }
        }
    }
}

void MatchesMap::robustifyMatches(std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints)
{
    DEBUG_1( std::cout << "\nRobustify matches with fundamental matrix:\n"; )
    for(register int it = 0; it < reliableMatch.size(); ++it)
    {
        if (reliableMatch[it])
        {
	  int cam1 = globalMatch[it].cam_id1;
	  int cam2 = globalMatch[it].cam_id2;
	  
	  std::vector<SiftGPU::SiftKeypoint> kps1 = (*set_of_keypoints)[cam1];
	  std::vector<SiftGPU::SiftKeypoint> kps2 = (*set_of_keypoints)[cam2];
	  std::vector< cv::Point2d > pts1, pts2;
	  extractPointsfromMatches( globalMatch[it].matches, kps1, kps2, pts1, pts2);
 	  DEBUG_3( std::cout << "matches size = " << globalMatch[it].matches.size() << "\n"; )
	  DEBUG_3( std::cout << "pts1 size = " << pts1.size() << "\n"; )
	  DEBUG_3( std::cout << "pts2 size = " << pts2.size() << "\n"; )
// 	  keyPointstoPoints(kps1, pts1);
// 	  keyPointstoPoints(kps2, pts2);
	  cv::Mat F_est;
	  int inlier = robustMatchesfromFundamental(pts1, pts2, globalMatch[it].matches, F_est, 5);
	  DEBUG_1( printf("MATCH: %04i -> %04i:\t#matches = %i\n", cam1, cam2, inlier); )
	  if( !((cam2 - cam1 == 1) && continuous) )
	  {
	      if (pts1.size() < min_nmatch_reliable) reliableMatch[it] = false;
	  }
        }
    }
}

void MatchesMap::depthFilter(std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints,
		         std::vector< cv::Mat > *set_of_depth, const int reliable )
{
    DEBUG_1( std::cout << "\nFilter matches with valid depth values:\n"; )
    for(register int it = 0; it < reliableMatch.size(); ++it)
    {
        if (reliableMatch[it])
        {
	  int cam1 = globalMatch[it].cam_id1;
	  int cam2 = globalMatch[it].cam_id2;
	  std::vector< cv::Point2d > pts1, pts2;
        
	  for (register int ft = 0; ft < globalMatch[it].matches.size(); ++ft)	//Extract pts1 from matches
	  {
	      int qid = globalMatch[it].matches[ft].queryIdx;
	      int tid = globalMatch[it].matches[ft].trainIdx;
	      pts1.push_back( cv::Point2d((*set_of_keypoints)[cam1][qid].x, (*set_of_keypoints)[cam1][qid].y) );
	      pts2.push_back( cv::Point2d((*set_of_keypoints)[cam2][tid].x, (*set_of_keypoints)[cam2][tid].y) );
	  }
	  // Remove bad points with depth projection
            std::vector<int> goodpt = removeBadPointsDual(pts1, pts2, (*set_of_depth)[cam1], (*set_of_depth)[cam2]);
	  int sum = sumVector(goodpt);
	  if (sum < reliable)
	  {
	      if( !((cam2 - cam1 == 1) && continuous) ) reliableMatch[it] = false;
// 	      std::cout << "false " ;
	  }
	  else
	  {
	      int k = 0;
	      for (int i = 0; i < globalMatch[it].matches.size(); i++) // Erase matches that do not present features with valid depths
	      {
		if (!goodpt[k])
		{
		    globalMatch[it].matches.erase( globalMatch[it].matches.begin() + i);
		    i--;
		}
		k++;
	      }
	      DEBUG_1( printf("MATCH: %04i -> %04i:\t#matches = %i\n", cam1, cam2, globalMatch[it].matches.size()); )
 	      DEBUG_2( std::cout << "Erase Matches, Size: " << globalMatch[it].matches.size() << "\n"; )
	  }
        }
    }
}

void MatchesMap::depthFilter(std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints,
		         std::vector< std::string > *depth_list, const int reliable )
{
    DEBUG_1( std::cout << "\nFilter matches with valid depth values:\n"; )
    for(register int it = 0; it < reliableMatch.size(); ++it)
    {
        if (reliableMatch[it])
        {
	  int cam1 = globalMatch[it].cam_id1;
	  int cam2 = globalMatch[it].cam_id2;
	  std::vector< cv::Point2d > pts1, pts2;
	  cv::Mat depth1 = cv::imread((*depth_list)[cam1], -1);
	  cv::Mat depth2 = cv::imread((*depth_list)[cam2], -1);
        
	  for (register int ft = 0; ft < globalMatch[it].matches.size(); ++ft)	//Extract pts1 from matches
	  {
	      int qid = globalMatch[it].matches[ft].queryIdx;
	      int tid = globalMatch[it].matches[ft].trainIdx;
	      pts1.push_back( cv::Point2d((*set_of_keypoints)[cam1][qid].x, (*set_of_keypoints)[cam1][qid].y) );
	      pts2.push_back( cv::Point2d((*set_of_keypoints)[cam2][tid].x, (*set_of_keypoints)[cam2][tid].y) );
	  }
	  // Remove bad points with depth projection
            std::vector<int> goodpt = removeBadPointsDual( pts1, pts2, depth1, depth2 );
	  int sum = sumVector(goodpt);
	  if (sum < reliable)
	  {
	      if( !((cam2 - cam1 == 1) && continuous) ) reliableMatch[it] = false;
// 	      std::cout << "false " ;
	  }
	  else
	  {
	      int k = 0;
	      for (int i = 0; i < globalMatch[it].matches.size(); i++) // Erase matches that do not present features with valid depths
	      {
		if (!goodpt[k])
		{
		    globalMatch[it].matches.erase( globalMatch[it].matches.begin() + i);
		    i--;
		}
		k++;
	      }
	      DEBUG_1( printf("MATCH: %04i -> %04i:\t#matches = %i\n", cam1, cam2, globalMatch[it].matches.size()); )
	      DEBUG_2( std::cout << "Erase Matches Size: " << globalMatch[it].matches.size() << "\n"; )
	  }
        }
    }
}


void MatchesMap::solveDB( HandleDB *mydb, std::vector< std::vector<SiftGPU::SiftKeypoint> > *keypointsGPU )
{
    actualfeature = 0;
    int base = 0;
    int cam = 1;
    int count = 0;
    int it = 0;
    // loop closure code
    while(count < reliableMatch.size())
    {
//         std::cout << "count = " << count << "\n";
//         std::cout << "it = " << it << "\n";
        if (reliableMatch[it])
        {
	  for (register int k = 0; k < globalMatch[it].matches.size(); ++k)
	  {
	      int cam1 = globalMatch[it].cam_id1;
	      int cam2 = globalMatch[it].cam_id2;
	      int idft_c1 = globalMatch[it].matches[k].queryIdx;
	      int idft_c2 = globalMatch[it].matches[k].trainIdx;
	      int sf1, sf2;
	      
	      bool find1 = mydb->searchFeature(cam1, idft_c1, &sf1); // Find if idft already exist for camera 1, return feature number
	      bool find2 = mydb->searchFeature(cam2, idft_c2, &sf2);
	      if (find1 && find2)
	      {
		continue;
	      }
	      else if (find1)
	      {
		mydb->insertRow( sf1, cam2, idft_c2,(*keypointsGPU)[cam2][idft_c2].x, (*keypointsGPU)[cam2][idft_c2].y );
// 		continue;
	      }
	      else if (find2)
	      {
		mydb->insertRow( sf2, cam1, idft_c1, (*keypointsGPU)[cam1][idft_c1].x,(*keypointsGPU)[cam1][idft_c1].y );
// 		continue;
	      }
	      else
	      {
		mydb->insertRow( actualfeature, cam1, idft_c1, (*keypointsGPU)[cam1][idft_c1].x, (*keypointsGPU)[cam1][idft_c1].y );
		mydb->insertRow( actualfeature, cam2, idft_c2, (*keypointsGPU)[cam2][idft_c2].x, (*keypointsGPU)[cam2][idft_c2].y );
		actualfeature++; // A new feature is added
	      }
	  }  
        }
        
        if (reliableMatch.size() > num_images ) // Condition to see how the matches were solved. If continuous it and count are the same
        {
	  it = it + (num_images - cam);
	  cam++;
	  if (num_images == cam+base)
	  {
	      base++;
	      it = base;
	      cam = 1;
	  }
        }
        else it++;
        count++;
    }
    //open loop code
//     for(int it = 0; it < reliableMatch.size(); it++)
//     {
//         if (reliableMatch[it])
//         {
// 	  for (int k = 0; k < globalMatch[it].matches.size(); k++)
// 	  {
// 	      int cam1 = globalMatch[it].cam_id1;
// 	      int cam2 = globalMatch[it].cam_id2;
// 	      int idft_c1 = globalMatch[it].matches[k].queryIdx;
// 	      int idft_c2 = globalMatch[it].matches[k].trainIdx;
// 	      int sf1, sf2;
// 	      
// 	      bool find1 = mydb->searchFeature(cam1, idft_c1, &sf1);
// 	      bool find2 = mydb->searchFeature(cam2, idft_c2, &sf2);
// 	      if (find1 && find2)
// 	      {
// 		continue;
// 	      }
// 	      else if (find1)
// 	      {
// 		mydb->insertRow( sf1, cam2, idft_c2,(*keypointsGPU)[cam2][idft_c2].x, (*keypointsGPU)[cam2][idft_c2].y );
// // 		continue;
// 	      }
// 	      else if (find2)
// 	      {
// 		mydb->insertRow( sf2, cam1, idft_c1, (*keypointsGPU)[cam1][idft_c1].x, (*keypointsGPU)[cam1][idft_c1].y );
// // 		continue;
// 	      }
// 	      else
// 	      {
// 		mydb->insertRow( actualfeature, cam1, idft_c1, (*keypointsGPU)[cam1][idft_c1].x, (*keypointsGPU)[cam1][idft_c1].y );
// 		mydb->insertRow( actualfeature, cam2, idft_c2, (*keypointsGPU)[cam2][idft_c2].x, (*keypointsGPU)[cam2][idft_c2].y );
// 		actualfeature++;
// 	      }
// 	  }  
//         }
//     }
}
	      
	      // 	      bool inc1 = mydb->insertUnique(actualfeature, cam1, idft_c1, 
	      // 				     (*keypointsGPU)[cam1][idft_c1].x, (*keypointsGPU)[cam1][idft_c1].y );
	      // 	      
	      // 	      bool inc2 = mydb->insertUnique(actualfeature, cam2, idft_c2, 
	      // 				     (*keypointsGPU)[cam2][idft_c2].x, (*keypointsGPU)[cam2][idft_c2].y );


void MatchesMap::solveDB3D( HandleDB *mydb, std::vector< std::vector<SiftGPU::SiftKeypoint> > *keypointsGPU,
		        std::vector< std::string > *depth_list, Eigen::Matrix3d &calibration )
{
    
    
    actualfeature = 0;
    int base = 0;
    int cam = 1;
    int count = 0;
    int it = 0;
    // loop closure code
    while(count < reliableMatch.size())
    {
//         std::cout << "count = " << count << "\n";
//         std::cout << "it = " << it << "\n";
        if (reliableMatch[it])
        {
	  int cam1 = globalMatch[it].cam_id1;
	  int cam2 = globalMatch[it].cam_id2;
	  cv::Mat depth1 = cv::imread( (*depth_list)[cam1], -1 );
	  cv::Mat depth2 = cv::imread( (*depth_list)[cam2], -1 );
	  
	  for (register int k = 0; k < globalMatch[it].matches.size(); ++k)
	  {
	      int idft_c1 = globalMatch[it].matches[k].queryIdx;
	      int idft_c2 = globalMatch[it].matches[k].trainIdx;
	      int sf1, sf2;
	      
	      bool find1 = mydb->searchFeature(cam1, idft_c1, &sf1); // Find if idft already exist for camera 1, return feature number
	      bool find2 = mydb->searchFeature(cam2, idft_c2, &sf2);
	      if (find1 && find2)
	      {
		continue;
	      }
	      else if (find1)
	      {
		int u = (int) round( (*keypointsGPU)[cam2][idft_c2].x );
		int v = (int) round( (*keypointsGPU)[cam2][idft_c2].y );
		Eigen::Vector3f vv = projection<int,double,float>( u, v, depth2, calibration);
		mydb->insertRow3D( sf1, cam2, idft_c2, vv(0), vv(1), vv(2) );
// 		continue;
	      }
	      else if (find2)
	      {
		int u = (int) round( (*keypointsGPU)[cam1][idft_c1].x );
		int v = (int) round( (*keypointsGPU)[cam1][idft_c1].y );
		Eigen::Vector3f vv = projection<int,double,float>( u, v, depth1, calibration);
		mydb->insertRow3D( sf2, cam1, idft_c1, vv(0), vv(1), vv(2) );
// 		continue;
	      }
	      else
	      {
		int u1 = (int) round( (*keypointsGPU)[cam1][idft_c1].x );
		int v1 = (int) round( (*keypointsGPU)[cam1][idft_c1].y );
		Eigen::Vector3f vv1 = projection<int,double,float>( u1, v1, depth1, calibration);
		int u2 = (int) round( (*keypointsGPU)[cam2][idft_c2].x );
		int v2 = (int) round( (*keypointsGPU)[cam2][idft_c2].y );
		Eigen::Vector3f vv2 = projection<int,double,float>( u2, v2, depth2, calibration);
		mydb->insertRow3D( actualfeature, cam1, idft_c1, vv1(0), vv1(1), vv1(2) );
		mydb->insertRow3D( actualfeature, cam2, idft_c2, vv2(0), vv2(1), vv2(2) );
		actualfeature++; // A new feature is added
	      }
	  }  
        }
        
        if (reliableMatch.size() > num_images ) // Condition to see how the matches were solved. If continuous it and count are the same
        {
	  it = it + (num_images - cam);
	  cam++;
	  if (num_images == cam+base)
	  {
	      base++;
	      it = base;
	      cam = 1;
	  }
        }
        else it++;
        count++;
    }
}


void MatchesMap::exportTXT(const char *file_txt, std::vector< std::vector<SiftGPU::SiftKeypoint> > *keypointsGPU)
{
    std::ofstream myfile1;
    myfile1.open (file_txt);
    DEBUG_1( std::cout << "Export matches to file: " << file_txt << "\n"; )
    for(int it = 0; it < reliableMatch.size(); it++)
    {
        if (reliableMatch[it])
        {
	  myfile1 << "\nMATCH " << it << "\n\n";
	  myfile1 << "camera\tidx\tcoordx\tcoordy\n";
	  
	  for (register int k = 0; k < globalMatch[it].matches.size(); ++k)
	  {
	      
	      int cam1 = globalMatch[it].cam_id1;
	      int cam2 = globalMatch[it].cam_id2;
	      int idft_c1 = globalMatch[it].matches[k].queryIdx;
	      int idft_c2 = globalMatch[it].matches[k].trainIdx;
	      
	      myfile1 << cam1 << "\t" <<  idft_c1 << "\t" << (*keypointsGPU)[cam1][idft_c1].x
		        << "\t" <<(*keypointsGPU)[cam1][idft_c1].y << "\n";
	      myfile1 << cam2 << "\t" <<  idft_c2 << "\t" << (*keypointsGPU)[cam2][idft_c2].x
		        << "\t" <<(*keypointsGPU)[cam2][idft_c2].y << "\n";
	       
	  }
        }
    }
    myfile1.close();
    return;
}

void MatchesMap::plot( std::vector< cv::Mat > *images, std::vector< std::vector< cv::KeyPoint > > *set_of_keypoints )
{
    cv::Mat draw_match;
    for (register int i=0; i < reliableMatch.size(); ++i)
    {
        if (reliableMatch[i])
        {
	  std::vector<cv::DMatch> match00 = globalMatch[i].matches;;
	  int cam1 = globalMatch[i].cam_id1;
	  int cam2 = globalMatch[i].cam_id2;
	  std::cout << "TOTAL MATCHES = " << match00.size() << '\n';
	  drawMatches( (*images)[cam1], (*set_of_keypoints)[cam1], (*images)[cam2], (*set_of_keypoints)[cam2],
		        match00, draw_match, cv::Scalar::all(-1), cv::Scalar::all(-1),
		        std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	  std::stringstream ss;
	  ss << "match, cam " << cam1 << ", with cam " << cam2;
		imshow( ss.str() , draw_match );
	  
	  cvWaitKey(0);
        }
    }
    return;
}