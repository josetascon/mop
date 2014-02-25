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

void MatchesMap::solveMatchesContinuous_subgroup(boost::shared_ptr< float_vv > descriptorsGPU, int init_image, int final_image)
{
    DEBUG_2( std::cout << "Start solveMatches\n"; )
    SiftMatchGPU matcher(num_goodmatch);
    
    if(matcher.VerifyContextGL() == 0)
    {
        fprintf(stderr, "Can't open OpenGL context for Matcher in SiftGPU\n");
        return;
    }
        
    num_images = final_image - init_image;
    int combinations = num_images-1;
    register int it = globalMatch.size();
    
    for(register int cam = init_image; cam < combinations; ++cam)
    {
        globalMatch.push_back( MatchQuery() );
        reliableMatch.push_back( false );
        
        int ft_per_im1 = int( descriptorsGPU->at(cam).size()/128 );
        int ft_per_im2 = int( descriptorsGPU->at(cam+1).size()/128 );
        matcher.SetDescriptors(0, ft_per_im1, &(descriptorsGPU->at(cam)[0])); // prepare descriptor1
        matcher.SetDescriptors(1, ft_per_im2, &(descriptorsGPU->at(cam+1)[0])); // prepare descritor2
        
        int match_buf[num_goodmatch][2];
        int nmatch = matcher.GetSiftMatch(num_goodmatch, match_buf); // match descriptors
        globalMatch[it].matches.clear();
        
        if (!continuous) // check if all continuous must be stored
        {
	  if (nmatch < min_nmatch_reliable) // check reliability of match
	  {
	      reliableMatch[it] = false;
	      continue;
	  }
        }
        
        reliableMatch[it] = true;
        for (int i = 0; i < nmatch ; i++) // copy matches to internal match object
        {
	  globalMatch[it].matches.push_back(cv::DMatch( match_buf[i][0], match_buf[i][1], i ));
        }
        globalMatch[it].cam_id1 = cam;
        globalMatch[it].cam_id2 = cam+1;
        DEBUG_1( printf("MATCH: %04i -> %04i:\t#matches = %i\n", cam, cam+1, nmatch); )
        it++;
    }
    DEBUG_2( std::cout << "End solveMatches\n"; )
}

void MatchesMap::solveMatchesPairs_subgroup(boost::shared_ptr< float_vv > descriptorsGPU, int init_image, int final_image )
{
    DEBUG_2( std::cout << "Start solveMatches\n"; )
    SiftMatchGPU matcher(num_goodmatch);
    
    if(matcher.VerifyContextGL() == 0)
    {
        fprintf(stderr, "Can't open OpenGL context for Matcher in SiftGPU\n");
        return;
    }
        
    num_images = final_image - init_image;
    int combinations = (num_images*(num_images-1))/2;
    register int cam = init_image;
    register int it = globalMatch.size();
    
    for(int k = 0; k < combinations; ++k)
    {
        for(register int stride = 1; cam + stride < final_image; stride++)
        {
	  globalMatch.push_back( MatchQuery() );
	  reliableMatch.push_back( false );
	  
	  int ft_per_im1 = int( descriptorsGPU->at(cam).size()/128 );
	  int ft_per_im2 = int( descriptorsGPU->at(cam+stride).size()/128 );
	  matcher.SetDescriptors(0, ft_per_im1, &(descriptorsGPU->at(cam)[0])); // prepare descriptor1
	  matcher.SetDescriptors(1, ft_per_im2, &(descriptorsGPU->at(cam+stride)[0])); // prepare descritor2

	  int match_buf[num_goodmatch][2];
	  int nmatch = matcher.GetSiftMatch(num_goodmatch, match_buf); // match descriptors
	  globalMatch[it].matches.clear();

	  if (nmatch < min_nmatch_reliable) // check reliability of match
	  {
	      if( !(continuous && (stride == 1)) ) // if ICP is enabled do this
	      {
// 		reliableMatch[it] = false;
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
        cam++;
    }
    DEBUG_2( std::cout << "End solveMatches\n"; )

}

void MatchesMap::solveMatchesOneElement_subgroupUp(boost::shared_ptr< float_vv > descriptorsGPU, int element, int final_image )
{
    DEBUG_2( std::cout << "Start solveMatches\n"; )
    SiftMatchGPU matcher(num_goodmatch);
    
    if(matcher.VerifyContextGL() == 0)
    {
        fprintf(stderr, "Can't open OpenGL context for Matcher in SiftGPU\n");
        return;
    }
        
    num_images = final_image - element;
    int combinations = num_images-1;
    register int base = element;
    register int it = globalMatch.size();
    int k = 0;
    for(register int stride = 1 + base; k < combinations; ++stride, ++k)
    {
        globalMatch.push_back( MatchQuery() );
        reliableMatch.push_back( false );
        
        int ft_per_im1 = int( descriptorsGPU->at(base).size()/128 );
        int ft_per_im2 = int( descriptorsGPU->at(stride).size()/128 );
        matcher.SetDescriptors(0, ft_per_im1, &(descriptorsGPU->at(base)[0])); // prepare descriptor1
        matcher.SetDescriptors(1, ft_per_im2, &(descriptorsGPU->at(stride)[0])); // prepare descritor2
        
        int match_buf[num_goodmatch][2];
        int nmatch = matcher.GetSiftMatch(num_goodmatch, match_buf); // match descriptors
        globalMatch[it].matches.clear();
        
        if (nmatch < min_nmatch_reliable) // check reliability of match
        {
	  if( !(continuous && (stride - base == 1)) ) // if ICP is enabled do this
	  {
// 		reliableMatch[it] = false;
	      it++;
	      continue;
	  }
        }
        
        reliableMatch[it] = true;
        for (int i = 0; i < nmatch ; i++) // copy matches to internal match object
        {
	  globalMatch[it].matches.push_back(cv::DMatch( match_buf[i][0], match_buf[i][1], i ));
        }
        globalMatch[it].cam_id1 = base;
        globalMatch[it].cam_id2 = stride;
        DEBUG_1( printf("MATCH: %04i -> %04i:\t#matches = %i\n", base, stride, nmatch); )
        it++;
    }
    DEBUG_2( std::cout << "End solveMatches\n"; )
}

void MatchesMap::solveMatchesOneElement_subgroupDown(boost::shared_ptr< float_vv > descriptorsGPU, int init_image, int element )
{
    DEBUG_2( std::cout << "Start solveMatches\n"; )
    SiftMatchGPU matcher(num_goodmatch);
    
    if(matcher.VerifyContextGL() == 0)
    {
        fprintf(stderr, "Can't open OpenGL context for Matcher in SiftGPU\n");
        return;
    }
        
    num_images = element - init_image;
    int combinations = num_images-1;
    register int base = element;
    register int it = globalMatch.size();
    int k = 0;
    for(register int cam = init_image; k < combinations; ++cam, ++k)
    {
        globalMatch.push_back( MatchQuery() );
        reliableMatch.push_back( false );
        
        int ft_per_im1 = int( descriptorsGPU->at(cam).size()/128 );
        int ft_per_im2 = int( descriptorsGPU->at(base).size()/128 );
        matcher.SetDescriptors(0, ft_per_im1, &(descriptorsGPU->at(cam)[0])); // prepare descriptor1
        matcher.SetDescriptors(1, ft_per_im2, &(descriptorsGPU->at(base)[0])); // prepare descritor2
        
        int match_buf[num_goodmatch][2];
        int nmatch = matcher.GetSiftMatch(num_goodmatch, match_buf); // match descriptors
        globalMatch[it].matches.clear();
        
        if (nmatch < min_nmatch_reliable) // check reliability of match
        {
	  if( !(continuous && (base - cam == 1)) ) // if ICP is enabled do this
	  {
// 		reliableMatch[it] = false;
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
        globalMatch[it].cam_id2 = base;
        DEBUG_1( printf("MATCH: %04i -> %04i:\t#matches = %i\n", cam, base, nmatch); )
        it++;
    }
    DEBUG_2( std::cout << "End solveMatches\n"; )
}

void MatchesMap::solveMatchesOneElement_subgroup(boost::shared_ptr< float_vv > descriptorsGPU, int element, int init_image, int final_image )
{
    solveMatchesOneElement_subgroupDown( descriptorsGPU, init_image, element );
    solveMatchesOneElement_subgroupUp( descriptorsGPU, element, final_image );
}

void MatchesMap::solveMatchesOneElement(boost::shared_ptr< float_vv > descriptorsGPU, int element)
{
    int start = 0;
    int end = descriptorsGPU->size();
    if ( element < end ) solveMatchesOneElement_subgroup( descriptorsGPU, element, start, end ); //verify boundary
}

void MatchesMap::solveMatchesContinuous(boost::shared_ptr< float_vv > descriptorsGPU)
{
    int init_image = 0;
    int final_image = descriptorsGPU->size();
    globalMatch.clear();
    reliableMatch.clear();
    DEBUG_1( std::cout << "\n================================ MATCH Features ==================================\n"; )
    solveMatchesContinuous_subgroup( descriptorsGPU, init_image, final_image );
    solveMatchesOneElement( descriptorsGPU, final_image - 1 ); // do this for loop closure
}

void MatchesMap::solveMatches(boost::shared_ptr< float_vv > descriptorsGPU)
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
     * 0) => 0-1; 1) => 0-2; 2) => 0-3; 3) => 0-4; 4) => 1-2; 5) => 1-3; 6) => 1-4; 7) => 2-3; 8) => 2-4; 9) => 3-4; \n
     * The combinations are computed in this order to get a good solution in solveDB where image features are introduced to achieve loop closure.
     */ 
    
    int init = 0;
    int final_image = descriptorsGPU->size();
    globalMatch.clear();
    reliableMatch.clear();
    DEBUG_1( std::cout << "\n================================ MATCH Features ==================================\n"; )
    solveMatchesPairs_subgroup( descriptorsGPU, init, final_image );
}

void MatchesMap::solveMatchesGroups(boost::shared_ptr< float_vv > descriptorsGPU, int groupsize)
{
    if( groupsize < 1 )
    {
        DEBUG_E( ("Group Size must be equal or greater than one") ); 
        exit(-1);
    }
    
    int gsize = groupsize;
    int total_images = descriptorsGPU->size();
    int groups = ceil( (float)total_images/ float(gsize) );
    DEBUG_1( std::cout << "\n================================ MATCH Features ==================================\n"; )
    for(register int cam = 0; cam < groups; ++cam)
    {
        int final_image = (cam + 1)*gsize + 1;
        int element = final_image;
        if( final_image >= total_images )
        {
	  final_image = total_images;
	  element = final_image - 1;
        }
        DEBUG_2( printf("Group: %i - %i, size: %i\n", cam*gsize, final_image, groups ); )
        solveMatchesPairs_subgroup( descriptorsGPU, cam*gsize, final_image );
        solveMatchesOneElement( descriptorsGPU, element );
    }
}


void MatchesMap::solveMatchesGroups(boost::shared_ptr< float_vv > descriptorsGPU, std::vector<int> *cluster)
{
    if( cluster->size() < 1 )
    {
        DEBUG_E( ("Group Size must be equal or greater than one") ); 
        exit(-1);
    }
    
    int previous_image = 0;
    int total_images = descriptorsGPU->size();
    DEBUG_1( std::cout << "\n================================ MATCH Features ==================================\n"; )
    for(int cam = 0; cam < cluster->size(); ++cam)
    {
        int final_image = cluster->at(cam);
        int element = final_image;
        if( final_image >= total_images )
        {
	  final_image = total_images;
	  element = final_image - 1;
        }
        solveMatchesPairs_subgroup( descriptorsGPU, previous_image, final_image );
        solveMatchesOneElement( descriptorsGPU, element );
        previous_image = final_image - 1;
    }
}
    
    

void MatchesMap::robustifyMatches(boost::shared_ptr< kpCV_vv > set_of_keypoints)
{
    DEBUG_1( std::cout << "\nRobustify matches with fundamental matrix:\n"; )
    for(register int it = 0; it < reliableMatch.size(); ++it)
    {
        if (reliableMatch[it])
        {
	  int cam1 = globalMatch[it].cam_id1;
	  int cam2 = globalMatch[it].cam_id2;
	  
	  std::vector<cv::KeyPoint> kps1 = set_of_keypoints->at(cam1);
	  std::vector<cv::KeyPoint> kps2 = set_of_keypoints->at(cam2);
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

void MatchesMap::robustifyMatches(boost::shared_ptr< kpGPU_vv > set_of_keypoints)
{
    DEBUG_1( std::cout << "\nRobustify matches with fundamental matrix:\n"; )
    for(register int it = 0; it < reliableMatch.size(); ++it)
    {
        if (reliableMatch[it])
        {
	  int cam1 = globalMatch[it].cam_id1;
	  int cam2 = globalMatch[it].cam_id2;
	  
	  std::vector<SiftGPU::SiftKeypoint> kps1 = set_of_keypoints->at(cam1);
	  std::vector<SiftGPU::SiftKeypoint> kps2 = set_of_keypoints->at(cam2);
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

void MatchesMap::depthFilter(boost::shared_ptr< kpGPU_vv > set_of_keypoints,
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
	      pts1.push_back( cv::Point2d( set_of_keypoints->at(cam1)[qid].x, set_of_keypoints->at(cam1)[qid].y) );
	      pts2.push_back( cv::Point2d( set_of_keypoints->at(cam2)[tid].x, set_of_keypoints->at(cam2)[tid].y) );
	  }
	  // Remove bad points with depth projection
            std::vector<int> goodpt = removeBadPointsDual(pts1, pts2, set_of_depth->at(cam1), set_of_depth->at(cam2));
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

void MatchesMap::depthFilter(boost::shared_ptr< kpGPU_vv > set_of_keypoints,
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
	  cv::Mat depth1 = cv::imread( depth_list->at(cam1), -1);
	  cv::Mat depth2 = cv::imread(depth_list->at(cam2), -1);
        
	  for (register int ft = 0; ft < globalMatch[it].matches.size(); ++ft)	//Extract pts1 from matches
	  {
	      int qid = globalMatch[it].matches[ft].queryIdx;
	      int tid = globalMatch[it].matches[ft].trainIdx;
	      pts1.push_back( cv::Point2d( set_of_keypoints->at(cam1)[qid].x, set_of_keypoints->at(cam1)[qid].y) );
	      pts2.push_back( cv::Point2d( set_of_keypoints->at(cam2)[tid].x, set_of_keypoints->at(cam2)[tid].y) );
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


void MatchesMap::solveDB( HandleDB *mydb, boost::shared_ptr< kpGPU_vv > keypointsGPU )
{
    actualfeature = 0;
    // loop closure code
    for (register int it = 0; it < reliableMatch.size(); ++it)
    {
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
		mydb->insertRow( sf1, cam2, idft_c2, keypointsGPU->at(cam2)[idft_c2].x, keypointsGPU->at(cam2)[idft_c2].y );
// 		continue;
	      }
	      else if (find2)
	      {
		mydb->insertRow( sf2, cam1, idft_c1, keypointsGPU->at(cam1)[idft_c1].x, keypointsGPU->at(cam1)[idft_c1].y );
// 		continue;
	      }
	      else
	      {
		mydb->insertRow( actualfeature, cam1, idft_c1, keypointsGPU->at(cam1)[idft_c1].x, keypointsGPU->at(cam1)[idft_c1].y );
		mydb->insertRow( actualfeature, cam2, idft_c2, keypointsGPU->at(cam2)[idft_c2].x, keypointsGPU->at(cam2)[idft_c2].y );
		actualfeature++; // A new feature is added
	      }
	  }  
        }
    }
}

// 	      bool inc1 = mydb->insertUnique(actualfeature, cam1, idft_c1, 
// 				     keypointsGPU->at(cam1)[idft_c1].x, keypointsGPU->at(cam1)[idft_c1].y );
// 	      
// 	      bool inc2 = mydb->insertUnique(actualfeature, cam2, idft_c2, 
// 				     keypointsGPU->at(cam2)[idft_c2].x, keypointsGPU->at(cam2)[idft_c2].y );


void MatchesMap::solveDB3D( HandleDB *mydb, boost::shared_ptr< kpGPU_vv > keypointsGPU,
		        std::vector< std::string > *depth_list, Eigen::Matrix3d &calibration )
{
    actualfeature = 0;
    // loop closure code
    for (register int it = 0; it < reliableMatch.size(); ++it)
    {
//         std::cout << "it = " << it << "\n";
        if (reliableMatch[it])
        {
	  int cam1 = globalMatch[it].cam_id1;
	  int cam2 = globalMatch[it].cam_id2;
	  cv::Mat depth1 = cv::imread( depth_list->at(cam1), -1 );
	  cv::Mat depth2 = cv::imread( depth_list->at(cam2), -1 );
	  
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
		int u = (int) round( keypointsGPU->at(cam2)[idft_c2].x );
		int v = (int) round( keypointsGPU->at(cam2)[idft_c2].y );
		Eigen::Vector3f vv = projection<int,double,float>( u, v, depth2, calibration);
		mydb->insertRow3D( sf1, cam2, idft_c2, vv(0), vv(1), vv(2) );
// 		continue;
	      }
	      else if (find2)
	      {
		int u = (int) round( keypointsGPU->at(cam1)[idft_c1].x );
		int v = (int) round( keypointsGPU->at(cam1)[idft_c1].y );
		Eigen::Vector3f vv = projection<int,double,float>( u, v, depth1, calibration);
		mydb->insertRow3D( sf2, cam1, idft_c1, vv(0), vv(1), vv(2) );
// 		continue;
	      }
	      else
	      {
		int u1 = (int) round( keypointsGPU->at(cam1)[idft_c1].x );
		int v1 = (int) round( keypointsGPU->at(cam1)[idft_c1].y );
		Eigen::Vector3f vv1 = projection<int,double,float>( u1, v1, depth1, calibration);
		int u2 = (int) round( keypointsGPU->at(cam2)[idft_c2].x );
		int v2 = (int) round( keypointsGPU->at(cam2)[idft_c2].y );
		Eigen::Vector3f vv2 = projection<int,double,float>( u2, v2, depth2, calibration);
		mydb->insertRow3D( actualfeature, cam1, idft_c1, vv1(0), vv1(1), vv1(2) );
		mydb->insertRow3D( actualfeature, cam2, idft_c2, vv2(0), vv2(1), vv2(2) );
		actualfeature++; // A new feature is added
	      }
	  }  
        }
    }
}


void MatchesMap::exportTXT( const char *file_txt, boost::shared_ptr< kpGPU_vv > keypointsGPU )
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
	      
	      myfile1 << cam1 << "\t" <<  idft_c1 << "\t" << keypointsGPU->at(cam1)[idft_c1].x
		        << "\t" << keypointsGPU->at(cam1)[idft_c1].y << "\n";
	      myfile1 << cam2 << "\t" <<  idft_c2 << "\t" << keypointsGPU->at(cam2)[idft_c2].x
		        << "\t" << keypointsGPU->at(cam2)[idft_c2].y << "\n";
	       
	  }
        }
    }
    myfile1.close();
    return;
}

void MatchesMap::plot( std::vector< cv::Mat > *images, boost::shared_ptr< kpCV_vv > set_of_keypoints )
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
	  drawMatches( images->at(cam1), set_of_keypoints->at(cam1), images->at(cam2), set_of_keypoints->at(cam2),
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