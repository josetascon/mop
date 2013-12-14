// Created: Aug/02/2013
// Author: José David Tascón Vidarte

#include "FeaturesEDM.hpp"

// ================================================================================================
// ================================= FUNCTIONS of CLASS SiftED ====================================
// ================================================================================================
SiftED::SiftED( std::vector< std::string > filenameIms )
{
    nameImages = filenameIms;
    num_images = nameImages.size();
    descriptorsGPU.resize(num_images);
    keypointsGPU.resize(num_images);
    
    set_of_images.resize(num_images);
    set_of_keypoints.resize(num_images);
    set_of_descriptors.resize(num_images);
};

SiftED::~SiftED()
{
//     nameImages.clear();
};   

void SiftED::loadImages()
{
    set_of_images.clear();
    for (int i = 0; i < nameImages.size(); i++ )	//Store all images in std::vector<cv::Mat>
    {
        set_of_images.push_back( cv::imread(nameImages[i], 1) );
    }
}

cv::Mat SiftED::image(int num)
{
    return set_of_images[num];
}

std::vector<cv::KeyPoint> SiftED::KeyPoint(int num)
{
    return set_of_keypoints[num];
}

void SiftED::solveSift()
{
    int ft_per_im[num_images];
    const char *files[num_images];
//     char *pfiles = &files[0];
    
    //create a SiftGPU instance
    SiftGPU sift;
    //processing parameters first
//     char * s_argv[] = {(char*)"-fo", (char*)"-1", (char*)"-v", (char*)"0"};
//     sift.ParseParam(4, s_argv);
    char * s_argv[] = {(char*)"-fo", (char*)"-1", (char*)"-v", (char*)"0", (char*)"-tc2", (char*)"1000" }; // tc2 limit max features
    sift.ParseParam(6, s_argv);
    
    //create an OpenGL context for computation
    int support = sift.CreateContextGL();
//     int argc = 1;
//     char **argv;
//     glutInit(&argc, argv);
//     glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
//     glutInitWindowSize(640, 480);
//     glutCreateWindow("Sift");
    
    
    //call VerfifyContexGL instead if using your own GL context
//     int support = sift.VerifyContextGL();
    if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED)
    {
        fprintf(stderr, "Can't open OpenGL context for SiftGPU\n");
        return;
    }
    
    for (int k = 0; k < num_images; k++) files[k] = nameImages[k].c_str();
    
    sift.SetImageList(num_images, files);
    std::cout << "\n================================ SIFT Features E&D ==================================\n";
    
    for (register int k = 0; k < num_images; ++k)
    {
        sift.RunSIFT(k); //process an image
        ft_per_im[k] = sift.GetFeatureNum(); //get feature count
        keypointsGPU[k].resize(ft_per_im[k]);
        descriptorsGPU[k].resize(128*ft_per_im[k]);
        
        sift.GetFeatureVector(&keypointsGPU[k][0], &descriptorsGPU[k][0]); //specify NULL if you don’t need keypoints or descriptors
        printf("SIFT: Image %04i, \t#features = %i\n", k, ft_per_im[k]);
//         std::string str = "SIFT: %04i,";
//         str.append(nameImages[k]);
//         str.append("\t #f = %i\n");
//         printf(str.c_str(),k,ft_per_im[k]);
    }
};

void SiftED::enableKeyPoint()
{
    for (register int k = 0; k < num_images; ++k)
    {
        int ft_per_im = keypointsGPU[k].size();
        set_of_keypoints[k].resize(ft_per_im);
        for (int i = 0; i < ft_per_im ; i++)
        {
	  set_of_keypoints[k][i] = cv::KeyPoint( cv::Point2f(keypointsGPU[k][i].x, keypointsGPU[k][i].y),
			  keypointsGPU[k][i].s, keypointsGPU[k][i].o );
        }
    }
}

// std::vector< std::vector<float> > SiftED::get_descriptorsGPU()
// {
//     return descriptorsGPU;
// }
//     
// std::vector< std::vector<SiftGPU::SiftKeypoint> > SiftED::get_keypointsGPU()
// {
//     return keypointsGPU;
// }


// ================================================================================================
// =============================== FUNCTIONS of CLASS MatchesMap ==================================
// ================================================================================================

std::vector<cv::DMatch> MatchesMap::match(int nmatch)
{
    return globalMatch[nmatch].matches;
}

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
//     int combinations = num_images - 1; // Due to solve only continous matches
    globalMatch.resize(num_images - 1);
    reliableMatch.resize(num_images - 1);
    std::cout << "\n================================ MATCH Features ==================================\n";
    
    for(register int cam = 0; cam < num_images - 1; ++cam)
    {
        int ft_per_im1 = int( (*descriptorsGPU)[cam].size()/128 );
        int ft_per_im2 = int( (*descriptorsGPU)[cam+1].size()/128 );
        matcher.SetDescriptors(0, ft_per_im1, &((*descriptorsGPU)[cam][0])); // prepare descriptor1
        matcher.SetDescriptors(1, ft_per_im2, &((*descriptorsGPU)[cam+1][0])); // prepare descritor2
        
        int match_buf[num_goodmatch][2];
        int nmatch = matcher.GetSiftMatch(num_goodmatch, match_buf); // match descriptors
        globalMatch[cam].matches.clear();
        
        if (nmatch < min_nmatch_reliable) // check reliability of match
        {
	  reliableMatch[cam] = false;
	  continue;
        }
        
        reliableMatch[cam] = true;
        for (int i = 0; i < nmatch ; i++) // copy matches to internal match object
        {
	  globalMatch[cam].matches.push_back(cv::DMatch( match_buf[i][0], match_buf[i][1], i ));
        }
        globalMatch[cam].cam_id1 = cam;
        globalMatch[cam].cam_id2 = cam+1;
        printf("MATCH: %04i -> %04i:\t#matches = %i\n", cam, cam+1, nmatch);
    }
//     std::cout << "End solveMatches\n";
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
    std::cout << "\n================================ MATCH Features ==================================\n";
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
	      reliableMatch[it] = false;
	      it++;
	      continue;
	  }
	  
	  reliableMatch[it] = true;
	  for (int i = 0; i < nmatch ; i++) // copy matches to internal match object
	  {
	      globalMatch[it].matches.push_back(cv::DMatch( match_buf[i][0], match_buf[i][1], i ));
	  }
	  globalMatch[it].cam_id1 = cam;
	  globalMatch[it].cam_id2 = cam+stride;
	  printf("MATCH: %04i -> %04i:\t#matches = %i\n", cam, cam+stride, nmatch);
	  it++;
        }
        stride++;
    }
//     std::cout << "End solveMatches\n";
}

void MatchesMap::robustifyMatches(std::vector< std::vector< cv::KeyPoint > > *set_of_keypoints)
{
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
// 	  std::cout << "matches size = " << globalMatch[it].matches.size() << "\n";
// 	  std::cout << "pts1 size = " << pts1.size() << "\n";
// 	  std::cout << "pts2 size = " << pts2.size() << "\n";
// 	  keyPointstoPoints(kps1, pts1);
// 	  keyPointstoPoints(kps2, pts2);
	  cv::Mat F_est;
	  int inlier = robustMatchesfromFundamental(pts1, pts2, globalMatch[it].matches, F_est, 5);
	  printf("MATCH: %04i -> %04i:\t#matches = %i\n", cam1, cam2, inlier);
	  if (pts1.size() < min_nmatch_reliable) reliableMatch[it] = false;
        }
    }
}

void MatchesMap::robustifyMatches(std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints)
{
    std::cout << "\nRobustify matches with fundamental matrix:\n";
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
// 	  std::cout << "matches size = " << globalMatch[it].matches.size() << "\n";
// 	  std::cout << "pts1 size = " << pts1.size() << "\n";
// 	  std::cout << "pts2 size = " << pts2.size() << "\n";
// 	  keyPointstoPoints(kps1, pts1);
// 	  keyPointstoPoints(kps2, pts2);
	  cv::Mat F_est;
	  int inlier = robustMatchesfromFundamental(pts1, pts2, globalMatch[it].matches, F_est, 5);
	  printf("MATCH: %04i -> %04i:\t#matches = %i\n", cam1, cam2, inlier);
	  if (pts1.size() < min_nmatch_reliable) reliableMatch[it] = false;
        }
    }
}

void MatchesMap::depthFilter(std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints,
		         std::vector< cv::Mat > *set_of_depth, const int reliable )
{
    std::cout << "\nFilter matches with valid depth values:\n";
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
	      reliableMatch[it] = false;
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
	      printf("MATCH: %04i -> %04i:\t#matches = %i\n", cam1, cam2, globalMatch[it].matches.size());
// 	      std::cout << "Erase Matches Size: " << globalMatch[it].matches.size() << "\n";
	  }
        }
    }
}

void MatchesMap::depthFilter(std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints,
		         std::vector< std::string > *depth_list, const int reliable )
{
    std::cout << "\nFilter matches with valid depth values:\n";
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
	      reliableMatch[it] = false;
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
	      printf("MATCH: %04i -> %04i:\t#matches = %i\n", cam1, cam2, globalMatch[it].matches.size());
// 	      std::cout << "Erase Matches Size: " << globalMatch[it].matches.size() << "\n";
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

// ================================================================================================
// ============================= FUNCTIONS of CLASS SimpleSiftGPU =================================
// ================================================================================================
SimpleSiftGPU::SimpleSiftGPU(int num_match): matcher(num_match), num_matches(num_match)
{
    //processing parameters first
//     char * s_argv[] = {(char*)"-fo", (char*)"-1", (char*)"-v", (char*)"1"};
//     sift.ParseParam(4, s_argv);
    
    char * s_argv[] = {(char*)"-fo", (char*)"-1", (char*)"-v", (char*)"1", (char*)"-tc2", (char*)"1000" }; // tc2 limit max features
    sift.ParseParam(6, s_argv);
    int support = sift.CreateContextGL();
    if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED)
    {
        fprintf(stderr, "Can't open OpenGL context for SiftGPU\n");
        return;
    }
};

SimpleSiftGPU::~SimpleSiftGPU() { };

void SimpleSiftGPU::solveFeatures( cv::Mat &Image, std::vector<SiftGPU::SiftKeypoint> &keypoints, std::vector<float> &descriptors )
{
    unsigned char *p_rgb = Image.data;
    sift.RunSIFT(Image.size().width, Image.size().height, p_rgb, GL_RGB, GL_UNSIGNED_BYTE);
//     sift.RunSIFT(nameImage.c_str());
    int num = sift.GetFeatureNum();//get feature count
    descriptors.resize(128*num);
    keypoints.resize(num);
    sift.GetFeatureVector(&keypoints[0], &descriptors[0]);
}

void SimpleSiftGPU::solveFeatures( cv::Mat &Image, std::vector<cv::KeyPoint> &keypoints, std::vector<float> &descriptors )
{
    std::vector<SiftGPU::SiftKeypoint> keyGPU;
    SimpleSiftGPU::solveFeatures( Image, keyGPU, descriptors );
    keypoints.clear();
    for (register int i = 0; i < keyGPU.size(); ++i) keypoints.push_back( cv::KeyPoint( cv::Point2f(keyGPU[i].x, keyGPU[i].y) , keyGPU[i].s, keyGPU[i].o ) );
}

void SimpleSiftGPU::solveFeatures( std::string nameImage, std::vector<SiftGPU::SiftKeypoint> &keypoints, std::vector<float> &descriptors )
{
    sift.RunSIFT(nameImage.c_str());
    int num = sift.GetFeatureNum();//get feature count
    descriptors.resize(128*num);
    keypoints.resize(num);
    sift.GetFeatureVector(&keypoints[0], &descriptors[0]);
}

int SimpleSiftGPU::solveMatches( std::vector<SiftGPU::SiftKeypoint> &keypoints1, std::vector<SiftGPU::SiftKeypoint> &keypoints2,
			   std::vector<float> &descriptors1, std::vector<float> &descriptors2, 
			     std::vector<cv::Point2d> &pt1, std::vector<cv::Point2d> &pt2 )
{
    if(matcher.VerifyContextGL() == 0)
    {
        fprintf(stderr, "Can't open OpenGL context for Matcher in SiftGPU\n");
        return -1;
    }
    //Set two sets of descriptor data to the matcher
    matcher.SetDescriptors(0, descriptors1.size(), &descriptors1[0]);
    matcher.SetDescriptors(1, descriptors2.size(), &descriptors2[0]);
    //Match and read back result to input buffer
    int match_buf[num_matches][2];
    int nmatch = matcher.GetSiftMatch(num_matches, match_buf);
    
    pt1.clear();
    pt2.clear();
    matches.clear();
    for (register int i = 0; i < nmatch ; ++i)
    {
        pt1.push_back(cv::Point2d(keypoints1[match_buf[i][0]].x,keypoints1[match_buf[i][0]].y));
        pt2.push_back(cv::Point2d(keypoints2[match_buf[i][1]].x,keypoints2[match_buf[i][1]].y));
        matches.push_back(cv::DMatch( match_buf[i][0], match_buf[i][1], i ));
    }
    return nmatch;
}

int SimpleSiftGPU::solveMatches( std::vector<SiftGPU::SiftKeypoint> &keyGPU1, std::vector<SiftGPU::SiftKeypoint> &keyGPU2,
			   std::vector<float> &descriptors1, std::vector<float> &descriptors2, 
			   std::vector<cv::Point2d> &pt1, std::vector<cv::Point2d> &pt2,
			   std::vector<cv::KeyPoint> &cvkeypoints1, std::vector<cv::KeyPoint> &cvkeypoints2 )
{
    if(matcher.VerifyContextGL() == 0)
    {
        fprintf(stderr, "Can't open OpenGL context for Matcher in SiftGPU\n");
        return -1;
    }
    //Set two sets of descriptor data to the matcher
    matcher.SetDescriptors(0, descriptors1.size(), &descriptors1[0]);
    matcher.SetDescriptors(1, descriptors2.size(), &descriptors2[0]);
    //Match and read back result to input buffer
    int match_buf[num_matches][2];
    int nmatch = matcher.GetSiftMatch(num_matches, match_buf);
    
    pt1.clear();
    pt2.clear();
    cvkeypoints1.clear();
    cvkeypoints2.clear();
    matches.clear();
    for (register int i = 0; i < nmatch ; ++i) // Only valid keypoints are returned, Matches have features id in order
    {
        cvkeypoints1.push_back( cv::KeyPoint(cv::Point2f(keyGPU1[match_buf[i][0]].x,keyGPU1[match_buf[i][0]].y),
			  keyGPU1[match_buf[i][0]].s, keyGPU1[match_buf[i][0]].o ));
        cvkeypoints2.push_back( cv::KeyPoint(cv::Point2f(keyGPU2[match_buf[i][1]].x,keyGPU2[match_buf[i][1]].y),
			  keyGPU2[match_buf[i][1]].s, keyGPU2[match_buf[i][1]].o ));
        pt1.push_back(cv::Point2d(keyGPU1[match_buf[i][0]].x,keyGPU1[match_buf[i][0]].y));
        pt2.push_back(cv::Point2d(keyGPU2[match_buf[i][1]].x,keyGPU2[match_buf[i][1]].y));
        matches.push_back(cv::DMatch( i, i, i ));
    }
    return nmatch;
}

int SimpleSiftGPU::solveMatches( std::vector<cv::KeyPoint> &cvkeypoints1, std::vector<cv::KeyPoint> &cvkeypoints2,
			   std::vector<float> &descriptors1, std::vector<float> &descriptors2, 
			   std::vector<cv::Point2d> &pt1, std::vector<cv::Point2d> &pt2 )
{
    if(matcher.VerifyContextGL() == 0)
    {
        fprintf(stderr, "Can't open OpenGL context for Matcher in SiftGPU\n");
        return -1;
    }
    //Set two sets of descriptor data to the matcher
    matcher.SetDescriptors(0, descriptors1.size(), &descriptors1[0]);
    matcher.SetDescriptors(1, descriptors2.size(), &descriptors2[0]);
    //Match and read back result to input buffer
    int match_buf[num_matches][2];
    int nmatch = matcher.GetSiftMatch(num_matches, match_buf);
    
    pt1.clear();
    pt2.clear();
    matches.clear();
    for (register int i = 0; i < nmatch ; ++i) // Use cvkeypoints to get points as std::vector<cv::Point2d>
    {
        pt1.push_back(cv::Point2d(cvkeypoints1[match_buf[i][0]].pt.x,cvkeypoints1[match_buf[i][0]].pt.y));
        pt2.push_back(cv::Point2d(cvkeypoints2[match_buf[i][1]].pt.x,cvkeypoints2[match_buf[i][1]].pt.y));
        matches.push_back(cv::DMatch( match_buf[i][0], match_buf[i][1], i ));			// matches saved with original feature id
    }
    return nmatch;
}

// ================================================================================================
// ============================== FUNCTIONS of CLASS ImagesFilter =================================
// ================================================================================================
void ImagesFilter::solveImages()
{
    SiftED feat_handler( filenames );
    feat_handler.solveSift();
    
    
    for(int i = 0; i < num_images_in ; ++i) 
    {
        
        ;
        
        
    }
    ///WORKING HERE
    
    
}





// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================
void keyPointstoPoints(const std::vector< cv::KeyPoint > &kps, std::vector< cv::Point2d > &ps)
{
	ps.clear();
	for (int i=0; i<kps.size(); i++) ps.push_back( cv::Point2d(kps[i].pt.x, kps[i].pt.y) );
}

void pointstoKeyPoints(const std::vector< cv::Point2d > &ps, std::vector< cv::KeyPoint > &kps)
{
	kps.clear();
	for (int i=0; i<ps.size(); i++) kps.push_back( cv::KeyPoint(  cv::Point2f(ps[i].x,ps[i].y) ,1.0f) );
}



void eigenfromMatches(std::vector< cv::DMatch > &matches,
		  std::vector< cv::KeyPoint > &keypoints1, std::vector< cv::KeyPoint > &keypoints2,
		  Eigen::MatrixXd &pts1, Eigen::MatrixXd &pts2)
{
    int num_features = matches.size();
    pts1 = Eigen::MatrixXd::Zero(3,num_features);
    pts2 = Eigen::MatrixXd::Zero(3,num_features);
    
    for( register int k = 0; k < num_features; ++k )
    {
        pts1(0,k) = keypoints1[matches[k].queryIdx].pt.x;
        pts1(1,k) = keypoints1[matches[k].queryIdx].pt.y;
        pts1(2,k) = 1.0;
        pts2(0,k) = keypoints2[matches[k].trainIdx].pt.x;
        pts2(1,k) = keypoints2[matches[k].trainIdx].pt.y;
        pts2(2,k) = 1.0;
    }
    // Debug
//     std::cout << "pts1: \n" << pts1.transpose() << "\n";
//     std::cout << "pts2: \n" << pts2.transpose() << "\n";
}

void eigenfromMatches(std::vector< cv::DMatch > &matches,
		  std::vector< SiftGPU::SiftKeypoint > &keypoints1, std::vector< SiftGPU::SiftKeypoint > &keypoints2,
		  Eigen::MatrixXd &pts1, Eigen::MatrixXd &pts2)
{
    int num_features = matches.size();
    pts1 = Eigen::MatrixXd::Zero(3,num_features);
    pts2 = Eigen::MatrixXd::Zero(3,num_features);
    
    for( register int k = 0; k < num_features; ++k )
    {
        pts1(0,k) = keypoints1[matches[k].queryIdx].x;
        pts1(1,k) = keypoints1[matches[k].queryIdx].y;
        pts1(2,k) = 1.0;
        pts2(0,k) = keypoints2[matches[k].trainIdx].x;
        pts2(1,k) = keypoints2[matches[k].trainIdx].y;
        pts2(2,k) = 1.0;
    }
    // Debug
//     std::cout << "pts1: \n" << pts1.transpose() << "\n";
//     std::cout << "pts2: \n" << pts2.transpose() << "\n";
}

void eigenfromSiftKeypoint(std::vector< SiftGPU::SiftKeypoint > &keys, Eigen::MatrixXf &pts)
{
    int num_features = keys.size();
    pts = Eigen::MatrixXf::Ones(3,num_features);
    for( register int k = 0; k < num_features; ++k )
    {
        pts(0,k) = keys[k].x;
        pts(1,k) = keys[k].y;
    }
}

void removeRepeatedKeyPoints( std::vector< cv::KeyPoint > &keypoints )
{
    // I create this due to the repeated keypoints features obtained with gridSiftFeature. O(n) algorithm
    std::vector< cv::KeyPoint > output;
    output.push_back(keypoints[0]); //Adding first element
    for (register int k = 1; k < keypoints.size(); ++k)
    {
        float distance = euclideanDistanceofcvPoints( keypoints[k].pt, keypoints[k-1].pt );
        if (distance > 2.0) // If distance between subsequent points is no greater than 2pixels units then is deleted
        {
	  output.push_back(keypoints[k]);
        }
    }
    keypoints = output;
    return;
};

void gridSiftFeatureDetect( cv::Mat &image, std::vector< cv::KeyPoint > &keypoints, 
		         int num_features, int grid_rows, int grid_cols)
{
	int features_per_grid = ceil( num_features/(grid_rows*grid_cols) );
	int col_n = ceil(image.cols/grid_cols);
	int row_n = ceil(image.rows/grid_rows);
	
	std::vector< cv::KeyPoint > sub_keypoints;
	cv::SiftFeatureDetector detector( features_per_grid, 3 , 0.06 ); // ( features_per_grid, 3 , 0.08 );
	keypoints.clear();
	for (int i = 0; i < grid_cols; i++)
	{
		for (int j = 0; j < grid_rows; j++)
		{
			
			cv::Mat sub_image (image, cv::Rect(i*col_n, j*row_n, col_n - 1, row_n - 1) );
			detector.detect( sub_image, sub_keypoints );
			for (register int k = 0; k < sub_keypoints.size(); ++k) 
			{
				sub_keypoints[k].pt = sub_keypoints[k].pt + cv::Point2f(i*col_n, j*row_n);
				keypoints.push_back( sub_keypoints[k]);
			}
		}
		
	}
	return;
	
}

void featureDetectandDescript(std::vector< cv::Mat > &set_of_images, 
		        std::vector< std::vector< cv::KeyPoint > > &set_of_keypoints,
		        std::vector< cv::Mat > &set_of_descriptors,
		        int num_features, int grid_rows, int grid_cols )
{
	std::cout << "Features Computation Start" << '\n';
	int num_set = set_of_images.size();
	cv::SiftDescriptorExtractor descriptor;
	
	set_of_keypoints.resize(num_set);
	set_of_descriptors.resize(num_set);
	
	for (register int i = 0; i < set_of_images.size(); ++i)
	{
		// ==================== Step 1: Detect the keypoints using SIFT Detector ====================
		gridSiftFeatureDetect ( set_of_images[i], set_of_keypoints[i], num_features, grid_rows, grid_cols);
		removeRepeatedKeyPoints( set_of_keypoints[i] );
		// ==================== Step 2: Calculate descriptors (feature vectors) ====================
		descriptor.compute( set_of_images[i], set_of_keypoints[i], set_of_descriptors[i] ); // Descritor computation  to cv::Mat descriptors
	}
	// Debug
	std::cout << "Features Computation Finish" << '\n';
	return;
}

void featureDetectandDescript(cv::Mat &image, 
			std::vector< cv::KeyPoint > &keypoints,
			cv::Mat &descriptors,
			int num_features, int grid_rows, int grid_cols )
{
	std::cout << "Features Computation Start" << '\n';
	// ==================== Step 1: Detect the keypoints using SIFT Detector ====================
	gridSiftFeatureDetect ( image, keypoints,num_features, grid_rows, grid_cols);
	removeRepeatedKeyPoints( keypoints );
	std::cout << "Features Extraction" << '\n';
	std::cout << "Total KeyPoints extracted = " << keypoints.size() <<'\n';
	// ==================== Step 2: Calculate descriptors (feature vectors) ====================
	cv::SiftDescriptorExtractor *descriptor = new cv::SiftDescriptorExtractor;
	descriptor->compute( image, keypoints, descriptors ); // Descritor computation  to cv::Mat descriptors
	delete descriptor;
	// Debug
	std::cout << "Features Descriptor" << '\n';
	std::cout << "Features Computation Finish" << '\n';
	
	return;
}

void eliminateRepeatedMatches( std::vector< cv::DMatch > &matches )
{
    std::vector< cv::DMatch > goodmatches;
    std::set<int> existing_trainIdx;
    for(register int i = 0; i < matches.size(); ++i )
    { 
        //std::cout << "Train Idxs = " << (matches)[i].trainIdx <<" ";
        if( existing_trainIdx.find((matches)[i].trainIdx) == existing_trainIdx.end() && 
           (matches)[i].trainIdx >= 0 )
        {
            goodmatches.push_back( (matches)[i]);
            existing_trainIdx.insert((matches)[i].trainIdx);
        }
    }
    
    matches = goodmatches;
    return;
}

void extractPointsfromMatches( std::vector< cv::DMatch > &goodmatches, 
			  std::vector< cv::KeyPoint > &keypoints1, std::vector< cv::KeyPoint > &keypoints2,
			  std::vector< cv::Point2d > &pts1, std::vector< cv::Point2d > &pts2)
{
    pts1.clear();
    pts2.clear();
    // **** Extracting KeyPoint indexes from good matches ****
    for(register int k = 0; k < goodmatches.size(); ++k )
    {
        cv::Point2f pts_tmp1 = keypoints1[goodmatches[k].queryIdx].pt;
        cv::Point2f pts_tmp2 = keypoints2[goodmatches[k].trainIdx].pt;
        pts1.push_back( cv::Point2d( (double)pts_tmp1.x, (double)pts_tmp1.y) );
        pts2.push_back( cv::Point2d( (double)pts_tmp2.x, (double)pts_tmp2.y) );
        // Debug
        // 		std::cout << "Match " << k << ":\t";
        // 		std::cout << keypoints1[goodmatches[k].queryIdx].pt << "\t||\t";
        // 		std::cout << keypoints2[goodmatches[k].trainIdx].pt << '\n';
    }
}

void extractPointsfromMatches( std::vector< cv::DMatch > &goodmatches, 
			  std::vector< SiftGPU::SiftKeypoint > &keypoints1, std::vector< SiftGPU::SiftKeypoint > &keypoints2,
			  std::vector< cv::Point2d > &pts1, std::vector< cv::Point2d > &pts2)
{
    pts1.clear();
    pts2.clear();
    // **** Extracting KeyPoint indexes from good matches ****
    for(register int k = 0; k < goodmatches.size(); ++k )
    {
        pts1.push_back( cv::Point2d( keypoints1[goodmatches[k].queryIdx].x, keypoints1[goodmatches[k].queryIdx].y) );
        pts2.push_back( cv::Point2d( keypoints2[goodmatches[k].trainIdx].x, keypoints2[goodmatches[k].trainIdx].y) );
        // Debug
        // 		std::cout << "Match " << k << ":\t";
        // 		std::cout << keypoints1[goodmatches[k].queryIdx].pt << "\t||\t";
        // 		std::cout << keypoints2[goodmatches[k].trainIdx].pt << '\n';
    }
}

void distanceRangeofMatches( std::vector< cv::DMatch > &matches, std::vector< double > &distances, double &min, double &max)
{
    // Calculating distance limits of matches
    double min_dist = 200.0;
    double max_dist = 0.0;
    distances.clear();
    // **** Quick calculation of max and min distances between keypoints ****
    std::cout << "Distances:\n";
    for(register int k = 0; k < matches.size(); ++k )
    { 
        double dist = matches[k].distance;
        if( dist < min_dist ) min_dist = dist;
		   if( dist > max_dist ) max_dist = dist;
		   distances.push_back(dist);
        //cout << "Match " << k << ": " << matches[k].distance << '\n';
    }
    std::cout << "\t-- Min dist : " << min_dist << '\n';
        std::cout << "\t-- Max dist : " << max_dist << '\n';
        min = min_dist;
        max = max_dist;
        return;
}



void goodMatchesfromDistances(std::vector< cv::DMatch > &matches, std::vector< double > &distances, int num_goodmatch,
			std::vector< cv::DMatch > &goodmatches)
{
    std::vector< int > index_match;
    SortObject myobject;
    int limit = matches.size() > num_goodmatch? num_goodmatch: matches.size();
    // **** Sorting Indexes of min distances for good matches ****
    index_match.resize(distances.size());
    for (register int i=0; i < index_match.size(); ++i) index_match[i] = i;
    myobject.value = distances;
    std::sort(index_match.begin(), index_match.end(), myobject);
    
    // **** Good matches assigment ****
    goodmatches.clear();
    // 	std::cout << "GOOD:\n";
    for(register int k = 0; k < limit; ++k )
    {
        // 		std::cout << "Match " << k << ": " << matches[index_match[k]].distance << '\n';
        goodmatches.push_back(matches[index_match[k]]);
    }
    return;
}

void goodMatcheswithEuclidean( std::vector< cv::DMatch > &matches,
			 std::vector< cv::KeyPoint > &keypoints1, std::vector< cv::KeyPoint > &keypoints2,
			 float thold, int num_ecm)
{
    float sigma_euclid = 20.0;
    float ecm_matches;
    std::vector< cv::DMatch > out_matches;
    //Eigen::VectorXd distVector = Eigen::VectorXd::Zero(matches.size());
    std::vector<float> distVector(matches.size(), 0.0); //initilize vector with 0
    
    for (register int k = 0; k < matches.size(); ++k)
    {
        //if (matches[k].distance > thold_dm) break;
        distVector[k] = euclideanDistanceofcvPoints(keypoints1[matches[k].queryIdx].pt, keypoints2[matches[k].trainIdx].pt);
    }
    //Eigen::VectorXd dVinit = distVector.segment(0,num_ecm); 
    std::vector<float> dVinit(distVector.begin(), distVector.begin() + num_ecm);
    std::sort(dVinit.begin(), dVinit.end());
    ecm_matches = dVinit[(int)(num_ecm/2)];
    //for (int k = 0; k < distVector.size(); k++)std::cout << "Euclidean Distance vector\n" << distVector[k] << '\n'; 
    //std::cout << "MEAN of Euclidean Distance vector\n" << ecm_matches << '\n';
    for (register int i=0; i < matches.size(); ++i)
    {
        //std::cout << "Match " << i << ", distance = " << matches[i].distance << '\n';
        if (matches[i].distance < thold)
        {
	  float eucd = distVector[i];
	  //std::cout << "Euclidean " << i << ", distance = " << eucd << '\n';
	  if (eucd < ecm_matches + sigma_euclid && eucd > ecm_matches - sigma_euclid)
	  {
	      out_matches.push_back(matches[i]);
	  }
	  
        }
        else break;
    }
    matches = out_matches;
//     std::cout << "Number of Final matches (out) " << out_matches.size() << '\n';
//     std::cout << "Number of Final matches (match) " << matches.size() << '\n';
    return;
}

void solveFeatures( cv::Mat &image1, cv::Mat &image2,
		std::vector< cv::KeyPoint > &keypoints1 , std::vector< cv::KeyPoint > &keypoints2,
		cv::Mat &descriptors1, cv::Mat &descriptors2, std::vector< cv::DMatch > &goodmatches,
		int num_features, int num_goodmatch,
		bool debug, bool draw)
{
    // Simply solve features with distances of matches
    
    // Internal parameters
    double min_dist, max_dist;
    
    cv::Mat draw_match, image_key1, image_key2;
    std::vector< cv::DMatch > matches;
    std::vector<double> distances;
    
    // ========================================== Features ==========================================
    featureDetectandDescript(image1, keypoints1, descriptors1, num_features);
    featureDetectandDescript(image2, keypoints2, descriptors2, num_features);
    
    //Print keypoints1 and keypoints2
//     if (debug)
//     {
//         for (int i=0; i < keypoints1.size(); i++)
//         {
// 	  std::cout << "Keypoint of Image1 number "<< i << ":\t"<< keypoints1[i].pt <<'\n';
//         }
//         for (int i=0; i < keypoints2.size(); i++)
//         {
// 	  std::cout << "Keypoint of Image2 number "<< i << ":\t"<< keypoints2[i].pt <<'\n';
//         }
//     }
    
    // ========================================== Matches ==========================================
    //DescriptorMatcher matcher;
    cv::FlannBasedMatcher matcher;
    matcher.match( descriptors1, descriptors2, matches );
    std::cout << "Fetures Matches" << '\n';
    std::cout << "# of matches = " << matches.size() << '\n';
    eliminateRepeatedMatches(matches);
    std::cout << "After eliminate repeated, # matches = " << matches.size() << '\n';
    distanceRangeofMatches(matches, distances, min_dist, max_dist);
    //std::cout << "Function goodMatchesfromDistances\n";
    goodMatchesfromDistances( matches, distances, num_goodmatch, goodmatches);
    //std::cout << "End Function goodMatchesfromDistances\n";
    
    if (draw)
    {
        drawKeypoints(image1, keypoints1, image_key1, cv::Scalar(0,0,255));
        drawKeypoints(image2, keypoints2, image_key2, cv::Scalar(0,0,255));
        
        imshow( "im1", image_key1 );
        imshow( "im2", image_key2 );
        
        cvWaitKey(0);
        
        drawMatches( image1, keypoints1, image2, keypoints2,
		goodmatches, draw_match, cv::Scalar::all(-1), cv::Scalar::all(-1),
		std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    
        imshow( "MATCHES 01", draw_match );
        cvWaitKey(0);
    }
    // Call externally 
    // extractPointsfromMatches( goodmatches, keypoints1, keypoints2, pts1, pts2);
    std::cout << "Solved Features\n";
}

void solveFeaturesEuclidean( cv::Mat &image1, cv::Mat &image2,
		std::vector< cv::KeyPoint > &keypoints1 , std::vector< cv::KeyPoint > &keypoints2,
		cv::Mat &descriptors1, cv::Mat &descriptors2, std::vector< cv::DMatch > &goodmatches,
		int num_features, int num_goodmatch,
		bool debug, bool draw)
{
     
    // Internal parameters
    int num_ecm = 12; 		// num_euclid_mean, number of euclidean distances to solve a mean value
    //float thold_dm = 200.0; 		// threshold of distance of matches reliable
    float thold_dm_max = 350.0; 	// maximum allowed and to check euclidean distance
    
    std::vector< cv::DMatch > matches;
    
    solveFeatures( image1, image2, keypoints1 , keypoints2, descriptors1, 
	         descriptors2, matches, num_features, num_goodmatch, debug, draw);
    // ========================================== Matches ==========================================
    //std::cout << "Function extractPointsfromMatches\n";
    //extractPointsfromMatches( matches, keypoints1, keypoints2, pts1, pts2);
    
    //function
    std::cout << "Function goodMatcheswithEuclidean\n";
    goodMatcheswithEuclidean(matches, keypoints1, keypoints2, thold_dm_max, num_ecm);
    goodmatches = matches;
    std::cout << "Number of Final matches " << goodmatches.size() << '\n';
    if (draw)
    {
        cv::Mat draw_match;
        drawMatches( image1, keypoints1, image2, keypoints2,
		goodmatches, draw_match, cv::Scalar::all(-1), cv::Scalar::all(-1),
		std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    
        imshow( "MATCHES After Euclidean Distance Check", draw_match );
        cvWaitKey(0);
    }
}

void solveFeaturesGPU( std::string nameImage1, std::string nameImage2,
		std::vector< cv::KeyPoint > &keypoints1 , std::vector< cv::KeyPoint > &keypoints2,
		std::vector< cv::DMatch > &matches,
		int num_goodmatch,
		bool debug, bool draw)
{
    //create a SiftGPU instance
    SiftGPU sift;
    //processing parameters first
    char * s_argv[] = {(char*)"-fo", (char*)"-1", (char*)"-v", (char*)"1"};
    //-fo -1, starting from -1 octave //-v 1,
    //only print out # feature and overall time
    sift.ParseParam(4, s_argv);
    
    //Initialize GLUT
//     glutInit(&argc, argv);
//     glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
//     glutInitWindowSize(640, 480);
//         //Create the window
//     glutCreateWindow("Image");
    
    //create an OpenGL context for computation
    int support = sift.CreateContextGL();
    //call VerfifyContexGL instead if using your own GL context
    //int support = sift.VerifyContextGL();
    if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED)
    {
        fprintf(stderr, "Can't open OpenGL context for SiftGPU\n");
        return;
    }
    
    sift.RunSIFT(nameImage1.c_str()); //process an image
    
    int num1 = sift.GetFeatureNum();//get feature count
    //allocate memory for readback
    std::vector<float> descriptors1(128*num1);
    std::vector<SiftGPU::SiftKeypoint> keys1(num1);
    //read back keypoints and normalized descritpros
    sift.GetFeatureVector(&keys1[0], &descriptors1[0]); //specify NULL if you don’t need keypoints or descriptors
    
    sift.RunSIFT(nameImage2.c_str());
    int num2 = sift.GetFeatureNum();//get feature count
    std::vector<float> descriptors2(128*num2);
    std::vector<SiftGPU::SiftKeypoint> keys2(num2);
    sift.GetFeatureVector(&keys2[0], &descriptors2[0]);
    
    if (debug)
    {
        std::cout << "Number of keypoints Image 1: " << keys1.size() << '\n';
        std::cout << "Number of keypoints Image 2: " << keys2.size() << '\n';
    }
    
    //specify the naximum number of features to match
    SiftMatchGPU matcher(num_goodmatch);
    //You can call SetMaxSift anytime to change this limit
    //You can call SetLanguage to select shader language
    //between GLSL/CUDA before initialization
    //Verify current OpenGL Context and do initialization
    if(matcher.VerifyContextGL() == 0)
    {
        fprintf(stderr, "Can't open OpenGL context for Matcher in SiftGPU\n");
        return;
    }
    //Set two sets of descriptor data to the matcher
    matcher.SetDescriptors(0, num1, &descriptors1[0]);
    matcher.SetDescriptors(1, num2, &descriptors2[0]);
    //Match and read back result to input buffer
    int match_buf[num_goodmatch][2];
    int nmatch = matcher.GetSiftMatch(num_goodmatch, match_buf);
    // You can also use homography and/or fundamental matrix to
    // guide the putative matching
    // Check function SiftMatchGPU::GetGuidedMatch
    // For more details of the above functions, check
    // SimpleSIFT.cpp and SiftGPU.h in the code package.
    
    keypoints1.clear();
    keypoints2.clear();
    matches.clear();
    
    if (debug)
    {
        for (register int i = 0; i < nmatch ; ++i)
        {
	  printf("Point of image1, id#%i : (%f,%f)" , match_buf[i][0], keys1[match_buf[i][0]].x, keys1[match_buf[i][0]].y);
	  printf("\t||\tPoint of image2, id#%i : (%f,%f)\n" , match_buf[i][1], keys2[match_buf[i][1]].x, keys2[match_buf[i][1]].y);
        }
    }
    
    for (register int i = 0; i < nmatch ; ++i)
    {
        keypoints1.push_back(cv::KeyPoint(cv::Point2f(keys1[match_buf[i][0]].x,keys1[match_buf[i][0]].y),
			  keys1[match_buf[i][0]].s, keys1[match_buf[i][0]].o ));
        keypoints2.push_back(cv::KeyPoint(cv::Point2f(keys2[match_buf[i][1]].x,keys2[match_buf[i][1]].y),
			  keys2[match_buf[i][1]].s, keys2[match_buf[i][1]].o ));
        matches.push_back(cv::DMatch( match_buf[i][0], match_buf[i][1], i ));
    }
    
    if (draw)
    {
        cv::Mat image1 = cv::imread(nameImage1);
        cv::Mat image2 = cv::imread(nameImage2);
        cv::Mat image_key1, image_key2;
    
        drawKeypoints(image1, keypoints1, image_key1, cv::Scalar(0,0,255));//, 4);
        drawKeypoints(image2, keypoints2, image_key2, cv::Scalar(0,0,255));//, 4);
    
        imshow( "im1", image_key1 );
        imshow( "im2", image_key2 );
        
        cvWaitKey(0);
        
        cv::Mat draw_match;
        drawMatches( image1, keypoints1, image2, keypoints2,
		matches, draw_match, cv::Scalar::all(-1), cv::Scalar::all(-1),
		std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        imshow( "MATCHES", draw_match );
        cvWaitKey(0);
    }
    
    return;
}

void globalContinuousFeatures( std::vector< cv::Mat > &images, std::vector< std::vector< cv::KeyPoint > > &keypoints,
			 std::vector< cv::Mat > &descriptors, std::vector< std::vector< cv::DMatch > > &matches,
		          int num_features, int num_goodmatch, bool debug, bool draw)

{
    // ERROR, extract features twice for all images
    int num_images = images.size();
    keypoints.resize( num_images );
    descriptors.resize( num_images );
    matches.resize( num_images - 1); // continuous matches between images
    
    for (register int i = 0; i < matches.size(); ++i)
    {
        solveFeaturesEuclidean(images[i], images[i+1], keypoints[i], keypoints[i+1], 
			 descriptors[i], descriptors[i+1], matches[i],
			 num_features, num_goodmatch, debug, draw);
        std::cout << "Final # of matches = " << matches[i].size() << '\n';
    }
    
}

void tagContinuosFeatures( std::vector< std::vector< cv::KeyPoint > > &keypoints, std::vector< std::vector< cv::DMatch > > &goodmatches, 
	        Eigen::Matrix<bool,-1,-1> &visibility, Eigen::Matrix<Eigen::Vector3d,-1,-1> &coordinates,
	        Eigen::MatrixXi &idxKPMatrix, std::vector< Eigen::Matrix<int,2,-1> > &idxKPVector)
{
    
    // ERROR like, CORRECT goodMatchesfromDistances(std::vector< cv::DMatch > &matches, std::vector< double > &distances, int num_goodmatch,
				//std::vector< cv::DMatch > &goodmatches)
    // int limit = matches.size() > num_goodmatch? num_goodmatch: matches.size();
    
    int num_features = 1000; // Defined large value of features
    int real_features = 0;
    int num_cameras = keypoints.size();
    Eigen::Matrix<int,2,1> initIDX = -1*Eigen::Matrix<int,2,1>::Ones();
    
    // initialization all zeros
    visibility = Eigen::Matrix<bool,-1,-1>::Zero(num_cameras,num_features);
    idxKPMatrix = Eigen::MatrixXi::Zero(num_cameras,num_features);
    coordinates.resize(num_cameras,num_features);
    idxKPVector.resize(num_cameras);
    for (int i = 0; i < num_cameras; i++) idxKPVector[i] = initIDX;
    
    // First camera addition. Initialization
    
    for (int k = 0; k < goodmatches[0].size(); k++)
    {
        int act_size1 = idxKPVector[0].cols();
        visibility(0,real_features) = true;
        idxKPMatrix(0,real_features) = goodmatches[0][k].queryIdx;
        visibility(1,real_features) = true;
        idxKPMatrix(1,real_features) = goodmatches[0][k].trainIdx;
        // coordinates
        cv::Point2f point1 = keypoints[0][goodmatches[0][k].queryIdx].pt;
        Eigen::Vector3d epoint1( (double)point1.x, (double)point1.y, 1.0);
        coordinates(0,real_features) = epoint1;
        cv::Point2f point2 = keypoints[1][goodmatches[0][k].trainIdx].pt;
        Eigen::Vector3d epoint2( (double)point2.x, (double)point2.y, 1.0);
        coordinates(1,real_features) = epoint2;
        
        idxKPVector[0].conservativeResize(2,act_size1+1);
        idxKPVector[0](0,act_size1) = real_features;
        idxKPVector[0](1,act_size1) = goodmatches[0][k].queryIdx;
        
        idxKPVector[1].conservativeResize(2,act_size1+1);
        idxKPVector[1](0,act_size1) = real_features;
        idxKPVector[1](1,act_size1) = goodmatches[0][k].trainIdx;
        
        real_features++;
    }
    //std::cout << "real_features value = " << real_features << '\n';
//     for (int k = 0; k < idxKPVector[0].cols(); k++)
//         std::cout << "Camera: 0\tFeature: "  << idxKPVector[0](0,k) << "\tIdx: " << idxKPVector[0](1,k) <<'\n';
//         //printf("Idx %i in camera 0 related with feature # %i\n", idxKPVector[0](1,k),idxKPVector[0](0,k));
    
    
    // Adding all features
    for (int cam = 1; cam < goodmatches.size(); cam++)
    {
        for (int k = 0; k < goodmatches[0].size(); k++)
        {
	  int search_idx = goodmatches[cam][k].queryIdx;
	  bool found_idx = false;
	  int tmp_f = real_features;
	  int act_size1 = idxKPVector[cam].cols();
	  int act_size2 = idxKPVector[cam+1].cols();
	  //search existent feature
	  for (int nf = 0; nf < act_size1; nf++)
	  {
	      if (idxKPVector[cam](1,nf) == search_idx)
	      {
		found_idx = true;
		tmp_f = idxKPVector[cam](0,nf);
		std::cout << "Found idx " << search_idx << " in camera " << cam << ", feature #" << tmp_f 
		<< " paired with trainidx " << goodmatches[cam][k].trainIdx <<'\n';
		break;
	      }
	  }
	  if (found_idx)
	  {
	      visibility(cam+1,tmp_f) = true;
	      idxKPMatrix(cam+1,tmp_f) = goodmatches[cam][k].trainIdx;
	      
	      cv::Point2f point2 = keypoints[cam+1][goodmatches[cam][k].trainIdx].pt;
	      Eigen::Vector3d epoint2( (double)point2.x, (double)point2.y, 1.0);
	      coordinates(cam+1,tmp_f) = epoint2;
	      
	      //if (act_size==1)idxKPVector[cam+1].conservativeResize(2,act_size+1);
	      idxKPVector[cam+1].conservativeResize(2,act_size2+1);
	      idxKPVector[cam+1](0,act_size2) = tmp_f;
	      idxKPVector[cam+1](1,act_size2) = goodmatches[cam][k].trainIdx;
	  }
	  else
	  {
	      visibility(cam,real_features) = true;
	      idxKPMatrix(cam,real_features) = goodmatches[cam][k].queryIdx;
	      
	      cv::Point2f point1 = keypoints[cam][goodmatches[cam][k].queryIdx].pt;
	      Eigen::Vector3d epoint1( (double)point1.x, (double)point1.y, 1.0);
	      coordinates(cam,real_features) = epoint1;
	      
	      idxKPVector[cam].conservativeResize(2,act_size1+1);
	      idxKPVector[cam](0,act_size1) = real_features;
	      idxKPVector[cam](1,act_size1) = goodmatches[cam][k].queryIdx;
	      
	      visibility(cam+1,real_features) = true;
	      idxKPMatrix(cam+1,real_features) = goodmatches[cam][k].trainIdx;
	      
	      cv::Point2f point2 = keypoints[cam+1][goodmatches[cam][k].trainIdx].pt;
	      Eigen::Vector3d epoint2( (double)point2.x, (double)point2.y, 1.0);
	      coordinates(cam+1,real_features) = epoint2;
	      
	      idxKPVector[cam+1].conservativeResize(2,act_size2+1);
	      idxKPVector[cam+1](0,act_size2) = real_features;
	      idxKPVector[cam+1](1,act_size2) = goodmatches[cam][k].trainIdx;
	      
	      real_features++;
	  }
        }
        //idxKPVector[cam]=idxKPVector[cam].rightCols(1);// delete initialized valued of -1
//         for (int j = 0; j < idxKPVector[cam].cols(); j++)
//         {
// 	  std::cout << "Camera: " << cam << "\tFeature: "  << idxKPVector[cam](0,j) << "\tIdx: " << idxKPVector[cam](1,j) <<'\n';
// 	  //printf("Idx %i in camera %i related with feature %i \n", idxKPVector[cam](1,j), cam, idxKPVector[cam](0,j));
//         }
        
    }
    
    visibility.conservativeResize(num_cameras, real_features);
    idxKPMatrix.conservativeResize(num_cameras, real_features);
//     std::cout << "Number of features\n" << real_features << '\n';
//     std::cout << "Visibility Matrix\n" << visibility << '\n';
//     std::cout << "Idx of KeyPoint Matrix\n" << idxKPMatrix << '\n';  
};

// void tagContinuosFeaturesUnDist( std::vector< std::vector< cv::KeyPoint > > &keypoints, std::vector< std::vector< cv::DMatch > > &goodmatches, 
// 	        Eigen::Matrix<bool,-1,-1> &visibility, Eigen::Matrix<Eigen::Vector3d,-1,-1> &coordinates,
// 	        Eigen::MatrixXi &idxKPMatrix, std::vector< Eigen::Matrix<int,2,-1> > &idxKPVector)
// {