// Created: Aug/02/2013
// File: Jan/13/2014
// Author: José David Tascón Vidarte

#include "FeaturesCV.hpp"

// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================
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