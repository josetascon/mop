// Created: Aug/02/2013
// File: Jan/13/2014
// Author: José David Tascón Vidarte

#include "FeaturesFunctions.hpp"


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