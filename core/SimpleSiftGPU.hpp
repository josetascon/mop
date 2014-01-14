/**
 * @file SimpleSiftGPU.hpp
 * @brief This file has the class SiftED used for SIFT Features applied to datasets. It uses at its core SiftGPU.
 *
 * @author José David Tascón Vidarte
 * @date Aug/02/2013
 */

#ifndef __SIMPLESIFTGPU_HPP__
#define __SIMPLESIFTGPU_HPP__

// OpenGl Libraries
#include <GL/glut.h>			// GL constants for SiftGPU function

// OpenCV Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

// Std Libraries
#include <iostream>

// SiftGPU Library
#include <SiftGPU.h>

// ================================================================================================
// ====================================== CLASS SimpleSiftGPU =====================================
// ================================================================================================
class SimpleSiftGPU
{
private:
    int num_matches;
    SiftGPU sift;
    SiftMatchGPU matcher;
    
public:
    std::vector< cv::DMatch > matches;
    
    SimpleSiftGPU(int num_match);
    ~SimpleSiftGPU();
    
    void solveFeatures( cv::Mat &Image, std::vector<SiftGPU::SiftKeypoint> &keypoints, std::vector<float> &descriptors );
    void solveFeatures( cv::Mat &Image, std::vector<cv::KeyPoint> &keypoints, std::vector<float> &descriptors );
    void solveFeatures( std::string nameImage, std::vector<SiftGPU::SiftKeypoint> &keypoints, std::vector<float> &descriptors );
    
    int solveMatches( std::vector<SiftGPU::SiftKeypoint> &keypoints1, std::vector<SiftGPU::SiftKeypoint> &keypoints2,
			   std::vector<float> &descriptors1, std::vector<float> &descriptors2, 
			     std::vector<cv::Point2d> &pt1, std::vector<cv::Point2d> &pt2 );
    
    int solveMatches( std::vector<SiftGPU::SiftKeypoint> &keyGPU1, std::vector<SiftGPU::SiftKeypoint> &keyGPU2,
			   std::vector<float> &descriptors1, std::vector<float> &descriptors2, 
			   std::vector<cv::Point2d> &pt1, std::vector<cv::Point2d> &pt2,
			   std::vector<cv::KeyPoint> &cvkeypoints1, std::vector<cv::KeyPoint> &cvkeypoints2 );
    
    int solveMatches( std::vector<cv::KeyPoint> &cvkeypoints1, std::vector<cv::KeyPoint> &cvkeypoints2,
			   std::vector<float> &descriptors1, std::vector<float> &descriptors2, 
			   std::vector<cv::Point2d> &pt1, std::vector<cv::Point2d> &pt2 );
};

// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================
//@date July/22/2013
void solveFeaturesGPU( std::string nameImage1, std::string nameImage2,
		std::vector< cv::KeyPoint > &keypoints1 , std::vector< cv::KeyPoint > &keypoints2,
		std::vector< cv::DMatch > &matches,
		int num_goodmatch = 50,
		bool debug = false, bool draw = false);

#endif