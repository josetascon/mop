/**
 * @file FeaturesCV.hpp
 * @brief This file has all functions related with Features extraction/description/matching
 *
 * @author José David Tascón Vidarte
 * @date Aug/02/2013
 */

#ifndef __FEATURESCV_HPP__
#define __FEATURESCV_HPP__

// OpenCV Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>

// Std Libraries
#include <iostream>

// Local Libraries
#include "FeaturesFunctions.hpp"

// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================
/**
 * ******************************************************************
 * @brief Description: Extract SIFT features from images.
 * The image is divided in a grid tryng to bring a stable distribution of features
 * 
 * @param image		(input)
 * @param keypoints		(output)
 * @param num_features 	Maximun number of features allowed in full image. Default value 100.
 * @param grid_rows 	Number of rows in the image grid. Default value 4
 * @param grid_cols 	Number of columns in the image grid. Default value 4
 * 
 * @date May/10/2013
 */
void gridSiftFeatureDetect ( cv::Mat &image, std::vector< cv::KeyPoint > &keypoints, 
		         int num_features = 100, int grid_rows = 4, int grid_cols = 4);

/**
 * ******************************************************************
 * @brief (SET VERSION) Description: Extract features and descriptors from a SET of images. Internal usage of gridSiftFeatureDetect function
 * 
 * @param set_of_images	(input)
 * @param set_of_keypoints	(output)
 * @param set_of_descriptors	(output)
 * @param num_features 	Maximun number of features allowed in full image. Default value 100.
 * @param grid_rows 	Number of rows in the image grid. Default value 4
 * @param grid_cols 	Number of columns in the image grid. Default value 4
 * 
 * @date May/10/2013
 */
void featureDetectandDescript(std::vector< cv::Mat > &set_of_images, 
			std::vector< std::vector< cv::KeyPoint > > &set_of_keypoints,
			std::vector< cv::Mat > &set_of_descriptors,
			int num_features = 100, int grid_rows = 4, int grid_cols = 4 );

/**
 * ******************************************************************
 * @brief (ONE IMAGE VERSION) Description: Extract features and descriptors from one image. Internal usage of gridSiftFeatureDetect function
 * 
 * @param image		(input)
 * @param keypoints		(output)
 * @param descriptors	(output)
 * @param num_features 	Maximun number of features allowed in full image. Default value 100.
 * @param grid_rows 	Number of rows in the image grid. Default value 4
 * @param grid_cols 	Number of columns in the image grid. Default value 4
 * 
 * @date May/10/2013
 */
void featureDetectandDescript(cv::Mat &image, 
			  std::vector< cv::KeyPoint > &keypoints,
			  cv::Mat &descriptors,
			  int num_features = 100, int grid_rows = 4, int grid_cols = 4 );

/**
 * ******************************************************************
 * @brief Description: Extract all distances from matches and bring the range of them (minimun and maximum)
 * 
 * @param matches		(input)
 * @param distances		(output) Vector of double with distances between matches
 * @param min		(output) Minimun distance value in matches
 * @param max		(output) Maximun distance value in matches
 * 
 * @date May/10/2013
 */
void distanceRangeofMatches( std::vector< cv::DMatch > &matches, std::vector< double > &distances, double &min, double &max);

/**
 * ******************************************************************
 * @brief Description: Struct used to sort matches(std::sort)  through their distance parameter
 * 
 * @date May/10/2013
 */
struct SortObject
{
	std::vector<double> value;
	bool operator() (int i,int j) { return (value[i]<value[j]);}
};

/**
 * ******************************************************************
 * @brief Description: Sort distances of matches and returns lesser ones as goodmatches.
 * The desired best number of good matches is defined by num_goodmatch.
 * 
 * @param matches		(input)
 * @param distances		(input)
 * @param num_goodmatch	(input)
 * @param goodmatches	(output)
 * 
 * @date May/10/2013
 */
void goodMatchesfromDistances(std::vector< cv::DMatch > &matches, std::vector< double > &distances, int num_goodmatch,
			std::vector< cv::DMatch > &goodmatches);

void goodMatcheswithEuclidean( std::vector< cv::DMatch > &matches,
			 std::vector< cv::KeyPoint > &keypoints1, std::vector< cv::KeyPoint > &keypoints2,
			 float thold, int num_ecm = 12);

void solveFeatures( cv::Mat &image1, cv::Mat &image2,
		std::vector< cv::KeyPoint > &keypoints1 , std::vector< cv::KeyPoint > &keypoints2,
		cv::Mat &descriptors1, cv::Mat &descriptors2, std::vector< cv::DMatch > &goodmatches,
		int num_features = 200, int num_goodmatch = 35,
		bool debug = false, bool draw = false);

void solveFeaturesEuclidean( cv::Mat &image1, cv::Mat &image2,
		std::vector< cv::KeyPoint > &keypoints1 , std::vector< cv::KeyPoint > &keypoints2,
		cv::Mat &descriptors1, cv::Mat &descriptors2, std::vector< cv::DMatch > &goodmatches,
		int num_features = 200, int num_goodmatch = 35,
		bool debug = false, bool draw = false);

void globalContinuousFeatures( std::vector< cv::Mat > &images, std::vector< std::vector< cv::KeyPoint > > &keypoints,
			 std::vector< cv::Mat > &descriptors, std::vector< std::vector< cv::DMatch > > &matches,
		          int num_features = 200, int num_goodmatch = 35, bool debug = false, bool draw = false);

void tagContinuosFeatures( std::vector< std::vector< cv::KeyPoint > > &keypoints, std::vector< std::vector< cv::DMatch > > &goodmatches, 
	        Eigen::Matrix<bool,-1,-1> &visibility, Eigen::Matrix<Eigen::Vector3d,-1,-1> &coordinates,
	        Eigen::MatrixXi &idxKPMatrix, std::vector< Eigen::Matrix<int,2,-1> > &idxKPVector);

#endif