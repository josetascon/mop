/**
 * @file FeaturesFunctions.hpp
 * @brief This file has all functions related with Features extraction/description/matching
 *
 * @author José David Tascón Vidarte
 * @date Aug/02/2013
 */

#ifndef __FEATURESFUNCTIONS_HPP__
#define __FEATURESFUNCTIONS_HPP__

// OpenCV Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>

// Std Libraries
#include <iostream>

// SiftGPU Library
#include <SiftGPU.h>

// Local Libraries
#include "Common.hpp"
// #include "HandleDB.hpp"
// #include "CameraPose.hpp"
// #include "DepthProjection.hpp"

// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================
/**
 * ******************************************************************
 * @brief Description: Transform a vector set of KeyPoint to Point2d 
 * 
 * @param kps		(input)
 * @param ps		(output)
 * 
 * @date May/10/2013
 */
void keyPointstoPoints(const std::vector< cv::KeyPoint > &kps, std::vector< cv::Point2d > &ps);

/**
 * ******************************************************************
 * @brief Description: Transform a vector set of Point2d to KeyPoint
 * 
 * @param ps 		(input)
 * @param kps 		(output)
 * 
 * @date May/10/2013
 */
void pointstoKeyPoints(const std::vector< cv::Point2d > &ps, std::vector< cv::KeyPoint > &kps);

/**
 * ******************************************************************
 * @brief Description: From a set of keypoints keep just the desired given by matches.
 * The output of points is in Eigen::MatrixXd with 3xn size format.
 * 
 * @param goodmatches	(input)
 * @param keypoints1	(input)
 * @param keypoints2	(input)
 * @param pts1		(output)
 * @param pts2		(output)
 * 
 * @date May/10/2013
 */
void eigenfromMatches(std::vector< cv::DMatch > &matches,
		  std::vector< cv::KeyPoint > &keypoints1, std::vector< cv::KeyPoint > &keypoints2,
		  Eigen::MatrixXd &pts1, Eigen::MatrixXd &pts2);

void eigenfromMatches(std::vector< cv::DMatch > &matches,
		  std::vector< SiftGPU::SiftKeypoint > &keypoints1, std::vector< SiftGPU::SiftKeypoint > &keypoints2,
		  Eigen::MatrixXd &pts1, Eigen::MatrixXd &pts2);

void eigenfromSiftKeypoint(std::vector< SiftGPU::SiftKeypoint > &keys, Eigen::MatrixXf &pts);

// NON EXISTENT
// void keepPointsfromGoodMatches( std::vector< cv::DMatch > &goodmatches, 
// 			  std::vector< cv::KeyPoint > &keypoints1, std::vector< cv::KeyPoint > &keypoints2,
// 			  std::vector< cv::Point2d > &pts1, std::vector< cv::Point2d > &pts2);


void removeRepeatedKeyPoints( std::vector< cv::KeyPoint > &keypoints );

void eliminateRepeatedMatches( std::vector< cv::DMatch > &matches );

void extractPointsfromMatches( std::vector< cv::DMatch > &goodmatches, 
			  std::vector< cv::KeyPoint > &keypoints1, std::vector< cv::KeyPoint > &keypoints2,
			  std::vector< cv::Point2d > &pts1, std::vector< cv::Point2d > &pts2);

void extractPointsfromMatches( std::vector< cv::DMatch > &goodmatches, 
			  std::vector< SiftGPU::SiftKeypoint > &keypoints1, std::vector< SiftGPU::SiftKeypoint > &keypoints2,
			  std::vector< cv::Point2d > &pts1, std::vector< cv::Point2d > &pts2);
#endif