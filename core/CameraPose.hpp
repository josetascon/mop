/**
 * @file CameraPose.hpp
 * @brief This file has all functions related with Projective and Epipolar Geometry
 *
 * @author José David Tascón Vidarte
 * @date May/12/2013
 */

#ifndef __CAMERAPOSE_HPP__
#define __CAMERAPOSE_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// OpenCV Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// Std Libraries
#include <iostream>

// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================

/**
 * ******************************************************************
 * @brief Decription: If determinat of Rot is -1 it multiplies the third column and corrects it to 1.
 * Rotation matrices with det = -1 are reflection matrices.
 * 
 * @param Rot 		(input/output) Rotation Matrix
 * 
 * @date Jul/08/2013
 */
bool determinantCorrection(Eigen::Matrix3d &Rot);

/**
 * ******************************************************************
 * @brief Description: Check if a Rotation Matrix has a determinant = -1 or 1 (orthogonality). 
 * If determinat of Rot is -1 it call determinantCorrection(Rot) function and corrects to 1.
 * Rotation matrices with det = -1 are reflection matrices.
 * 
 * @param Rot 		(input) Rotation Matrix
 * @return (bool) true value if det(Rot) = -1 or 1.
 * 
 * @date Jul/08/2013
 */
bool checkCoherentRotation(Eigen::Matrix3d &Rot);

//@date Jul/18/2013
Eigen::MatrixXd buildProjectionMatrix( Eigen::Matrix3d &kalibration, Eigen::Matrix3d &Rotation, Eigen::Vector3d &translation);

/**
 * ******************************************************************
 * @brief Description: Get fundamental matrix with a robust algorithm (RANSAC) and remove points marked as outliers from pts1 and pts2
 * 
 * @param pts1 		(input/output) Points coordinates of features from image1
 * @param pts2 		(input/output) Related (in order) features from image2 
 * @param matches		(output)
 * @param Fundamental	(output) Fundamental Matrix (rank 2 matrix)
 * 
 * @date May/12/2013
 */
void getFundamentalandRemoveOutliers(std::vector< cv::Point2d > &pts1, std::vector< cv::Point2d > &pts2, std::vector< cv::DMatch > &matches,
					cv::Mat &Fundamental);

int robustMatchesfromFundamental(std::vector< cv::Point2d > &pts1, std::vector< cv::Point2d > &pts2, std::vector<cv::DMatch> &matches, cv::Mat &Fund, int max_iter);

/**
 * ******************************************************************
 * @brief Description: Find essential matrix from fundamental matrix and calibration matrix.
 * Use the linear algorithm in HZ2004 to find Rotation and translation from the essential matrix.
 * So this algorithm offers four solutions because his output are 2 Rotation matrices and 2 translation vector.
 * 
 * @param Fundamental	(input) Fundamental Matrix (rank 2 matrix)
 * @param kalibration	(input) Calibration matrix k (intrinsics parameters of a camera)
 * @param Rot1		(output) First rotation matrix solution
 * @param Rot2		(output) Second rotation matrix solution
 * @param translation1	(output) First translation vector solution
 * @param translation2	(output) Second translation vector solution
 * 
 * @date May/12/2013
 */
void posefromFundamental( cv::Mat &Fundamental, cv::Mat &kalibration,
						cv::Mat &Essential, cv::Mat &Rot1, cv::Mat &Rot2, cv::Mat &translation1, cv::Mat &translation2);

void essentialfromFundamental( Eigen::Matrix3d &Fundamental, Eigen::Matrix3d &kalibration,
		        Eigen::Matrix3d &Essential);

/**
 * ******************************************************************
 * @brief Description: Use the linear algorithm in HZ2004 to find Rotation and translation from the essential matrix.
 * So this algorithm offers four solutions because his output are 2 Rotation matrices and 2 translation vector.
 * 
 * @param Essential		(input) Essential Matrix (rank 2 matrix)
 * @param Rot1		(output) First rotation matrix solution
 * @param Rot2		(output) Second rotation matrix solution
 * @param translation1	(output) First translation vector solution
 * @param translation2	(output) Second translation vector solution
 * 
 * @date Jul/15/2013
 */

void posefromEssential( Eigen::Matrix3d &Essential, Eigen::Matrix3d &Rot1, Eigen::Matrix3d &Rot2, 
		    Eigen::Vector3d &ttr1, Eigen::Vector3d &ttr2);
/**
 * ******************************************************************
 * @brief Description: posefrom3DPoints is a 3D to 3D camera pose estimation using linear method from [ARUM87] && [MARMONE07]
 * 
 * @param WorldData1	(input) 3D Points of a scene, initial position (first shot)
 * @param WorldData2 	(input) 3D Points of the same scene, from a different position (second shot)
 * @param Rotation 		(output) Found Camera Rotation
 * @param translation	(output) Found translation from first and second shot
 * 
 * @date Jul/08/2013
 */
void posefrom3DPoints( Eigen::MatrixXd &WorldData1, Eigen::MatrixXd &WorldData2, 
		   Eigen::Matrix3d &Rotation, Eigen::Vector3d &translation );

void posefrom3DPoints( Eigen::MatrixXf &WorldData1, Eigen::MatrixXf &WorldData2, 
		   Eigen::Matrix3f &Rotation, Eigen::Vector3f &translation );

Eigen::MatrixXd linearCamera( Eigen::MatrixXd &xpt, Eigen::MatrixXd &Xpt);

#endif