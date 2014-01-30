/**
 * @file MultipleCamera.hpp
 * @brief This file has all functions related 
 *
 * @author José David Tascón Vidarte
 * @date Jul/16/2013
 */

#ifndef __MULTIPLECAMERA_HPP__
#define __MULTIPLECAMERA_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// OpenCV Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

// Std Libraries
#include <iostream>
#include <vector>
#include <utility>		// std::pair

// Local Libraries
#include "Debug.hpp"
#include "Common.hpp"
#include "FeaturesEDM.hpp"
#include "CameraPose.hpp"
#include "Triangulation.hpp"
#include "Optimizer.hpp"

// ================================================================================================
// ============================================ CLASS SfM =========================================
// ================================================================================================

// @date Aug/06/2013
class SfM
{
private:
    int num_cameras;
    int num_features;
    Eigen::Matrix3d Calibration;
    std::vector< Eigen::Matrix3d > Fundamental;
//     std::vector< Eigen::Vector3d > t12_RCV;
//     std::vector< Eigen::Vector3d > t13_RCV;
    
public:
    std::vector< Eigen::Quaternion<double> > Quat_relative;
    std::vector< Eigen::Vector3d > tr_relative;
    std::vector< Eigen::MatrixXd > Cameras_RCV;
    std::vector< Eigen::Quaternion<double> > Qn_global;
    std::vector< Eigen::Matrix3d > Rot_global;
    std::vector< Eigen::Vector3d > tr_global;
    Eigen::MatrixXd Structure;
    Eigen::VectorXi plotSt; // plot TESTING
    
    // Constructor
    SfM(int cams, int feats, Eigen::Matrix3d Calib) : num_cameras(cams), num_features(feats), Calibration(Calib) { };
    // Destructor
    ~SfM() { ; };
    
    //solve Pose, use continuous matches
    void solvePose(std::vector< MatchQuery > *globalMatch, std::vector< std::vector< cv::KeyPoint > > *set_of_keypoints);
    void solvePose(Eigen::Matrix<bool,-1,-1> *visibility, Eigen::Matrix<Eigen::Vector3d,-1,-1> *coordinates );
    void solveStructure(Eigen::Matrix<bool,-1,-1> *visibility, Eigen::Matrix<Eigen::Vector3d,-1,-1> *coordinates );
    
    void updateCamera();
};

// ================================================================================================
// =================================== CLASS IncrementalBA ========================================
// ================================================================================================
class IncrementalBA
{
private:
    // Variables
    Eigen::Matrix<bool,-1,-1> *visibility; // [num_cams x num_features]
    Eigen::Matrix<Eigen::Vector3d,-1,-1> *coordinates; // [num_cams x num_features]
    std::vector<double> *intrinsics;
    std::vector<double> *distortion;
    
public:
    std::vector< Eigen::Quaternion<double> > quaternion;
    std::vector< Eigen::Vector3d > translation;
    Eigen::MatrixXd structure;
    std::vector< Eigen::MatrixXd > Camera;
    
    //Constructor
    IncrementalBA(Eigen::Matrix<bool,-1,-1> *view_matrix, 
	        Eigen::Matrix<Eigen::Vector3d,-1,-1> *coord ): visibility(view_matrix), coordinates(coord) { };
    //Destructor
    ~IncrementalBA() { ; };
    
    void setIntrinsics( std::vector<double> *internal_param ) { intrinsics = internal_param; };
    void setDistortion( std::vector<double> *coefficients) { distortion = coefficients; };
    
    void runC();
    void runF();
    void updateCamera();
};

// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================
/**
 * ******************************************************************
 * @brief Decription:
 * 
 * @param ia 		(input)
 * @param oa 		(output)
 * 
 * @return (TYPE) 
 * 
 * @date Jul/16/2013
 */
void findCameraExtrinsicsAdapter( std::vector< cv::KeyPoint > &keypoints1, std::vector< cv::KeyPoint > &keypoints2, 
		       std::vector< cv::DMatch > &matches, Eigen::Matrix3d &calib, 
		       Eigen::Matrix3d &Fundamental, Eigen::Matrix3d &Rot, Eigen::Vector3d &tr );

void findCameraExtrinsics(std::vector< cv::Point2d > &pts1, std::vector< cv::Point2d > &pts2, Eigen::Matrix3d &calib, 
		       Eigen::Matrix3d &Fundamental, Eigen::Matrix3d &Rot, Eigen::Vector3d &tr );

//@date Jul/18/2013
void triangulateMultipleCamera( Eigen::Matrix<bool,-1,-1> &visibility, 
			  Eigen::Matrix<Eigen::Vector3d,-1,-1> &coordinates,
			  std::vector<Eigen::MatrixXd> &Cameras,
			  Eigen::MatrixXd &Structure);

void solveStructureInitial(const Eigen::Matrix<bool,-1,-1> &visibility,
		        const Eigen::Matrix<Eigen::Vector3d,-1,-1> &coordinates, 
		        Eigen::MatrixXd &Structure, std::vector< Eigen::MatrixXd > &Cameras_RCV, int cam_range1, int cam_range2 );

void scalefromTranslations( std::vector< Eigen::Matrix3d > &Rot_global, std::vector< Eigen::Vector3d > &t12_RCV,
		        std::vector< Eigen::Vector3d > &t13_RCV, std::vector< Eigen::Vector3d > &tr_global );






#endif