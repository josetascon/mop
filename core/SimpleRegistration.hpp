/**
 * @file SimpleRegistration.hpp
 * @brief This file has all functions related 
 *
 * @author José David Tascón Vidarte
 * @date Dec/09/2013
 */

#ifndef __SIMPLEREGISTRATION_HPP__
#define __SIMPLEREGISTRATION_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// OpenCV Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

// Std Libraries
#include <iostream>
#include <vector>

// Local Libraries
#include "Debug.hpp"
#include "Common.hpp"
#include "Pose3D.hpp"
// #include "CameraPose.hpp"
#include "FeaturesEDM.hpp"
#include "Optimizer.hpp"
#include "DepthProjection.hpp"

// ================================================================================================
// ================================= CLASS SimpleRegistration =====================================
// ================================================================================================
class SimpleRegistration		// Registration Aligment of Points based on Image Features
{
private:
    int num_cameras;
    int num_features;
    Eigen::Matrix3d Calibration;
    
    
    Eigen::MatrixXd X1, X2;
    Eigen::Matrix3d Rot;
    Eigen::Vector3d tr;
    
    void initializeGlobalVariables();
    void initializeCoordinatesOrigin();
    void updateGlobal(int iteration);
    void printGlobal(int iteration);
    
public:
    std::vector< Eigen::MatrixXd > Cameras_RCV;
    std::vector< Eigen::Matrix3d > Rot_global;
    std::vector< Eigen::Quaternion<double> > Qn_global;
    std::vector< Eigen::Vector3d > tr_global;
    std::vector< Eigen::MatrixXd > Xmodel;	// Created to easily access 3D features. ( e.g. when I need to Plot elipsoids )
    std::vector< Eigen::MatrixXd > Variance;
    
    // Constructor
    SimpleRegistration(int cams, int feats, Eigen::Matrix3d Calib) : num_cameras(cams), num_features(feats), Calibration(Calib) { };
    // Destructor
    ~SimpleRegistration() { ; };
    
    //solve Pose, use continuous matches
    void solvePose(std::vector< MatchQuery > *globalMatch, std::vector< std::vector< cv::KeyPoint > > *set_of_keypoints, 
		std::vector< cv::Mat > *set_of_depth);
    
    void solvePose(Eigen::Matrix<bool,-1,-1> *visibility, Eigen::Matrix<Eigen::Vector3d,-1,-1> *coordinates, 
		std::vector< std::string > &depth_list, bool optimal = true);
    
    void solvePose(Eigen::Matrix<bool,-1,-1> *visibility, Eigen::Matrix<Eigen::Vector4d,-1,-1> *coordinates, bool optimal = true);
    
    void updateQuaternion();
    
    /** Use SimpleRegistration::Ptr as a clean nomenclature to create an smart pointer of SimpleRegistration type.\n 
     * E.g.: \n
     * > SimpleRegistration::Ptr myobject(new SimpleRegistration);
     */
    typedef boost::shared_ptr< SimpleRegistration > Ptr;
};



#endif