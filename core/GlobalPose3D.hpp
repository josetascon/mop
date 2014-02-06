/**
 * @file GlobalPose3D.hpp
 * @brief This file has all functions related 
 *
 * @author José David Tascón Vidarte
 * @date Jan/31/2014
 */

#ifndef __GLOBALPOSE3D_HPP__
#define __GLOBALPOSE3D_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// OpenCV Libraries
// #include <opencv2/core/core.hpp>
// #include <opencv2/core/eigen.hpp>

// Std Libraries
#include <iostream>
#include <vector>

// Local Libraries
#include "Debug.hpp"
#include "DepthProjection.hpp"
#include "Optimizer.hpp" 

// ================================================================================================
// ==================================== CLASS GlobalPose3D ========================================
// ================================================================================================
class GlobalPose3D
{
private:
    int num_cameras;
    int num_features;
    

public:
    Eigen::MatrixXd Structure;
    Eigen::MatrixXd Covariance;
    
    
    // Constructor
    GlobalPose3D() { ; };
    // Destructor
    ~GlobalPose3D() { ; };
    
    void solve(Eigen::Matrix<bool,-1,-1> *visibility, Eigen::Matrix<Eigen::Vector4d,-1,-1> *coordinates,
		     Eigen::Matrix3d *Calibration,
		     std::vector< Eigen::Quaternion<double> > *Qn_global, std::vector< Eigen::Vector3d > *tr_global);
    
};

#endif