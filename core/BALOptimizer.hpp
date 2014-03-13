/**
 * @file BALOptimizer.hpp
 * @brief Optimizer, class with functions & data to optimize CameraPose and Structure
 *
 * @author José David Tascón Vidarte
 * @date Jul/31/2012
 */

#ifndef __BALOPTIMIZER_HPP__
#define __BALOPTIMIZER_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// Ceres Solver Libraries
#include "gflags/gflags.h"
#include <glog/logging.h>
#include <ceres/ceres.h>

// Std Libraries
#include <iostream>

// Local Libraries
#include "Debug.hpp"
#include "Common.hpp"
#include "reprojection_error.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::CauchyLoss;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

// ================================================================================================
// ==================================== CLASS BALOptimizer ========================================
// ================================================================================================
class BALOptimizer
{
private:
    // Variables
    Eigen::Matrix<bool,-1,-1> *visibility; // [num_cams x num_features]
    Eigen::Matrix<Eigen::Vector3d,-1,-1> *coordinates; // [num_cams x num_features]
    std::vector< Eigen::Quaternion<double> > *quaternion;
    std::vector< Eigen::Vector4d > q_vector;
    Eigen::MatrixXd *translation_and_intrinsics;
    Eigen::MatrixXd *structure;
    
public:
    //Constructor
    BALOptimizer(Eigen::Matrix<bool,-1,-1> *visibility,
	       Eigen::Matrix<Eigen::Vector3d,-1,-1> *coordinates,
	       std::vector< Eigen::Quaternion<double> > *quaternion,
	       Eigen::MatrixXd *translation_and_intrinsics, Eigen::MatrixXd *structure) 
	      : visibility(visibility), coordinates(coordinates), quaternion(quaternion), 
	      translation_and_intrinsics(translation_and_intrinsics), structure(structure) { };
    //Destructor
    ~BALOptimizer() { ; };
    
    void update();
    // Functions
    void runBAL();
};




#endif