/**
 * @file IncrementalOptimizer.hpp
 * @brief Optimizer, class with functions & data to optimize CameraPose and Structure
 *
 * @author José David Tascón Vidarte
 * @date Jul/31/2012
 */

#ifndef __INCREMENTALOPTIMIZER_HPP__
#define __INCREMENTALOPTIMIZER_HPP__

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
// ================================ CLASS IncrementalOptimizer ====================================
// ================================================================================================
class IncrementalOptimizer
{
private:
    // Variables pointers
    Eigen::Matrix<bool,-1,-1> *visibility; // [num_cams x num_features]
    Eigen::Matrix<Eigen::Vector3d,-1,-1> *coordinates; // [num_cams x num_features]
    Eigen::Matrix3d *intrinsics;
    std::vector<double> *distortion;
    std::vector< Eigen::Quaternion<double> > *quaternion;
    std::vector< Eigen::Vector3d > *translation;
    Eigen::MatrixXd *structure;
    
    std::vector< Eigen::Vector4d > q_vector;
    
public:
    //Constructor
    IncrementalOptimizer(Eigen::Matrix<bool,-1,-1> *view_matrix, Eigen::Matrix<Eigen::Vector3d,-1,-1> *coord,
	        std::vector< Eigen::Quaternion<double> > *quaternion, std::vector< Eigen::Vector3d > *translation,
	        Eigen::MatrixXd *structure, Eigen::Matrix3d *intrinsics, std::vector<double> *distortion)
    : visibility(view_matrix), coordinates(coord), quaternion(quaternion), translation(translation), structure(structure),
      intrinsics(intrinsics), distortion(distortion) { };
    //Destructor
    ~IncrementalOptimizer() { ; };
    
    void update(int constant_cam, int num_cams);
    void run(int constant_cam, int num_cams);
};

#endif