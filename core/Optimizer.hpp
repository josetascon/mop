/**
 * @file Optimizer.hpp
 * @brief Optimizer, class with functions & data to optimize CameraPose and Structure
 *
 * @author José David Tascón Vidarte
 * @date Jul/31/2012
 */

#ifndef __OPTIMIZER_HPP__
#define __OPTIMIZER_HPP__

// // Eigen Libraries
// #include <eigen3/Eigen/Dense>
// 
// // Ceres Solver Libraries
// #include "gflags/gflags.h"
// #include <glog/logging.h>
// #include <ceres/ceres.h>
// 
// // Std Libraries
// #include <iostream>
// 
// // Local Libraries
// #include "Debug.hpp"
// #include "Common.hpp"
// #include "reprojection_error.hpp"
// 
// using ceres::AutoDiffCostFunction;
// using ceres::CostFunction;
// using ceres::CauchyLoss;
// using ceres::Problem;
// using ceres::Solve;
// using ceres::Solver;

#include "GlobalOptimizerSfM.hpp"
#include "OptimizerG3D.hpp"
#include "LocalOptimizer.hpp"
#include "IncrementalOptimizer.hpp"
#include "BALOptimizer.hpp"

#endif