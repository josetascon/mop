/**
 * @file OptimizerG3D.hpp
 * @brief Optimizer, class with functions & data to optimize CameraPose and Structure
 *
 * @author José David Tascón Vidarte
 * @date Jul/31/2012
 */

#ifndef __OPTIMIZERG3D_HPP__
#define __OPTIMIZERG3D_HPP__

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
// ==================================== CLASS OptimizerG3D ========================================
// ================================================================================================
class OptimizerG3D
{
private:
    Eigen::MatrixXd *Pts;
    Eigen::MatrixXd *Variance;
    std::vector< Eigen::Quaternion<double> > *Qns; 
    std::vector< Eigen::Vector3d > *trs;
    
    Eigen::Matrix<bool,-1,-1> *visibility;
    Eigen::Matrix<Eigen::Vector4d,-1,-1> *coordinates;
    
    std::vector< std::vector<double> > qv_internal;
    
public:
    // Constructor
    OptimizerG3D() { ; };
    // Destructor
    ~OptimizerG3D() { ; };

    void setParameters(Eigen::MatrixXd *Structure, Eigen::MatrixXd *Covariance,
		   Eigen::Matrix<bool,-1,-1> *view_matrix, Eigen::Matrix<Eigen::Vector4d,-1,-1> *coord,
		   std::vector< Eigen::Quaternion<double> > *Qn_global, std::vector< Eigen::Vector3d > *tr_global)
    {
        Pts = Structure;
        Variance = Covariance;
        Qns = Qn_global;
        trs = tr_global;
        visibility = view_matrix;
        coordinates = coord;
        
        quaternion_vector2vector_vector( *Qn_global, qv_internal ); // QUATERNION to Vector
    }
    
    // For pose_LSQ
    void setParameters(Eigen::MatrixXd *Structure, Eigen::Matrix<bool,-1,-1> *view_matrix, Eigen::Matrix<Eigen::Vector4d,-1,-1> *coord,
		   std::vector< Eigen::Quaternion<double> > *Qn_global, std::vector< Eigen::Vector3d > *tr_global)
    {
        Pts = Structure;
        Qns = Qn_global;
        trs = tr_global;
        visibility = view_matrix;
        coordinates = coord;
        
        quaternion_vector2vector_vector( *Qn_global, qv_internal ); // QUATERNION to Vector
    }
    
    void updateQuaternion()
    {
        vector_vector2quaternion_vector( qv_internal, *Qns );
        for (int cam = 0; cam < Qns->size(); cam++)
        {
	  (*Qns)[cam].normalize();
        }
    }
    
    void pose_LSQ();
    void pose_Covariance();

};

#endif