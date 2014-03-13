/**
 * @file LocalOptimizer.hpp
 * @brief Optimizer, class with functions & data to optimize CameraPose and Structure
 *
 * @author José David Tascón Vidarte
 * @date Jul/31/2012
 */

#ifndef __LOCALOPTIMIZER_HPP__
#define __LOCALOPTIMIZER_HPP__

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
// =================================== CLASS LocalOptimizer =======================================
// ================================================================================================
class LocalOptimizer
{
private:
    Eigen::MatrixXd *observation1;
    Eigen::MatrixXd *observation2;
    Eigen::MatrixXd *variance1;
    Eigen::MatrixXd *variance2;
    Eigen::Matrix3d *rotation;
    Eigen::Vector3d *translation;
    Eigen::MatrixXd *structure;
    std::vector<double> calibration;
    std::vector<double> *intrinsics;
    std::vector<double> *distortion;
    
    //internal used in optimization
    std::vector<double> quaternionV;
//     std::vector< std::vector<double> > structureV;
    
    void updatePose3Dto2D();
    void update();
    
public:
    //Constructor
    LocalOptimizer();
    //Destructor
    ~LocalOptimizer();
    void setParameters(Eigen::MatrixXd *obs1, Eigen::MatrixXd *obs2, Eigen::Matrix3d *Rot, Eigen::Vector3d *tr);
    void setParameters3Dto3D(Eigen::MatrixXd *obs1, Eigen::MatrixXd *obs2, Eigen::Matrix3d *Rot, Eigen::Vector3d *tr);
    void setParameters3Dto3D(Eigen::MatrixXd *obs1, Eigen::MatrixXd *obs2, Eigen::Matrix3d *Rot, Eigen::Vector3d *tr, 
		         Eigen::MatrixXd *var1, Eigen::MatrixXd *var2);
    void setParameters3Dto2D(Eigen::MatrixXd *obs1, Eigen::MatrixXd *obs2, 
		   Eigen::Matrix3d *Rot, Eigen::Vector3d *tr, Eigen::MatrixXd *st );
    void setIntrinsics( std::vector<double> *internal_param ){ intrinsics = internal_param; };	// [fx,fy,cx,cy]
    void setIntrinsics( Eigen::Matrix3d *K )
    {
        double tmp[4] = { (*K)(0,0), (*K)(1,1), (*K)(0,2), (*K)(1,2) }; // [fx,fy,cx,cy]
        calibration = std::vector< double >(&tmp[0], &tmp[4]);
        intrinsics = &calibration;
    };
    
    void setDistortion( std::vector<double> *coefficients ){ distortion = coefficients; };	// [k1,k2,p1,p2,k3]
    
    void poseCamera();
    void pose3Dto2D();
    void pose3Dto3D();
    void pose3Dto3D_Covariance();
    
//     void fundamental(Eigen::MatrixXd *F, std::vector<double> *X); // TODO
//     void structureOptimization();
    
};

#endif