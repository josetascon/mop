/**
 * @file GlobalOptimizerSfM.hpp
 * @brief Optimizer, class with functions & data to optimize CameraPose and Structure
 *
 * @author José David Tascón Vidarte
 * @date Jul/31/2012
 */

#ifndef __GLOBALOPTIMIZERSFM_HPP__
#define __GLOBALOPTIMIZERSFM_HPP__

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
// ================================= CLASS GlobalOptimizerSfM =====================================
// ================================================================================================
class GlobalOptimizerSfM
{
private:
    // Variables
    double initial_reprojection_error;
    double final_reprojection_error;
    double time;
    Eigen::Matrix<bool,-1,-1> *visibility; // [num_cams x num_features]
    Eigen::Matrix<Eigen::Vector3d,-1,-1> *coordinates; // [num_cams x num_features]
    std::vector< Eigen::Vector3d > *translation;
    std::vector< Eigen::Matrix3d > *rot;
//     std::vector< std::vector<double> > *rotation; //Rodriguez angles
//     std::vector< std::vector<double> > *quaternion;
    std::vector< Eigen::Quaternion<double> > *quaternion;
    //std::vector< std::vector<double> > *structure;
    Eigen::MatrixXd *structure;
    std::vector<double> calibration;
    std::vector<double> *intrinsics;
    std::vector<double> *distortion;
    std::vector<double> scale;
    
    // internal parameters
    std::vector< std::vector<double> > qv_internal;
    std::vector< std::vector<double> > qv_continuos;
    
public:
    //Constructor
    GlobalOptimizerSfM();
    //Destructor
    ~GlobalOptimizerSfM();
    
    // Assignation methods
    void setVisibility( Eigen::Matrix<bool,-1,-1> *view_matrix );		//done
    void setCoordinates( Eigen::Matrix<Eigen::Vector3d,-1,-1> *coord );	//done
    void setTranslation( std::vector< Eigen::Vector3d > *tr ); 			//done
//     void setRotation( std::vector< Eigen:Matrix3d > *rot;			//done
//     void setRotation( std::vector< std::vector<double> > *rot);			//done
//     void setQuaternion( std::vector< std::vector<double> > *quat );		//done
    void setQuaternion( std::vector< Eigen::Quaternion<double> > *quat );		//done
//     void setStructure( std::vector< std::vector<double> > *st );			//done
    void setStructure( Eigen::MatrixXd *st );					//done
    void setIntrinsics( std::vector<double> *internal_param );			//done
    void setIntrinsics( Eigen::Matrix3d *K );					//done
    void setDistortion( std::vector<double> *coefficients);				//done
    
    // Some set functions are done, but they need a proper setParameters function.
    void setParameters( Eigen::Matrix<bool,-1,-1> *view_matrix,
	       Eigen::Matrix<Eigen::Vector3d,-1,-1> *coord,
	       std::vector< Eigen::Quaternion<double> > *quat, std::vector< Eigen::Vector3d > *tr,
	       Eigen::MatrixXd *st  );
//     void setParameters( Eigen::Matrix<bool,-1,-1> *view_matrix,
// 	       Eigen::Matrix<Eigen::Vector3d,-1,-1> *coord,
// 	       std::vector< std::vector<double> > *quat, std::vector< Eigen::Vector3d > *tr,
// 	       std::vector< std::vector<double> > *st  );
    void update();
    void stats(double &initial_error, double &final_error, double &time);
    
    // Functions
    void runBA();//char *argv0);
//     void run();
};


// ================================================================================================
// ======================================= Add Functions ==========================================
// ================================================================================================
double reprojectionErrorCalculation(Eigen::Matrix<bool,-1,-1> *visibility, Eigen::Matrix<Eigen::Vector3d,-1,-1> *coordinates,
		    std::vector<double> *intrinsics, std::vector< Eigen::Quaternion<double> > *quaternion, 
		    std::vector< Eigen::Vector3d > *translation, Eigen::MatrixXd *structure);

#endif