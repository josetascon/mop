/**
 * @file Optimizer.hpp
 * @brief Optimizer, class with functions & data to optimize CameraPose and Structure
 *
 * @author José David Tascón Vidarte
 * @date Jul/31/2012
 */

#ifndef __Optimizer_HPP__
#define __Optimizer_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// Ceres Solver Libraries
#include "gflags/gflags.h"
#include <glog/logging.h>
#include <ceres/ceres.h>

// Std Libraries
#include <iostream>

// Local Libraries
#include "Common.hpp"
#include "reprojection_error.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::CauchyLoss;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

// ================================================================================================
// ================================== CLASS GlobalOptimizer =======================================
// ================================================================================================
class GlobalOptimizer
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
    std::vector<double> *intrinsics;
    std::vector<double> *distortion;
    std::vector<double> scale;
    
    // internal parameters
    std::vector< std::vector<double> > qv_internal;
    std::vector< std::vector<double> > qv_continuos;
    
public:
    //Constructor
    GlobalOptimizer();
    //Destructor
    ~GlobalOptimizer();
    
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
    void setDistortion( std::vector<double> *coefficients ){ distortion = coefficients; };	// [k1,k2,p1,p2,k3]
    
    void pose3Dto2D();
    void pose3Dto3D();
    void pose3Dto3D_Covariance();
    
//     void fundamental(Eigen::MatrixXd *F, std::vector<double> *X); // TODO
//     void structureOptimization();
    
};

// ================================================================================================
// =================================== CLASS IncrementalBA ========================================
// ================================================================================================
class PartialIncremental
{
private:
    // Variables pointers
    Eigen::Matrix<bool,-1,-1> *visibility; // [num_cams x num_features]
    Eigen::Matrix<Eigen::Vector3d,-1,-1> *coordinates; // [num_cams x num_features]
    std::vector<double> *intrinsics;
    std::vector<double> *distortion;
    std::vector< Eigen::Quaternion<double> > *quaternion;
    std::vector< Eigen::Vector3d > *translation;
    Eigen::MatrixXd *structure;
    
    std::vector< Eigen::Vector4d > q_vector;
    
public:
    //Constructor
    PartialIncremental(Eigen::Matrix<bool,-1,-1> *view_matrix, Eigen::Matrix<Eigen::Vector3d,-1,-1> *coord,
	        std::vector< Eigen::Quaternion<double> > *quaternion, std::vector< Eigen::Vector3d > *translation,
	        Eigen::MatrixXd *structure, std::vector<double> *intrinsics, std::vector<double> *distortion)
    : visibility(view_matrix), coordinates(coord), quaternion(quaternion), translation(translation), structure(structure),
      intrinsics(intrinsics), distortion(distortion) { };
    //Destructor
    ~PartialIncremental() { ; };
    
    void update(int constant_cam, int num_cams);
    void run(int constant_cam, int num_cams);
};


// ================================================================================================
// ================================== CLASS BALOptimizer =======================================
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


double reprojectionErrorCalculation(Eigen::Matrix<bool,-1,-1> *visibility, Eigen::Matrix<Eigen::Vector3d,-1,-1> *coordinates,
		    std::vector<double> *intrinsics, std::vector< Eigen::Quaternion<double> > *quaternion, 
		    std::vector< Eigen::Vector3d > *translation, Eigen::MatrixXd *structure);




#endif