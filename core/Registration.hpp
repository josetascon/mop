/**
 * @file Registration.hpp
 * @brief This file has all functions related 
 *
 * @author José David Tascón Vidarte
 * @date Dec/09/2013
 */

#ifndef __REGISTRATION_HPP__
#define __REGISTRATION_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// OpenCV Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

// Std Libraries
#include <iostream>
#include <vector>
#include <utility>		// std::pair

//Local Libraries
#include "Common.hpp"
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
    
public:
    std::vector< Eigen::MatrixXd > Cameras_RCV;
    std::vector< Eigen::Matrix3d > Rot_global;
    std::vector< Eigen::Quaternion<double> > Qn_global;
    std::vector< Eigen::Vector3d > tr_global;
    std::vector< Eigen::MatrixXd > Xmodel;
    std::vector< Eigen::MatrixXd > Variance;
    Eigen::MatrixXd Structure; //from now recover pose only
    std::vector< std::vector<cv::Point3d> > StructurePlot;// plot TESTING
    
    
    // Constructor
    SimpleRegistration(int cams, int feats, Eigen::Matrix3d Calib) : num_cameras(cams), num_features(feats), Calibration(Calib) { };
    // Destructor
    ~SimpleRegistration() { ; };
    
    //solve Pose, use continuous matches
    void solvePose(std::vector< MatchQuery > *globalMatch, std::vector< std::vector< cv::KeyPoint > > *set_of_keypoints, 
		std::vector< cv::Mat > *set_of_depth);
    
    void solvePose(Eigen::Matrix<bool,-1,-1> *visibility, Eigen::Matrix<Eigen::Vector3d,-1,-1> *coordinates,
	         std::vector< cv::Mat > *set_of_depth);
    
    void solvePoseOptimal(Eigen::Matrix<bool,-1,-1> *visibility, Eigen::Matrix<Eigen::Vector3d,-1,-1> *coordinates, 
		std::vector< cv::Mat > *set_of_depth);
    
    void solvePose(Eigen::Matrix<bool,-1,-1> *visibility, Eigen::Matrix<Eigen::Vector4d,-1,-1> *coordinates, bool optimal = true);
    
    void updateCamera();
    
    /** Use SimpleRegistration::Ptr as a clean nomenclature to create an smart pointer of SimpleRegistration type.\n 
     * E.g.: \n
     * > SimpleRegistration::Ptr myobject(new SimpleRegistration);
     */
    typedef boost::shared_ptr< SimpleRegistration > Ptr;
};


struct PoseQuery
{
    int cam_id1;
    int cam_id2;
    int num_matches;
    Eigen::Quaternion<double> quaternion;
    Eigen::Vector3d translation;
    
    /** Constructor where all variables are provided. */
    PoseQuery(int cam1, int cam2, int nmatch, Eigen::Quaternion<double> quat, Eigen::Vector3d tr)
    : cam_id1(cam1), cam_id2(cam2), num_matches(nmatch), quaternion(quat), translation(tr) { };
    
    /** Use PoseQuery::vector as a clean nomenclature to create an object std::vector< PoseQuery >. \n
     * E.g.: \n
     * > PoseQuery::vector my_vpq;
     */
    typedef std::vector< PoseQuery > vector;
};

int find_id_pair(std::vector< std::pair <int,int> > edges, int v1, int v2);

// ================================================================================================
// ======================================== CLASS GraphPose =======================================
// ================================================================================================
class GraphPose
{
private:
    int vertices;
    int edges;
    std::vector< float > weights;
    std::vector< std::pair <int,int> > edges_pairs;
    
public:
    PoseQuery::vector localPose;
    std::vector< Eigen::Quaternion<double> > Qn_global;
    std::vector< Eigen::Vector3d > tr_global;
    
    //Constructor
    GraphPose() { };
    //Destructor
    ~GraphPose() { };
    
    void solvePose( std::vector<bool> *reliableMatch, std::vector< MatchQuery > *globalMatch,
		    std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints,
		    std::vector< std::string > *list_depth, Eigen::Matrix3d *Calibration);
    
    void solveEdges();
    int getNumVertex() { return vertices; }
    int getNumEdges() { return edges; }
    std::vector< float > getWeights() { return weights; }
    std::vector< std::pair <int,int> > getEdgesPairs() { return edges_pairs; }
    
    void solveGraph(std::vector< int > &discover, std::vector< int > &parent);
    void solveGraphContinuous();
    void runTORO();
};


#endif