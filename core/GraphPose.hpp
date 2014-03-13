/**
 * @file GraphPose.hpp
 * @brief This file has all functions related 
 *
 * @author José David Tascón Vidarte
 * @date Dec/09/2013
 */

#ifndef __GRAPHPOSE_HPP__
#define __GRAPHPOSE_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// OpenCV Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

// Std Libraries
#include <iostream>
#include <fstream>		// ofstream and/or ifstream
#include <vector>
#include <utility>		// std::pair

// Local Libraries
#include "Debug.hpp"
#include "Common.hpp"
#include "Pose3D.hpp"
#include "PoseICP.hpp"
#include "GraphBFS.hpp"
#include "FeaturesEDM.hpp"

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

// ================================================================================================
// ======================================== CLASS GraphPose =======================================
// ================================================================================================
class GraphPose
{
private:
    bool load_calib;
    bool fallback_icp;
    
    int vertices;
    int edges;
    std::vector< float > weights;
    std::vector< std::pair <int,int> > edges_pairs;
    std::vector< std::pair <int,int> > match_pairs;
    
    std::vector< std::string> *images_rgb;
    std::vector< std::string> *images_depth;
    
    std::vector< int > discover_time;
    std::vector< int > parent_node;
    
    Eigen::Matrix3d Calibration;
    Eigen::Matrix3d Rot;
    Eigen::Vector3d tr;
    std::vector< cv::Point2d > pts1, pts2;
    
    int find_id_pair(std::vector< std::pair <int,int> > edges, int v1, int v2);
    void poseInID( int id, std::vector< std::string > *list_depth, 
	std::vector< MatchQuery > *globalMatch, std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints, bool optimal = true );
    
    PoseQuery::vector localPose;
//     std::vector< Eigen::Quaternion<double> > Qn_global;
//     std::vector< Eigen::Vector3d > tr_global;
    
    typedef std::vector< Eigen::Quaternion<double> > Qd_vector;
    typedef std::vector< Eigen::Vector3d > V3d_vector;
    
    boost::shared_ptr< Qd_vector > Qn_global;
    boost::shared_ptr< V3d_vector > tr_global;
    
public:
    //Constructor
    GraphPose() 
    {
        load_calib = false;
        fallback_icp = false;
        initializePtrs();
    };
    GraphPose( Eigen::Matrix3d &Calib )
    {
        setCalibration( Calib );
        fallback_icp = false;
        initializePtrs();
    };
    //Destructor
    ~GraphPose() { };
    
    void initializePtrs()
    {
        Qn_global = boost::shared_ptr< Qd_vector >( new Qd_vector());
        tr_global = boost::shared_ptr< V3d_vector >( new V3d_vector());
    };
    
    void run( std::vector< std::string > *list_depth,
	    std::vector<bool> *reliableMatch, std::vector< MatchQuery > *globalMatch,
	    std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints, bool optimal = true );
    
    // Solve pairwise the poses defined in globalMatch
    void solveLocalPoseAllNodes( std::vector< std::string > *list_depth,
		    std::vector<bool> *reliableMatch, std::vector< MatchQuery > *globalMatch,
		    std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints, bool optimal = true );
    
    void solveEdgesAllNodes(); // Use after solveLocalPoseAllNodes
    void solveGlobalPoseAllNodes(); // Use after solveEdgesAllNodes
    void solveGlobalPoseContinuous(); // Use after solveEdgesAllNodes
    
    // Graph based solution with Breath First Search algorithm
    void solveGraph( int initbfs = -1 );
    
    // Solution without pairwise local poses, only calculate in solveGlobalPose necessary poses defined after solveGraph.
    void solveEdges(std::vector<bool> *reliableMatch, std::vector< MatchQuery > *globalMatch);
    void solveGlobalPose( std::vector< std::string > *list_depth, std::vector< MatchQuery > *globalMatch, 
		        std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints, bool optimal = true );
    

    void exportGRAPH( const char *filename );
    
    // Set functions
    void setCalibration( Eigen::Matrix3d &Calib) 
    { 
        Calibration = Calib;
        load_calib = true;
    };
    
    void setFallBackICPOn( std::vector< std::string> *rgb, std::vector< std::string> *depth )//, int min_points = 3 ) // Minimal number of points to solve pose
    {
        images_rgb = rgb;
        images_depth = depth;
        fallback_icp = true;
//         valid_min_points = min_points;
    }
    
    // Get functions
    int getNumVertex() { return vertices; }
    int getNumEdges() { return edges; }
    std::vector< float > getWeights() { return weights; }
    std::vector< std::pair <int,int> > getEdgesPairs() { return edges_pairs; }
    
    boost::shared_ptr< std::vector< Eigen::Quaternion<double> > > getPtrGlobalQuaternion()
    {
//         return boost::shared_ptr< std::vector< Eigen::Quaternion<double> > >( &Qn_global );
        return Qn_global;
    }
    
    boost::shared_ptr< std::vector< Eigen::Vector3d > > getPtrGlobalTranslation()
    {
//         return boost::shared_ptr< std::vector< Eigen::Vector3d > >( &tr_global );
        return tr_global;
    }
};


#endif