/**
 * @file ReconstructionModel.hpp
 * @brief This file has all functions related Point Cloud Library PCL
 *
 * @author José David Tascón Vidarte
 * @date Feb/14/2014
 */

#ifndef __RECONSTRUCTIONMODEL_HPP__
#define __RECONSTRUCTIONMODEL_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// PCL Libraries
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>

// OpenCV Libraries
#include <opencv2/opencv.hpp>

// Std Libraries
#include <iostream>
#include <string>
#include <vector>
#include <libgen.h>

// Local libraries
#include "DepthProjection.hpp"
#include "Debug.hpp"
#include "Common.hpp"
#include "HandleDB.hpp"
#include "Interface.hpp"
#include "InterfacePCL.hpp"
#include "MergeClouds.hpp"
#include "DOTWriter.hpp"

#include "FeaturesEDM.hpp"
#include "Registration.hpp"
#include "Optimizer.hpp"
#include "Plot.hpp"


// ================================================================================================
// =================================== CLASS ReconstructionModel ==================================
// ================================================================================================

class ReconstructionModel
{
protected:
    // Definitions, known between friend, and child classes only
    typedef std::vector< Eigen::Quaternion<double> > Qd_vector;
    typedef std::vector< Eigen::Vector3d > V3d_vector;
    typedef std::vector< Eigen::MatrixXd > MXd_vector;
    
    typedef std::vector< std::vector<float> > float_vv;
    typedef std::vector< std::vector<SiftGPU::SiftKeypoint> > kpGPU_vv;
    
    bool load_list;
    bool load_calib;
    bool subgroups;
    bool unified_model;
    bool non_unified_model;
    int num_features;
    int num_cameras;
    int num_depth_filter;
    
    Eigen::Matrix3d Calibration;
    std::vector<std::string> *image_list;
    std::vector<std::string> *depth_list;
    std::vector<int> *subgroup_boundaries;

    boost::shared_ptr< Qd_vector > Qn_global;
    boost::shared_ptr< V3d_vector > tr_global;
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Xmodel;
    boost::shared_ptr< Eigen::MatrixXd > Variance;
    std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > set_model;
    std::vector< boost::shared_ptr< Eigen::MatrixXd > > set_variance;
    
    boost::shared_ptr< SiftED > features;
    boost::shared_ptr< MatchesMap > matchMap;
    boost::shared_ptr< FeaturesMap > featMap;
    
    timer_wall timer1;
    
    void initilizeAuxiliarVariables()
    {
        load_calib = false; 
        load_list = false;
        subgroups = false;
        unified_model = false;
        non_unified_model = false;
    }
    
    virtual void initilizePtrs()
    {
//         Qn_global = boost::shared_ptr< Qd_vector >( new Qd_vector());
//         tr_global = boost::shared_ptr< V3d_vector >( new V3d_vector());
//         Xmodel = boost::shared_ptr< MXd_vector >( new MXd_vector());
//         Variance = boost::shared_ptr< MXd_vector >( new MXd_vector());
        
        features = boost::shared_ptr< SiftED >( new SiftED(image_list) );
        matchMap = boost::shared_ptr< MatchesMap >( new MatchesMap(500,35) );
        featMap = boost::shared_ptr< FeaturesMap >(new FeaturesMap());
    };
    
public:
    // Constructors
    ReconstructionModel( ) { };
//     ReconstructionModel( ) { load_calib = false; load_list = false; subgroups = false; num_depth_filter = 3; };
    
    ReconstructionModel( std::vector<std::string> *imagelist, std::vector<std::string> *depthlist, Eigen::Matrix3d &Calib, int &df_value )
    {
        initilizeAuxiliarVariables();
        setLists( imagelist, depthlist );
        setCalibration( Calib );
        num_depth_filter = df_value;
        initilizePtrs();
    };
    
    ~ReconstructionModel( ) { };
    
    // Functions
    virtual void runSimple()
    {
        solveFeatures();
        simpleModel();
        globalOptimization();
    };
    
    virtual void runGraph()
    {
        solveFeatures();
        graphModel();
        globalOptimization();
    };
    
    void solveFeatures();
    void freeMemoryFeatures();
    
    virtual void graphModel();
    virtual void simpleModel();
    
    void globalOptimization();
    
    void solveUnifiedModel();
    void solveNonUnifiedModel();
    
    void visualizeNonUnifiedModel();
    void visualizeUnifiedModel();
    
    // Set Functions
    void setCalibration( Eigen::Matrix3d &Calib ) 
    { 
        Calibration = Calib;
        load_calib = true;
    };
    
    virtual void setRGBList( std::vector<std::string> *imagelist ) { image_list = imagelist; };
    virtual void setDepthList( std::vector<std::string> *depthlist ) { depth_list = depthlist; };
    virtual void setLists( std::vector<std::string> *imagelist, std::vector<std::string> *depthlist ) 
    {
        if (imagelist->size() != depthlist->size())
        {
	  DEBUG_E( ("Number of color and depth images files are different. Please check your xml files.") );
	  exit(-1);
        }
        setRGBList( imagelist );
        setDepthList( depthlist );
        load_list = true;
    };
    
    void setMatchSubgroup( std::vector<int> *subgroup_limits )
    {
        subgroups = true;
        subgroup_boundaries = subgroup_limits;
    }
    
    // Get Functions
    boost::shared_ptr< std::vector< Eigen::Quaternion<double> > > getPtrGlobalQuaternion() { return Qn_global; }
    boost::shared_ptr< std::vector< Eigen::Vector3d > > getPtrGlobalTranslation() { return tr_global; }
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getPtrCloudModel() { return Xmodel; }
    boost::shared_ptr< Eigen::MatrixXd > getPtrVariance() { return Variance; }
    
    boost::shared_ptr< SiftED > getPtrFeatures() { return features; };
    boost::shared_ptr< MatchesMap > getPtrMatchesMap() { return matchMap; };
    boost::shared_ptr< FeaturesMap > getPtrFeaturesMap() { return featMap; };
    
};

#endif