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
private:
    
    bool load_list;
    bool load_calib;
    int num_features;
    int num_cameras;
    int num_depth_filter;
    
    Eigen::Matrix3d Calibration;
    std::vector<std::string> *image_list;
    std::vector<std::string> *depth_list;
    
    typedef std::vector< Eigen::Quaternion<double> > Qd_vector;
    typedef std::vector< Eigen::Vector3d > V3d_vector;
    typedef std::vector< Eigen::MatrixXd > MXd_vector;
    boost::shared_ptr< Qd_vector > Qn_global;
    boost::shared_ptr< V3d_vector > tr_global;
    boost::shared_ptr< MXd_vector > Xmodel;
    boost::shared_ptr< MXd_vector > Variance;
    
    boost::shared_ptr< SiftED > features;
    boost::shared_ptr< MatchesMap > matchMap;
    boost::shared_ptr< FeaturesMap > featMap;
    
    timer_wall timer1;
    
public:
    // Constructors
    ReconstructionModel( ) { load_calib = false; load_list = false; num_depth_filter = 3; };
    
    ReconstructionModel( std::vector<std::string> *imagelist, std::vector<std::string> *depthlist, Eigen::Matrix3d &Calib, int &df_value )
    {
        setLists( imagelist, depthlist );
        setCalibration( Calib );
        num_depth_filter = df_value;
        initilizePtrs();
    };
    
    ~ReconstructionModel( ) { };
    
    void initilizePtrs()
    {
        Qn_global = boost::shared_ptr< Qd_vector >( new Qd_vector());
        tr_global = boost::shared_ptr< V3d_vector >( new V3d_vector());
        Xmodel = boost::shared_ptr< MXd_vector >( new MXd_vector());
        Variance = boost::shared_ptr< MXd_vector >( new MXd_vector());
        
        features = boost::shared_ptr< SiftED >( new SiftED(image_list) );
        matchMap = boost::shared_ptr< MatchesMap >( new MatchesMap(500,35) );
        featMap = boost::shared_ptr< FeaturesMap >(new FeaturesMap());
    };
    
    // Functions
    void solveFeatures();
    void freeMemoryFeatures();
    
    void graphModel();
    
    void simpleModel();
    
    void globalOptimization();
    
    void visualizeModel_NonUnified();
    
    // Set Functions
    void setCalibration( Eigen::Matrix3d &Calib ) 
    { 
        Calibration = Calib;
        load_calib = true;
    };
    
    void setRGBList( std::vector<std::string> *imagelist ) { image_list = imagelist; };
    void setDepthList( std::vector<std::string> *depthlist ) { depth_list = depthlist; };
    void setLists( std::vector<std::string> *imagelist, std::vector<std::string> *depthlist ) 
    {
        setRGBList( imagelist );
        setDepthList( depthlist );
        load_list = true;
    };
    
    // Get Functions
    boost::shared_ptr< std::vector< Eigen::Quaternion<double> > > getPtrGlobalQuaternion() { return Qn_global; }
    
    boost::shared_ptr< std::vector< Eigen::Vector3d > > getPtrGlobalTranslation() { return tr_global; }
    
    boost::shared_ptr< std::vector< Eigen::MatrixXd > > getPtrXmodel() { return Xmodel; }
    
    boost::shared_ptr< std::vector< Eigen::MatrixXd > > getPtrVariance() { return Variance; }
    
};

#endif