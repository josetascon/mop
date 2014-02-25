/**
 * @file MergeClouds.hpp
 * @brief This file has all functions related Point Cloud Library PCL
 *
 * @author José David Tascón Vidarte
 * @date Feb/14/2014
 */

#ifndef __MERGECLOUDS_HPP__
#define __MERGECLOUDS_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// PCL Libraries
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

// OpenCV Libraries
#include <opencv2/opencv.hpp>

// Std Libraries
#include <iostream>
#include <string>
#include <vector>

// Local Libraries
#include "Debug.hpp"
#include "Common.hpp"
#include "DepthProjection.hpp"
#include "InterfacePCL.hpp"


// ================================================================================================
// ======================================= CLASS MergeClouds ======================================
// ================================================================================================

class MergeClouds
{
private:
    
    bool load_list;
    bool load_calib;
    
    Eigen::Matrix3d Calibration;
    std::vector<std::string> *image_list;
    std::vector<std::string> *depth_list;
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2;
    boost::shared_ptr< Eigen::MatrixXd > covariance1;
    boost::shared_ptr< Eigen::MatrixXd > covariance2;
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model_cloud;
    boost::shared_ptr< Eigen::MatrixXd > model_covariance;
    
    void loadDenseCloud( std::string &image, std::string &depth, 
			  Eigen::Quaternion<double> &Qn, Eigen::Vector3d &tr,
			  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out,
			  boost::shared_ptr< Eigen::MatrixXd > &covariance);
    
public:
    // Constructors
    MergeClouds( ) { load_calib = false; load_list = false; };
    
    MergeClouds( Eigen::Matrix3d &Calib ) { setCalibration( Calib ); load_list = false; };
    
    MergeClouds( std::vector<std::string> *imagelist, std::vector<std::string> *depthlist )
    {
        setLists( imagelist, depthlist );
        load_calib = false;
    };
    
    MergeClouds( std::vector<std::string> *imagelist, std::vector<std::string> *depthlist, Eigen::Matrix3d &Calib )
    {
        setLists( imagelist, depthlist );
        setCalibration( Calib );
    }
    
    ~MergeClouds( ) { };
    
    // Functions
    void mergeTwo_PCL(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pc1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pc2,
	      boost::shared_ptr< Eigen::MatrixXd > &variance1, boost::shared_ptr< Eigen::MatrixXd > &variance2, 
	      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out, boost::shared_ptr< Eigen::MatrixXd > &covariance_out );
    
    void mergeSet_PCL( std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &set_cloud, 
		std::vector< boost::shared_ptr< Eigen::MatrixXd > > &set_covariance,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &model);
    
    void mergeTwo( std::string &image1, std::string &depth1, std::string &image2, std::string &depth2, 
		        Eigen::Quaternion<double> &qn, Eigen::Vector3d &tr );
    
    void mergeSet( std::vector< Eigen::Quaternion<double> > &Qn_global, std::vector< Eigen::Vector3d > &tr_global );
    
    
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
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloud() { return model_cloud; };
    boost::shared_ptr< Eigen::MatrixXd > getCovariance() { return model_covariance; };
    
};

void mergeClouds( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pc1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pc2,
	      boost::shared_ptr< Eigen::MatrixXd > &covariance1, boost::shared_ptr< Eigen::MatrixXd > &covariance2, 
	      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out, boost::shared_ptr< Eigen::MatrixXd > &covariance_out );

void mergeCloudSet( std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &set_cloud, 
		std::vector< boost::shared_ptr< Eigen::MatrixXd > > &set_covariance,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &model);



#endif