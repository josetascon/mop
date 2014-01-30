/**
 * @file InterfacePCL.hpp
 * @brief This file has all functions related Point Cloud Library PCL
 *
 * @author José David Tascón Vidarte
 * @date Nov/04/2013
 */

#ifndef __INTERFACEPCL_HPP__
#define __INTERFACEPCL_HPP__

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


// ====================================================================================================================================
// =====================================================  FUNCTIONS PCL ===============================================================
// ====================================================================================================================================
typedef union
{
    struct
    {
      unsigned char Blue;
      unsigned char Green;
      unsigned char Red;
      unsigned char Alpha;
    };
    float float_value;
    uint32_t long_value;
} RGBValue;

template <typename T_eig>
void eigen2pointcloud(Eigen::Matrix<T_eig,-1,-1> &MM, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)
{
    if ( MM.rows() < 3)
    {
        std::cerr << "Error: Eigen Matrix does not have appropiate dimensions\n";
        exit(0);
    }
    int num_points = MM.cols();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
    
    for (register int k = 0; k < num_points ; ++k)
    {
        pcl::PointXYZ pt;
        pt.x = MM(0,k);
        pt.y = MM(1,k);
        pt.z = MM(2,k);
        cloud->push_back(pt);
    }
    
    cloud->sensor_origin_.setZero ();
    cloud->sensor_orientation_.w () = 1.0f;
    cloud->sensor_orientation_.x () = 0.0f;
    cloud->sensor_orientation_.y () = 0.0f;
    cloud->sensor_orientation_.z () = 0.0f;
    
    cloud->is_dense = true;
    DEBUG_2( std::cout << "Cloud size = " << cloud->width*cloud->height << "\n"; )
    cloud_out = cloud;
}

void cv2PointCloud(cv::Mat &depth, Eigen::Matrix3d &calibration, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_xyz);

void cv2PointCloud(cv::Mat &image, cv::Mat &depth, Eigen::Matrix3d &calibration, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out);

/**
 * ******************************************************************
 * @brief cv2PointCloudDense save only values in the allowed range, creating a continuos PCL array. (cv2PointCloud save the cloud as an image array with NAN values 		wasting a lot of space)
 * @author José David Tascón Vidarte
 * @date Sep/12/2013
 */
void cv2PointCloudDense(cv::Mat &image, cv::Mat &depth, Eigen::Matrix3d &calibration, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out);

void cv2PointCloudDense(cv::Mat &image, cv::Mat &depth, Eigen::Matrix3d &calibration,
	         pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out, boost::shared_ptr< Eigen::MatrixXd > &covariance);

void computeCovariance(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, Eigen::Matrix3d &calibration,
		   boost::shared_ptr< Eigen::MatrixXd > &covariance);

void cv2PointCloudSet(std::vector<cv::Mat> &image, std::vector<cv::Mat> &depth, Eigen::Matrix3d &calibration, 
		std::vector<Eigen::Quaternion<double> > &Qn, std::vector< Eigen::Vector3d > &tr,
		std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &set_cloud);

void cv2PointCloudSet(std::vector<cv::Mat> &image, std::vector<cv::Mat> &depth, Eigen::Matrix3d &calibration, 
		std::vector<Eigen::Quaternion<double> > &Qn, std::vector< Eigen::Vector3d > &tr,
		std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &set_cloud,
		std::vector< boost::shared_ptr< Eigen::MatrixXd > > &set_covariance);

void cv2PointCloudSet(std::vector<std::string> &image_list, std::vector<std::string> &depth_list, Eigen::Matrix3d &calibration, 
		std::vector<Eigen::Quaternion<double> > &Qn, std::vector< Eigen::Vector3d > &tr,
		std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &set_cloud,
		std::vector< boost::shared_ptr< Eigen::MatrixXd > > &set_covariance);

void sparse2dense( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_sparse, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_dense);

void set2unique( std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &data, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out);

void mergeClouds( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pc1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pc2,
	      boost::shared_ptr< Eigen::MatrixXd > &covariance1, boost::shared_ptr< Eigen::MatrixXd > &covariance2, 
	      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out, boost::shared_ptr< Eigen::MatrixXd > &covariance_out );

void mergeCloudSet( std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &set_cloud, 
		std::vector< boost::shared_ptr< Eigen::MatrixXd > > &set_covariance,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &model);

#endif