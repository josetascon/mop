/**
 * @file PoseICP.hpp
 * @brief This file has all functions related 
 *
 * @author José David Tascón Vidarte
 * @date Feb/07/2014
 */

#ifndef __POSEICP_HPP__
#define __POSEICP_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// Boost Libraries
#include <boost/shared_ptr.hpp>

// OpenCV Libraries
#include <opencv2/core/core.hpp>

// PCL Libraries
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

// Std Libraries
#include <iostream>
#include <vector>

// Local Libraries
#include "Debug.hpp"
#include "InterfacePCL.hpp"


// ================================================================================================
// ====================================== CLASS PoseICP ===========================================
// ================================================================================================
class PoseICP		// Registration Aligment of two sets
{
private:
    Eigen::Matrix3d Calibration;
    Eigen::Matrix3d Rotation;
    Eigen::Vector3d translation;
    
    boost::shared_ptr< cv::Mat > image1;
    boost::shared_ptr< cv::Mat > image2;
    boost::shared_ptr< cv::Mat > depth1;
    boost::shared_ptr< cv::Mat > depth2;
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2;
    
    bool load_calib;
    bool load_images;
    bool load_clouds;
    
    void setRGBImages( std::string &rgb1, std::string &rgb2 )
    {
        boost::shared_ptr< cv::Mat > im1_ptr (new cv::Mat); // smart pointer to free memory properly
        boost::shared_ptr< cv::Mat > im2_ptr (new cv::Mat);
        cv::Mat im1 = cv::imread( rgb1, -1 ); //load image
        cv::Mat im2 = cv::imread( rgb2, -1 );
        *im1_ptr = im1;
        *im2_ptr = im2;
        image1 = im1_ptr; // return pointer value
        image2 = im2_ptr; 
    }
    
    void setRGBImages( cv::Mat *rgb1, cv::Mat *rgb2 )
    {
        image1 = boost::shared_ptr<cv::Mat>(rgb1);
        image2 = boost::shared_ptr<cv::Mat>(rgb2);
//         image1 = rgb1;
//         image2 = rgb2;
    }
    
    void setDepthImages( cv::Mat *range1, cv::Mat *range2 )
    {
        depth1 = boost::shared_ptr<cv::Mat>(range1);;
        depth2 = boost::shared_ptr<cv::Mat>(range2);
    }
    
    void setDepthImages( std::string &range1, std::string &range2 )
    {
        boost::shared_ptr< cv::Mat > im1_ptr (new cv::Mat); // smart pointer to free memory properly
        boost::shared_ptr< cv::Mat > im2_ptr (new cv::Mat);
        cv::Mat im1 = cv::imread( range1, -1 ); //load image
        cv::Mat im2 = cv::imread( range2, -1 );
//         boost::shared_ptr< cv::Mat > im1_ptr (&im1); // smart pointer to free memory properly
//         boost::shared_ptr< cv::Mat > im2_ptr (&im2);
        *im1_ptr = im1;
        *im2_ptr = im2;
        depth1 = im1_ptr; // return pointer value
        depth2 = im2_ptr; 
    }
    
public:
    // Constructors
    PoseICP( )
    {
        load_calib = false;
        load_images = false;
        load_clouds = false;
    };
    
    PoseICP( Eigen::Matrix3d &Calib )
    {
        setCalibration( Calib );
        load_images = false;
        load_clouds = false;
    };
    
    PoseICP( cv::Mat *rgb1, cv::Mat *range1, cv::Mat *rgb2, cv::Mat *range2, Eigen::Matrix3d &Calib )
    {
        setCalibration( Calib );
        setImages( rgb1, range1, rgb2, range2 );
        load_clouds = false;
    };
    
    PoseICP( std::string &rgb1, std::string &range1, std::string &rgb2, std::string &range2, Eigen::Matrix3d &Calib )
    {
        setCalibration( Calib );
        setImages( rgb1, range1, rgb2, range2 );
        load_clouds = false;
    };
    
    PoseICP( cv::Mat *rgb1, cv::Mat *range1, cv::Mat *rgb2, cv::Mat *range2 )
    {
        setImages( rgb1, range1, rgb2, range2 );
        load_clouds = false;
    };
    
    PoseICP( std::string &rgb1, std::string &range1, std::string &rgb2, std::string &range2 )
    {
        setImages( rgb1, range1, rgb2, range2 );
        load_calib = false;
        load_clouds = false;
    };
    
    PoseICP( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pc1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pc2, Eigen::Matrix3d &Calib )
    {
        setCalibration( Calib );
        setClouds( pc1, pc2 );
        load_images = false;
    }
    
    PoseICP( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pc1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pc2 )
    {
        setClouds( pc1, pc2 );
        load_calib = false;
        load_images = false;
    }
    
    // Destructor
    ~PoseICP() { ; };
    
    // Function to run the entire process
    void run();
    
    // Function to extract cloud1 and cloud2 from images
    void solveClouds();
    
    // Function to import cloud1 and cloud2 from pcd files
    void importPCD( const char *file_pcd1, const char *file_pcd2);
    
    // Function to solve pose with icp using cloud1 and cloud2
    void solvePose();
    
    // SET Functions
    void setCalibration( Eigen::Matrix3d &Calib) 
    { 
        Calibration = Calib;
        load_calib = true;
    };
    
    void simpleCalibration( double focal = 520.0 )
    {
        if ( !load_images )
        {
	  DEBUG_E( ("Not loaded Images. Use properly the constructor or set parameters manually") ); 
	  exit(-1);
        }
        double wimg = image1->size().width - 1.0;
        double himg = image1->size().height - 1.0;
        Eigen::Matrix3d K;
        K << focal, 0.0, wimg/2, 0.0, focal, himg/2, 0.0, 0.0, 1.0;		// Camera Matrix (intrinsics)
        
        setCalibration( K );
    }
    
    void setImages( std::string &rgb1, std::string &range1, std::string &rgb2, std::string &range2 )
    {
        setRGBImages( rgb1, rgb2 );
        setDepthImages(range1, range2);
        load_images = true;
    }
    
    void setImages( cv::Mat *rgb1, cv::Mat *range1, cv::Mat *rgb2, cv::Mat *range2 )
    {
        setRGBImages( rgb1, rgb2 );
        setDepthImages(range1, range2);
        load_images = true;
    }
    
    void setClouds( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pc1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pc2 ) 
    {
        cloud1 = pc1;
        cloud2 = pc2;
        load_clouds = true;
    };
    
    // GET Functions
    Eigen::Quaternion<double> getQuaternion() { Eigen::Quaternion<double> Qn(Rotation); return Qn; };
    Eigen::Matrix3d getRotation() { return Rotation; };
    Eigen::Vector3d getTranslation() { return translation; };
    Eigen::Matrix4d getTransformationMatrix() 
    { 
        Eigen::Matrix4d MM;
        MM << Rotation, translation, 0.0, 0.0, 0.0, 0.1;
    }

};

#endif