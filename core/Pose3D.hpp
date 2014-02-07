/**
 * @file Pose3D.hpp
 * @brief This file has all functions related 
 *
 * @author José David Tascón Vidarte
 * @date Dec/09/2013
 */

#ifndef __POSE3D_HPP__
#define __POSE3D_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// OpenCV Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>

// Boost Libraries
#include <boost/shared_ptr.hpp>

// Std Libraries
#include <iostream>
#include <vector>

// Local Libraries
#include "Debug.hpp"
#include "Common.hpp"
#include "CameraPose.hpp"
#include "FeaturesEDM.hpp"
#include "Optimizer.hpp"
#include "DepthProjection.hpp"

// ================================================================================================
// ======================================= CLASS Pose3D ===========================================
// ================================================================================================
class Pose3D		// Registration Aligment of two sets
{
private:
    cv::Mat KOCV;
    Eigen::Matrix3d Calibration;
    Eigen::Matrix3d Rotation;
    Eigen::Vector3d translation;
    
    Eigen::MatrixXd X1;
    Eigen::MatrixXd X2;
    Eigen::MatrixXd variance1;
    Eigen::MatrixXd variance2;
    
    cv::Mat *image1;
    cv::Mat *image2;
    cv::Mat *depth1;
    cv::Mat *depth2;
    
    bool load_images;
    bool load_calib;
    bool load_points;
    
    void setRGBImages( std::string &rgb1, std::string &rgb2 )
    {
        boost::shared_ptr< cv::Mat > im1_ptr (new cv::Mat); // smart pointer to free memory properly
        boost::shared_ptr< cv::Mat > im2_ptr (new cv::Mat);
        *im1_ptr = cv::imread( rgb1, -1 ); //load image
        *im2_ptr = cv::imread( rgb2, -1 );
        image1 = im1_ptr.get(); // return pointer value
        image2 = im2_ptr.get(); 
    }
    
    void setRGBImages( cv::Mat *rgb1, cv::Mat *rgb2 )
    {
        image1 = rgb1;
        image2 = rgb2;
    }
    
    void setDepthImages( cv::Mat *range1, cv::Mat *range2 )
    {
        depth1 = range1;
        depth2 = range2;
    }
    
    void setDepthImages( std::string &range1, std::string &range2 )
    {
        boost::shared_ptr< cv::Mat > im1_ptr (new cv::Mat); // smart pointer to free memory properly
        boost::shared_ptr< cv::Mat > im2_ptr (new cv::Mat);
        *im1_ptr = cv::imread( range1, -1 ); //load image
        *im2_ptr = cv::imread( range2, -1 );
        depth1 = im1_ptr.get(); // return pointer value
        depth2 = im2_ptr.get(); 
    }
    
public:
    
    // Constructors
    Pose3D( )
    {
        load_calib = false;
        load_images = false;
        load_points = false;
    };
    Pose3D( Eigen::Matrix3d &Calib )
    {
        setCalibration( Calib );
        load_images = false;
        load_points = false;
    };
    
    Pose3D( cv::Mat *rgb1, cv::Mat *range1, cv::Mat *rgb2, cv::Mat *range2, Eigen::Matrix3d &Calib )
    {
        setCalibration( Calib );
        setImages( rgb1, range1, rgb2, range2 );
        load_points = false;
    };
    
    Pose3D( std::string &rgb1, std::string &range1, std::string &rgb2, std::string &range2, Eigen::Matrix3d &Calib )
    {
        setCalibration( Calib );
        setImages( rgb1, range1, rgb2, range2 );
        load_points = false;
    };
    
    Pose3D( cv::Mat *rgb1, cv::Mat *range1, cv::Mat *rgb2, cv::Mat *range2 )
    {
        setImages( rgb1, range1, rgb2, range2 );
        load_calib = false;
        load_points = false;
    };
    
    Pose3D( std::string &rgb1, std::string &range1, std::string &rgb2, std::string &range2 )
    {
        setImages( rgb1, range1, rgb2, range2 );
        load_calib = false;
        load_points = false;
    };
    
    Pose3D( Eigen::MatrixXd &points1, Eigen::MatrixXd &points2, Eigen::Matrix3d &Calib )
    {
        setCalibration( Calib );
        setPoints( points1, points2 );
        load_images = false;
    }
    
    Pose3D( Eigen::MatrixXd &points1, Eigen::MatrixXd &points2 )
    {
        setPoints( points1, points2 );
        load_calib = false;
        load_images = false;
    }
    
    // Destructor
    ~Pose3D() { ; };
    
    // Function to run the entire process
    void run();
    
    // Function to extract points X1 and X2
    void solvePoints();
    
    // Function to extract points X1 and X2 from opencv points
    void adaptPoints( std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, cv::Mat &range1, cv::Mat &range2 );
    
    // Function to solve pose from X1 and X2
    void solvePose(bool optimal = true);
    
    // SET Functions
    void setCalibration( Eigen::Matrix3d &Calib) 
    { 
        Calibration = Calib;
        KOCV = (cv::Mat_<double>(3,3) << Calibration(0,0), 0.0, Calibration(0,2), 0.0, Calibration(1,1), Calibration(1,2), 0.0, 0.0, 1.0);
        load_calib = true;
    };
    
    void simpleCalibration( double focal = 520.0 )
    {
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
    
    void setPoints( Eigen::MatrixXd &points1, Eigen::MatrixXd &points2 ) 
    {
        X1 = points1;
        X2 = points2;
        load_points = true;
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
    
    Eigen::MatrixXd getPointsX1() { return X1; };
    Eigen::MatrixXd getPointsX2() { return X2; };
    
    Eigen::MatrixXd getVariance1() { return variance1; };
    Eigen::MatrixXd getVariance2() { return variance2; };

};

#endif