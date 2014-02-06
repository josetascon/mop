/**
 * @file DepthProjection.hpp
 * @brief This file has all functions related with projection of depth with Kinect data.
 *
 * @author José David Tascón Vidarte
 * @date May/08/2013
 */

// AXIS Y without -1, postive coordinates downward

#ifndef __DEPTHPROJECTION_HPP__
#define __DEPTHPROJECTION_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// OpenCV Libraries
#include <opencv2/opencv.hpp>

// Std Libraries
#include <iostream>
#include <cmath>

// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================

/**
 * ******************************************************************
 * @brief Description: Check if a point has a valid measurement in Depth image
 * Depth Values are scaled, then 5000 equivalent to 1m
 * If close_range Bad points aren't stored (equals to 0, > 3 meters, or < 0.5 meters).
 * 
 * @param image_point	(input/output) Points that you want to check
 * @param measurez 		(input) Depth image with z values
 * @param close_range 	(input) Valid range  0.5 < z < 3 meters. Default value = true
 * 
 * @date May/08/2013
 */
void removeBadPoints(std::vector< cv::Point2d > &image_point, cv::Mat &measurez, bool close_range = true);

/**
 * ******************************************************************
 * @brief Description: Check if a point from any input image_point has a valid measurement in Depth image.
 * This function work with dual of images (if one point is bad it removes the dual set).
 * Depth Values are scaled, then 5000 equivalent to 1m.
 * If close_range Bad points aren't stored (equals to 0, > 3 meters, or < 0.5 meters).
 * 
 * @param image_point1	(input/output) Points that you want to check from image 1
 * @param image_point2	(input/output) Points that you want to check from image 2
 * @param measurez1		(input) Depth image 1 with z values
 * @param measurez2		(input) Depth image 2 with z values
 * @param close_range 	(input) Valid range  0.5 < z < 3 meters. Default value = true
 * 
 * @date May/08/2013
 */
std::vector<int> removeBadPointsDual(std::vector< cv::Point2d > &image_point1, std::vector< cv::Point2d > &image_point2,
						 cv::Mat &measurez1, cv::Mat &measurez2, bool close_range = true);

// Eigen::Matrix<int,-1, 1> removeBadPointsDual(std::vector< Eigen::Vector3d > &image_point1, std::vector< Eigen::Vector3d > &image_point2,
// 				        cv::Mat &measurez1, cv::Mat &measurez2, Eigen::MatrixXd &x1, Eigen::MatrixXd &x2, 
// 				        bool close_range = true);


template< typename Tp1, typename Tp2 >
void projectiongeneral(Tp1 image_point[2], Tp2 &measurez, Tp2 intrinsic[4], Tp2 structure[3], Tp2 factor = 5000.0)
{
    // image point = u & v coordinates
    // measurez = depth from range sensor image
    // intrinsic = calibration param as cx, cy, fx, fy;
    // structure = 3d point output
//     Tp2 factor = 5000.0;		// Global scale factor. For the 16-bit PNG files 5000 equivalent to 1m. Use 50.0 to change scale to cm
    // ====================================        World Point Calculation      ====================================
    structure[2] = measurez / factor;
    structure[0] = (static_cast<Tp2> (image_point[0]) - intrinsic[0]) * structure[2] / intrinsic[2];
    structure[1] = (static_cast<Tp2> (image_point[1]) - intrinsic[1]) * structure[2] / intrinsic[3];
}

template < typename Tp>
bool boundarykinect( Tp &measurez, Tp factor = 5000.0 )
{
    // Detects if a Z value (measurez) is inside Kinect boundary
    if (measurez == 0 || measurez < 0.45*factor || measurez > 3.0*factor) 
        return false;
    else return true;
}


/**
 * ******************************************************************
 * @brief Description: Projects a 2d point to a 3d point from depth measurements and calibration parameters.
 * Depth Values are scaled, then 5000 equivalent to 1m
 * 
 * @param image_point	(input) Image point coordinates
 * @param measurez		(input) Depth image with z values
 * @param cx		(input) Central pixel of x axis
 * @param cy		(input) Central pixel of y axis
 * @param fx		(input) Focal length of x axis
 * @param fy		(input) Focal length of y axis
 * 
 * @return 		(cv::Point3d) 3D point projected from image point
 * 
 * @date May/08/2013
 */
cv::Point3d projection(cv::Point2d &image_point, cv::Mat &measurez, double &cx, double &cy, double &fx, double &fy);

template < typename Tp, typename Teig1, typename Teig2 >
Eigen::Matrix<Teig2, 3, 1> projection(Tp &x, Tp &y, cv::Mat &measurez, Eigen::Matrix<Teig1, 3, 3> &calibration)
{
    Tp uv[2] = { x, y };
    Teig2 intrinsic[4] = { (Teig2)calibration(0,2), (Teig2)calibration(1,2), (Teig2)calibration(0,0), (Teig2)calibration(1,1) };
    Teig2 xyz[3];
    Teig2 depth = (Teig2) measurez.at<short>( uv[1], uv[0] );
    
    projectiongeneral< Tp, Teig2 >( uv, depth, intrinsic, xyz) ;
    return Eigen::Matrix<Teig2, 3, 1>(xyz[0], xyz[1], xyz[2]);
//     Teig factor = 5000.0; 			// for the 16-bit PNG files 5000 equivalent to 1m. Use 50.0 to change scale to cm
//     // ====================================        World Point Calculation      ====================================
//     Teig z_ = depth.at<short>(y,x) / factor;
//     Teig x_ = (x - calibration(0,2)) * z_ / calibration(0,0);
//     Teig y_ = (y - calibration(1,2)) * z_ / calibration(1,1);
//     // Debug
// //     printf("2D_Point = [%i, %i]\t||\t3D_Point = [%f, %f, %f]\n", x, y, x_, y_, z_);
//     return Eigen::Matrix<Teig, 3, 1>(x_, y_, z_);
}

template <typename Teig3, typename Teig2, typename Teig1 >
Eigen::Matrix<Teig1, 3, 1> projection(Eigen::Matrix<Teig2, 3, 1> &image_point, cv::Mat &measurez, Eigen::Matrix<Teig3, 3, 3> &calibration)
{
    Teig2 uv[2] = { image_point(0), image_point(1) };
    Teig1 intrinsic[4] = { (Teig1)calibration(0,2), (Teig1)calibration(1,2), (Teig1)calibration(0,0), (Teig1)calibration(1,1) };
    Teig1 xyz[3];
    Teig1 depth = (Teig1) measurez.at<short>( uv[1], uv[0] );
    
    projectiongeneral< Teig2, Teig1 >( uv, depth, intrinsic, xyz) ;
    return Eigen::Matrix<Teig1, 3, 1>(xyz[0], xyz[1], xyz[2]);
}

/**
 * ******************************************************************
 * @brief Description: Extract color data from a Image with a coordinate points.
 * 
 * @param image_point	(input) Desired image points coordinates that want to extract color information
 * @param color_image	(input) RGB image data
 * 
 * @return (cv::Point3f) Color data in RGB format
 * 
 * @date May/08/2013
 */
cv::Point3f colorExtraction(cv::Point2d &image_point, cv::Mat &color_image);
float colorExtraction(int x, int y, cv::Mat &color_image);
float colorExtraction(Eigen::Vector2i &image_point, cv::Mat &color_image);


// NOTE CONSIDET TO DELETE
// template <typename Teig>
// void extractDepth(Eigen::Matrix<Teig,-1,-1> &x2d, cv::Mat &depth, Eigen::Matrix<Teig,-1,-1> &matrix_z)
// {
//     int num_points = x2d.cols();
//     Teig factor = 5000.0; // 1m = 5000.0
//     matrix_z = Eigen::Matrix<Teig,-1,-1>::Zero(3,num_points);
//     for ( register int i = 0; i < num_points; ++i)
//     {
//         int u = int(x2d(0,i));
//         int v = int(x2d(1,i));
//         matrix_z.col(i) = Eigen::Matrix<Teig,3,1>::Ones()*( (Teig( depth.at<short>(v,u) ))/factor );
//     } // Create a Matrix 3xn where each column has the same Z per column
// };
// 
// // It was more efficient to use calc3Dfrom2D with std::vector<cv::Point3d> as output
// template <typename Teig>
// void calc3Dfrom2DEigen(Eigen::Matrix<Teig,-1,-1> &x2d, cv::Mat &depth, Eigen::Matrix3d kalibration, Eigen::Matrix<Teig,-1,-1> &X3D)
// {
//     // Backprojection with the inverse of K, and using depth measurement form Kinect
//     int num_points = x2d.cols();
//     X3D = Eigen::Matrix<Teig,-1,-1>::Ones(3,num_points);
//     Eigen::Matrix<Teig,3,3> T = (kalibration.inverse()).template cast<Teig>();;
//     
//     Eigen::Matrix<Teig,-1,-1> matrix_z;
//     extractDepth(x2d, depth, matrix_z);
//     X3D = T*( x2d.cwiseProduct(matrix_z) );
// };


/**
 * ******************************************************************
 * @brief (KEYPOINTS VERSION) Description: Projects KeyPoints from a Image to a Scene with Depth measurements.
 * Depth Values are scaled, then 5000 equivalent to 1m
 * 
 * @param corners		(input) Desired image points coordinates to project in KeyPoint format
 * @param depth		(input) Depth image data
 * @param kalibration	(input) Calibration Matrix
 * @param world_point3d	(output) 3D pojected points set
 * 
 * @date May/08/2013
 */
void calc3Dfrom2D(std::vector<cv::KeyPoint> &corners, cv::Mat &depth, cv::Mat &kalibration,
						std::vector<cv::Point3d> &world_point3d);

/**
 * ******************************************************************
 * @brief (POINTS VERSION) Description: Projects KeyPoints from a Image to a Scene with Depth measurements.
 * Depth Values are scaled, then 5000 equivalent to 1m
 * 
 * @param corners		(input) Desired image points coordinates to project in Point2d format
 * @param depth		(input) Depth image data
 * @param kalibration	(input) Calibration Matrix
 * @param world_point3d	(output) 3D pojected points set
 * 
 * @date May/08/2013
 */
void calc3Dfrom2D(std::vector<cv::Point2d> &corners, cv::Mat &depth, cv::Mat &kalibration,
						std::vector<cv::Point3d> &world_point3d);

void calc3Dfrom2D(Eigen::MatrixXd x2d, cv::Mat &depth, Eigen::Matrix3d &kalibration, Eigen::MatrixXd &X3D);


/**
 * ******************************************************************
 * @brief (KEYPOINTS VERSION) Description: Projects KeyPoints from a Image to a Scene with Depth measurements.
 * It extracts colors from Image too.
 * Depth Values are scaled, then 5000 equivalent to 1m
 * 
 * @param corners		(input) Desired image points coordinates to project in KeyPoint format
 * @param depth		(input) Depth image data
 * @param kalibration	(input) Calibration Matrix
 * @param image 		(input) RGB image data
 * @param world_point3d	(output) 3D pojected points set
 * @param color		(output) Color of 3D points (Size = [npoints]x[3(RGB)] double)
 * 
 * @date May/08/2013
 */
void calc3DandRGBfrom2D(std::vector<cv::KeyPoint> &corners, cv::Mat &depth, cv::Mat &kalibration, cv::Mat &image, 
							std::vector<cv::Point3d> &world_point3d, std::vector< cv::Point3f > &color);

/**
 * ******************************************************************
 * @brief (POINTS VERSION) Description: Projects KeyPoints from a Image to a Scene with Depth measurements.
 * It extracts colors from Image too.
 * Depth Values are scaled, then 5000 equivalent to 1m
 * 
 * @param corners		(input) Desired image points coordinates to project in Point2d format
 * @param depth		(input) Depth image data
 * @param kalibration	(input) Calibration Matrix
 * @param image 		(input) RGB image data
 * @param world_point3d	(output) 3D pojected points set
 * @param color		(output) Color of 3D points (Size = [npoints]x[3(RGB)] double)
 * 
 * @date May/08/2013
 */
void calc3DandRGBfrom2D(std::vector<cv::Point2d> &corners, cv::Mat &depth, cv::Mat &kalibration, cv::Mat &image, 
							std::vector<cv::Point3d> &world_point3d, std::vector< cv::Point3f > &color);


// Variance
void varianceDepth(double &z, double &sigma);
void varianceDepth(Eigen::RowVectorXd &z, Eigen::RowVectorXd &sigma);
void varianceKinect( Eigen::Vector3d &X, Eigen::Matrix3d &K, Eigen::Vector3d &W, double vx = 1.0, double vy = 1.0);
void varianceKinectSet( Eigen::MatrixXd &X, Eigen::Matrix3d &K, Eigen::MatrixXd &W, double vx = 1.0, double vy = 1.0);



#endif