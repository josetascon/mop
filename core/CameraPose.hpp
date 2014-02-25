/**
 * @file CameraPose.hpp
 * @brief This file has all functions related with Projective and Epipolar Geometry
 *
 * @author José David Tascón Vidarte
 * @date May/12/2013
 */

#ifndef __CAMERAPOSE_HPP__
#define __CAMERAPOSE_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// OpenCV Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// Std Libraries
#include <iostream>

// Local Libraries
#include "Debug.hpp"
#include "Common.hpp"

// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================

/**
 * ******************************************************************
 * @brief Decription: If determinat of Rot is -1 it multiplies the third column and corrects it to 1.
 * Rotation matrices with det = -1 are reflection matrices.
 * 
 * @param Rot 		(input/output) Rotation Matrix
 * 
 * @date Jul/08/2013
 */
bool determinantCorrection(Eigen::Matrix3d &Rot);

/**
 * ******************************************************************
 * @brief Description: Check if a Rotation Matrix has a determinant = -1 or 1 (orthogonality). 
 * If determinat of Rot is -1 it call determinantCorrection(Rot) function and corrects to 1.
 * Rotation matrices with det = -1 are reflection matrices.
 * 
 * @param Rot 		(input) Rotation Matrix
 * @return (bool) true value if det(Rot) = -1 or 1.
 * 
 * @date Jul/08/2013
 */
bool checkCoherentRotation(Eigen::Matrix3d &Rot);

//@date Jul/18/2013
Eigen::MatrixXd buildProjectionMatrix( Eigen::Matrix3d &kalibration, Eigen::Matrix3d &Rotation, Eigen::Vector3d &translation);


/// Set Origin of coordinates to [0 0 0]
template< typename Tp >
void setCoordinatestoOrigin(std::vector< Eigen::Quaternion<Tp> > &Qn_global, std::vector< Eigen::Matrix<Tp,3,1> > &tr_global )
{
    Eigen::Quaternion<Tp> q0 = Qn_global[0];
    Eigen::Matrix<Tp,3,1> t0 = tr_global[0];
    for (register int k = 0; k < Qn_global.size(); ++k)
    {
        Qn_global[k] = Qn_global[k]*q0.conjugate();
        tr_global[k] = tr_global[k] - Qn_global[k]*t0;
    }
}

/// Set coordinates to an specific point
template< typename Tp >
void setCoordinatestoDesiredPosition(std::vector< Eigen::Quaternion<Tp> > &Qn_global, std::vector< Eigen::Matrix<Tp,3,1> > &tr_global,
		    Eigen::Quaternion<Tp> &qn_desired, Eigen::Matrix<Tp,3,1> &tr_desired, int camera = 0 )
{
    // Other idea
    {
//     Eigen::Matrix<Tp,3,1> center = Qn_global[camera].conjugate()*(-tr_global[camera]);
//     Eigen::Matrix<Tp,3,1> center_desired = qn_desired.conjugate()*(-tr_desired);
//     Eigen::Matrix<Tp,3,1> c0 = -center + center_desired;
    }
    
    Eigen::Quaternion<Tp> q0 = Qn_global[camera].conjugate()*qn_desired;
    Eigen::Matrix<Tp,3,1> t0 = qn_desired.conjugate()*(tr_global[camera] - tr_desired);
    // when the update take place for camera then:
//     Qn_global[camera] = q0*Qn_global[camera]
//     tr_global[camera] = tr_global[camera] - Qn_global[camera]*t0 // HERE Qn_global[camera] is now qn_desired
    
    for (register int k = 0; k < Qn_global.size(); ++k)
    {
        Qn_global[k] = Qn_global[k]*q0;
        tr_global[k] = tr_global[k] - Qn_global[k]*t0;
    }
}



/**
 * ******************************************************************
 * @brief Description: Get fundamental matrix with a robust algorithm (RANSAC) and remove points marked as outliers from pts1 and pts2
 * 
 * @param pts1 		(input/output) Points coordinates of features from image1
 * @param pts2 		(input/output) Related (in order) features from image2 
 * @param matches		(output)
 * @param Fundamental	(output) Fundamental Matrix (rank 2 matrix)
 * 
 * @date May/12/2013
 */
void getFundamentalandRemoveOutliers(std::vector< cv::Point2d > &pts1, std::vector< cv::Point2d > &pts2, std::vector< cv::DMatch > &matches,
					cv::Mat &Fundamental);

int robustMatchesfromFundamental(std::vector< cv::Point2d > &pts1, std::vector< cv::Point2d > &pts2, std::vector<cv::DMatch> &matches, cv::Mat &Fund, int max_iter);

/**
 * ******************************************************************
 * @brief Description: Find essential matrix from fundamental matrix and calibration matrix.
 * Use the linear algorithm in HZ2004 to find Rotation and translation from the essential matrix.
 * So this algorithm offers four solutions because his output are 2 Rotation matrices and 2 translation vector.
 * 
 * @param Fundamental	(input) Fundamental Matrix (rank 2 matrix)
 * @param kalibration	(input) Calibration matrix k (intrinsics parameters of a camera)
 * @param Rot1		(output) First rotation matrix solution
 * @param Rot2		(output) Second rotation matrix solution
 * @param translation1	(output) First translation vector solution
 * @param translation2	(output) Second translation vector solution
 * 
 * @date May/12/2013
 */
void poseFundamental( cv::Mat &Fundamental, cv::Mat &kalibration,
		cv::Mat &Essential, cv::Mat &Rot1, cv::Mat &Rot2, cv::Mat &translation1, cv::Mat &translation2);

void fundamental2essential( Eigen::Matrix3d &Fundamental, Eigen::Matrix3d &kalibration,
		        Eigen::Matrix3d &Essential);

/**
 * ******************************************************************
 * @brief Description: Use the linear algorithm in HZ2004 to find Rotation and translation from the essential matrix.
 * So this algorithm offers four solutions because his output are 2 Rotation matrices and 2 translation vector.
 * 
 * @param Essential		(input) Essential Matrix (rank 2 matrix)
 * @param Rot1		(output) First rotation matrix solution
 * @param Rot2		(output) Second rotation matrix solution
 * @param translation1	(output) First translation vector solution
 * @param translation2	(output) Second translation vector solution
 * 
 * @date Jul/15/2013
 */

void poseEssential( Eigen::Matrix3d &Essential, Eigen::Matrix3d &Rot1, Eigen::Matrix3d &Rot2, 
		    Eigen::Vector3d &ttr1, Eigen::Vector3d &ttr2);
/**
 * ******************************************************************
 * @brief Description: posefrom3DPoints is a 3D to 3D camera pose estimation using linear method from [ARUM87] && [MARMONE07]
 * 
 * @param WorldData1	(input) 3D Points of a scene, initial position (first shot)
 * @param WorldData2 	(input) 3D Points of the same scene, from a different position (second shot)
 * @param Rotation 		(output) Found Camera Rotation
 * @param translation	(output) Found translation from first and second shot
 * 
 * @date Jul/08/2013
 */
template < typename Teig >
void poseArun( Eigen::Matrix<Teig,-1,-1> &WorldData1, Eigen::Matrix<Teig,-1,-1> &WorldData2, 
		   Eigen::Matrix<Teig,3,3> &Rotation, Eigen::Matrix<Teig,3,1> &translation )
{
    // Calc Centroids (mean)
    Eigen::Matrix<Teig,-1,1> centroidA = WorldData1.rowwise().mean();
    Eigen::Matrix<Teig,-1,1> centroidB = WorldData2.rowwise().mean();
    // Debug:
//     std::cout << "Mean of data 1, centroidA:\n" << centroidA << '\n';
//     std::cout << "Mean of data 2, centroidB:\n" << centroidB << '\n';
    // Matrix H as the covariance matrix
    Eigen::Matrix<Teig,-1,-1> H1 = (WorldData2.colwise() - centroidB);
    Eigen::Matrix<Teig,-1,-1> H = (WorldData1.colwise() - centroidA) * H1.transpose();
    // H = 3x3 || NOW 3xn * 3xn'
    Eigen::JacobiSVD<Eigen::Matrix<Teig,-1,-1>> svd(H, Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::Matrix<Teig,-1,-1> VV = svd.matrixV();
    Eigen::Matrix<Teig,-1,-1> UU = svd.matrixU();
    
    Rotation = (Eigen::Matrix<Teig,3,3>)(VV*UU.transpose()); // Recovered Rotation Matrix
    translation = Eigen::Matrix<Teig,3,1>(-Rotation*centroidA + centroidB); //Recover translation
}

/// Estimate a Camera matrix with 2D (xpt) and 3D points (Xpt) with a linear SVD algorithm
Eigen::MatrixXd linearCamera( Eigen::MatrixXd &xpt, Eigen::MatrixXd &Xpt);

/// Given a camera matrix P (with input Calibration) extract Rotation and translation.
template < typename Teig >
void poseCameraMatrix3x4( Eigen::Matrix<Teig,-1,-1> &Pmatrix, Eigen::Matrix<Teig,3,3> &Calibration, 
	  Eigen::Matrix<Teig,3,3> &Rotation, Eigen::Matrix<Teig,3,1> &translation )
{
    Eigen::Matrix<Teig,-1,-1> P = Pmatrix;
    Eigen::Matrix<Teig,3,3> RR, R_angle;
    Eigen::Matrix<Teig,3,1> tt, angles_vec;
    
    P = Calibration.inverse()*P;
//     RR << P(0,0), P(0,1), P(0,2), P(1,0), P(1,1), P(1,2), P(2,0), P(2,1), P(2,2);	// Extract rotation [0:2 x 0:2]
    RR = P.block(0,0,3,3);		// Extract rotation [0:2 x 0:2]
    tt = P.col(3); 			// Extract translation, last column
    
    // Procedure to achieve a consistent Rotation with det = 1
    Teig sd = sign(RR.determinant());
    Eigen::JacobiSVD< Eigen::Matrix<Teig,-1,-1> > svd(RR, Eigen::ComputeThinU | Eigen::ComputeThinV); // Normalize with the greatest singular value
    Teig nv = 1/((svd.singularValues())(0));
    tt = sd*nv*tt; 				// Normalization is also applied to tt 
    RR = sd*nv*RR; 				// Rotation matrix must have 3 singular values = to 1, in order to achieve a det = 1;
    rotation2angles<Teig>(RR, angles_vec);	// Find Euler angles.
					// Do not use direct conversion from RR to Quaternion because the det of RR at this point is not 1
    R_angle = Eigen::AngleAxisd(angles_vec(0)*CONSTANT_PI/180, Eigen::Vector3d::UnitX()) * 
    Eigen::AngleAxisd(angles_vec(1)*CONSTANT_PI/180,  Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(angles_vec(2)*CONSTANT_PI/180, Eigen::Vector3d::UnitZ());
    //Debug
//     std::cout << "nv = " << nv << "\n";
//     std::cout << "sd = " << sd << "\n";
//     std::cout << "P3\n" << P2 << "\n";
//     std::cout << "Rot\n" << RR << "\n";
    DEBUG_3( std::cout << "Rotation angles:\n" << angles_vec.transpose() << "\n"; )
    DEBUG_3( std::cout << "translation:\n" << tt.transpose() << "\n"; )
    
    Rotation = R_angle;
    translation = tt;
}

#endif