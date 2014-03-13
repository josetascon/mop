// Created: Dec/09/2013
// Author: José David Tascón Vidarte

#include "SimpleRegistrationSfM.hpp"
#include "LinearEstimator.hpp"
#include "Triangulation.hpp"


// ================================================================================================
// ========================== FUNCTIONS of CLASS SimpleRegistrationSfM ============================
// ================================================================================================
void SimpleRegistrationSfM::solvePose( boost::shared_ptr< MXb > visibility, boost::shared_ptr< MX_V3d > coordinates, 
		std::vector< std::string > &depth_list, bool optimal)
{
    // Initialization
    initializeGlobalVariables();
    initializeCoordinatesOrigin();
    Structure = Eigen::MatrixXd::Zero(3,num_features);
    Covariance = Eigen::MatrixXd::Zero(3,num_features);
    
    std::vector<int> ft_number;
    std::vector< cv::Point2d > pts1, pts2;
    std::vector< cv::Point3d > WP1, WP2;
    std::vector< cv::Point3d > pts1copy, pts2copy, pts2image;
    std::vector< Eigen::Vector4d > common_st;
    
    initilizeStructure( visibility, coordinates, depth_list[0] );
    
    std::cout << "\n================================ POSE Estimation ==================================\n";
    for(int k = 0; k < (num_cameras - 1); k++) // pose 1-2. Solve pose for continous cameras
    {
        int cam1 = k;
        int cam2 = k+1;
        pts1.clear(); pts2.clear();
        pts1copy.clear(); pts2copy.clear();
        pts2image.clear();
        common_st.clear();
        ft_number.clear();
        
        for (register int ft = 0; ft < num_features; ++ft)		// Select valid features for corresponding camera
        {
	  if ((*visibility)(cam1,ft) && (*visibility)(cam2,ft) )   // Temporal saving of image coordinates for mutual points
	  {
	      Eigen::Vector3d point1 = (*coordinates)(cam1,ft);
	      Eigen::Vector3d point2 = (*coordinates)(cam2,ft);
	      pts1.push_back( cv::Point2d(point1(0), point1(1)) );
	      pts2.push_back( cv::Point2d(point2(0), point2(1)) );
	      pts1copy.push_back( cv::Point3d(point1(0), point1(1), 1.0) );
	      pts2copy.push_back( cv::Point3d(point2(0), point2(1), 1.0) );
	      ft_number.push_back(ft);
	      
	      if ( Structure(2,ft) != 0 )
	      {
		pts2image.push_back( cv::Point3d(point2(0), point2(1), 1.0) ); //Store data in case of sfm fallback
		Eigen::Vector3d st = Structure.col(ft);
		common_st.push_back( Eigen::Vector4d( st(0),st(1),st(2),1.0 ) );
	      }
	  }
        }
        
        // Debug
        DEBUG_1( printf("POSE: %04i -> %04i:\n", cam1, cam2); )
        DEBUG_1( std::cout << "Total Pts: " << pts1.size() << "\n"; )
        
        cv::Mat depth1 = cv::imread( depth_list[cam1], -1 );
        cv::Mat depth2 = cv::imread( depth_list[cam2], -1 );
        
        Pose3D pose( Calibration );
        pose.adaptPoints(pts1, pts2, depth1, depth2);
        
        if ( pts1.size() > valid_sfm_points ) 
        {
	  //Estimate Rotation with Arun + optimization
	  pose.solvePose(optimal);
	  Rot = pose.getRotation();
	  tr = pose.getTranslation();
        }
        else 
        {
	  // SfM FALLBACK. Finding new camera from 3D point to 2D relation
	  Eigen::MatrixXd x2dd, X2temp;
	  Eigen::MatrixXd P2(3,4);
	  point3_vector2eigen( pts2image, x2dd );
	  eigen_vector2eigen(common_st, X2temp);
	  P2 = linearCamera( x2dd, X2temp);
	  poseCameraMatrix3x4( P2, Calibration, Rot, tr );
        }
        
        // COMPILER FLAG
        XMODEL
        (
	  Xmodel->push_back( pose.getPointsX1() );
	  Xmodel->push_back( pose.getPointsX2() );
	  Variance->push_back( pose.getVariance1() );
	  Variance->push_back( pose.getVariance2() );
        )
        
        updateGlobal( k+1 );
        Eigen::MatrixXd x1_im, x2_im;
        point3_vector2eigen( pts1copy, x1_im );
        point3_vector2eigen( pts2copy, x2_im );
        updateStructure( k+1, ft_number, x1_im, x2_im, depth2 );
        
        DEBUG_2( printGlobal( k+1 ); )
    }
}

void SimpleRegistrationSfM::initilizeStructure( boost::shared_ptr< MXb > visibility, boost::shared_ptr< MX_V3d > coordinates, std::string file_depth)
{
    cv::Mat depth1 = cv::imread( file_depth, -1 );
    for (register int ft = 0; ft < num_features; ++ft)		// Initialize structure in first camera
    {
        if ((*visibility)(0,ft))
        {
	  Eigen::Vector3d image_point = (*coordinates)(0,ft);
	  Eigen::Vector3d pt_space = projection<double,double,double>(image_point, depth1, Calibration);
	  Eigen::Vector3d W;
	  if ( boundarykinect( pt_space(2), 1.0 ) ) 		// Check kinect valid range. Only initialize valid points in first image
	  {
	      Structure.col(ft) = pt_space;
	      varianceKinect( pt_space, Calibration, W);
	      Covariance.col(ft) = W;
	  }
        }
    }
}

void SimpleRegistrationSfM::updateStructure( int iteration, std::vector<int> &index, Eigen::MatrixXd &x1d, Eigen::MatrixXd &x2d, 
				     cv::Mat &depth2 )
{
    // x1d and x2d are image points [3 x n]
    double variance_xyz = 0.2*0.2;
    printf( "Size x1 [%i x %i]\n", x1d.rows(), x1d.cols() );
    printf( "Size x2 [%i x %i]\n", x2d.rows(), x2d.cols() );
    for (int i = 0; i < index.size(); ++i)
    {
        Eigen::Vector3d pa, va_tmp, pv, vv_temp;
        
        // Point1: actual
        pa = Structure.col(index[i]);
        va_tmp = Covariance.col(index[i]);
        
        // Point2: viewed
        Eigen::Vector3d image_point2 = x2d.col(i);
        Eigen::Vector3d temp1 = projection<double,double,double>(image_point2, depth2, Calibration);
        if ( boundarykinect( temp1(2), 1.0 ) ) varianceKinect( temp1, Calibration, vv_temp);
        else
        {
	  Eigen::Vector3d image_point1 = x1d.col(i);
	  Eigen::Vector4d temp2 = linearTriangulation( image_point1, image_point2, Cameras_RCV->at(iteration - 1), Cameras_RCV->at(iteration) );
	  pv = temp2.head(3);
	  vv_temp = Eigen::Vector3d( variance_xyz, variance_xyz, variance_xyz );
        }
        
        LinearEstimator<double> blue( pa, va_tmp, pv, vv_temp, Rot, tr );
        blue.estimate();
        
        //Update
        Structure.col(index[i]) = blue.getPoint();
        Covariance.col(index[i]) = blue.getVariance();
    }
     
}