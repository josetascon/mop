// Created: Dec/09/2013
// Author: José David Tascón Vidarte

#include "SimpleRegistrationSfM.hpp"

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
    std::vector< cv::Point3d > pts2copy;
    std::vector< Eigen::Vector4d > common_st;
    
    initilizeStructure( visibility, coordinates, depth_list[0] );
    
    std::cout << "\n================================ POSE Estimation ==================================\n";
    for(int k = 0; k < (num_cameras - 1); k++) // pose 1-2. Solve pose for continous cameras
    {
        int cam1 = k;
        int cam2 = k+1;
        pts1.clear();
        pts2.clear();
        pts2copy.clear();
        common_st.clear();
        
        for (register int ft = 0; ft < num_features; ++ft)		// Select valid features for corresponding camera
        {
	  if ((*visibility)(cam1,ft) && (*visibility)(cam2,ft) )   // Temporal saving of image coordinates for mutual points
	  {
	      Eigen::Vector3d point1 = (*coordinates)(cam1,ft);
	      Eigen::Vector3d point2 = (*coordinates)(cam2,ft);
	      pts1.push_back( cv::Point2d(point1(0), point1(1)) );
	      pts2.push_back( cv::Point2d(point2(0), point2(1)) );
	      ft_number.push_back(ft);
	      
	      if ( Structure(2,ft) != 0 )
	      {
		pts2copy.push_back( cv::Point3d(point2(0), point2(1), 1.0) ); //Store data in case of sfm fallback
		common_st.push_back( Structure.col(ft) );
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
	  point3_vector2eigen( pts2copy, x2dd );
	  eigen_vector2eigen(common_st, X2temp);
	  P2 = linearCamera( x2dd, X2temp);
	  poseCameraMatrix3x4( P2, Calibration, Rot, tr );
	  /// UPDATE STRUCTURE AND COVARIANCE HERE
	  
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