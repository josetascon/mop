// Created: Dec/09/2013
// Author: José David Tascón Vidarte

#include "SimpleRegistration.hpp"

// ================================================================================================
// =========================== FUNCTIONS of CLASS SimpleRegistration ==============================
// ================================================================================================
void SimpleRegistration::initializeGlobalVariables()
{
     // initialization
    Cameras_RCV.resize(num_cameras);
    Rot_global.resize(num_cameras);
    Qn_global.resize(num_cameras);
    tr_global.resize(num_cameras);
}

void SimpleRegistration::initializeCoordinatesOrigin()
{
    Rot_global[0] = Eigen::Matrix3d::Identity();
    tr_global[0] = Eigen::Vector3d::Zero();
    Qn_global[0] = Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0); //save quaternion
    Cameras_RCV[0] = buildProjectionMatrix( Calibration, Rot_global[0], tr_global[0] );
}

void SimpleRegistration::updateGlobal(int iteration)
{
    // The updated vector element is k+1 OR (iteration + 1)
    register int k = iteration;
    Rot_global[k] = Rot*Rot_global[k-1]; // Rot;
    tr_global[k] = Rot*tr_global[k-1] + tr; // tr_global[k] = tr;
    Qn_global[k] = Eigen::Quaternion<double>(Rot_global[k]); //save quaternion
    Cameras_RCV[k] = buildProjectionMatrix( Calibration, Rot_global[k], tr_global[k] );    
}

void SimpleRegistration::printGlobal( int iteration )
{
    register int k = iteration;
    Eigen::Vector3d angles_vec1;
    
    // Debug Info:
    rotation2angles( Rot_global[k], angles_vec1);
    std::cout << "Rotation " <<  k << ":\n" << Rot_global[k] << "\n";
    std::cout << "Rotation angles " <<  k << ":\n" << angles_vec1.transpose() << "\n";
    std::cout << "Equivalent quaternion Cam " << k << ":\n" << Qn_global[k].w() << " " << Qn_global[k].vec().transpose() << '\n';
    std::cout << "Translation " <<  k << ":\n" << tr_global[k].transpose() << "\n";
    std::cout << "Camera Matrix Recover " <<  k << ":\n" << Cameras_RCV[k] << "\n\n";
}

void SimpleRegistration::solvePose(std::vector< MatchQuery > *globalMatch, std::vector< std::vector< cv::KeyPoint > > *set_of_keypoints, 
		std::vector< cv::Mat > *set_of_depth)
{
    initializeGlobalVariables();
    initializeCoordinatesOrigin();
    
    std::vector< cv::Point2d > pts1, pts2;
    std::vector< cv::Point3d > WP1, WP2;
    
    std::cout << "\n================================ POSE Estimation ==================================\n";
    for(register int k = 0; k < (num_cameras - 1); ++k) // pose 1-2
    {
        std::vector< cv::DMatch > match_c = (*globalMatch)[k].matches;
        int cam1 = (*globalMatch)[k].cam_id1;
        int cam2 = (*globalMatch)[k].cam_id2;
        extractPointsfromMatches( match_c, (*set_of_keypoints)[cam1], (*set_of_keypoints)[cam2], pts1, pts2);
        
        Pose3D pose( Calibration );
        pose.adaptPoints(pts1, pts2, (*set_of_depth)[cam1], (*set_of_depth)[cam2]);
        pose.solvePose();
        Rot = pose.getRotation();
        tr = pose.getTranslation();
        
        updateGlobal( k+1 );
        DEBUG_2( printGlobal( k+1 ); )
    }
    return;
}


void SimpleRegistration::solvePose(Eigen::Matrix<bool,-1,-1> *visibility, Eigen::Matrix<Eigen::Vector3d,-1,-1> *coordinates, 
		std::vector< std::string > &depth_list, bool optimal)
{
    // Initialization
    initializeGlobalVariables();
    initializeCoordinatesOrigin();
    
    std::vector<int> ft_number;
    std::vector< cv::Point2d > pts1, pts2;
    std::vector< cv::Point3d > WP1, WP2;
    
    std::cout << "\n================================ POSE Estimation ==================================\n";
    for(int k = 0; k < (num_cameras - 1); k++) // pose 1-2. Solve pose for continous cameras
    {
        int cam1 = k;
        int cam2 = k+1;
        pts1.clear();
        pts2.clear();
        
        for (register int ft = 0; ft < num_features; ++ft)		// Select valid features for corresponding camera
        {
	  if ((*visibility)(cam1,ft) && (*visibility)(cam2,ft) )
	  {
	      Eigen::Vector3d point1 = (*coordinates)(cam1,ft);
	      Eigen::Vector3d point2 = (*coordinates)(cam2,ft);
	      pts1.push_back( cv::Point2d(point1(0), point1(1)) );
	      pts2.push_back( cv::Point2d(point2(0), point2(1)) );
	      ft_number.push_back(ft);
	  }
        }
        // Debug
        DEBUG_1( printf("POSE: %04i -> %04i:\n", cam1, cam2); )
        DEBUG_1( std::cout << "Total Pts: " << pts1.size() << "\n"; )
        
        cv::Mat depth1 = cv::imread( depth_list[cam1], -1 );
        cv::Mat depth2 = cv::imread( depth_list[cam2], -1 );
        
        Pose3D pose( Calibration );
        pose.adaptPoints(pts1, pts2, depth1, depth2);
        pose.solvePose(optimal);
        Rot = pose.getRotation();
        tr = pose.getTranslation();
        
        Xmodel.push_back( pose.getPointsX1() );
        Xmodel.push_back( pose.getPointsX2() );
        Variance.push_back( pose.getVariance1() );
        Variance.push_back( pose.getVariance2() );
        
        updateGlobal( k+1 );
        DEBUG_2( printGlobal( k+1 ); )
        
        // TEST print temporal pair of files
// 	  X1.transposeInPlace();
// 	  X2.transposeInPlace();
// 	  char buf[256];
// 	  sprintf(buf, "data_%0.4d_1", k);
// 	  writeTextFileEigen(&buf[0], X1);
// 	  sprintf(buf, "data_%0.4d_2", k);
// 	  writeTextFileEigen(&buf[0], X2);
// 	  X1.transposeInPlace();
// 	  X2.transposeInPlace();
        // END TEST
    }
}

void SimpleRegistration::solvePose(Eigen::Matrix<bool,-1,-1> *visibility, Eigen::Matrix<Eigen::Vector4d,-1,-1> *coordinates, bool optimal)
{
    // initialization
    initializeGlobalVariables();
    initializeCoordinatesOrigin();
    
    Eigen::MatrixXd Xtmp1, Xtmp2;
    
    DEBUG_1( std::cout << "\n================================ POSE Estimation ==================================\n"; )
    for(int k = 0; k < (num_cameras - 1); k++) // pose 1-2
    {
        int cam1 = k;
        int cam2 = k+1;
        int count_ft = 0;
        Xtmp1 = Xtmp2 = Eigen::MatrixXd::Zero(4,num_features);
        
        for (register int ft = 0; ft < num_features; ++ft)
        {
	  if ((*visibility)(cam1,ft) && (*visibility)(cam2,ft) )
	  {
	      Eigen::Vector4d point1 = (*coordinates)(cam1,ft);
	      Eigen::Vector4d point2 = (*coordinates)(cam2,ft);
	      Xtmp1.col(count_ft) = point1;
	      Xtmp2.col(count_ft) = point2;
	      count_ft++;
	  }
        }
        // Debug
        DEBUG_1( printf("POSE: %04i -> %04i:\n", cam1, cam2); )
        DEBUG_1( std::cout << "Total Pts: " << count_ft << "\n"; )
        X1 = Xtmp1.block(0,0,3,count_ft);
        X2 = Xtmp2.block(0,0,3,count_ft);
        
        Pose3D pose( X1, X2, Calibration );
        pose.solvePose(optimal);
        Rot = pose.getRotation();
        tr = pose.getTranslation();

        Xmodel.push_back( pose.getPointsX1() );
        Xmodel.push_back( pose.getPointsX2() );
        Variance.push_back( pose.getVariance1() );
        Variance.push_back( pose.getVariance2() );
        
        updateGlobal( k+1 );
        DEBUG_2( printGlobal( k+1 ); )
    }
}

// Update camera with new quaternion values (after global optimization like BA)
void SimpleRegistration::updateQuaternion()
{
    for (register int cam = 0; cam < num_cameras; cam++)
    {
        Qn_global[cam].normalize();
        Rot_global[cam] = Qn_global[cam].toRotationMatrix();
        Cameras_RCV[cam] = buildProjectionMatrix( Calibration, Rot_global[cam], tr_global[cam] );
        DEBUG_2( printGlobal( cam ); )
    }
}