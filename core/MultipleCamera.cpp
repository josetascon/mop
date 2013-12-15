// Created: Jul/16/2013
// Author: José David Tascón Vidarte

#include "MultipleCamera.hpp"

// ================================================================================================
// =================================== FUNCTIONS of CLASS SfM =====================================
// ================================================================================================
// struct MatchQuery
// {
//     std::vector< cv::DMatch > matches;
//     int cam_id1;
//     int cam_id2;
// };
void SfM::solvePose(std::vector< MatchQuery > *globalMatch, std::vector< std::vector< cv::KeyPoint > > *set_of_keypoints)
{
    // initialization
    int num_matches_pose = (num_cameras - 1) + (num_cameras - 2); //matches used in pose 1-2 and 1-3
    Fundamental.resize(num_matches_pose);
    Quat_relative.resize(num_cameras);
    tr_relative.resize(num_cameras);
    Cameras_RCV.resize(num_cameras);
    Qn_global.resize(num_cameras);
    Rot_global.resize(num_cameras);
    tr_global.resize(num_cameras);
    
//     t12_RCV.resize(num_cameras-1);
//     t13_RCV.resize(num_cameras-2);
    
    Quat_relative[0] = Qn_global[0] = Rot_global[0] = Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0);
    tr_relative[0] = tr_global[0] = Eigen::Vector3d::Zero();
    Cameras_RCV[0] = buildProjectionMatrix( Calibration, Rot_global[0], tr_global[0] );
    
    std::vector< cv::DMatch > match_c;
    Eigen::Matrix3d Rot_chosen;
    Eigen::Vector3d tr_chosen, angles_vec1;
    std::cout << "\n================================ POSE Estimation ==================================\n";
    for(int k=0; k < (num_cameras - 1); k++) // pose 1-2
    {
        match_c = (*globalMatch)[k].matches;
        int cam1 = (*globalMatch)[k].cam_id1;
        int cam2 = (*globalMatch)[k].cam_id2;
        printf("POSE: %04i -> %04i:\n", cam1, cam2);
        findCameraExtrinsicsAdapter( (*set_of_keypoints)[cam1], (*set_of_keypoints)[cam2], match_c,
			Calibration, Fundamental[k], Rot_chosen, tr_chosen );
        Quat_relative[k+1] = Eigen::Quaternion<double>(Rot_chosen);
        tr_relative[k+1] = tr_chosen;
        // comment this in order to use scalefromTranslations
        Rot_global[k+1] = Rot_chosen*Rot_global[k];
        Qn_global[k+1] = Quat_relative[k+1]*Qn_global[k];
        tr_global[k+1] = Rot_chosen*tr_global[k] + tr_chosen;
        Cameras_RCV[k+1] = buildProjectionMatrix( Calibration, Rot_global[k+1], tr_global[k+1] );
        //Debug
//         std::cout << "Camera Matrix Recover " <<  cam2 << ":\n" << Cameras_RCV[cam2] << "\n\n";
    }
//     for(int k=0; k < (num_cameras - 2); k++) // pose 1-3
//     {
//         int kk = k + num_cameras - 1;
//         match_c = (*globalMatch)[kk].matches;
//         int cam1 = (*globalMatch)[kk].cam_id1;
//         int cam2 = (*globalMatch)[kk].cam_id2;
//         printf("POSE: %04i -> %04i:\n", cam1, cam2);
//         findCameraExtrinsicsAdapter( (*set_of_keypoints)[cam1], (*set_of_keypoints)[cam2], match_c, 
// 			Calibration, Fundamental[kk], Rot_chosen, tr_chosen );
//         t13_RCV[k] = tr_chosen;
//     }
//     scalefromTranslations( Rot_global, t12_RCV, t13_RCV, tr_relative ); ///orig scale (not working properly)
    return;
}

void SfM::solvePose(Eigen::Matrix<bool,-1,-1> *visibility,
			Eigen::Matrix<Eigen::Vector3d,-1,-1> *coordinates)
{
    // initialization
    Fundamental.resize(num_cameras - 1);
    Quat_relative.resize(num_cameras);
    tr_relative.resize(num_cameras);
    Cameras_RCV.resize(num_cameras);
    Qn_global.resize(num_cameras);
    Rot_global.resize(num_cameras);
    tr_global.resize(num_cameras);
    
    Quat_relative[0] = Qn_global[0] = Rot_global[0] = Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0);
    tr_relative[0] = tr_global[0] = Eigen::Vector3d::Zero();
    Cameras_RCV[0] = buildProjectionMatrix( Calibration, Rot_global[0], tr_global[0] );
    
    std::vector< cv::Point2d > pts1, pts2;
    Eigen::Matrix3d Rot_chosen;
    Eigen::Vector3d tr_chosen, angles_vec1;
    std::cout << "\n================================ POSE Estimation ==================================\n";
    for(int k=0; k < (num_cameras - 1); k++) // pose 1-2
    {
        int cam1 = k;
        int cam2 = k+1;
        pts1.clear();
        pts2.clear();
        
        for (int ft = 0; ft < num_features; ft++)
        {
	  if ((*visibility)(cam1,ft) && (*visibility)(cam2,ft) )
	  {
	      Eigen::Vector3d point1 = (*coordinates)(cam1,ft);
	      Eigen::Vector3d point2 = (*coordinates)(cam2,ft);
// 	      std::cout << "Point1: " << point1.transpose() << "\n";
// 	      std::cout << "Point2: " << point2.transpose() << "\n";
	      pts1.push_back( cv::Point2d(point1(0), point1(1)) );
	      pts2.push_back( cv::Point2d(point2(0), point2(1)) );
	  }
        }
        printf("POSE: %04i -> %04i:\n", cam1, cam2);
        findCameraExtrinsics(pts1, pts2, Calibration, Fundamental[k], Rot_chosen, tr_chosen);
        Quat_relative[k+1] = Eigen::Quaternion<double>(Rot_chosen);
        tr_relative[k+1] = tr_chosen;
        
        Rot_global[k+1] = Rot_chosen*Rot_global[k];
        Qn_global[k+1] = Quat_relative[k+1]*Qn_global[k];
        tr_global[k+1] = Rot_chosen*tr_global[k] + tr_chosen;
        Cameras_RCV[k+1] = buildProjectionMatrix( Calibration, Rot_global[k+1], tr_global[k+1] );
        //Debug
//         rotation2angles(Rot_global[cam2], angles_vec1);
//         std::cout << "Rotation " <<  cam2 << ":\n" << Rot_global[cam2] << "\n";
//         std::cout << "Rotation angles " <<  cam2 << ":\n" << angles_vec1.transpose() << "\n";
//         std::cout << "Equivalent quaternion Cam " << cam2 << ":\n" << Quat_relative[cam2].w() << " " << Quat_relative[cam2].vec().transpose() << '\n';
//         std::cout << "Translation " <<  cam2 << ":\n" << tr_relative[cam2].transpose() << "\n";
        std::cout << "Camera Matrix Recover " <<  cam2 << ":\n" << Cameras_RCV[cam2] << "\n\n";
    }
    return;
}

void SfM::solveStructure(Eigen::Matrix<bool,-1,-1> *visibility,
			Eigen::Matrix<Eigen::Vector3d,-1,-1> *coordinates )
{
    //[camera x features] = [i x j]
    std::vector<Eigen::Vector3d> xdata;
    std::vector<Eigen::MatrixXd> PP;
    //Structure.resize(4,visibility.cols());
    Structure = Eigen::MatrixXd::Zero(4,visibility->cols());
    plotSt = Eigen::VectorXi::Zero(visibility->cols());//plot code TESTING
    bool first = true; int first_cam=-1;//plot code TESTING
    for (int j = 0; j < visibility->cols(); j++)
    {
        xdata.clear();
        PP.clear();
        first = true;//plot code TESTING
        first_cam = -1;//plot code TESTING
        for (int i = 0; i < visibility->rows(); i++)
        {
	  if ((*visibility)(i,j))
	  {
// 	      printf("Feature %i in camera %i, ",j,i);
// 	      std::cout << "Coordinates:\n" << (*coordinates)(i,j) << '\n';
	      xdata.push_back((*coordinates)(i,j));
	      PP.push_back(Cameras_RCV[i]);
	      
	      //plot code TESTING
	      if (first)
	      {
		first_cam = i;
		first = false;
	      }
	  }
        }
        if (PP.size() > 1) Structure.col(j) = linearTriangulation( xdata, PP );
        plotSt(j) = first_cam;//plot code TESTING
    }
    
    return;
}

void SfM::updateCamera()
{
    Eigen::Vector3d angles_vec1;
    for (int cam = 0; cam < num_cameras; cam++)
    {
        Qn_global[cam].normalize();
        Rot_global[cam] = Qn_global[cam].toRotationMatrix();
        Cameras_RCV[cam] = buildProjectionMatrix( Calibration, Rot_global[cam], tr_global[cam] );
        rotation2angles(Rot_global[cam], angles_vec1);
        std::cout << "Rotation " <<  cam << ":\n" << Rot_global[cam] << "\n";
        std::cout << "Rotation angles " <<  cam << ":\n" << angles_vec1.transpose() << "\n";
        std::cout << "Equivalent quaternion Cam " << cam << ":\n" << Qn_global[cam].w() << " " << Qn_global[cam].vec().transpose() << '\n';
        std::cout << "Translation " <<  cam << ":\n" << tr_global[cam].transpose() << "\n";
        std::cout << "Camera Matrix Recover " <<  cam << ":\n" << Cameras_RCV[cam] << "\n\n";
    } 
}


// ================================================================================================
// ============================== FUNCTIONS of CLASS IncrementalBA ================================
// ================================================================================================
void IncrementalBA::runC()
{
    int num_cams = visibility->rows();
    int num_features = visibility->cols();
    structure = Eigen::MatrixXd::Zero(4,visibility->cols());
    
    Eigen::Matrix3d K, F;
    K << (*intrinsics)[0], 0.0, (*intrinsics)[2], 0.0, (*intrinsics)[1], (*intrinsics)[3], 0.0, 0.0, 1.0;
    
    // ========================================== Two View Initialization ==========================================
    Eigen::MatrixXd x1data, x2data, x1hom, x2hom;
    std::vector< cv::Point2d > pts1, pts2;
    Eigen::Matrix3d Rot;
    Eigen::Vector3d tr, angles_vec1;
    
    int cam1 = 0;
    int cam2 = 1;
    std::vector<int> index;
    std::cout << "Actual camera Iteration = " << cam1 << "-" << cam2 << "/" << num_cams-1 << "\n";
    for (register int ft = 0; ft < num_features; ++ft)
    {
        if ( (*visibility)(cam1,ft) && (*visibility)(cam2,ft) )
        {
	  Eigen::Vector3d point1 = (*coordinates)(cam1,ft);
	  Eigen::Vector3d point2 = (*coordinates)(cam2,ft);
	  pts1.push_back( cv::Point2d(point1(0), point1(1)) );
	  pts2.push_back( cv::Point2d(point2(0), point2(1)) );
	  index.push_back(ft);
        }
    }
//     printf("POSE: %04i -> %04i:\n", cam1, cam2);
    findCameraExtrinsics(pts1, pts2, K, F, Rot, tr);
    point_vector2eigen( pts1, x1data );
    point_vector2eigen( pts2, x2data );
    homogeneous( x1data, x1hom );
    homogeneous( x2data, x2hom );
    
    Eigen::MatrixXd P1(3,4);
    Eigen::MatrixXd P2(3,4);
    P1 << K, Eigen::Vector3d::Zero(3);
    P2 << buildProjectionMatrix( K, Rot, tr);
    Eigen::MatrixXd Xtemp = linearTriangulationNormalized( x1hom, x2hom, P1, P2);
//     structure.block(0,0,4,Xtemp.cols()) = Xtemp;
    
    // Initial optimization for two views
    LocalOptimizer opt01;
    opt01.setParameters3Dto2D( &x1data, &x2data, &Rot, &tr, &Xtemp );
    opt01.setIntrinsics( intrinsics );
    opt01.setDistortion( distortion );
    opt01.pose3Dto2D();
    
    for (register int i = 0; i < index.size(); ++i) structure.col(index[i]) = Xtemp.col(i);
    
    quaternion.push_back(Eigen::Quaternion<double>::Identity());
    quaternion.push_back(Eigen::Quaternion<double>(Rot));
    
    translation.push_back(Eigen::Vector3d::Zero(3));
    translation.push_back(tr);
    
    Camera.push_back( P1 );
    Camera.push_back( buildProjectionMatrix(K, Rot, tr) );
    
    
    // ========================================== END Two View Initialization ==========================================
    
    
    // ========================================== Adding new cameras ==========================================
    for (int cam2 = 2; cam2 < num_cams; cam2++)
    {
        int cam0 = cam2-2;
        int cam1 = cam2-1;
        index.clear();
        std::vector< cv::Point3d > pts3;
        std::vector< Eigen::Vector4d > common_st;
        std::vector< Eigen::Vector3d > xx1, xx2;
        for (register int ft = 0; ft < num_features; ++ft)
        {
	  if ( (*visibility)(cam2,ft) && (*visibility)(cam1,ft) && (*visibility)(cam0,ft) ) // Check correspondences in three views
// 	  if ( (*visibility)(cam2,ft) && (structure(3,ft) == 1) ) // from available points
	  {
	      Eigen::Vector3d point1 = (*coordinates)(cam2,ft);
	      pts3.push_back( cv::Point3d(point1(0), point1(1), 1.0) ); 	// improve this convertVector3EtoEigen (like convertVector4EtoEigen)
	      common_st.push_back( structure.col(ft) );			// Save viewable structure and 2d observation in cam2 to find a initial camera
	  }
	  if( (*visibility)(cam2,ft) && (*visibility)(cam1,ft) && (structure(3,ft) == 0) ) //Initialize new structure added
	  {
	      xx1.push_back( (*coordinates)(cam1,ft) );
	      xx2.push_back( (*coordinates)(cam2,ft) );
	      index.push_back(ft);
	  }
        }
        
        // Finding new camera from 3D point to 2D relation
        point3_vector2eigen( pts3, x1data );
        eigen_vector2eigen(common_st, Xtemp);
        P1 = Camera[cam1];
        P2 = linearCamera( x1data, Xtemp); //Actual camera from 3D point to 2D relation
        //Debug
        std::cout << "Actual camera Iteration = " << cam2 << "/" << num_cams-1 << "\n";
        std::cout << "Pts to calc new camera = " << pts3.size() << "\n";
//         std::cout << "common_st\n" << common_st.size() << "\n";
//         std::cout << "x1data size\n" << x1data.cols() << "\n";
//         std::cout << "Xtemp size\n" << Xtemp.cols() << "\n";
//         std::cout << "x1data \n" << x1data.transpose() << "\n";
//         std::cout << "structure temp\n" << Xtemp.transpose() << "\n";
//         std::cout << "structure\n" << structure.transpose() << "\n";

        // Initialize new camera, solve R and t
        P2 = K.inverse()*P2;
        Eigen::Matrix3d RR;
        RR << P2(0,0), P2(0,1), P2(0,2), P2(1,0), P2(1,1), P2(1,2), P2(2,0), P2(2,1), P2(2,2);
        
        Eigen::Vector3d tt;
        tt = P2.col(3);
        double sd = sign(RR.determinant());
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(RR, Eigen::ComputeThinU | Eigen::ComputeThinV); // Normalize with the greatest singular value
        double nv = 1/((svd.singularValues())(0));
        tt = sd*nv*tt; 				// Normalization is also applied to 
        RR = sd*nv*RR; 				// Rotation matrix must have 3 singular values = to 1, in order to achieve a det = 1;
        rotation2angles(RR, angles_vec1); 	// Find Euler angles
        Eigen::Matrix3d R_angle; 			// Do not use direct conversion from RR to Quaternion because the det of RR at this point is not 1
        R_angle = Eigen::AngleAxisd(angles_vec1(0)*pi/180, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(angles_vec1(1)*pi/180,  Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(angles_vec1(2)*pi/180, Eigen::Vector3d::UnitZ());
        //Debug
//         std::cout << "nv = " << nv << "\n";
//         std::cout << "sd = " << sd << "\n";
//         std::cout << "P3\n" << P2 << "\n";
//         std::cout << "Rot\n" << RR << "\n";
        std::cout << "Rotation angles:\n" << angles_vec1.transpose() << "\n";
        std::cout << "translation:\n" << tt.transpose() << "\n";

        quaternion.push_back(Eigen::Quaternion<double>(R_angle));
        translation.push_back(tt);
        
        // Optimization
        int max_feature = 0;
        for (register int i = 0; i < visibility->cols(); ++i) if ((*visibility)(cam2,i)) max_feature = i;
        Eigen::Matrix<bool,-1,-1> view = visibility->block(0,0,cam2+1,max_feature); 		// Reduce the matrix to search
        PartialIncremental opt02( &view, coordinates, &quaternion, &translation, &structure, intrinsics, distortion  );
        opt02.run(cam2,cam2+1);
        RR = quaternion[cam2].toRotationMatrix();
        Camera.push_back( buildProjectionMatrix(K, RR, translation[cam2]) );
        
        //Initialize additional 3D points. This is at the end to allow the camera to be optimized
        P2 = Camera[cam2];
        for (register int i = 0; i < index.size(); ++i)  structure.col(index[i]) = linearTriangulation( xx1[i], xx2[i], P1, P2 );
//         std::cout << "End = " << cam2 << "\n\n";
    }
}

void IncrementalBA::runF()
{
    int num_cams = visibility->rows();
    int num_features = visibility->cols();
    structure = Eigen::MatrixXd::Zero(4,visibility->cols());
    
    Eigen::Matrix3d K, F;
    K << (*intrinsics)[0], 0.0, (*intrinsics)[2], 0.0, (*intrinsics)[1], (*intrinsics)[3], 0.0, 0.0, 1.0;
    
    // ========================================== Two View Initialization ==========================================
    Eigen::MatrixXd x1data, x2data, x1hom, x2hom;
    std::vector< cv::Point2d > pts1, pts2;
    Eigen::Matrix3d Rot;
    Eigen::Vector3d tr, angles_vec1;
    
    int cam1 = 0;
    int cam2 = 1;
    std::vector<int> index;
    std::cout << "Actual camera Iteration = " << cam1 << "-" << cam2 << "/" << num_cams-1 << "\n";
    for (register int ft = 0; ft < num_features; ++ft)
    {
        if ( (*visibility)(cam1,ft) && (*visibility)(cam2,ft) )
        {
	  Eigen::Vector3d point1 = (*coordinates)(cam1,ft);
	  Eigen::Vector3d point2 = (*coordinates)(cam2,ft);
	  pts1.push_back( cv::Point2d(point1(0), point1(1)) );
	  pts2.push_back( cv::Point2d(point2(0), point2(1)) );
	  index.push_back(ft);
        }
    }
//     printf("POSE: %04i -> %04i:\n", cam1, cam2);
    findCameraExtrinsics(pts1, pts2, K, F, Rot, tr);
    point_vector2eigen( pts1, x1data );
    point_vector2eigen( pts2, x2data );
    homogeneous( x1data, x1hom );
    homogeneous( x2data, x2hom );
    
    Eigen::MatrixXd P1(3,4);
    Eigen::MatrixXd P2(3,4);
    P1 << K, Eigen::Vector3d::Zero(3);
    P2 << buildProjectionMatrix( K, Rot, tr);
    Eigen::MatrixXd Xtemp = linearTriangulationNormalized( x1hom, x2hom, P1, P2);
//     structure.block(0,0,4,Xtemp.cols()) = Xtemp;
    
    // Initial optimization for two views
    LocalOptimizer opt01;
    opt01.setParameters3Dto2D( &x1data, &x2data, &Rot, &tr, &Xtemp );
    opt01.setIntrinsics( intrinsics );
    opt01.setDistortion( distortion );
    opt01.pose3Dto2D();
    
    for (register int i = 0; i < index.size(); ++i) structure.col(index[i]) = Xtemp.col(i);
    
    quaternion.push_back(Eigen::Quaternion<double>::Identity());
    quaternion.push_back(Eigen::Quaternion<double>(Rot));
    
    translation.push_back(Eigen::Vector3d::Zero(3));
    translation.push_back(tr);
    
    Camera.push_back( P1 );
    Camera.push_back( buildProjectionMatrix(K, Rot, tr) );
    
    
    // ========================================== END Two View Initialization ==========================================
    
    
    // ========================================== Adding new cameras ==========================================
    for (int cam2 = 2; cam2 < num_cams; cam2++)
    {
        int cam1 = cam2-1;
        index.clear();
        pts1.clear();
        pts2.clear();
        std::vector< Eigen::Vector3d > xx1, xx2;
        for (register int ft = 0; ft < num_features; ++ft)
        {
	  if ( (*visibility)(cam2,ft) && (*visibility)(cam1,ft) ) // Check correspondences in two views
	  {
	      Eigen::Vector3d point1 = (*coordinates)(cam1,ft);
	      Eigen::Vector3d point2 = (*coordinates)(cam2,ft);
	      pts1.push_back( cv::Point2d(point1(0), point1(1)) );
	      pts2.push_back( cv::Point2d(point2(0), point2(1)) );
	      if ( (structure(3,ft) == 0) ) //Initialize new structure added
	      {
		xx1.push_back( (*coordinates)(cam1,ft) );
		xx2.push_back( (*coordinates)(cam2,ft) );
		index.push_back(ft);
	      }
	  }
        }
        std::cout << "Actual camera Iteration = " << cam2 << "/" << num_cams-1 << "\n";
        // Finding new camera from Fundamental
        findCameraExtrinsics(pts1, pts2, K, F, Rot, tr);
        tr = Rot*translation[cam1] + tr;
        Eigen::Matrix3d Rot_cum = quaternion[cam1].toRotationMatrix();
        Rot = Rot*Rot_cum;
        P1 = Camera[cam1];
        P2 = buildProjectionMatrix(K, Rot, tr);
        quaternion.push_back(Eigen::Quaternion<double>(Rot));
        translation.push_back(tr);
        
        // Optimization
        int max_feature = 0;
        for (register int i = 0; i < visibility->cols(); ++i) if ((*visibility)(cam2,i)) max_feature = i;
        Eigen::Matrix<bool,-1,-1> view = visibility->block(0,0,cam2+1,max_feature); 		// Reduce the matrix to search
        PartialIncremental opt02( &view, coordinates, &quaternion, &translation, &structure, intrinsics, distortion  );
        opt02.run(cam2,cam2+1);
        Eigen::Matrix3d RR = quaternion[cam2].toRotationMatrix();
        Camera.push_back( buildProjectionMatrix(K, RR, translation[cam2]) );
        
        //Initialize additional 3D points
        for (register int i = 0; i < index.size(); ++i)  structure.col(index[i]) = linearTriangulation( xx1[i], xx2[i], P1, P2 );
    }
}

void IncrementalBA::updateCamera()
{
    Eigen::Matrix3d K;
    K << (*intrinsics)[0], 0.0, (*intrinsics)[2], 0.0, (*intrinsics)[1], (*intrinsics)[3], 0.0, 0.0, 1.0;
    for (int cam = 0; cam < quaternion.size(); cam++)
    {
        quaternion[cam].normalize();
        Eigen::Matrix3d Rot = quaternion[cam].toRotationMatrix();
        Camera[cam] = buildProjectionMatrix( K, Rot, translation[cam] );
    }
}


// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================
void findCameraExtrinsicsAdapter( std::vector< cv::KeyPoint > &keypoints1, std::vector< cv::KeyPoint > &keypoints2, 
		       std::vector< cv::DMatch > &matches, Eigen::Matrix3d &calib, 
		       Eigen::Matrix3d &Fundamental, Eigen::Matrix3d &Rot, Eigen::Vector3d &tr )
{

    std::vector< cv::Point2d > pts1, pts2;
    
    extractPointsfromMatches( matches, keypoints1, keypoints2, pts1, pts2);
    //Debug
//     for(int i=0; i < pts1.size(); i++)
//     {
//         std::cout << "Point of image1, " << i << ": " << pts1[i];
//         std::cout << "\t||\tPoint of image2, " << i << ": " << pts2[i] << '\n';
//     }
    findCameraExtrinsics( pts1, pts2, calib, Fundamental, Rot, tr );
}
    
void findCameraExtrinsics(std::vector< cv::Point2d > &pts1, std::vector< cv::Point2d > &pts2, Eigen::Matrix3d &calib, 
		       Eigen::Matrix3d &Fundamental, Eigen::Matrix3d &Rot, Eigen::Vector3d &tr )
{
    cv::Mat Fund;
    Eigen::MatrixXd x1data, x2data, x1hom, x2hom;
    Eigen::MatrixXd X3r1t1, X3r1t2, X3r2t1, X3r2t2;
    Eigen::MatrixXd P1(3,4), P2r1t1(3,4), P2r1t2(3,4), P2r2t1(3,4), P2r2t2(3,4);
    Eigen::Matrix3d FundE, Essential;
    Eigen::Matrix3d Rot1, Rot2;
    Eigen::Vector3d tr1, tr2, angles_vec1, angles_vec2;

    // ========================================== Fundamental & Essential Matrix ==========================================
    Fund = findFundamentalMat(pts1, pts2, CV_FM_8POINT); // eight point alg here because there is no outliers from FeaturesEDM
//     getFundamentalandRemoveOutliers(pts1, pts2, matches, Fund);
    cv2eigen( Fund, FundE );
    Fundamental = FundE;
    essentialfromFundamental(FundE, calib, Essential);
    posefromEssential( Essential, Rot1, Rot2, tr1, tr2 );
//     std::cout << "Essential Matrix:\n" << Essential << '\n';
    if (determinantCorrection(Rot1))
    {
        Essential = -Essential;
        posefromEssential( Essential, Rot1, Rot2, tr1, tr2 );
//         std::cout << "Recalculating Essential Matrix:\n" << Essential << '\n';
    }
    checkCoherentRotation(Rot1);
    rotation2angles( Rot1, angles_vec1);
    rotation2angles( Rot2, angles_vec2);
    
//     std::cout << "Rotation Matrix Rot1(W):\n " << Rot1 <<'\n';
//     std::cout << "Rotation Matrix Rot2(WT):\n " << Rot2 <<'\n';
//     std::cout << "Rotation Value (W):\n " << angles_vec1.transpose() <<'\n';
//     std::cout << "Rotation Value (WT):\n " << angles_vec2.transpose() <<'\n';
//     std::cout << "Translation Value (+u):\n" << tr1.transpose() <<'\n';
//     std::cout << "Translation Value (-u):\n" << tr2.transpose() <<'\n';
    
    // ============================== Selection of Rotation Matrix & translation Vector ===================================
    // Rotation selection as a constrained version due to little motion
    Rot = angles_vec1.norm() < angles_vec2.norm()? Rot1 : Rot2; // orig < 
    rotation2angles( Rot, angles_vec1);
    
    point_vector2eigen( pts1, x1data );
    point_vector2eigen( pts2, x2data );
    
    homogeneous(x1data,x1hom);
    homogeneous(x2data,x2hom);
    
    P1 << calib, Eigen::Vector3d::Zero(3);
    P2r1t1 << buildProjectionMatrix(calib, Rot, tr1);
    P2r1t2 << buildProjectionMatrix(calib, Rot, tr2);
    
    X3r1t1 = linearTriangulationNormalized( x1hom, x2hom, P1, P2r1t1);
    X3r1t2 = linearTriangulationNormalized( x1hom, x2hom, P1, P2r1t2);
//     std::cout << "Triangulation (+u):\n" << X3r1t1.transpose() <<'\n';
//     std::cout << "Triangulation (-u):\n" << X3r1t2.transpose() <<'\n';
    // rigth hand >
    tr = countPositivesInRow(X3r1t1, 2) > countPositivesInRow(X3r1t2, 2)? tr1 : tr2;  // orig  SS > SS ? tr1 : tr2
    std::cout << "Selected Rotation Matrix:\n " << Rot <<"\n";
    std::cout << "Rotation Angles:\n " << angles_vec1.transpose() <<"\n";
    std::cout << "Selected translation vector:\n " << tr.transpose() <<"\n\n";
    /*    
    TESTING optimization
    Eigen::MatrixXd P2(3,4);
    P2 << buildProjectionMatrix( calib, Rot, tr);
    
    Eigen::MatrixXd X3 = linearTriangulationNormalized( x1hom, x2hom, P1, P2);
    double intrinsics[4] = { calib(0,0), calib(1,1), calib(0,2), calib(1,2) };
    std::vector< double > intrinsics_param(&intrinsics[0], &intrinsics[4]);
    double distcoeff[5] = {2.5552679187075661e-01, -5.8740292343503686e-01, -3.0863014649845459e-04, 1.9066445284294834e-03, 5.1108649981093257e-01};
    std::vector< double > coefficients(&distcoeff[0], &distcoeff[5]);
    
    LocalOptimizer opt01;
    opt01.setParameters3Dto2D( &x1data, &x2data, &Rot, &tr, &X3 );
    opt01.setIntrinsics( &intrinsics_param );
    opt01.setDistortion( &coefficients );
    opt01.pose3Dto2D();

    rotation2angles( Rot, angles_vec1);
    std::cout << "Selected Rotation Matrix:\n " << Rot <<"\n";
    std::cout << "Rotation Angles:\n " << angles_vec1.transpose() <<"\n";
    std::cout << "Selected translation vector:\n " << tr.transpose() <<"\n\n";
    END TESTING optimization ***NOTE: SUCCESS :D
    */
    /*
    // Rotation selection check all
    point_vector2eigen( pts1, x1data );
    point_vector2eigen( pts2, x2data );
    
    homogeneous(x1data,x1hom);
    homogeneous(x2data,x2hom);
    
    P1 << calib, Vector3d::Zero(3);
    P2r1t1 << buildProjectionMatrix(calib, Rot1, tr1);
    P2r1t2 << buildProjectionMatrix(calib, Rot1, tr2);
    P2r2t1 << buildProjectionMatrix(calib, Rot2, tr1);
    P2r2t2 << buildProjectionMatrix(calib, Rot2, tr2);
    
    X3r1t1 = linearTriangulationNormalized( x1hom, x2hom, P1, P2r1t1);
    X3r1t2 = linearTriangulationNormalized( x1hom, x2hom, P1, P2r1t2);
    X3r2t1 = linearTriangulationNormalized( x1hom, x2hom, P1, P2r2t1);
    X3r2t2 = linearTriangulationNormalized( x1hom, x2hom, P1, P2r2t2);
//     std::cout << "Triangulation (R1,+u):\n" << X3r1t1.transpose() <<'\n';
//     std::cout << "Triangulation (R1,-u):\n" << X3r1t2.transpose() <<'\n';
//     std::cout << "Triangulation (R2,+u):\n" << X3r2t1.transpose() <<'\n';
//     std::cout << "Triangulation (R2,-u):\n" << X3r2t2.transpose() <<'\n';
    Eigen::Vector4i countsPos( countPositivesInRow(X3r1t1, 2), countPositivesInRow(X3r1t2, 2), 
			  countPositivesInRow(X3r2t1, 2), countPositivesInRow(X3r2t2, 2) );
    Vector4i::Index maxPos;
    countsPos.maxCoeff(&maxPos);
    std::cout << "Vector with count:\n" << countsPos << "\n";
    std::cout << "Max index = " << maxPos << "\n";
    
    switch (maxPos)
    {
        case 0:
	  Rot = Rot1;
	  tr = tr1;
	  break;
        case 1:
	  Rot = Rot1;
	  tr = tr2;
	  break;
        case 2:
	  Rot = Rot2;
	  tr = tr1;
	  break;
        case 3:
	  Rot = Rot2;
	  tr = tr2;
	  break;
    }
    rotation2angles( Rot, angles_vec1);
    std::cout << "Selected Rotation Matrix:\n " << Rot <<"\n";
    std::cout << "Rotation Angles:\n " << angles_vec1.transpose() <<"\n";
    std::cout << "Selected translation vector:\n " << tr.transpose() <<"\n\n";
    */
}

void triangulateMultipleCamera( Eigen::Matrix<bool,-1,-1> &visibility, 
			  Eigen::Matrix<Eigen::Vector3d,-1,-1> &coordinates,
			  std::vector<Eigen::MatrixXd> &Cameras,
			  Eigen::MatrixXd &Structure)
{
    //[camera x features] = [i x j]
    std::vector<Eigen::Vector3d> xdata;
    std::vector<Eigen::MatrixXd> PP;
    //Structure.resize(4,visibility.cols());
    Structure = Eigen::MatrixXd::Zero(4,visibility.cols());
    
    for (int j = 0; j < visibility.cols(); j++)
    {
        xdata.clear();
        PP.clear();
        for (int i = 0; i < visibility.rows(); i++)
        {
	  if (visibility(i,j) == 1)
	  {
// 	      printf("Feature %i in camera %i, ",j,i);
// 	      std::cout << "Coordinates:\n" << coordinates(i,j) << '\n';
	      xdata.push_back(coordinates(i,j));
	      PP.push_back(Cameras[i]);
	  }
        }
        if (PP.size() > 1) Structure.col(j) = linearTriangulation( xdata, PP );
    }
    //Eigen::Vector4d linearTriangulation( std::vector<Eigen::Vector3d> &xdata, std::vector<Eigen::MatrixXd> &P )
}

void solveStructureInitial(const Eigen::Matrix<bool,-1,-1> &visibility,
		        const Eigen::Matrix<Eigen::Vector3d,-1,-1> &coordinates, 
		        Eigen::MatrixXd &Structure, std::vector< Eigen::MatrixXd > &Cameras_RCV, int cam_range1, int cam_range2 )
{
    //[camera x features] = [i x j]
    std::vector<Eigen::Vector3d> xdata;
    std::vector<Eigen::MatrixXd> PP;
    //Structure.resize(4,visibility.cols());
    for (int j = 0; j < visibility.cols(); j++)
    {
        xdata.clear();
        PP.clear();
        for (int i = cam_range1; i <= cam_range2; i++)
        {
	  if (visibility(i,j))
	  {
// 	      printf("Feature %i in camera %i, ",j,i);
// 	      std::cout << "Coordinates:\n" << (*coordinates)(i,j) << '\n';
	      xdata.push_back(coordinates(i,j));
	      PP.push_back(Cameras_RCV[i-cam_range1]);	      
	  }
        }
        if (PP.size() > 1) Structure.col(j) = linearTriangulation( xdata, PP );
    }    
    return;
}

void scalefromTranslations( std::vector< Eigen::Matrix3d > &Rot_global, std::vector< Eigen::Vector3d > &t12_RCV,
		        std::vector< Eigen::Vector3d > &t13_RCV, std::vector< Eigen::Vector3d > &tr_global )
{
    int num_cameras = Rot_global.size();
    // size of tr_global = num_cameras
    // size of t12_RCV = num_cameras - 1, size of t13_RCV = num_cameras - 2
    tr_global[0] = Eigen::Vector3d::Zero();
    tr_global[1] = t12_RCV[0];
    
    for (int cam = 0; cam < num_cameras-2; cam++)
    {
        Eigen::Matrix3d R23 = Rot_global[cam+2]*(Rot_global[cam+1].transpose());
        // Minimize the ratio ||a2vn - lambda*a2vd||
        Eigen::Vector3d a2vn = -t13_RCV[cam].cross(R23*t12_RCV[cam]);
        Eigen::Vector3d a2vd = t13_RCV[cam].cross(t12_RCV[cam+1]);
        double a2 = 1.0; // initial value
        a2 = a2 + pseudoInverse(a2vd)*(a2vn - a2*a2vd); // Normal equations
        
        // Minimize the ratio ||a1vn - lambda*a1vd||
        Eigen::Vector3d a1vn = R23*t12_RCV[cam] + a2*t12_RCV[cam+1];
        Eigen::Vector3d a1vd = t13_RCV[cam];
        double a1 = 1.0; // initial value
        a1 =  a1 + pseudoInverse(a1vd)*(a1vn - a1*a1vd); // Normal equations
//         std::cout << "\n******Scale from Translation******\n";
//         std::cout << "R23:\n" << R23 << "\n";
//         std::cout << "t13_RCV" << t13_RCV[cam].transpose() << "\n";
//         std::cout << "t12_RCV" << t12_RCV[cam].transpose() << "\n";
//         std::cout << "t12_RCV" << t12_RCV[cam+1].transpose() << "\n";
//         printf("a2 = %f\na1 = %f\n", a2, a1);
        // Update
        t13_RCV[cam] = a1*t13_RCV[cam];
        t12_RCV[cam+1] = a2*t12_RCV[cam+1];
        tr_global[cam+2] = R23*tr_global[cam+1] + t12_RCV[cam+1];
//         std::cout << "t13_RCV" << t13_RCV[cam].transpose() << "\n";
//         std::cout << "t12_RCV" << t12_RCV[cam+1].transpose() << "\n\n";
    }
    return;
}