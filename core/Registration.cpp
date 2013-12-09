// Created: Dec/09/2013
// Author: José David Tascón Vidarte

#include "Registration.hpp"


// ================================================================================================
// =========================== FUNCTIONS of CLASS SimpleRegistration ==============================
// ================================================================================================
void SimpleRegistration::solvePose(std::vector< MatchQuery > *globalMatch, std::vector< std::vector< cv::KeyPoint > > *set_of_keypoints, 
		std::vector< cv::Mat > *set_of_depth)
{
    // initialization
    Cameras_RCV.resize(num_cameras);
    Rot_global.resize(num_cameras);
    Qn_global.resize(num_cameras);
    tr_global.resize(num_cameras);
    StructurePlot.resize(num_cameras-1); // plot TESTING
    
    Rot_global[0] = Eigen::Matrix3d::Identity();
    tr_global[0] = Eigen::Vector3d::Zero();
    Qn_global[0] = Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0); //save quaternion
    Cameras_RCV[0] = buildProjectionMatrix( Calibration, Rot_global[0], tr_global[0] );
    cv::Mat KOCV = (cv::Mat_<double>(3,3) << Calibration(0,0), 0.0, Calibration(0,2), 0.0, Calibration(1,1), Calibration(1,2), 0.0, 0.0, 1.0);
    
    std::vector< cv::Point2d > pts1, pts2;
    std::vector< cv::Point3d > WP1, WP2;
    Eigen::MatrixXd X1, X2;
    Eigen::Matrix3d Rot;
    Eigen::Vector3d tr, angles_vec1;
    std::cout << "\n================================ POSE Estimation ==================================\n";
    for(register int k=0; k < (num_cameras - 1); ++k) // pose 1-2
    {
        std::vector< cv::DMatch > match_c = (*globalMatch)[k].matches;
        int cam1 = (*globalMatch)[k].cam_id1;
        int cam2 = (*globalMatch)[k].cam_id2;
        extractPointsfromMatches( match_c, (*set_of_keypoints)[cam1], (*set_of_keypoints)[cam2], pts1, pts2);
        removeBadPointsDual(pts1, pts2, (*set_of_depth)[cam1], (*set_of_depth)[cam2]);
        calc3Dfrom2D(pts1, (*set_of_depth)[cam1], KOCV, WP1);
        calc3Dfrom2D(pts2, (*set_of_depth)[cam2], KOCV, WP2);
        convertPoint3_toEigen(WP1, X1);
        convertPoint3_toEigen(WP2, X2);
        posefrom3DPoints( X1, X2, Rot, tr );
//         std::cout << "X1 =\n" << X1.transpose() << "\n";
//         std::cout << "X2 =\n" << X2.transpose() << "\n";
        
        // TESTING OPT without variance
//         LocalOptimizer opt01;
//         opt01.setParameter3Dto3D( &X1, &X2, &Rot, &tr );
//         opt01.pose3Dto3D();
        // TESTING OPT. **** NOTE: There is nothing to optimize, the basic algorithm has a small cost value
        
        Rot_global[k+1] = Rot*Rot_global[k]; // Rot;
        tr_global[k+1] = Rot*tr_global[k] + tr; // tr_global[k+1] = tr;
        Qn_global[k+1] = Eigen::Quaternion<double>(Rot_global[k+1]); //save quaternion
        Cameras_RCV[k+1] = buildProjectionMatrix( Calibration, Rot_global[k+1], tr_global[k+1] );
        StructurePlot[k] = WP1; // plot TESTING
        // Debug:
        anglesfromRotation( Rot_global[k+1], angles_vec1);
        std::cout << "Rotation " <<  k+1 << ":\n" << Rot_global[k+1] << "\n";
        std::cout << "Rotation angles " <<  k+1 << ":\n" << angles_vec1.transpose() << "\n";
//         std::cout << "Equivalent quaternion Cam " << k+1 << ":\n" << Qn_global[k+1].w() << " " << Qn_global[k+1].vec().transpose() << '\n';
        std::cout << "Translation " <<  k+1 << ":\n" << tr_global[k+1].transpose() << "\n";
        std::cout << "Camera Matrix Recover " <<  k+1 << ":\n" << Cameras_RCV[k+1] << "\n\n";
    }
    return;
}


void SimpleRegistration::solvePose(Eigen::Matrix<bool,-1,-1> *visibility, Eigen::Matrix<Eigen::Vector3d,-1,-1> *coordinates, 
		std::vector< cv::Mat > *set_of_depth)
{
    // initialization
    Cameras_RCV.resize(num_cameras);
    Rot_global.resize(num_cameras);
    Qn_global.resize(num_cameras);
    tr_global.resize(num_cameras);
    Structure = Eigen::MatrixXd::Zero(4,visibility->cols());
    
    Rot_global[0] = Eigen::Matrix3d::Identity();
    tr_global[0] = Eigen::Vector3d::Zero();
    Qn_global[0] = Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0); //save quaternion
    Cameras_RCV[0] = buildProjectionMatrix( Calibration, Rot_global[0], tr_global[0] );
    cv::Mat KOCV = (cv::Mat_<double>(3,3) << Calibration(0,0), 0.0, Calibration(0,2), 0.0, Calibration(1,1), Calibration(1,2), 0.0, 0.0, 1.0);
    
    std::vector<int> ft_number;
    std::vector< cv::Point2d > pts1, pts2;
    std::vector< cv::Point3d > WP1, WP2;
    Eigen::MatrixXd X1, X2;
    Eigen::Matrix3d Rot;
    Eigen::Vector3d tr, angles_vec1;
    std::cout << "\n================================ POSE Estimation ==================================\n";
    for(int k=0; k < (num_cameras - 1); k++) // pose 1-2
    {
        int cam1 = k;
        int cam2 = k+1;
        pts1.clear();
        pts2.clear();
        ft_number.clear();
        
        for (register int ft = 0; ft < num_features; ++ft)
        {
	  if ((*visibility)(cam1,ft) && (*visibility)(cam2,ft) )
	  {
	      Eigen::Vector3d point1 = (*coordinates)(cam1,ft);
	      Eigen::Vector3d point2 = (*coordinates)(cam2,ft);
// 	      std::cout << "Point1: " << point1.transpose() << "\n";
// 	      std::cout << "Point2: " << point2.transpose() << "\n";
	      pts1.push_back( cv::Point2d(point1(0), point1(1)) );
	      pts2.push_back( cv::Point2d(point2(0), point2(1)) );
	      ft_number.push_back(ft);
	  }
        }
        std::vector<int> goodpt = removeBadPointsDual(pts1, pts2, (*set_of_depth)[cam1], (*set_of_depth)[cam2]);
        // Debug
        printf("POSE: %04i -> %04i:\n", cam1, cam2);
        printVector(goodpt);
        
        calc3Dfrom2D(pts1, (*set_of_depth)[cam1], KOCV, WP1);
        calc3Dfrom2D(pts2, (*set_of_depth)[cam2], KOCV, WP2);
        convertPoint3_toEigen(WP1, X1);
        convertPoint3_toEigen(WP2, X2);
        posefrom3DPoints( X1, X2, Rot, tr );
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
       
        Rot_global[k+1] = Rot*Rot_global[k]; // Rot;
        tr_global[k+1] = Rot*tr_global[k] + tr; // tr_relative[k+1] = tr;
        Qn_global[k+1] = Eigen::Quaternion<double>(Rot_global[k+1]); //save quaternion
        Cameras_RCV[k+1] = buildProjectionMatrix( Calibration, Rot_global[k+1], tr_global[k+1] );
        
        // Rudimentary structure computation based on MEAN
//         Eigen::MatrixXd Xtmp = (Rot_global[k]*X1).colwise() + tr_global[k];
// //         std::cout << "Temporal X, cam " << k << ":\n";				//Debug
//         register int idx = 0;
//         for (register int i = 0; i < goodpt.size(); ++i)
//         {
// 	  if (goodpt[i])
// 	  {
// 	      Eigen::Vector4d v1 = Eigen::Vector4d( Xtmp(0,idx), Xtmp(1,idx), Xtmp(2,idx), 1);
// 	      Eigen::Vector4d v2 = Structure.col(ft_number[i]);
// 	      Structure.col(ft_number[i]) = v2(3)? 0.5*(v2 + v1): v1;
// 	      idx++;
// // 	      std::cout << "v1, ft " << ft_number[i] << ": "<< v1.transpose() << ":\n";		//Debug
// // 	      std::cout << "v2, ft " << ft_number[i] << ": "<< v2.transpose() << ":\n";		//Debug
// // 	      std::cout << "St, ft " << ft_number[i] << ": "<< Structure.col(ft_number[i]).transpose() << ":\n";		//Debug
// 	  }
//         }

        // Debug:
        anglesfromRotation( Rot_global[k+1], angles_vec1);
        std::cout << "Rotation " <<  k+1 << ":\n" << Rot_global[k+1] << "\n";
        std::cout << "Rotation angles " <<  k+1 << ":\n" << angles_vec1.transpose() << "\n";
//         std::cout << "Equivalent quaternion Cam " << k+1 << ":\n" << Qn_global[k+1].w() << " " << Qn_global[k+1].vec().transpose() << '\n';
        std::cout << "Translation " <<  k+1 << ":\n" << tr_global[k+1].transpose() << "\n";
        std::cout << "Camera Matrix Recover " <<  k+1 << ":\n" << Cameras_RCV[k+1] << "\n\n";
    }
    
    return;
}

void SimpleRegistration::solvePoseOptimal(Eigen::Matrix<bool,-1,-1> *visibility, Eigen::Matrix<Eigen::Vector3d,-1,-1> *coordinates, 
		std::vector< cv::Mat > *set_of_depth)
{
    // initialization
    Cameras_RCV.resize(num_cameras);
    Rot_global.resize(num_cameras);
    Qn_global.resize(num_cameras);
    tr_global.resize(num_cameras);
    Structure = Eigen::MatrixXd::Zero(4,visibility->cols());
    
    Rot_global[0] = Eigen::Matrix3d::Identity();
    tr_global[0] = Eigen::Vector3d::Zero();
    Qn_global[0] = Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0); //save quaternion
    Cameras_RCV[0] = buildProjectionMatrix( Calibration, Rot_global[0], tr_global[0] );
    cv::Mat KOCV = (cv::Mat_<double>(3,3) << Calibration(0,0), 0.0, Calibration(0,2), 0.0, Calibration(1,1), Calibration(1,2), 0.0, 0.0, 1.0);
    
    std::vector<int> ft_number;
    std::vector< cv::Point2d > pts1, pts2;
    std::vector< cv::Point3d > WP1, WP2;
    Eigen::MatrixXd X1, X2, variance1, variance2;
    Eigen::Matrix3d Rot;
    Eigen::Vector3d tr, angles_vec1;
    std::cout << "\n================================ POSE Estimation ==================================\n";
    for(int k=0; k < (num_cameras - 1); k++) // pose 1-2
    {
        int cam1 = k;
        int cam2 = k+1;
        pts1.clear();
        pts2.clear();
        
        for (register int ft = 0; ft < num_features; ++ft)
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
        std::vector<int> goodpt = removeBadPointsDual(pts1, pts2, (*set_of_depth)[cam1], (*set_of_depth)[cam2]);
        // Debug
        printf("POSE: %04i -> %04i:\n", cam1, cam2);
        printVector(goodpt);
        
        calc3Dfrom2D(pts1, (*set_of_depth)[cam1], KOCV, WP1);
        calc3Dfrom2D(pts2, (*set_of_depth)[cam2], KOCV, WP2);
        convertPoint3_toEigen(WP1, X1);
        convertPoint3_toEigen(WP2, X2);
        posefrom3DPoints( X1, X2, Rot, tr );
        
        varianceKinectSet( X1, Calibration, variance1 );
        varianceKinectSet( X2, Calibration, variance2 );
//         std::cout << "Std dev 1:\n" << variance1.transpose() << ":\n";
        
        // TESTING OPT 
        LocalOptimizer opt01;
        opt01.setParameters3Dto3D( &X1, &X2, &Rot, &tr, &variance1, &variance2 );
        opt01.pose3Dto3D_Covariance();
        
        Xmodel.push_back(X1);
        Xmodel.push_back(X2);
        Variance.push_back(variance1);
        Variance.push_back(variance2);
        // TESTING OPT.
       
        Rot_global[k+1] = Rot*Rot_global[k]; // Rot;
        tr_global[k+1] = Rot*tr_global[k] + tr; // tr_relative[k+1] = tr;
        Qn_global[k+1] = Eigen::Quaternion<double>(Rot_global[k+1]); //save quaternion
        Cameras_RCV[k+1] = buildProjectionMatrix( Calibration, Rot_global[k+1], tr_global[k+1] );

        // Debug:
        anglesfromRotation( Rot_global[k+1], angles_vec1);
        std::cout << "Rotation " <<  k+1 << ":\n" << Rot_global[k+1] << "\n";
        std::cout << "Rotation angles " <<  k+1 << ":\n" << angles_vec1.transpose() << "\n";
//         std::cout << "Equivalent quaternion Cam " << k+1 << ":\n" << Qn_global[k+1].w() << " " << Qn_global[k+1].vec().transpose() << '\n';
        std::cout << "Translation " <<  k+1 << ":\n" << tr_global[k+1].transpose() << "\n";
        std::cout << "Camera Matrix Recover " <<  k+1 << ":\n" << Cameras_RCV[k+1] << "\n\n";
    }
    
    return;
}

void SimpleRegistration::solvePose(Eigen::Matrix<bool,-1,-1> *visibility, Eigen::Matrix<Eigen::Vector4d,-1,-1> *coordinates, bool optimal)
{
    // initialization
    Cameras_RCV.resize(num_cameras);
    Rot_global.resize(num_cameras);
    Qn_global.resize(num_cameras);
    tr_global.resize(num_cameras);
    Structure = Eigen::MatrixXd::Zero(4,visibility->cols());
    
    Rot_global[0] = Eigen::Matrix3d::Identity();
    tr_global[0] = Eigen::Vector3d::Zero();
    Qn_global[0] = Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0); //save quaternion
    Cameras_RCV[0] = buildProjectionMatrix( Calibration, Rot_global[0], tr_global[0] );
    
    Eigen::MatrixXd X1, X2, Xtmp1, Xtmp2, variance1, variance2;
    Eigen::Matrix3d Rot;
    Eigen::Vector3d tr, angles_vec1;
    std::cout << "\n================================ POSE Estimation ==================================\n";
    for(int k=0; k < (num_cameras - 1); k++) // pose 1-2
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
        printf("POSE: %04i -> %04i:\n", cam1, cam2);
        std::cout << "Total Pts: " << count_ft << "\n";
        X1 = Xtmp1.block(0,0,3,count_ft);
        X2 = Xtmp2.block(0,0,3,count_ft);
        posefrom3DPoints( X1, X2, Rot, tr );
        varianceKinectSet( X1, Calibration, variance1 );
        varianceKinectSet( X2, Calibration, variance2 );

        LocalOptimizer opt01;
        opt01.setParameters3Dto3D( &X1, &X2, &Rot, &tr, &variance1, &variance2 );
        if (optimal) opt01.pose3Dto3D_Covariance();
        else opt01.pose3Dto3D();
        
        Xmodel.push_back(X1);
        Xmodel.push_back(X2);
        Variance.push_back(variance1);
        Variance.push_back(variance2);
       
        Rot_global[k+1] = Rot*Rot_global[k]; // Rot;
        tr_global[k+1] = Rot*tr_global[k] + tr; // tr_relative[k+1] = tr;
        Qn_global[k+1] = Eigen::Quaternion<double>(Rot_global[k+1]); //save quaternion
        Cameras_RCV[k+1] = buildProjectionMatrix( Calibration, Rot_global[k+1], tr_global[k+1] );

        // Debug:
        anglesfromRotation( Rot_global[k+1], angles_vec1);
        std::cout << "Rotation " <<  k+1 << ":\n" << Rot_global[k+1] << "\n";
        std::cout << "Rotation angles " <<  k+1 << ":\n" << angles_vec1.transpose() << "\n";
//         std::cout << "Equivalent quaternion Cam " << k+1 << ":\n" << Qn_global[k+1].w() << " " << Qn_global[k+1].vec().transpose() << '\n';
        std::cout << "Translation " <<  k+1 << ":\n" << tr_global[k+1].transpose() << "\n";
        std::cout << "Camera Matrix Recover " <<  k+1 << ":\n" << Cameras_RCV[k+1] << "\n\n";
    }
}

void SimpleRegistration::updateCamera()
{
    Eigen::Vector3d angles_vec1;
    for (int cam = 0; cam < num_cameras; cam++)
    {
        Qn_global[cam].normalize();
        Rot_global[cam] = Qn_global[cam].toRotationMatrix();
        Cameras_RCV[cam] = buildProjectionMatrix( Calibration, Rot_global[cam], tr_global[cam] );
        anglesfromRotation(Rot_global[cam], angles_vec1);
        std::cout << "Rotation " <<  cam << ":\n" << Rot_global[cam] << "\n";
        std::cout << "Rotation angles " <<  cam << ":\n" << angles_vec1.transpose() << "\n";
        std::cout << "Equivalent quaternion Cam " << cam << ":\n" << Qn_global[cam].w() << " " << Qn_global[cam].vec().transpose() << '\n';
        std::cout << "Translation " <<  cam << ":\n" << tr_global[cam].transpose() << "\n";
        std::cout << "Camera Matrix Recover " <<  cam << ":\n" << Cameras_RCV[cam] << "\n\n";
    } 
}

// ================================================================================================
// ================================ FUNCTIONS of CLASS GraphPose ==================================
// ================================================================================================
void GraphPose::solvePose( std::vector<bool> *reliableMatch, std::vector< MatchQuery > *globalMatch,
		    std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints,
		    std::vector< std::string > *list_depth, Eigen::Matrix3d *Calibration)
{
    edges = 0;
    vertex = list_depth->size();
    localPose.clear();
    cv::Mat KOCV = (cv::Mat_<double>(3,3) << (*Calibration)(0,0), 0.0, (*Calibration)(0,2), 0.0, (*Calibration)(1,1), (*Calibration)(1,2), 0.0, 0.0, 1.0);
    std::cout << "\n================================ GRAPH POSE Estimation ==================================\n";
    for(register int it = 0; it < reliableMatch->size(); ++it)
    {
        
        if ((*reliableMatch)[it])
        {
	  int cam1 = (*globalMatch)[it].cam_id1;
	  int cam2 = (*globalMatch)[it].cam_id2;
	  std::vector< cv::Point2d > pts1, pts2;
	  Eigen::Matrix3d Rot;
	  Eigen::Vector3d tr;
	  std::vector< cv::Point3d > WP1, WP2;
	  Eigen::MatrixXd X1, X2, variance1, variance2;
	  cv::Mat depth1 = cv::imread( (*list_depth)[cam1], -1 );
	  cv::Mat depth2 = cv::imread( (*list_depth)[cam2], -1 );
	  
	  for (register int ft = 0; ft < (*globalMatch)[it].matches.size(); ++ft)	//Extract pts1 from matches
	  {
	      int qid = (*globalMatch)[it].matches[ft].queryIdx;
	      int tid = (*globalMatch)[it].matches[ft].trainIdx;
	      pts1.push_back( cv::Point2d((*set_of_keypoints)[cam1][qid].x, (*set_of_keypoints)[cam1][qid].y) );
	      pts2.push_back( cv::Point2d((*set_of_keypoints)[cam2][tid].x, (*set_of_keypoints)[cam2][tid].y) );
	  }
	  // Remove bad points with depth projection
	  std::vector<int> goodpt = removeBadPointsDual(pts1, pts2, depth1, depth2);
	  
	  calc3Dfrom2D(pts1, depth1, KOCV, WP1);
	  calc3Dfrom2D(pts2, depth2, KOCV, WP2);
	  convertPoint3_toEigen(WP1, X1);
	  convertPoint3_toEigen(WP2, X2);
	  posefrom3DPoints( X1, X2, Rot, tr );
	  varianceKinectSet( X1, *Calibration, variance1 );
	  varianceKinectSet( X2, *Calibration, variance2 );
	  printf("POSE: %04i -> %04i:\n", cam1, cam2);
	  
	  // Optimization with Mahalanobis distance
	  LocalOptimizer opt01;
	  opt01.setParameters3Dto3D( &X1, &X2, &Rot, &tr, &variance1, &variance2 );
	  opt01.pose3Dto3D_Covariance();
	  std::cout << "\n";
	  
	  // Save pose to global object
	  Eigen::Quaternion<double> quat(Rot);
	  localPose.push_back( PoseQuery(cam1, cam2, goodpt.size(), quat, tr) );
	  edges++;
        }
    }
}


void GraphPose::solveEdges()
{
    weights.clear();
    edges_pairs.clear();
    weights.resize(edges);
    edges_pairs.resize(edges);
    
    for(register int it = 0; it < edges; ++it)
    {   
        weights[it] = float( localPose[it].num_matches );
        edges_pairs[it] = std::make_pair( localPose[it].cam_id1, localPose[it].cam_id2);
    }
}

void GraphPose::solveGraph()
{
    ;
}

int find_id_pair(std::vector< std::pair <int,int> > edges, int v1, int v2)
{
    int index = 0;
    for (std::vector<std::pair <int,int> >::iterator it = edges.begin() ; it != edges.end(); ++it, ++index)
        if ( (it->first == v1) &&  (it->second == v2)) return index;
}

void GraphPose::solveGraphContinuous()
{
    Qn_global.clear();
    tr_global.clear();
    Qn_global.resize(vertex);
    tr_global.resize(vertex);
    
    Qn_global[0] = Eigen::Quaternion<double>::Identity();
    tr_global[0] = Eigen::Vector3d::Zero();
    
    ct_global.clear();
    ct_global.resize(vertex);
    ct_global[0] = Eigen::Vector3d::Zero();
//     std::cout << "Global Vertex Pose " << "\n";
//     std::cout << "Vertex size: " << vertex << "\n";
//     std::cout << "local pose #: " << localPose.size() << "\n";    
    for(register int k = 0; k < vertex-1; ++k)
    {
        int idx = find_id_pair(edges_pairs, k, k+1 );
//         std::cout << "idx: " << idx << "\n";
//         std::cout << "k: " << k << "\n";
        Qn_global[k+1] = localPose[idx].quaternion*Qn_global[k];
        tr_global[k+1] = localPose[idx].quaternion*tr_global[k] + localPose[idx].translation;
        ct_global[k+1] = Qn_global[k+1].conjugate()*(-tr_global[k+1]);
    }
//     std::cout << "End | Global Vertex Pose " << "\n";
}

void GraphPose::runTORO()
{
    char *file_txt = (char*)"pose.graph";
    
    std::ofstream myfile1;
    myfile1.open (file_txt);
    myfile1.precision(12);

    for(int it = 0; it < Qn_global.size(); it++)
    {
        Eigen::Matrix3d rr = Qn_global[it].toRotationMatrix();
//         Eigen::Vector3d tr = tr_global[it];
        Eigen::Vector3d tr = ct_global[it]; // TESTING Center
        Eigen::Vector3d angles;
        anglesfromRotation( rr, angles, false ); //false for radians units
        
        myfile1 << "VERTEX3 ";
        myfile1 << it << " ";
        myfile1 << tr.transpose() << " ";
        myfile1 << angles.transpose();
        myfile1 << "\n";
    }
    
    for(int it = 0; it < localPose.size(); it++)
    {
        int cam1 = localPose[it].cam_id1;
        int cam2 = localPose[it].cam_id2;
        Eigen::Matrix<double,6,1> p_j, p_i, dji;
        Eigen::Matrix3d Ri, rot;
        Eigen::Vector3d tr, angles;
        
        rot = localPose[cam2].quaternion.toRotationMatrix();
//         tr = localPose[cam2].translation;
        tr = rot.transpose()*(-localPose[cam2].translation);
        anglesfromRotation( rot, angles, false );//false for radians units
        
        p_j << tr, angles;
        
        rot = localPose[cam1].quaternion.toRotationMatrix();
//         tr = localPose[cam1].translation;
        tr = rot.transpose()*(-localPose[cam1].translation);
        anglesfromRotation( rot, angles, false );//false for radians units
        
        p_i << tr, angles;
        
        Ri = Qn_global[cam1].toRotationMatrix();
        Eigen::Matrix<double,6,6> R6i = Eigen::Matrix<double,6,6>::Identity();
        R6i.block(0,0,3,3) = Ri;
        
        dji = R6i.transpose()*(p_j - p_i);
        
        myfile1 << "EDGE3 ";
        myfile1 << cam1 << " ";
        myfile1 << cam2 << " ";
//         myfile1 << p_j;
//         myfile1 << p_i;
        myfile1 << dji.transpose();
//         myfile1 << tr.transpose() << " ";
//         myfile1 << angles.transpose(); 
//         myfile1 << "  1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1" << "\n";
        myfile1 << "\n";
    }
    myfile1.close();
}
