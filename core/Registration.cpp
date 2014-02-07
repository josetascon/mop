// Created: Dec/09/2013
// Author: José David Tascón Vidarte

#include "Registration.hpp"

// ================================================================================================
// ================================ FUNCTIONS of CLASS GraphPose ==================================
// ================================================================================================
void GraphPose::solvePose( std::vector<bool> *reliableMatch, std::vector< MatchQuery > *globalMatch,
		    std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints,
		    std::vector< std::string > *list_depth, Eigen::Matrix3d *Calibration)
{
    edges = 0;
    vertices = list_depth->size();
    localPose.clear();
    cv::Mat KOCV = (cv::Mat_<double>(3,3) << (*Calibration)(0,0), 0.0, (*Calibration)(0,2), 0.0, (*Calibration)(1,1), (*Calibration)(1,2), 0.0, 0.0, 1.0);
    DEBUG_1( std::cout << "\n================================ GRAPH POSE Estimation ==================================\n"; )
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
	  DEBUG_1( printf("POSE: %04i -> %04i\n", cam1, cam2); )
	  
	  // Remove bad points with depth projection
	  std::vector<int> goodpt = removeBadPointsDual(pts1, pts2, depth1, depth2);
	  
	  calc3Dfrom2D(pts1, depth1, KOCV, WP1);
	  calc3Dfrom2D(pts2, depth2, KOCV, WP2);
	  point3_vector2eigen(WP1, X1);
	  point3_vector2eigen(WP2, X2);
	  poseArun( X1, X2, Rot, tr );
	  varianceKinectSet( X1, *Calibration, variance1 );
	  varianceKinectSet( X2, *Calibration, variance2 );
	  
	  // Optimization with Mahalanobis distance
	  LocalOptimizer opt01;
	  opt01.setParameters3Dto3D( &X1, &X2, &Rot, &tr, &variance1, &variance2 );
	  opt01.pose3Dto3D_Covariance();
	  DEBUG_2( std::cout << "\n"; )
	  
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
//     weights.resize(edges);
//     edges_pairs.resize(edges);
    DEBUG_1( std::cout << "\n================================ GRAPH, Edge List ==================================\n"; )
    DEBUG_1( std::cout << "Edges:\n"; )
    
    for(register int it = 0; it < edges; ++it)
    {
        weights.push_back( float( localPose[it].num_matches ) );
        edges_pairs.push_back( std::make_pair( localPose[it].cam_id1, localPose[it].cam_id2) );
        
        // Debug
        DEBUG_1( std::cout << it << ": " << localPose[it].cam_id1 << " -> " << localPose[it].cam_id2 << "\n"; )
        
//         weights[it] = float( localPose[it].num_matches );
//         edges_pairs[it] = std::make_pair( localPose[it].cam_id1, localPose[it].cam_id2);
    }
}

/// SOLVE Qn_global and tr_global with GRAPH (min distance)

void GraphPose::solveGraph(std::vector< int > &discover, std::vector< int > &parent)
{
    Qn_global.clear();
    tr_global.clear();
    Qn_global.resize(vertices);
    tr_global.resize(vertices);
    
    Qn_global[ discover[0] ] = Eigen::Quaternion<double>::Identity();
    tr_global[ discover[0] ] = Eigen::Vector3d::Zero();
    DEBUG_1( std::cout << "\n================================ GRAPH, Solution to relative poses ==================================\n"; )
    for (register int k = 1; k < discover.size(); ++k)
    {
        int v2 = discover[k];
        int v1 = parent[ discover[k] ];
        bool reverse = (v1 > v2);
        
        if (reverse) // Check for inversion of local pose
        {
	  int idp = find_id_pair( edges_pairs, v2, v1 );
	  // Debug
	  DEBUG_1( std::cout << "Find: " << v1 << " -> " << v2 << "\t||\tEdge: "<< idp <<"\n"; )
	  
	  Qn_global[v2] = localPose[idp].quaternion.conjugate()*Qn_global[v1];
	  tr_global[v2] = localPose[idp].quaternion.conjugate()*(tr_global[v1] - localPose[idp].translation);
        }
        else
        {
	  int idp = find_id_pair( edges_pairs, v1, v2 );
	  // Debug
	  DEBUG_1( std::cout << "Find: " << v1 << " -> " << v2 << "\t||\tEdge: "<< idp <<"\n"; )
	  
	  Qn_global[v2] = localPose[idp].quaternion*Qn_global[v1];
	  tr_global[v2] = localPose[idp].quaternion*tr_global[v1] + localPose[idp].translation;
        }
    }
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
    Qn_global.resize(vertices);
    tr_global.resize(vertices);
    
    Qn_global[0] = Eigen::Quaternion<double>::Identity();
    tr_global[0] = Eigen::Vector3d::Zero();

//     std::cout << "Global Vertex Pose " << "\n";
//     std::cout << "Vertex size: " << vertices << "\n";
//     std::cout << "local pose #: " << localPose.size() << "\n";    
    for(register int k = 0; k < vertices-1; ++k)
    {
        int idx = find_id_pair(edges_pairs, k, k+1 );
//         std::cout << "idx: " << idx << "\n";
//         std::cout << "k: " << k << "\n";
        Qn_global[k+1] = localPose[idx].quaternion*Qn_global[k];
        tr_global[k+1] = localPose[idx].quaternion*tr_global[k] + localPose[idx].translation;
    }
//     std::cout << "End | Global Vertex Pose " << "\n";
}

void GraphPose::exportGRAPH( const char *filename )
{
//     char *file_txt = (char*)"pose.graph";
    std::ofstream myfile1;
    myfile1.open (filename);
    myfile1.precision(12);

    for(int it = 0; it < Qn_global.size(); it++)
    {
        Eigen::Matrix3d rr = Qn_global[it].toRotationMatrix();
        Eigen::Vector3d tr = rr.transpose()*(-tr_global[it]);
        Eigen::Vector3d angles;
        rotation2angles( rr, angles, false ); //false for radians units
        
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
        
        Eigen::Matrix3d rot1 = localPose[it].quaternion.toRotationMatrix();
        Eigen::Vector3d tr1 = rot1.transpose()*(-localPose[it].translation);
        Eigen::Matrix<double,6,1> p_j, p_i, dji;
        Eigen::Vector3d angles1, angles2;
        
        rotation2angles_DetectZero( rot1, angles1, false );//false for radians units
        dji << tr1, angles1;
        
        myfile1 << "EDGE3 ";
        myfile1 << cam1 << " ";
        myfile1 << cam2 << " ";
        myfile1 << dji.transpose();
//         myfile1 << tr.transpose() << " ";
//         myfile1 << angles.transpose(); 
//         myfile1 << "  1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1" << "\n";
        myfile1 << "\n";
    }
    myfile1.close();
}