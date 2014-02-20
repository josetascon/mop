// Created: Dec/09/2013
// Author: José David Tascón Vidarte

#include "GraphPose.hpp"

// ================================================================================================
// ================================ FUNCTIONS of CLASS GraphPose ==================================
// ================================================================================================
int GraphPose::find_id_pair(std::vector< std::pair <int,int> > edges, int v1, int v2)
{
    int index = 0;
    for (std::vector<std::pair <int,int> >::iterator it = edges.begin() ; it != edges.end(); ++it, ++index)
        if ( (it->first == v1) &&  (it->second == v2)) return index;
}

void GraphPose::poseInID( int id, std::vector< std::string > *list_depth, 
	std::vector< MatchQuery > *globalMatch, std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints, bool optimal )
{
    pts1.clear();
    pts2.clear();
    
    int cam1 = (*globalMatch)[id].cam_id1;
    int cam2 = (*globalMatch)[id].cam_id2;
    
    cv::Mat depth1 = cv::imread( (*list_depth)[cam1], -1 );
    cv::Mat depth2 = cv::imread( (*list_depth)[cam2], -1 );
    extractPointsfromMatches( (*globalMatch)[id].matches, (*set_of_keypoints)[cam1], (*set_of_keypoints)[cam2], pts1, pts2 );
    
    Pose3D pose( Calibration );
    if (fallback_icp) pose.setFallBackPoseOn();
    pose.adaptPoints(pts1, pts2, depth1, depth2);
    bool is_solved = pose.solvePose(optimal); // true for optimal
    
    if (is_solved)
    {
        Rot = pose.getRotation();
        tr = pose.getTranslation();
    }
    else
    {
        DEBUG_1( std::cout << "Running ICP. Patience ...\n"; )
        PoseICP icp_solver( (*images_rgb)[cam1] , (*images_depth)[cam1], (*images_rgb)[cam2], (*images_depth)[cam2], Calibration );
        icp_solver.run();
        Rot = icp_solver.getRotation();
        tr = icp_solver.getTranslation();
    }
}

void GraphPose::run(  std::vector< std::string > *list_depth,
		    std::vector<bool> *reliableMatch, std::vector< MatchQuery > *globalMatch,
		    std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints, bool optimal )
{
    vertices = list_depth->size();
    solveEdges(reliableMatch, globalMatch);
    solveGraph();
    solveGlobalPose( list_depth, globalMatch, set_of_keypoints, optimal );
}

void GraphPose::solveLocalPoseAllNodes( std::vector< std::string > *list_depth,
		    std::vector<bool> *reliableMatch, std::vector< MatchQuery > *globalMatch,
		    std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints, bool optimal )
{
    edges = 0;
    vertices = list_depth->size();
    localPose.clear();
    
    DEBUG_1( std::cout << "\n================================ GRAPH POSE Estimation ==================================\n"; )
    for(register int it = 0; it < reliableMatch->size(); ++it)
    {
        
        if ((*reliableMatch)[it])
        {
	  int cam1 = (*globalMatch)[it].cam_id1;
	  int cam2 = (*globalMatch)[it].cam_id2;
	  // Debug
	  DEBUG_1( printf("POSE: %04i -> %04i\n", cam1, cam2); )
	  
	  poseInID( it, list_depth, globalMatch, set_of_keypoints, optimal );
	  
	  // Save pose to global object
	  Eigen::Quaternion<double> quat(Rot);
	  localPose.push_back( PoseQuery(cam1, cam2, pts1.size(), quat, tr) );
// 	  localPose.push_back( PoseQuery(cam1, cam2, goodpt.size(), quat, tr) );
	  edges++;
        }
    }
}


void GraphPose::solveEdgesAllNodes()
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

void GraphPose::solveEdges(std::vector<bool> *reliableMatch, std::vector< MatchQuery > *globalMatch)
{
    weights.clear();
    edges_pairs.clear();
    edges = 0;
    
    DEBUG_1( std::cout << "\n================================ GRAPH, Finding Edges ==================================\n"; )
    DEBUG_1( std::cout << "Edges:\n"; )
    for(register int it = 0; it < reliableMatch->size(); ++it)
    {
        if ((*reliableMatch)[it])
        {
	  int cam1 = (*globalMatch)[it].cam_id1;
	  int cam2 = (*globalMatch)[it].cam_id2;
	  weights.push_back( 1.0 );
	  edges_pairs.push_back( std::make_pair(cam1,cam2) );
	  edges++;
	  
	  match_pairs.push_back( std::make_pair(cam1,cam2) ); // Record all match information
	  
	  // Debug
	  DEBUG_1( std::cout << it << ": " << cam1 << " -> " << cam2 << "\n"; )
        }
        else match_pairs.push_back( std::make_pair(-1,-1) );
    }
}

void GraphPose::solveGraph( int initbfs )
{
    GraphBFS bfs( vertices, edges, edges_pairs, weights );
    if ( initbfs >= 0 ) bfs.setInitBFS( initbfs );
    bfs.solveBFS( discover_time, parent_node );
    DEBUG_2( std::cout << "BFS sucess\n"; )
}

void GraphPose::solveGlobalPose( std::vector< std::string > *list_depth, std::vector< MatchQuery > *globalMatch, 
		        std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints, bool optimal )
{
    Qn_global->clear();
    tr_global->clear();
    Qn_global->resize(vertices);
    tr_global->resize(vertices);
    
    Qn_global->at( discover_time[0] ) = Eigen::Quaternion<double>::Identity();
    tr_global->at( discover_time[0] ) = Eigen::Vector3d::Zero();
    DEBUG_1( std::cout << "\n================================ GRAPH, Solution to relative poses ==================================\n"; )
    for (register int k = 1; k < discover_time.size(); ++k)
    {
        int v2 = discover_time[k];
        int v1 = parent_node[ discover_time[k] ];
        bool reverse = (v1 > v2);
        
        if (reverse) // Check for inversion of local pose
        {
	  int idp = find_id_pair( match_pairs, v2, v1 );
	  // Debug
	  DEBUG_1( std::cout << "Find: " << v1 << " -> " << v2 << "\t||\tEdge: "<< idp <<"\n"; )
	  
	  poseInID( idp, list_depth, globalMatch, set_of_keypoints, optimal );
	  Eigen::Quaternion<double> quat(Rot);
	  
	  Qn_global->at(v2) = quat.conjugate()*Qn_global->at(v1);
	  tr_global->at(v2) = quat.conjugate()*(tr_global->at(v1) - tr);
        }
        else
        {
	  int idp = find_id_pair( match_pairs, v1, v2 );
	  // Debug
	  DEBUG_1( std::cout << "Find: " << v1 << " -> " << v2 << "\t||\tEdge: "<< idp <<"\n"; )
	  
	  poseInID( idp, list_depth, globalMatch, set_of_keypoints, optimal );
	  Eigen::Quaternion<double> quat(Rot);
	  
	  Qn_global->at(v2) = quat*Qn_global->at(v1);
	  tr_global->at(v2) = quat*tr_global->at(v1) + tr;
        }
    }
}

void GraphPose::solveGlobalPoseAllNodes()
{
    Qn_global->clear();
    tr_global->clear();
    Qn_global->resize(vertices);
    tr_global->resize(vertices);
    
    Qn_global->at( discover_time[0] ) = Eigen::Quaternion<double>::Identity();
    tr_global->at( discover_time[0] ) = Eigen::Vector3d::Zero();
    DEBUG_1( std::cout << "\n================================ GRAPH, Solution to relative poses ==================================\n"; )
    for (register int k = 1; k < discover_time.size(); ++k)
    {
        int v2 = discover_time[k];
        int v1 = parent_node[ discover_time[k] ];
        bool reverse = (v1 > v2);
        
        if (reverse) // Check for inversion of local pose
        {
	  int idp = find_id_pair( edges_pairs, v2, v1 );
	  // Debug
	  DEBUG_1( std::cout << "Find: " << v1 << " -> " << v2 << "\t||\tEdge: "<< idp <<"\n"; )
	  
	  Qn_global->at(v2) = localPose[idp].quaternion.conjugate()*Qn_global->at(v1);
	  tr_global->at(v2) = localPose[idp].quaternion.conjugate()*(tr_global->at(v1) - localPose[idp].translation);
        }
        else
        {
	  int idp = find_id_pair( edges_pairs, v1, v2 );
	  // Debug
	  DEBUG_1( std::cout << "Find: " << v1 << " -> " << v2 << "\t||\tEdge: "<< idp <<"\n"; )
	  
	  Qn_global->at(v2) = localPose[idp].quaternion*Qn_global->at(v1);
	  tr_global->at(v2) = localPose[idp].quaternion*tr_global->at(v1) + localPose[idp].translation;
        }
    }
}

void GraphPose::solveGlobalPoseContinuous()
{
    Qn_global->clear();
    tr_global->clear();
    Qn_global->resize(vertices);
    tr_global->resize(vertices);
    
    Qn_global->at(0) = Eigen::Quaternion<double>::Identity();
    tr_global->at(0) = Eigen::Vector3d::Zero();

//     std::cout << "Global Vertex Pose " << "\n";
//     std::cout << "Vertex size: " << vertices << "\n";
//     std::cout << "local pose #: " << localPose.size() << "\n";    
    for(register int k = 0; k < vertices-1; ++k)
    {
        int idx = find_id_pair(edges_pairs, k, k+1 );
//         std::cout << "idx: " << idx << "\n";
//         std::cout << "k: " << k << "\n";
        Qn_global->at(k+1) = localPose[idx].quaternion*Qn_global->at(k);
        tr_global->at(k+1) = localPose[idx].quaternion*tr_global->at(k) + localPose[idx].translation;
    }
//     std::cout << "End | Global Vertex Pose " << "\n";
}

void GraphPose::exportGRAPH( const char *filename )
{
//     char *file_txt = (char*)"pose.graph";
    std::ofstream myfile1;
    myfile1.open (filename);
    myfile1.precision(12);

    for(int it = 0; it < Qn_global->size(); it++)
    {
        Eigen::Matrix3d rr = Qn_global->at(it).toRotationMatrix();
        Eigen::Vector3d tr = rr.transpose()*(-tr_global->at(it));
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