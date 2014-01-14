// Created: Sep/04/2013
// Author: José David Tascón Vidarte

#include "BALReader.hpp"

// ================================================================================================
// ============================== FUNCTIONS of CLASS BALReader ====================================
// ================================================================================================
void BALReader::importBAL(Eigen::Matrix<bool,-1,-1> &visibility, Eigen::Matrix<Eigen::Vector3d,-1,-1> &coordinates,
	    std::vector< Eigen::Quaternion<double> > &quaternion, Eigen::MatrixXd &translation_and_intrinsics, Eigen::MatrixXd &structure)
{
    std::ifstream myfile1;
    myfile1.open(file_BAL);
    if (!myfile1.is_open())
    {
        std::cerr << "Error: unable to open file " << file_BAL << "\n";
        exit(0);
    }
    
    myfile1 >> num_cameras;
    myfile1 >> num_features;
    myfile1 >> num_observations;
    
    visibility = Eigen::Matrix<bool,-1,-1>::Zero(num_cameras,num_features);
    coordinates.resize(num_cameras,num_features);
    translation_and_intrinsics = Eigen::MatrixXd::Zero(6,num_cameras);
    quaternion.resize(num_cameras);
    structure = Eigen::MatrixXd::Ones(4,num_features);
    
    register int camera_idx;
    register int point_idx;
    double coordx;
    double coordy;
    // Reading observations
    for (register int j = 0; j < num_observations; ++j)
    {
        myfile1 >> camera_idx;
        myfile1 >> point_idx;
        myfile1 >> coordx;
        myfile1 >> coordy;
        visibility(camera_idx,point_idx) = true;
        coordinates(camera_idx,point_idx) = Eigen::Vector3d(coordx, coordy, 1.0);
    }
    double qtmp[4];
    double angleaxis[3];
    // Reading cameras
    for (register int j = 0; j < num_cameras; ++j)
    {
        myfile1 >> angleaxis[0];
        myfile1 >> angleaxis[1];
        myfile1 >> angleaxis[2];
        AngleAxisRToQuaternion(&angleaxis[0], &qtmp[0]);
        quaternion[j] = Eigen::Quaternion<double>(qtmp[0],qtmp[1],qtmp[2],qtmp[3]);
        for(register int i = 0; i < 6; ++i) myfile1 >> translation_and_intrinsics(i,j);
    }
    // Reading points
    for (register int j = 0; j < num_features; ++j)
    {
        myfile1 >> structure(0,j);
        myfile1 >> structure(1,j);
        myfile1 >> structure(2,j);
    }
    // Format
//     <num_cameras> <num_points> <num_observations>
//     <camera_index_1> <point_index_1> <x_1> <y_1>
//     ...
//     <camera_index_num_observations> <point_index_num_observations> <x_num_observations> <y_num_observations>
//     <camera_1>
//     ...
//     <camera_num_cameras>
//     <point_1>
//     ...
//     <point_num_points>
}