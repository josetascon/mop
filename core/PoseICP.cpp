// Created: Feb/07/2013
// Author: José David Tascón Vidarte

#include "PoseICP.hpp"

// ================================================================================================
// ================================= FUNCTIONS of CLASS PoseICP ===================================
// ================================================================================================
// Function to run the entire process
void PoseICP::run()
{
    solveClouds();
    solvePose();
}

// Function to extract cloud1 and cloud2 from images
void PoseICP::solveClouds()
{
    if ( !load_calib || !load_images )
    {
        DEBUG_E( ("Not loaded images or Calibration Matrix. Use properly the constructor or set parameters manually") ); 
        exit(-1);
    }
    
    DEBUG_3( printf( "Image 1 size [%i x %i]\n", image1->size().width, image1->size().height ); )
    DEBUG_3( printf( "Depth 1 size [%i x %i]\n", depth1->size().width, depth1->size().height ); )
    cv2PointCloudDense(*image1, *depth1, Calibration, cloud1);
    cv2PointCloudDense(*image2, *depth2, Calibration, cloud2);
    load_clouds = true;
}

// Function to import cloud1 and cloud2 from pcd files
void PoseICP::importPCD( const char *file_pcd1, const char *file_pcd2)
{
    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read (file_pcd1, *cloud1); // Read the file 1
    reader.read (file_pcd2, *cloud2); // Read the file 2
    load_clouds = true;
}

// Function to solve pose with icp using cloud1 and cloud2
void PoseICP::solvePose()
{
    if ( !load_clouds)
    {
        DEBUG_E( ("Not loaded points. Load properly the points or execute solvePoints() first") ); 
        exit(-1);
    }
    
    timer_wall timer1;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    // ICP
    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputSource(cloud1);
    icp.setInputTarget(cloud2);
    
    timer1.start();
    icp.align(*cloud_out1);
    timer1.elapsed_s();
    
    // Debug information:
    DEBUG_2( std::cout << "\n================ ICP Results ================\n"; )
    DEBUG_2( std::cout << "Algorithm has converged:" << icp.hasConverged() << "\tscore: " << icp.getFitnessScore() << std::endl; )
    DEBUG_2( std::cout << "Time to solve: " << timer1.lap() << " s\n"; )
    DEBUG_3( std::cout << "Transformation Matrix:\n"; )
    DEBUG_3( std::cout << icp.getFinalTransformation() << std::endl; )
    
    Eigen::Matrix4f TMatrix = icp.getFinalTransformation();
    Eigen::Matrix3f Rot = TMatrix.block(0,0,3,3);
    Eigen::Vector3f tr = TMatrix.block(0,3,3,1);
    
    Eigen::Quaternion<float> qt(Rot);
    Eigen::Quaternion<double> qd;
    qd = qt.template cast<double>();
    DEBUG_2( std::cout << "Orientation:\n" << qd.w() << " " << qd.vec().transpose() << "\n"; )
    DEBUG_2( std::cout << "Translation:\n" << tr.transpose() << "\n\n"; )
    
    Rotation = qd.toRotationMatrix();
    translation = tr.template cast<double>();
}