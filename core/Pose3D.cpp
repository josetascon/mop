// Created: Feb/07/2013
// Author: José David Tascón Vidarte

#include "Pose3D.hpp"

// ================================================================================================
// ================================= FUNCTIONS of CLASS Pose3D ====================================
// ================================================================================================
// Function to run the entire process
void Pose3D::run()
{
    solvePoints();
    solvePose();
}

void Pose3D::solvePoints()
{
    if ( !load_calib || !load_images )
    {
        DEBUG_E( ("Not loaded images or Calibration Matrix. Use properly the constructor or set parameters manually") ); 
        exit(-1);
    }
    
    cv::Mat Fund;
    std::vector<SiftGPU::SiftKeypoint> keyGPU1, keyGPU2;
    std::vector<float> descriptors1, descriptors2;
    std::vector<cv::Point2d> pts1, pts2;
    
    SimpleSiftGPU ss_features(400);    
    ss_features.solveFeatures( *image1, keyGPU1, descriptors1 );
    ss_features.solveFeatures( *image2, keyGPU2, descriptors2 );
    ss_features.solveMatches( keyGPU1, keyGPU2, descriptors1, descriptors2, pts1, pts2);
    
    robustMatchesfromFundamental(pts1, pts2, ss_features.matches, Fund, 3);
    
    adaptPoints( pts1, pts2, *depth1, *depth2 );
}

// Function to extract points X1 and X2 from opencv points
void Pose3D::adaptPoints( std::vector<cv::Point2d> &pts1, std::vector<cv::Point2d> &pts2, cv::Mat &range1, cv::Mat &range2 )
{
    if ( !load_calib )
    {
        DEBUG_E( ("Not loaded Calibration Matrix. Use properly the constructor or set parameters manually") ); 
        exit(-1);
    }
    
    std::vector< cv::Point3d > WP1, WP2;
    
    removeBadPointsDual(pts1, pts2, range1, range2);
    
    calc3Dfrom2D(pts1, *depth1, KOCV, WP1);
    calc3Dfrom2D(pts2, *depth2, KOCV, WP2);
    point3_vector2eigen(WP1, X1);
    point3_vector2eigen(WP2, X2);
    
    load_points = true;
}
    
// Function to solve pose from X1 and X2
void Pose3D::solvePose(bool optimal)
{
    if ( !load_points)
    {
        DEBUG_E( ("Not loaded points. Load properly the points or execute solvePoints() first") ); 
        exit(-1);
    }
    
    poseArun( X1, X2, Rotation, translation );			// Initial pose with Arun's algorithm
    
    if ( optimal )
    {
        // Optimization routine (with Covariance)
        varianceKinectSet( X1, Calibration, variance1 );
        varianceKinectSet( X2, Calibration, variance2 );
        
        LocalOptimizer opt01;
        opt01.setParameters3Dto3D( &X1, &X2, &Rotation, &translation, &variance1, &variance2 );
        opt01.pose3Dto3D_Covariance();
    }
    else
    {
        // Optimization routine
        LocalOptimizer opt01;
        opt01.setParameters3Dto3D( &X1, &X2, &Rotation, &translation);
        opt01.pose3Dto3D();
    }
}

 
    