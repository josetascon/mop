// Created: Feb/14/2014
// Author: José David Tascón Vidarte

#include "ReconstructionModel.hpp"

// ================================================================================================
// =========================== FUNCTIONS of CLASS ReconstructionModel =============================
// ================================================================================================
void ReconstructionModel::solveFeatures()
{   
    std::string filename_db = ":memory:";
    HandleDB mydb( (char*)filename_db.c_str() );
    mydb.openDB();
    mydb.createFeaturesTable3D();
    mydb.createIndex1();
    if (!mydb.isOpen()) 
    {
        DEBUG_E( ("Fail to reconstruct a reconstruction model. Cloud not open database (SQLite)") ); 
        exit(-1);
    }
    
    // Extract and Descript Features with SIFT
    features->solveSift();
    
    // Matches
    matchMap->setAllContinousOn(); // ENABLE CONTINOUS MATCHES ALWAYS TO ALLOW ICP WORK
    if (subgroups) matchMap->solveMatchesGroups(features->getDescriptorsGPU(), subgroup_boundaries);
    else matchMap->solveMatches(features->getDescriptorsGPU());
    matchMap->robustifyMatches(features->getKeypointsGPU());
    matchMap->depthFilter(features->getKeypointsGPU(), depth_list, num_depth_filter);
    timer1.start();
    matchMap->solveDB3D( &mydb, features->getKeypointsGPU(), depth_list, Calibration );
    DEBUG_1( std::cout << "Elapsed time to solve DB: " << timer1.elapsed_s() << " [s]\n"; )
    
    // Visibility with FeaturesMap
    featMap->solveVisibility3D( &mydb );
    num_cameras = featMap->getNumberCameras();
    num_features = featMap->getNumberFeatures();
    mydb.closeDB();
}

void ReconstructionModel::freeMemoryFeatures()
{
    // Free memory used with Features information
    features.reset();
    matchMap.reset();
    featMap.reset();
}

void ReconstructionModel::graphModel()
{
    boost::shared_ptr< GraphPose > gp(new GraphPose(Calibration) );
    gp->setFallBackICPOn( image_list, depth_list );
    gp->run( depth_list, &matchMap->reliableMatch, &matchMap->globalMatch, (features->getKeypointsGPU()).get(), true ); // true for optimal
    Qn_global = gp->getPtrGlobalQuaternion();
    tr_global = gp->getPtrGlobalTranslation();
}

void ReconstructionModel::simpleModel()
{
    boost::shared_ptr< SimpleRegistration > sr01( new SimpleRegistration( num_cameras, num_features, Calibration ) );
    sr01->setFallBackICPOn( image_list, depth_list, num_depth_filter );
    timer1.start();
    sr01->solvePose3D( featMap->getVisibility(), featMap->getCoordinates3D(), true ); // true for optimal
    std::cout << "Elapsed time to solve Pose: " << timer1.elapsed_s() << " [s]\n";
    Qn_global = sr01->getPtrGlobalQuaternion();
    tr_global = sr01->getPtrGlobalTranslation();
}

void ReconstructionModel::globalOptimization()
{
    GlobalPose3D global01;
    global01.solve(featMap->getVisibility(), featMap->getCoordinates3D(), &Calibration, Qn_global, tr_global );
//     global01.solve_LSQ(featMap->getVisibility(), featMap->getCoordinates3D(), &Calibration, Qn_global, tr_global );
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloudE;
//     eigen2pointcloud( global01.Structure, cloudE );
    setCoordinatestoOrigin(*Qn_global, *tr_global);
}

void ReconstructionModel::visualizeNonUnifiedModel()
{
    if(!non_unified_model)
    {
        DEBUG_E( ("Unable to find Non-Unified model. Execute solveNonUnifiedModel() first\n") ); 
        exit(-1);
    }
       
    // ============================================ Visualization ============================================
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = visualizeCloudSet( set_model );
    visualizeCameras( viewer, *Qn_global, *tr_global );
    
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void ReconstructionModel::solveUnifiedModel()
{
    MergeClouds mrg( image_list, depth_list, Calibration );
    mrg.mergeSet( *Qn_global, *tr_global );
    Xmodel = mrg.getCloud();
    Variance = mrg.getCovariance();
    unified_model = true;
}

void ReconstructionModel::solveNonUnifiedModel()
{
    cv2PointCloudSet(*image_list, *depth_list, Calibration, *Qn_global, *tr_global, set_model, set_variance);
    non_unified_model = true;
}

void ReconstructionModel::visualizeUnifiedModel()
{
    if(!unified_model)
    {
        DEBUG_E( ("Unable to find Unified model. Execute solveUnifiedModel() first\n") ); 
        exit(-1);
    }
       
    // ============================================ Visualization ============================================
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = visualizeCloud(Xmodel);
    visualizeCameras( viewer, *Qn_global, *tr_global );
    
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}