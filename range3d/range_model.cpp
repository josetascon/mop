// Created: Nov/07/2013
// Author: José David Tascón Vidarte

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// PCL Libraries
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>

// OpenCV Libraries
#include <opencv2/opencv.hpp>

// Std Libraries
#include <iostream>
#include <string>
#include <vector>

// Local libraries
#include "DepthProjection.hpp"
#include "Common.hpp"
#include "FeaturesMap.hpp"
#include "HandleDB.hpp"
#include "Interface.hpp"
#include "InterfacePCL.hpp"
#include "FeaturesEDM.hpp"
#include "Registration.hpp"
#include "Optimizer.hpp"
#include "Plot.hpp"

#include "DOTWriter.hpp"
#include "GraphBFS.hpp"

// using namespace cv;
// using namespace std;
// using namespace Eigen;

/**
* ******************************************************************
* @function help
*/
void help()
{ 
	std::cout << "\033[1;33m Usage: ./range_model -i <rgb_list.xml> -d <depth_list.xml> -o <output.txt>\033[0m\n"
				"\tOptions:\n" 
				"\t[-i]\t XML input file name with rgb images\n" 
				"\t[-d]\t XML input file name with depth images\n" 
				"\t[-v]\t Enable verbose mode\n"				
				"\t[-o]\t Output text filename\n"
				"\t[-k]\t .TXT input file name with calibration matrix\n"
				"\t[-df]\t Valid matches in Depth Filter\n"
				"\t[-db]\t .DB input file name with database\n"
				
	<< std::endl;
	
}

// ================================================= MAIN ==============================================
int main(int argc, char* argv[])
{
    // ========================================== Parameters ===========================================
    bool ram_db = true;
    bool verbose = false;
    bool load_k = false;
    bool save_ply = false;
    bool undis = false;
    int num_depth_filter = 3;		// Minimal valid match
    const char* filename_calib;
    const char* filename_undis_coeff;
    const char* inputFilename_rgb;
    const char* inputFilename_depth;
    const char* outputFilename;
    std::string filename_db = ":memory:";
    
    std::vector< std::string > imageList_rgb;
    std::vector< std::string > imageList_depth;
    
    // ========================================== Check parameters ==========================================
    if( argc < 2 )
    {
        help();
        return 0;
    }
    else
    {
        for(int i = 1; i < argc; i++ )
        {
	  const char* s = argv[i];
	  
	  if( strcmp( s, "-i" ) == 0 )
	  {
	      i++;
	      inputFilename_rgb = argv[i];
	  }
	  else if( strcmp( s, "-d" ) == 0 )
	  {
	      i++;
	      inputFilename_depth = argv[i];
	  }
	  else if( strcmp( s, "-o" ) == 0 )
	  {
	      i++;
	      outputFilename = argv[i];
	      save_ply = true;
	  }
	  else if( strcmp( s, "-k" ) == 0 )
	  {
	      i++;
	      filename_calib = argv[i];
	      load_k = true;
	  }
	  else if( strcmp( s, "-u" ) == 0 )
	  {
	      i++;
	      filename_undis_coeff = argv[i];
	      undis = true;
	  }
	  else if( strcmp( s, "-v" ) == 0 )
	  {
	      verbose = true;
	  }
	  else if( strcmp( s, "-df" ) == 0 )
	  {
	      i++;
	      num_depth_filter = pchar2number<int>( argv[i] );
	  }
	  else if( strcmp( s, "-db" ) == 0 )
	  {
	      i++;
	      filename_db = argv[i];
	      ram_db = false;
	  }
	  else
	  {
	      help();
	      return fprintf( stderr, "Unknown option %s\n", s ), -1;
	  }
        }
        
    }
    if (verbose) cout << "Parameters read...\t\t[OK]" << '\n';
    if (verbose) cout << "Data init...\t\t\t";
    
    // ========================================== Varible declaration ==========================================
    int num_goodmatch = 35;
    int num_features;
    int num_cameras;
    timer_wall timer1;

    // ========================================== Read Images ==========================================
    if (verbose)
    {
        std::cout << "[OK]" << '\n';		// DATA initialization OK
        std::cout << "Reading Files:\t" << inputFilename_rgb << " & " << inputFilename_depth << '\n';
        std::cout << "File list...\t\t\t";
    }
    importXMLImageList(inputFilename_rgb, imageList_rgb);		//Reading list of images from XML list
    importXMLImageList(inputFilename_depth, imageList_depth);
    
    if (verbose)
    {
        std::cout << "[OK]\n";
        std::cout << "Loading Images...\t\t";
    }
    
    if (verbose)
    {
        cv::Mat image0 = cv::imread(imageList_rgb[0], 1);
        std::cout << "[OK]\n";
        std::cout << "Number of Images: " << imageList_rgb.size() << "\n";
        printf("Size of Images: \tWidth: %i\t||\t Height: %i\n", image0.size().width, image0.size().height);
        std::cout << "\nDetection and Descriptor...\t";
    }
   
    // ======================================== END Read Images ========================================
    
    Eigen::Matrix3d K;
    if (load_k)						// If Calibration matrix is taken from txt file
    {
        Eigen::MatrixXd K_load;
        importTXTEigen(filename_calib, K_load);
        K << K_load;
    }
    else
    {
        cv::Mat image0 = cv::imread(imageList_rgb[0], 1);
        double wimg = image0.size().width - 1.0;
        double himg = image0.size().height - 1.0;
        double f = 520.0;
        K << f, 0.0, wimg/2, 0.0, f, himg/2, 0.0, 0.0, 1.0;		// Camera Matrix (intrinsics)
    }
    if (verbose) std::cout << "\nCalibration Matrix:\n" << K << "\n";
    
    Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(5,1);
    std::vector< std::string > undistort_files;
    if (undis)
    {
        importTXTEigen(filename_undis_coeff, coeff);
        
        undistortImages( (const char *)"undistort/rgb/", imageList_rgb, K, coeff, (const char *)"rgb_undis.xml", undistort_files );
        imageList_rgb = undistort_files;
        undistortImages( (const char *)"undistort/depth/", imageList_depth, K, coeff, (const char *)"depth_undis.xml", undistort_files );
        imageList_depth = undistort_files;
    }
    if (verbose) std::cout << "\nDistortion Coefficients:\n" << coeff.transpose() << "\n";
    
    // Graph Variables
    int num_vertex;
    int num_edges;
    std::vector<float> weights;
    std::vector< std::pair<int,int> > edges_pairs;
    
    
    // ====================================================================================================
    // ========================================== Start execution =========================================
    // ====================================================================================================
    
    HandleDB mydb( (char*)filename_db.c_str() );
    mydb.openDB();
    if ( ram_db )						// If database is in ram, create table and index
    {
        mydb.createFeaturesTable3D();
        mydb.createIndex1();
    }
    if (!mydb.isOpen()) exit(0);
    
//     if ( ram_db )						// If database is in ram, solve features map and store in db
//     {
        SiftED myfeat(imageList_rgb);
        myfeat.solveSift();
        
        MatchesMap my_mmap(400,35);
        my_mmap.solveMatches(&myfeat.descriptorsGPU);
//         my_mmap.solveMatchesContinuous(&myfeat.descriptorsGPU);
        my_mmap.robustifyMatches(&myfeat.keypointsGPU);
        my_mmap.depthFilter(&myfeat.keypointsGPU, &imageList_depth, num_depth_filter);
        timer1.start();
        my_mmap.solveDB3D( &mydb, &myfeat.keypointsGPU, &imageList_depth, K );
        std::cout << "Elapsed time to solve DB: " << timer1.elapsed_s() << " [s]\n";
        
        GraphPose gp;
        gp.solvePose( &my_mmap.reliableMatch, &my_mmap.globalMatch, &myfeat.keypointsGPU, &imageList_depth, &K);
        gp.solveEdges();
        
        num_vertex = gp.getNumVertex();
        num_edges = gp.getNumEdges();
        weights = gp.getWeights();
        edges_pairs = gp.getEdgesPairs();
        
        DOTWriter dotw("graph");
//         dotw.exportDOT( (const char *)"figs/test.dot", &edges_pairs );
        dotw.exportDOT( (const char *)"figs/gin_std.dot", &edges_pairs, NULL, &weights );
        
        std::vector< int > discover;
        std::vector< int > parent;
        GraphBFS bfs( num_vertex, num_edges, edges_pairs, weights );
        int initbfs = 0;
        bfs.setInitBFS( initbfs );
        bfs.solveBFS( discover, parent );
        
        gp.solveGraph( discover, parent );
        

        
//         gp.solveGraphContinuous();
//         exportGRAPH( (char*)"gin_opt.graph", gp.Qn_global, gp.tr_global ); 
//         gp.runTORO();
//         exportTXTQuaternionVector((char*)"pose_rot.txt", gp.Qn_global);
//         exportTXTTranslationVector((char*)"pose_tr.txt", gp.tr_global);
//     }
    
//     FeaturesMap featM;
//     featM.solveVisibility3D( &mydb );
//     printf("Visibility Matrix [%d x %d]\n",featM.visibility.rows(),featM.visibility.cols());
//     num_cameras = featM.cameras();
//     num_features = featM.features();
//     mydb.closeDB();
//     
//     SimpleRegistration sr01( num_cameras, num_features, K );
//     timer1.start();
//     sr01.solvePose( &featM.visibility, &featM.coordinates3D, true ); // true for optimal
//     std::cout << "Elapsed time to solve Pose: " << timer1.elapsed_s() << " [s]\n";
/*    
    // Print lin and opt
    SimpleRegistration sr02( num_cameras, num_features, K );
    timer1.start();
    sr02.solvePose( &featM.visibility, &featM.coordinates3D, false ); // true for optimal
    std::cout << "Elapsed time to solve Pose: " << timer1.elapsed_s() << " [s]\n";*/
    
    // Write files of global quaternion and translation
//     exportTXTQuaternionVector((char*)"liv_rot_opt.txt", sr01.Qn_global);
//     exportTXTTranslationVector((char*)"liv_tr_opt.txt", sr01.tr_global);
//     exportTXTQuaternionVector((char*)"liv_rot_lin.txt", sr02.Qn_global);
//     exportTXTTranslationVector((char*)"liv_tr_lin.txt", sr02.tr_global);
    
    std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > set_cloud;
    std::vector< boost::shared_ptr< Eigen::MatrixXd > > set_covariance;
//     cv2PointCloudSet(imageList_rgb, imageList_depth, K, sr01.Qn_global, sr01.tr_global, set_cloud, set_covariance);
    cv2PointCloudSet(imageList_rgb, imageList_depth, K, gp.Qn_global, gp.tr_global, set_cloud, set_covariance);
    
//     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_join;
//     mergeCloudSet( set_cloud, set_covariance, cloud_join );
    
    // Test one merge. Use to show in blue the proximity
//     boost::shared_ptr< Eigen::MatrixXd > Cov1, Cov2, Cov_New;
//     computeCovariance( set_cloud[0], K, Cov1 );
//     computeCovariance( set_cloud[1], K, Cov2 );    
//     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_join;
//     mergeClouds( set_cloud[0], set_cloud[1], Cov1, Cov2, cloud_join, Cov_New );
    
    // TODO Remember to free the memory here, images, Features, etc
    
    // ============================================ Visualization ============================================
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    
//     std::cout << "set cloud size:" << set_cloud.size() << "\n";
    viewer = visualizeCloudSet( set_cloud );
    
//     viewer = visualizeCloud(set_cloud[0]);
//     viewer = visualizeCloud(cloud_join);
    
    
//     visualizeCameras( viewer, sr01.Qn_global, sr01.tr_global );
    visualizeCameras( viewer, gp.Qn_global, gp.tr_global );
    
//     visualizeCameras(viewer, imageList_rgb, sr01.Qn_global, sr01.tr_global );
    
//     visualizeNoise(viewer, sr01.Xmodel, sr01.Variance, sr01.Qn_global, sr01.tr_global, 500 );
    
//     visualizeGraph( num_vertex, num_edges, weights, edges_pairs );
    
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    
//     if (save_ply)
//     {
//         pcl::PLYWriter wr_ply;
//         wr_ply.write(outputFilename, *model);
//     }
    
    // ========================================== END Visualization ==========================================
    return EXIT_SUCCESS;
    exit(0);
}