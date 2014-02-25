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
#include <libgen.h>

// Local libraries
#include "DepthProjection.hpp"
#include "Debug.hpp"
#include "Common.hpp"
#include "HandleDB.hpp"
#include "Interface.hpp"
#include "InterfacePCL.hpp"
#include "MergeClouds.hpp"
#include "DOTWriter.hpp"

#include "FeaturesEDM.hpp"
#include "Registration.hpp"
#include "Optimizer.hpp"
#include "Plot.hpp"

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
				"\t[-op]\t Output ply filename\n"
				"\t[-ot]\t Output txt filename\n"
				"\t[-k]\t .TXT input file name with calibration matrix\n"
				"\t[-df]\t Valid matches in Depth Filter\n"
				"\t[-db]\t .DB input file name with database\n"
				
	<< std::endl;
	
}

void readMultipleFiles( std::vector< std::string > &files_input, std::vector< std::string > &image_list, std::vector<int> &boundaries )
{
    std::cout << "Reading\n";
    std::string ext = "xml";
    for(int k = 0; k < files_input.size(); k++)
    {
        verifyFileExtension( files_input[k].c_str(), ext, true );
        importXMLImageList( files_input[k].c_str(), image_list, false); // false for unclear the vector
        boundaries.push_back(image_list.size());
        std::cout << "Number: " << image_list.size() << "\n";
    }
//     exit(0);
}

// ================================================= MAIN ==============================================
int main(int argc, char* argv[])
{
    // ========================================== Parameters ===========================================
    bool ram_db = true;
    bool verbose = false;
    bool multiple = false;
    bool load_k = false;
    bool save_ply = false;
    bool save_txt = false;
    bool undis = false;
    int num_depth_filter = 3;		// Minimal valid match
    const char* filename_calib;
    const char* filename_undis_coeff;
    const char* inputFilename_rgb;
    const char* inputFilename_depth;
    const char* outputFilename;
    
    std::vector< std::string > files_input_rgb;
    std::vector< std::string > files_input_depth;
    
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
	  else if( strcmp( s, "-im" ) == 0 )
	  {
	      i++;
	      int input = pchar2number<int>( argv[i] );
	      for(int k = 0; k < input; k++)
	      {
		i++;
		files_input_rgb.push_back( argv[i] );
	      }
	      multiple = true;
	  }
// 	  else if( strcmp( s, "-dm" ) == 0 )
// 	  {
// 	      
// 	  }
	  else if( strcmp( s, "-op" ) == 0 )
	  {
	      i++;
	      outputFilename = argv[i];
	      save_ply = true;
	  }
	  else if( strcmp( s, "-ot" ) == 0 )
	  {
	      save_txt = true;
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
    if (verbose) std::cout << "Parameters read...\t\t[OK]" << '\n';
    if (verbose) std::cout << "Data init...\t\t\t";
    
    std::vector<int> bb;
    if( multiple ) readMultipleFiles(files_input_rgb , imageList_rgb, bb );
    
    // ========================================== Varible declaration ==========================================
    int num_goodmatch = 35;
    int num_features;
    int num_cameras;
    timer_wall timer1;
    std::string base_filename = basename((char*)inputFilename_rgb);
    base_filename = base_filename.substr(0, base_filename.rfind("."));
    
    std::string ext = "xml";
    verifyFileExtension( inputFilename_rgb, ext, true );
    verifyFileExtension( inputFilename_depth, ext, true );
    
    // ========================================== Read Images ==========================================
    if (verbose)
    {
        std::cout << "[OK]" << '\n';		// DATA initialization OK
        std::cout << "Reading Files:\t" << inputFilename_rgb << " & " << inputFilename_depth << '\n';
        std::cout << "File list...\t\t\t";
    }
    importXMLImageList(inputFilename_rgb, imageList_rgb);		//Reading list of images from XML list
    importXMLImageList(inputFilename_depth, imageList_depth);
    
    if (imageList_depth.size() != imageList_rgb.size())
    {
        DEBUG_E( ("Number of color and depth images files are different. Please check your xml files.") );
        exit(-1);
    }
    
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
    typedef std::vector< Eigen::Quaternion<double> > Qd_vector;
    typedef std::vector< Eigen::Vector3d > V3d_vector;
    boost::shared_ptr< Qd_vector > Qn_global;
    boost::shared_ptr< V3d_vector > tr_global;
    
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
        boost::shared_ptr< SiftED > myfeat( new SiftED(&imageList_rgb) );
        myfeat->solveSift();
        
        boost::shared_ptr< MatchesMap > my_mmap( new MatchesMap(500,35) );
        my_mmap->setAllContinousOn(); // ENABLE CONTINOUS MATCHES ALWAYS TO ALLOW ICP WORK
        my_mmap->solveMatches(myfeat->getDescriptorsGPU());
//         my_mmap->solveMatchesContinuous(myfeat->getDescriptorsGPU());
//         my_mmap->solveMatchesGroups(myfeat->getDescriptorsGPU(), 10);
/*        std::vector<int> bb;
        bb.push_back(5); bb.push_back(10); bb.push_back(20), bb.push_back(imageList_rgb.size());
        my_mmap->solveMatchesGroups(myfeat->getDescriptorsGPU(), &bb);  */      
        my_mmap->robustifyMatches(myfeat->getKeypointsGPU());
        my_mmap->depthFilter(myfeat->getKeypointsGPU(), &imageList_depth, num_depth_filter);
        timer1.start();
        my_mmap->solveDB3D( &mydb, myfeat->getKeypointsGPU(), &imageList_depth, K );
        DEBUG_1( std::cout << "Elapsed time to solve DB: " << timer1.elapsed_s() << " [s]\n"; )
        
//         boost::shared_ptr< GraphPose > gp(new GraphPose(K) );
//         gp->setFallBackICPOn( &imageList_rgb, &imageList_depth );
//         gp->run( &imageList_depth, &my_mmap->reliableMatch, &my_mmap->globalMatch, (myfeat->getKeypointsGPU()).get(), true ); // true for optimal
// //         gp->solveLocalPoseAllNodes( &imageList_depth, &my_mmap->reliableMatch, &my_mmap->globalMatch, myfeat->getKeypointsGPU() );
// //         gp->solveEdgesAllNodes();
// //         gp->solveGraph();
// //         gp->solveGlobalPoseAllNodes();
// // //         gp->solveGlobalPoseContinuous();        
//         Qn_global = gp->getPtrGlobalQuaternion();
//         tr_global = gp->getPtrGlobalTranslation();
        
//         weights = gp->getWeights();
//         edges_pairs = gp->getEdgesPairs();
//         
//         DOTWriter dotw("graph");
//         char buf[256];
//         char *file_dot = &buf[0];
//         sprintf(file_dot, "figs/%s.dot", base_filename.c_str());
//         dotw.exportDOT( file_dot.c_str(), &edges_pairs );
//         dotw.exportDOT( file_dot, &edges_pairs, NULL, &weights );
//         
//         std::string graph_file1, graph_file2;
//         graph_file1 = graph_file2 = base_filename;
//         graph_file1.append("_gp->graph");
//         graph_file2.append("_ex.graph");
//         gp->exportGRAPH( graph_file1.c_str() );
//         exportGRAPH( graph_file2.c_str(), gp->Qn_global, gp->tr_global );
//         
//     }
    
    boost::shared_ptr< FeaturesMap > featM (new FeaturesMap());
    featM->solveVisibility3D( &mydb );
    printf("Visibility Matrix [%d x %d]\n",featM->getVisibility()->rows(),featM->getVisibility()->cols());
    num_cameras = featM->getNumberCameras();
    num_features = featM->getNumberFeatures();
    mydb.closeDB();
    
    boost::shared_ptr< SimpleRegistration > sr01( new SimpleRegistration( num_cameras, num_features, K ) );
    sr01->setFallBackICPOn( &imageList_rgb, &imageList_depth, num_depth_filter );
    timer1.start();
    sr01->solvePose3D( featM->getVisibility(), featM->getCoordinates3D(), true ); // true for optimal
    std::cout << "Elapsed time to solve Pose: " << timer1.elapsed_s() << " [s]\n";
    Qn_global = sr01->getPtrGlobalQuaternion();
    tr_global = sr01->getPtrGlobalTranslation();
//     
//     // Print lin and opt
//     SimpleRegistration sr02( num_cameras, num_features, K );
//     timer1.start();
//     sr02.solvePose3D( &featM->visibility, &featM->coordinates3D, false ); // true for optimal
//     std::cout << "Elapsed time to solve Pose: " << timer1.elapsed_s() << " [s]\n";
    
    // Test one merge. Use to show in blue the proximity
//     boost::shared_ptr< Eigen::MatrixXd > Cov1, Cov2, Cov_New;
//     computeCovariance( set_cloud[0], K, Cov1 );
//     computeCovariance( set_cloud[1], K, Cov2 );    
//     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_join;
//     mergeClouds( set_cloud[0], set_cloud[1], Cov1, Cov2, cloud_join, Cov_New );
    
    // TODO Remember to free the memory here, images, Features, etc
    
    
    // RUN GLOBAL Optimization
    GlobalPose3D global01;
    global01.solve(featM->getVisibility(), featM->getCoordinates3D(), &K, Qn_global, tr_global );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudE;
    eigen2pointcloud( global01.Structure, cloudE );
    
    setCoordinatestoOrigin(*Qn_global, *tr_global);
//     Eigen::Quaternion<double> q_desired( 0.92388, 0.0, -0.38268, 0.0 ); // 45 degrees y axis
//     Eigen::Vector3d t_desired(-0.70711, 0.0, -0.70711 ); // center = [1,0,0]
//     setCoordinatestoDesiredPosition( gp->Qn_global, gp->tr_global, q_desired, t_desired, 4 ); // move camera 5 to desired point
    
    // Load ground truth
//     Qd_vector ground_qn_global;
//     V3d_vector ground_tr_global;
//     importTXTQuaternionVector( "freiburg1_room_ground_rot.txt", ground_qn_global );
//     importTXTTranslationVector( "freiburg1_room_ground_tr.txt", ground_tr_global );
// //     importTXTQuaternionVector( "freiburg3_ground_rot.txt", ground_qn_global );
// //     importTXTTranslationVector( "freiburg3_ground_tr.txt", ground_tr_global );
//     setCoordinatestoOrigin(ground_qn_global, ground_tr_global);
    
    // Write files of global quaternion and translation
    if ( save_txt )
    {
        std::string txt_file1, txt_file2;
        txt_file1 = txt_file2 = base_filename;
        txt_file1.append("_rot");
        txt_file2.append("_tr");
        
//         exportTXTQuaternionVector( (txt_file1 + "_gp->txt").c_str(), *gp->getPtrGlobalQuaternion() );
//         exportTXTTranslationVector( (txt_file2 + "_gp->txt").c_str(), *gp->getPtrGlobalTranslation() );
//         exportTXTQuaternionVector( (txt_file1 + "_sr_opt.txt").c_str(), *sr01->getPtrGlobalQuaternion() );
//         exportTXTTranslationVector( (txt_file2 + "_sr_opt.txt").c_str(), *sr01->getPtrGlobalTranslation() );
//         exportTXTQuaternionVector( (txt_file1 + "_sr_lin.txt").c_str(), *sr02.getPtrGlobalQuaternion() );
//         exportTXTTranslationVector( (txt_file2 + "_sr_lin.txt").c_str(), *sr02.getPtrGlobalTranslation() );
        exportTXTQuaternionVector( (txt_file1 + "_gl_opt.txt").c_str(), *Qn_global);
        exportTXTTranslationVector( (txt_file2 + "_gl_opt.txt").c_str(), *tr_global);
    }
    
    // Free memory
    myfeat.reset();
    my_mmap.reset();
    featM.reset();
    
    // Model
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_join;
    std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > set_cloud;
    std::vector< boost::shared_ptr< Eigen::MatrixXd > > set_covariance;
    cv2PointCloudSet(imageList_rgb, imageList_depth, K, *Qn_global, *tr_global, set_cloud, set_covariance);
//     mergeCloudSet( set_cloud, set_covariance, cloud_join );
//     set2unique( set_cloud, cloud_join );
    
//     MergeClouds mrg( &imageList_rgb, &imageList_depth, K);
//     mrg.mergeSet( *Qn_global, *tr_global );
//     cloud_join = mrg.getCloud();
    
    // ============================================ Visualization ============================================
//     Qd_vector qnl = *Qn_global;
//     V3d_vector tnl = *tr_global;
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    
//     std::cout << "set cloud size:" << set_cloud.size() << "\n";
    // Visualize Cloud
    viewer = visualizeCloudSet( set_cloud );
//     viewer = visualizeCloud(cloudE);
//     viewer = visualizeCloud(set_cloud[0]);
//     viewer = visualizeCloud(cloud_join);
//     viewer->setBackgroundColor (1.0, 1.0, 1.0);
    
    // Visualize Camera positions
//     visualizeCameras( viewer, *sr01->getPtrGlobalQuaternion(), *sr01->getPtrGlobalTranslation() );
//     visualizeCameras( viewer, *sr02.getPtrGlobalQuaternion(), *sr02.getPtrGlobalTranslation() );
//     visualizeCameras( viewer, *gp->getPtrGlobalQuaternion(), *gp->getPtrGlobalTranslation() );
    visualizeCameras( viewer, *Qn_global, *tr_global );

//     visualizeCameras(viewer, imageList_rgb, *Qn_global, *tr_global );
    
    // Other visualizations
    visualizeTrack( viewer, *Qn_global, *tr_global );
//     visualizeTrack( viewer, *gp->getPtrGlobalQuaternion(), *gp->getPtrGlobalTranslation() );
//     visualizeTrack( viewer, *sr01->getPtrGlobalQuaternion(), *sr01->getPtrGlobalTranslation() );
//     visualizeTrack( viewer, ground_qn_global, ground_tr_global, Eigen::Vector3d( 0.9, 0.0, 0.0 ) );
    
//     visualizeNoise(viewer, *sr01->getPtrXmodel(), *sr01->getPtrVariance(), *sr01->getPtrGlobalQuaternion(), *sr01->getPtrGlobalTranslation(), 500 );
    
//     visualizeGraph( num_vertex, num_edges, weights, edges_pairs );
    
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    
    if (save_ply)
    {
        pcl::PLYWriter wr_ply;
        wr_ply.write(outputFilename, *cloud_join);
    }
    
    // ========================================== END Visualization ==========================================
    return EXIT_SUCCESS;
    exit(0);
}