// Created: Oct/06/2013
// Author: José David Tascón Vidarte

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// PCL Libraries
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
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
#include "Common.hpp"
#include "FeaturesMap.hpp"
#include "HandleDB.hpp"
#include "Interface.hpp"
#include "InterfacePCL.hpp"
#include "FeaturesEDM.hpp"
#include "Registration.hpp"
#include "Optimizer.hpp"
#include "Plot.hpp"

// using namespace cv;
// using namespace std;
// using namespace Eigen;

//**
// * ******************************************************************
// * @function help
// */
void help()
{ 
	std::cout << "\033[1;33m Usage: ./test_cloud -i <rgb_list.xml> -d <depth_list.xml> -o <output.txt>\033[0m\n"
				"\tOptions:\n" 
				"\t[-i]\t XML input file name with rgb images\n" 
				"\t[-d]\t XML input file name with depth images\n" 
				"\t[-v]\t Enable verbose mode\n"
				"\t[-m]\t Enable to show images matches\n"
				"\t[-o]\t Output text filename\n"
				"\t[-k]\t .TXT input file name with calibration matrix\n"
				"\t[-db]\t .DB input file name with database\n"
	<< std::endl;
	
}

// ================================================= MAIN ==============================================
int main(int argc, char* argv[])
{
    // ========================================== Parameters ===========================================
    bool ram_db = true;
    bool verbose = false;
    bool drawM = false;
    bool load_k = false;
    bool save_ply = false;
    const char* filename_calib;
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
	  else if( strcmp( s, "-db" ) == 0 )
	  {
	      i++;
	      filename_db = argv[i];
	      ram_db = false;
	  }
	  else if( strcmp( s, "-k" ) == 0 )
	  {
	      i++;
	      filename_calib = argv[i];
	      load_k = true;
	  }
	  else if( strcmp( s, "-v" ) == 0 )
	  {
	      verbose = true;
	  }
	  else if( strcmp( s, "-m" ) == 0 )
	  {
	      drawM = true;
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
    std::vector< cv::Mat > images, images_depth;//, images_matches;

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
    for (int i = 0; i < (int)imageList_rgb.size(); i++ )	//Store all images in std::vector<cv::Mat>
    {
        images.push_back( cv::imread(imageList_rgb[i], 1) );
        images_depth.push_back( cv::imread(imageList_depth[i], -1) );
    }
    
    if (verbose)
    {
        std::cout << "[OK]\n";
        std::cout << "Number of Images: " << images.size() << "\n";
        printf("Size of Images: \tWidth: %i\t||\t Height: %i\n", images[0].size().width, images[0].size().height);
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
        double wimg = images[0].size().width - 1.0;
        double himg = images[0].size().height - 1.0;
        double f = 520.0;
        K << f, 0.0, wimg/2, 0.0, f, himg/2, 0.0, 0.0, 1.0;		// Camera Matrix (intrinsics)
    }
    if (verbose) std::cout << "\nCalibration Matrix:\n" << K << "\n";
    
    // ====================================================================================================
    // ========================================== Start execution =========================================
    // ====================================================================================================
    
    HandleDB mydb( (char*)filename_db.c_str() );
    mydb.openDB();
    if ( ram_db )						// If database is in ram, create table and index
    {
        mydb.createFeaturesTable();
        mydb.createIndex1();
    }
    if (!mydb.isOpen()) exit(0);
    
    if ( ram_db )						// If database is in ram, solve features map and store in db
    {
        SiftED myfeat(imageList_rgb);
        myfeat.solveSift();
    //     myfeat.loadImages();
        if( drawM ) myfeat.enableKeyPoint();
        
        MatchesMap my_mmap(400,35);
        my_mmap.solveMatches(&myfeat.descriptorsGPU);
        my_mmap.robustifyMatches(&myfeat.keypointsGPU);
        my_mmap.depthFilter(&myfeat.keypointsGPU, &images_depth, 3);
        timer1.start();
        my_mmap.solveDB( &mydb, &myfeat.keypointsGPU );
        std::cout << "Elapsed time to solve DB: " << timer1.elapsed_s() << " [s]\n";
    //     my_mmap.txt((char*)"./match.txt", &myfeat.keypointsGPU);
        if( drawM ) my_mmap.plot( &images, &myfeat.set_of_keypoints );
    }
    
    FeaturesMap featM;
    featM.solveVisibility( &mydb );
//     std::cout << "Visibility =\n" << featM.visibility << "\n";
    printf("Visibility Matrix [%d x %d]\n",featM.visibility.rows(),featM.visibility.cols());
//     featM.txt((char*)"./visibility.txt");
    num_cameras = featM.cameras();
    num_features = featM.features();
    mydb.closeDB();
    
    // ============================================ Registration ============================================
    
    SimpleRegistration sr01( num_cameras, num_features, K );
    timer1.start();
    sr01.solvePose( &featM.visibility, &featM.coordinates, imageList_depth );
    std::cout << "Elapsed time to solve Pose: " << timer1.elapsed_s() << " [s]\n";
//     sr01.solvePose( &my_mmap.globalMatch, &myfeat.set_of_keypoints, &images_depth );
//     std::cout << "Recover structure matrix:\n" << sr01.Structure.transpose() << "\n";

    std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > set_cloud;
    cv2PointCloudSet(images, images_depth, K, sr01.Qn_global, sr01.tr_global, set_cloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model;
    set2unique( set_cloud, model );
    
    // ========================================== END Registration ==========================================

    
    
    // ============================================ Visualization ============================================
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = visualizeCloud(model);
    
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    
    // To save as ply apply his rotation to the points
    if (save_ply)
    {
        pcl::PLYWriter wr_ply;
        wr_ply.write(outputFilename, *model);
    }
//     pcl::io::savePCDFileASCII ("gin_std.pcd", model);
//     pcl::io::savePLYFileASCI("gin_std.ply", model);
    
    // ========================================== END Visualization ==========================================

    exit(0);
}