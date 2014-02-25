// Created: May/15/2013
// Author: José David Tascón Vidarte

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// OpenCV Libraries
#include <opencv2/opencv.hpp>

// Std Libraries
#include <iostream>
#include <string>
#include <vector>

// Local libraries
#include "Common.hpp"
#include "Interface.hpp"
#include "FeaturesMap.hpp"
#include "FeaturesEDM.hpp"
#include "MultipleCamera.hpp"
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
    std::cout << "\033[1;33m Usage: ./test_initBA -i <rgb_list.xml> \033[0m\n"//-o <output.txt>\n"
	      "\tOptions:\n" 
	      "\t[-i]\t .XML input file name with rgb images\n"
	      "\t[-k]\t .TXT input file name with calibration matrix\n"
	      "\t[-db]\t .DB input file name with database\n"
	      "\t[-v]\t Enable verbose mode\n"
	      "\t[-m]\t Enable to show images matches\n"
// 		"\t[-o]\t Output text filename\n"
	<< std::endl;
}

// ================================================= MAIN ==============================================
int main(int argc, char* argv[])
{
    // ========================================== Parameters ===========================================
    bool verbose = false;
    bool drawM = false;
    bool ram_db = true;
    bool load_k = false;
    std::string filename_db = ":memory:";
    const char* filename_calib;
    const char* inputFilename_rgb;
    const char* outputFilename;
    
    std::vector< std::string > imageList_rgb;
    
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
	  else if( strcmp( s, "-o" ) == 0 )
	  {
	      i++;
	      outputFilename = argv[i];
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
    if (verbose) cout << "\nParameters read...\t\t[OK]" << '\n';
    if (verbose) cout << "Data init...\t\t\t";
    
    // ========================================== Varible declaration ==========================================
    
    int num_features;
    int num_cameras;
    timer_wall timer1;
    std::vector< cv::Mat > images, images_depth;//, images_matches;

    // ====================================================================================================
    // ========================================== Start execution =========================================
    // ====================================================================================================

    
    // ========================================== Read Images ==========================================
    if (verbose)
    {
        std::cout << "[OK]" << '\n';		// DATA initialization OK
        std::cout << "Reading Files:\t" << inputFilename_rgb << "\n";
        std::cout << "File list...\t\t\t";
    }
    importXMLImageList(inputFilename_rgb, imageList_rgb);		//Reading list of images from XML list
    
    if (verbose)
    {
        std::cout << "[OK]\n";
        std::cout << "Loading Images...\t\t";
    }
    for (int i = 0; i < (int)imageList_rgb.size(); i++ )	//Store all images in std::vector<cv::Mat>
    {
        images.push_back( cv::imread(imageList_rgb[i], 1) );
    }
    
    if (verbose)
    {
        std::cout << "[OK]\n";
        std::cout << "Number of Images: " << images.size() << "\n";
        printf("Size of Images: \tWidth: %i  ||  Height: %i\n", images[0].size().width, images[0].size().height);
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
        double wimg = images[0].size().width;
        double himg = images[0].size().height;
        double f = 1000.0;
        K << f, 0.0, wimg/2, 0.0, f, himg/2, 0.0, 0.0, 1.0;		// Camera Matrix (intrinsics)
    }
    if (verbose) std::cout << "\nCalibration Matrix:\n" << K << "\n";
    
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
        boost::shared_ptr< SiftED > myfeat( new SiftED(&imageList_rgb) );
        myfeat->solveSift();
//         myfeat->loadImages();
        if( drawM ) myfeat->enableKeyPoint();
        
        boost::shared_ptr< MatchesMap > my_mmap( new MatchesMap(500,35) );
        my_mmap->solveMatches(myfeat->getDescriptorsGPU());
        my_mmap->robustifyMatches(myfeat->getKeypointsGPU());
        timer1.start();
        my_mmap->solveDB( &mydb, myfeat->getKeypointsGPU() );
        std::cout << "Elapsed time to solve DB: " << timer1.elapsed_s() << " [s]\n";
//         my_mmap.txt((char*)"./match.txt", myfeat->getKeypointsGPU());
        if( drawM ) my_mmap->plot( &images, myfeat->getKeypointsSet() );
    }
    
    boost::shared_ptr< FeaturesMap > featM (new FeaturesMap());
    featM->solveVisibility( &mydb );
    printf("Visibility Matrix [%d x %d]\n",featM->getVisibility()->rows(),featM->getVisibility()->cols());
    num_cameras = featM->getNumberCameras();
    num_features = featM->getNumberFeatures();
    mydb.closeDB();
    
    
//     SfM sfm01( num_cameras, num_features, K );
// //     sfm01.solvePose( &my_mmap.globalMatch, &myfeat.set_of_keypoints);
//     sfm01.solvePose( (featM->getVisibility()).get(), (featM->getCoordinates()).get() );
//     sfm01.solveStructure( (featM->getVisibility()).get(), (featM->getCoordinates()).get() );
        
//     std::cout << "Structure =\n" << sfm01.Structure.transpose() << "\n";
    mydb.closeDB();
    
    // ========================================== Optimization ==========================================
    
    double intrinsics[4] = { K(0,0), K(1,1), K(0,2), K(1,2) };
    std::vector< double > intrinsics_param(&intrinsics[0], &intrinsics[4]);
    double distcoeff[5] = {2.5552679187075661e-01, -5.8740292343503686e-01, -3.0863014649845459e-04, 1.9066445284294834e-03, 5.1108649981093257e-01};
    std::vector< double > coefficients(&distcoeff[0], &distcoeff[5]);
//     std::vector< double > coefficients(5,0.0);/// active for dinosaur
    /*
    GlobalOptimizer opt01;
    opt01.setParameters( (featM->getVisibility()).get(), (featM->getCoordinates()).get(), &sfm01.Quat_cumulative, &sfm01.tr_global, &sfm01.Structure );
    opt01.setIntrinsics( &intrinsics_param );
    opt01.setDistortion( &coefficients );
    opt01.runBA();// argv[0] );// bundle adjustment to all data
    
    // FINAL DATA PRINTING ***************************
    sfm01.updateCamera();
    
//     std::cout << "Recover structure matrix:\n" << sfm01.Structure.transpose() << "\n";    
    writePMVS("./pmvs", imageList_rgb, sfm01.Cameras_RCV, K, coefficients);
    */
    timer1.start();
    IncrementalBA opt01( (featM->getVisibility()).get(), (featM->getCoordinates()).get() );
    opt01.setIntrinsics( &intrinsics_param );
    opt01.setDistortion( &coefficients );
    opt01.runC();
    
    GlobalOptimizer opt03;
    opt03.setParameters( (featM->getVisibility()).get(), (featM->getCoordinates()).get(), &opt01.quaternion, &opt01.translation, &opt01.structure );
    opt03.setIntrinsics( &intrinsics_param );
    opt03.setDistortion( &coefficients );
    opt03.runBA();// argv[0] );// bundle adjustment to all data
    std::cout << "Incrementel BA time: "<< timer1.elapsed_s() << " [s]\n"; 
    opt01.updateCamera();
    
    std::vector< Eigen::Matrix3d > Rotation(opt01.quaternion.size());
    for (register int i = 0; i < Rotation.size(); ++i) Rotation[i] = opt01.quaternion[i].toRotationMatrix();
    exportPMVS("./pmvs", imageList_rgb, opt01.Camera, K, coefficients);
    
    // ========================================== END Optimization ==========================================
    
    // plot TESTING
//     int max_WP = sfm01.plotSt.maxCoeff();
//     std::cout << "maximum in plot WP = " << max_WP << "\n";
//     std::cout << "plotSt = " << sfm01.plotSt.transpose() << "\n";
//     std::vector< std::vector<cv::Point3d> > WP(max_WP+1);
//     std::vector<cv::Point3d> x3plot;
//     for (int cam = 0; cam < max_WP+1; cam++)
//     {
//         x3plot.clear();
//         for (int ft = 0; ft < num_features; ft++)
//         {
// 	  if (cam == sfm01.plotSt(ft))
// 	  {
// 	      x3plot.push_back( cv::Point3d(sfm01.Structure(0,ft),sfm01.Structure(1,ft),sfm01.Structure(2,ft)) );
// 	  }
//         }
//         std::cout << "x3plot size = " << x3plot.size() << "\n";
//         std::cout << "cam = " << cam << "\n";
//         WP[cam] = x3plot;
//     }
    // end plot TESTING
    
    // Plot for IncrementalBA structure in one vector (Global)
    std::vector< std::vector<cv::Point3d> > WP;
    std::vector<cv::Point3d> pts;
    eigen2point3_vector( opt01.structure , pts );
    WP.push_back(pts);
    
    // sfm data
    PlotGL::viewer.setRotation( &Rotation );
    PlotGL::viewer.setTranslation( &opt01.translation );
    PlotGL::viewer.setStructure( &WP );
//     PlotGL::viewer.setColor( &Color );
    
    
    // PLOT for sfm object structure in one vector (Global) 
//     std::vector< std::vector<cv::Point3d> > WP;
//     std::vector<cv::Point3d> pts;
//     eigen2point3_vector( sfm01.Structure , pts );
//     WP.push_back(pts);
//     
//     // sfm data
//     PlotGL::viewer.setRotation( &sfm01.Rot_global );
//     PlotGL::viewer.setTranslation( &sfm01.tr_global );
//     PlotGL::viewer.setStructure( &WP );
// //     PlotGL::viewer.setColor( &Color );
    
    PlotGL::viewer.run(argc, argv, !ram_db); // false);
    
    exit(0);
}