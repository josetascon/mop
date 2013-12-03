// Created: Oct/03/2013
// Author: José David Tascón Vidarte

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// PCL Libraries
#include <pcl/io/ply_io.h>

// Std Libraries
#include <iostream>
#include <string>

// Classes
#include "Common.hpp"
#include "Interface.hpp"
#include "InterfacePCL.hpp"
#include "Optimizer.hpp"
#include "Plot.hpp"

#define pi 3.14159265358979323846

// using namespace cv;
// using namespace std;
// using namespace Eigen;

//**
// * ******************************************************************
// * @function help
// */
void help()
{
    std::cout << "\033[1;33m Usage: ./bal -f <filename.txt> \033[0m\n"//-o <output.txt>\n"
	      "\tOptions:\n" 
	      "\t[-f]\t .TXT input file name with bal problem\n"
	      "\t[-v]\t Enable verbose mode\n"
	      "\t[-s]\t Enable visualization\n"
	      "\t[-o]\t Output ply filename\n"
	<< std::endl;
}

// ================================================= MAIN ==============================================
int main(int argc, char* argv[])
{
    // ========================================== Parameters ===========================================
    bool verbose = false;
    bool visualize = false;
    bool output = false;
    const char* input_filename;
    std::string output_filename = "output.ply";
    
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
	  if( strcmp( s, "-f" ) == 0 )
	  {
	      i++;
	      input_filename = argv[i];
	  }
	  else if( strcmp( s, "-s" ) == 0 )
	  {
	      visualize = true;
	  }
	  else if( strcmp( s, "-v" ) == 0 )
	  {
	      verbose = true;
	  }
	  else if( strcmp( s, "-o" ) == 0 )
	  {
	      i++;
	      output_filename = argv[i];
	      output = true;
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
    
    BALProblem bal(input_filename);
    
    Eigen::Matrix<bool,-1,-1> visibility;
    Eigen::Matrix<Eigen::Vector3d,-1,-1> coordinates;
    std::vector< Eigen::Quaternion<double> > quaternion;
    Eigen::MatrixXd tr_and_int;
    Eigen::MatrixXd structure;
    
    // ====================================================================================================
    // ========================================== Start execution =========================================
    // ====================================================================================================

    // ========================================== Read Files ==========================================
    if (verbose)
    {
        std::cout << "[OK]" << '\n';		// DATA initialization OK
        std::cout << "Reading File:\t" << input_filename << "\n";
        std::cout << "File data...\t\t\t";
    }
    
    bal.read(visibility,coordinates,quaternion,tr_and_int,structure); 
    
    if (verbose) std::cout << "[OK]\n";
    
    // ========================================== Optimization ==========================================
    
    std::cout << "Start Optimization Problem\n";
    
    timer1.start();
    BALOptimizer opt01(&visibility,&coordinates,&quaternion,&tr_and_int,&structure);
    opt01.runBAL();
    std::cout << "Optimization with BA time: "<< timer1.elapsed_s() << " [s]\n"; 
    
    std::vector< Eigen::Matrix3d > Rotation(quaternion.size());
    std::vector< Eigen::Vector3d > translation(quaternion.size());
    for (register int i = 0; i < Rotation.size(); ++i)
    {
        Rotation[i] = quaternion[i].toRotationMatrix();
        Eigen::MatrixXd tr_temp = tr_and_int.col(i);
        translation[i] = Eigen::Vector3d(tr_temp(0), tr_temp(1), tr_temp(2));
    }
    
    // ========================================== END Optimization ==========================================
    if( output )
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr model;
        eigen2PointCloud(structure, model);
    
        pcl::PLYWriter wr_ply;
        wr_ply.write(output_filename.c_str(), *model);
        std::cout << "Save model as PLY file\n";
    }
    
    // Plot for BAL structure in one vector (Global)
    std::vector< std::vector<cv::Point3d> > WP;
    std::vector<cv::Point3d> pts;
    convertEigentoPoint3_( structure , pts );
    WP.push_back(pts);
    
    // sfm data
    PlotGL::viewer.setRotation( &Rotation );
    PlotGL::viewer.setTranslation( &translation );
    PlotGL::viewer.setStructure( &WP );
    
    PlotGL::viewer.run(argc, argv, true); // false);
    
    exit(0);
}