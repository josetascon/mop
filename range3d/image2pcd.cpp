// Feb/05/2013
// Author: José David Tascón Vidarte

// libc
#include <iostream>
#include <libgen.h>
#include <cmath>

// OpenCV Libraries
#include <opencv2/opencv.hpp>

// PCL Libraries
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

// Local
#include "Interface.hpp"
#include "InterfacePCL.hpp"


void help(char* arg0)
{ 
    std::cout << "\033[1;33m Usage: " << arg0 << " -i <rgb image> <depth image> -o <pcd output> \033[0m\n"
	  "\tOptions:\n" 
	  "\t[-i]\t Filename of rgb and depth images\n"
	  "\t[-o]\t Filename of output pcd\n"
	  "\t[-k]\t .TXT input file name with calibration matrix\n"
	  "\t[-d]\t Debug information\n"
    << std::endl;
}

int main(int argc, char* argv[])
{
    const char* filename_rgb;
    const char* filename_depth;
    const char* filename_calib;
    char* filename_output = (char*)"out.pcd";
    bool outname = false;
    bool load_k = false;
    bool debug = false;
    if( argc < 2 )
    {
        help(argv[0]);
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
	      filename_rgb = argv[i];
	      i++;
	      filename_depth = argv[i];
	  }
	  else if( strcmp( s, "-o" ) == 0 )
	  {
	      i++;
	      filename_output = argv[i];
	      outname = true;
	  }
	  else if( strcmp( s, "-d" ) == 0 )
	  {
	      debug = true;
	  }
	  else if( strcmp( s, "-k" ) == 0 )
	  {
	      i++;
	      filename_calib = argv[i];
	      load_k = true;
	  }
	  else
	  {
	      help(argv[0]);
	      return fprintf( stderr, "Unknown option %s\n", s ), -1;
	  }
        }
    }
    
    cv::Mat image = cv::imread( filename_rgb, -1 );
    cv::Mat depth = cv::imread( filename_depth, -1 );
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    Eigen::Matrix3d K;
    if (load_k)						// If Calibration matrix is taken from txt file
    {
        Eigen::MatrixXd K_load;
        importTXTEigen(filename_calib, K_load);
        K << K_load;
    }
    else
    {
        double wimg = image.size().width - 1.0;
        double himg = image.size().height - 1.0;
        double f = 520.0;
        K << f, 0.0, wimg/2, 0.0, f, himg/2, 0.0, 0.0, 1.0;		// Camera Matrix (intrinsics)
    }
    
    std::string file_out;
    if( !outname )
    {
        file_out = basename((char*)filename_rgb);
        file_out = file_out.substr(0, file_out.rfind("."));
        file_out.append(".pcd");
    }
    else file_out = filename_output;
    
    
    std::cout << "\nGenerate pcd file " << file_out << "\t...\n";
    
    cv2PointCloudDense(image, depth, K, cloud);
    
    pcl::PCDWriter wr_pcd;
    wr_pcd.write(file_out.c_str(), *cloud);
    std::cout << "Done. End Program\n\n";
    
    return 0;
}