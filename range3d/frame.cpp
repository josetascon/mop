// Created: Jan/15/2013
// Author: José David Tascón Vidarte

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// OpenCV Libraries
#include <opencv2/opencv.hpp>

// Std Libraries
#include <iostream>
#include <string>
#include <vector>
#include <libgen.h>

// Local libraries
#include "DepthProjection.hpp"
#include "Common.hpp"
#include "Interface.hpp"
#include "FeaturesEDM.hpp"
#include "FrameSelector.hpp"
#include "Registration.hpp"


// using namespace cv;
// using namespace std;
// using namespace Eigen;

/**
* ******************************************************************
* @function help
*/
void help()
{ 
	std::cout << "\033[1;33m Usage: ./frame -i <rgb_list.xml> -d <depth_list.xml> -o <output.txt>\033[0m\n"
				"\tOptions:\n" 
				"\t[-i]\t XML input file name with rgb images\n" 
				"\t[-d]\t XML input file name with depth images\n" 
				"\t[-v]\t Enable verbose mode\n"
				"\t[-k]\t .TXT input file name with calibration matrix\n"
				"\t[-df]\t Valid matches in Depth Filter\n"
				
	<< std::endl;
	
}

// ================================================= MAIN ==============================================
int main(int argc, char* argv[])
{
    // ========================================== Parameters ===========================================
    
    bool verbose = false;
    bool load_k = false;
    int num_depth_filter = 30;		// Minimal valid window + 40 matches
    const char* filename_calib;
    const char* inputFilename_rgb;
    const char* inputFilename_depth;
    
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
	  else if( strcmp( s, "-df" ) == 0 )
	  {
	      i++;
	      num_depth_filter = pchar2number<int>( argv[i] );
	  }
	  else
	  {
	      help();
	      return fprintf( stderr, "Unknown option %s\n", s ), -1;
	  }
        }
    }
    
    importXMLImageList(inputFilename_rgb, imageList_rgb);		//Reading list of images from XML list
    importXMLImageList(inputFilename_depth, imageList_depth);	//Reading list of depth images from XML list
    
    if (imageList_depth.size() != imageList_rgb.size())
    {
        DEBUG_E( ("Number of color and depth images files are different. Please check your xml files.") );
        exit(-1);
    }
    
    std::string base_filename = basename((char*)inputFilename_rgb);
    base_filename = base_filename.substr(0, base_filename.rfind("."));
    std::string file_out_rgb = "out_" + base_filename + "_rgb.xml";
    std::string file_out_depth = "out_" + base_filename + "_depth.xml";
    
    FrameSelector fm01(imageList_rgb, 40, num_depth_filter);
//     fm01.solve();
    fm01.solvewithDepth( imageList_depth );
    fm01.exportXML( file_out_rgb.c_str(), imageList_rgb );
    fm01.exportXML( file_out_depth.c_str(), imageList_depth );
    
    
    exit(0);
}
    