// Created: Nov/07/2013
// Author: José David Tascón Vidarte

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// Local Libraries
#include "ReconstructionModel.hpp"

// using namespace cv;
// using namespace std;
// using namespace Eigen;

/**
* ******************************************************************
* @function help
*/
void help(char* arg0)
{ 
	std::cout << "\033[1;33m Usage: " << arg0 << " -i <rgb_list.xml> -d <depth_list.xml> -o <output.txt>\033[0m\n"
				"\tOptions:\n" 
				"\t[-i]\t XML input file name with rgb images\n" 
				"\t[-d]\t XML input file name with depth images\n"
				"\t[-im]\t Multiple XML input file name with rgb images. E.g. -im 2 <file1.xml> <file2.xml>\n" 
				"\t[-dm]\t Multiple XML input file name with depth images. E.g. -im 2 <file1.xml> <file2.xml>\n"
				"\t[-op]\t Output ply filename\n"
				"\t[-ot]\t Output txt filename\n"
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
	  else if( strcmp( s, "-dm" ) == 0 )
	  {
	      i++;
	      int input = pchar2number<int>( argv[i] );
	      for(int k = 0; k < input; k++)
	      {
		i++;
		files_input_depth.push_back( argv[i] );
	      }
	      multiple = true;
	  }
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
	  else if( strcmp( s, "-h" ) == 0 )
	  {
	      help(argv[0]);
	      exit(0);
	  }
	  else
	  {
	      help(argv[0]);
	      return fprintf( stderr, "Unknown option %s\n", s ), -1;
	  }
        }
        
    }
    
    DEBUG_1( std::cout << "Parameters read...\t\t[OK]" << '\n';
	   std::cout << "Data init...\t\t\t"; )
    
    // ========================================== Varible declaration ==========================================
    int num_goodmatch = 35;
    int num_features;
    int num_cameras;
    std::string base_filename;
    std::vector<int> bound_d, bound_c;
    timer_wall timer1;
        
    // ========================================== Read Images ==========================================
    
    DEBUG_1( std::cout << "[OK]" << '\n'; 		// DATA initialization OK
	   if( !multiple) std::cout << "Reading Files:\t" << inputFilename_rgb << " & " << inputFilename_depth << '\n';
	   std::cout << "File list...\t\t\t"; )
    
    if ( multiple )
    {
        base_filename = basename( (char*)files_input_rgb[0].c_str() );
        base_filename = base_filename.substr(0, base_filename.rfind("."));
        importXMLMultipleImageList(files_input_rgb , imageList_rgb, bound_c );
        importXMLMultipleImageList(files_input_depth , imageList_depth, bound_d );
    }
    else
    {
        base_filename = basename((char*)inputFilename_rgb);
        base_filename = base_filename.substr(0, base_filename.rfind("."));
    
        std::string ext = "xml";
        verifyFileExtension( inputFilename_rgb, ext, true );
        verifyFileExtension( inputFilename_depth, ext, true );
    
        importXMLImageList(inputFilename_rgb, imageList_rgb);		//Reading list of images from XML list
        importXMLImageList(inputFilename_depth, imageList_depth);
    }
    
    if (imageList_depth.size() != imageList_rgb.size())
    {
        DEBUG_E( ("Number of color and depth images files are different. Please check your xml files.") );
        exit(-1);
    }
    
    DEBUG_1( std::cout << "[OK]\n";
	   std::cout << "Loading Images...\t\t";
	   cv::Mat image0 = cv::imread(imageList_rgb[0], 1);
	   std::cout << "[OK]\n";
	   std::cout << "Number of Images: " << imageList_rgb.size() << "\n";
	   printf("Size of Images: \tWidth: %i\t||\t Height: %i\n", image0.size().width, image0.size().height); )
   
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
    DEBUG_1( std::cout << "\nCalibration Matrix:\n" << K << "\n"; )
    
    // ====================================================================================================
    // ========================================== Start execution =========================================
    // ====================================================================================================
    
    // Model
    ReconstructionModel rc_model( &imageList_rgb, &imageList_depth, K, num_depth_filter );
    if (multiple) rc_model.setMatchSubgroup( &bound_c );
    rc_model.runGraph();
//     rc_model.runSimple();
    rc_model.freeMemoryFeatures();
    
    if (save_ply)
    {
        rc_model.solveUnifiedModel();
//         rc_model.visualizeUnifiedModel();
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_join = rc_model.getPtrCloudModel();
        pcl::PLYWriter wr_ply;
        wr_ply.write(outputFilename, *cloud_join);
    }
    else
    {
        rc_model.solveNonUnifiedModel();
        rc_model.visualizeNonUnifiedModel();
    }

    exit(0);
}