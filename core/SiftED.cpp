// Created: Aug/02/2013
// File: Jan/13/2014
// Author: José David Tascón Vidarte

#include "SiftED.hpp"

// ================================================================================================
// ================================= FUNCTIONS of CLASS SiftED ====================================
// ================================================================================================
void SiftED::loadImages()
{
//     set_of_images = boost::shared_ptr< std::vector< cv::Mat > >( new std::vector< cv::Mat >());
    set_of_images->clear();
    for (int i = 0; i < nameImages->size(); i++ )	//Store all images in std::vector<cv::Mat>
    {
        set_of_images->push_back( cv::imread(nameImages->at(i), 1) );
    }
    load_images = true;
}

cv::Mat SiftED::getImage(int num)
{
    if( load_images ) return set_of_images->at(num);
    else
    {
        DEBUG_E( ("Images are not loaded yet. Try loadImages() first") ); 
        exit(-1);
    }
}

std::vector<cv::KeyPoint> SiftED::getKeyPoint(int num)
{
    if( keypoint_available ) return set_of_keypoints->at(num);
    else
    {
        DEBUG_E( ("Keypoints unavailable. Try enableKeyPoint() first") ); 
        exit(-1);
    }
}

void SiftED::solveSift()
{
    int ft_per_im[num_images];
    const char *files[num_images];
//     char *pfiles = &files[0];
    
    //create a SiftGPU instance
    SiftGPU sift;
    //processing parameters first
//     char * s_argv[] = {(char*)"-fo", (char*)"-1", (char*)"-v", (char*)"0"};
//     sift.ParseParam(4, s_argv);
    char * s_argv[] = {(char*)"-fo", (char*)"-1", (char*)"-v", (char*)"0", (char*)"-tc2", (char*)"1000" }; // tc2 limit max features
    sift.ParseParam(6, s_argv);
    
    //create an OpenGL context for computation
    int support = sift.CreateContextGL();
//     int argc = 1;
//     char **argv;
//     glutInit(&argc, argv);
//     glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
//     glutInitWindowSize(640, 480);
//     glutCreateWindow("Sift");
    
    
    //call VerfifyContexGL instead if using your own GL context
//     int support = sift.VerifyContextGL();
    if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED)
    {
        fprintf(stderr, "Can't open OpenGL context for SiftGPU\n");
        return;
    }
    
    for (int k = 0; k < num_images; k++) files[k] = nameImages->at(k).c_str();
    
    sift.SetImageList(num_images, files);
    DEBUG_1( std::cout << "\n================================ SIFT Features E&D ==================================\n"; )
    
    for (register int k = 0; k < num_images; ++k)
    {
        sift.RunSIFT(k); //process an image
        ft_per_im[k] = sift.GetFeatureNum(); //get feature count
        keypointsGPU->at(k).resize(ft_per_im[k]);
        descriptorsGPU->at(k).resize(128*ft_per_im[k]);
        
        sift.GetFeatureVector(&keypointsGPU->at(k)[0], &descriptorsGPU->at(k)[0]); //specify NULL if you don’t need keypoints or descriptors
//         DEBUG_1( printf("SIFT: Image %04i, \t#features = %i\n", k, ft_per_im[k]); )
        DEBUG_1( std::string str = "SIFT: Image %04i, ";
	  str.append( basename(strdup(nameImages->at(k).c_str())) );
	  str.append("\t #f = %i\n");
	  printf(str.c_str(),k,ft_per_im[k]);
        )
    }
};

void SiftED::enableKeyPoint()
{
//     set_of_keypoints = boost::shared_ptr< kpCV_vv >( new kpCV_vv());
    for (register int k = 0; k < num_images; ++k)
    {
        int ft_per_im = keypointsGPU->at(k).size();
        set_of_keypoints->at(k).resize(ft_per_im);
        for (int i = 0; i < ft_per_im ; i++)
        {
	  set_of_keypoints->at(k)[i] = cv::KeyPoint( cv::Point2f(keypointsGPU->at(k)[i].x, keypointsGPU->at(k)[i].y),
			  keypointsGPU->at(k)[i].s, keypointsGPU->at(k)[i].o );
        }
    }
    keypoint_available = true;
}

