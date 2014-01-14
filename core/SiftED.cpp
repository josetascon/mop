// Created: Aug/02/2013
// File: Jan/13/2014
// Author: José David Tascón Vidarte

#include "SiftED.hpp"

// ================================================================================================
// ================================= FUNCTIONS of CLASS SiftED ====================================
// ================================================================================================
SiftED::SiftED( std::vector< std::string > filenameIms )
{
    nameImages = filenameIms;
    num_images = nameImages.size();
    descriptorsGPU.resize(num_images);
    keypointsGPU.resize(num_images);
    
    set_of_images.resize(num_images);
    set_of_keypoints.resize(num_images);
    set_of_descriptors.resize(num_images);
};

SiftED::~SiftED()
{
//     nameImages.clear();
};   

void SiftED::loadImages()
{
    set_of_images.clear();
    for (int i = 0; i < nameImages.size(); i++ )	//Store all images in std::vector<cv::Mat>
    {
        set_of_images.push_back( cv::imread(nameImages[i], 1) );
    }
}

cv::Mat SiftED::image(int num)
{
    return set_of_images[num];
}

std::vector<cv::KeyPoint> SiftED::KeyPoint(int num)
{
    return set_of_keypoints[num];
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
    
    for (int k = 0; k < num_images; k++) files[k] = nameImages[k].c_str();
    
    sift.SetImageList(num_images, files);
    std::cout << "\n================================ SIFT Features E&D ==================================\n";
    
    for (register int k = 0; k < num_images; ++k)
    {
        sift.RunSIFT(k); //process an image
        ft_per_im[k] = sift.GetFeatureNum(); //get feature count
        keypointsGPU[k].resize(ft_per_im[k]);
        descriptorsGPU[k].resize(128*ft_per_im[k]);
        
        sift.GetFeatureVector(&keypointsGPU[k][0], &descriptorsGPU[k][0]); //specify NULL if you don’t need keypoints or descriptors
        printf("SIFT: Image %04i, \t#features = %i\n", k, ft_per_im[k]);
//         std::string str = "SIFT: %04i,";
//         str.append(nameImages[k]);
//         str.append("\t #f = %i\n");
//         printf(str.c_str(),k,ft_per_im[k]);
    }
};

void SiftED::enableKeyPoint()
{
    for (register int k = 0; k < num_images; ++k)
    {
        int ft_per_im = keypointsGPU[k].size();
        set_of_keypoints[k].resize(ft_per_im);
        for (int i = 0; i < ft_per_im ; i++)
        {
	  set_of_keypoints[k][i] = cv::KeyPoint( cv::Point2f(keypointsGPU[k][i].x, keypointsGPU[k][i].y),
			  keypointsGPU[k][i].s, keypointsGPU[k][i].o );
        }
    }
}

// std::vector< std::vector<float> > SiftED::get_descriptorsGPU()
// {
//     return descriptorsGPU;
// }
//     
// std::vector< std::vector<SiftGPU::SiftKeypoint> > SiftED::get_keypointsGPU()
// {
//     return keypointsGPU;
// }