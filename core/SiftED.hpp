/**
 * @file SiftED.hpp
 * @brief This file has the class SiftED used for SIFT Features applied to datasets. It uses at its core SiftGPU.
 *
 * @author José David Tascón Vidarte
 * @date Aug/02/2013
 */

#ifndef __SIFTED_HPP__
#define __SIFTED_HPP__

// OpenGl Libraries
#include <GL/glut.h>			// GL constants for SiftGPU function

// OpenCV Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

// Std Libraries
#include <iostream>

// SiftGPU Library
#include <SiftGPU.h>

// ================================================================================================
// ========================================== CLASS SiftED ========================================
// ================================================================================================
class SiftED
{
private:
//     bool verbose;
    int num_images;
    std::vector< std::string > nameImages;
    
    //CV INFO
    std::vector< cv::Mat > set_of_descriptors;

public:
    std::vector< cv::Mat > set_of_images;
    std::vector< std::vector< cv::KeyPoint > > set_of_keypoints;
    
    std::vector< std::vector<SiftGPU::SiftKeypoint> > keypointsGPU;
    std::vector< std::vector<float> > descriptorsGPU;
    
    //Constructor
    SiftED( std::vector< std::string > filenameIms );
    //Destructor
    ~SiftED();

    void solveSift();

    void loadImages();
    void enableKeyPoint();		//enable CV KeyPoint
    cv::Mat image( int num );
    std::vector<cv::KeyPoint> KeyPoint( int num );

    void convertDesGPUtoCV(); 	// TODO, useless until now
    void convertDataGPUtoCV(); 	// TODO, useless until now
    
//     std::vector< std::vector<float> > get_descriptorsGPU();
//     std::vector< std::vector<SiftGPU::SiftKeypoint> > get_keypointsGPU();
};
#endif