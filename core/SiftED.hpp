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

// Boost
#include <boost/shared_ptr.hpp>

// Std Libraries
#include <iostream>
#include <libgen.h>

// SiftGPU Library
#include <SiftGPU.h>

// Local Libraries
#include "Debug.hpp"

// ================================================================================================
// ========================================== CLASS SiftED ========================================
// ================================================================================================
class SiftED
{
private:
    typedef std::vector< std::vector<float> > float_vv;
    typedef std::vector< std::vector<SiftGPU::SiftKeypoint> > kpGPU_vv;
    typedef std::vector< std::vector< cv::KeyPoint > > kpCV_vv;
    
    bool load_images;
    bool keypoint_available;
    
    int num_images;
    std::vector< std::string > *nameImages;
//     boost::shared_ptr< std::vector< std::string > > nameImages;
    
    //CV INFO
    boost::shared_ptr< std::vector< cv::Mat > > set_of_images;
    boost::shared_ptr< std::vector< cv::Mat > > set_of_descriptors;
    boost::shared_ptr< kpCV_vv > set_of_keypoints;
    
    boost::shared_ptr< float_vv > descriptorsGPU;
    boost::shared_ptr< kpGPU_vv > keypointsGPU;
    
public:
    
    //Constructor
    SiftED( std::vector< std::string > *filenameIms )
    {
        initilizePtrs();
        load_images = false;
        keypoint_available = false;
        
        nameImages = filenameIms;
        num_images = nameImages->size();
        descriptorsGPU->resize(num_images);
        keypointsGPU->resize(num_images);
    };
    //Destructor
    ~SiftED() { };
//     {
//         nameImages.reset();
//         descriptorsGPU.reset();
//         keypointsGPU.reset();
//         set_of_images.reset();
//         set_of_descriptors.reset();
//         set_of_keypoints.reset();
//     };
    
    void initilizePtrs()
    {
//         nameImages = boost::shared_ptr< std::vector< std::string > >( new std::vector< std::string >());
        set_of_images = boost::shared_ptr< std::vector< cv::Mat > >( new std::vector< cv::Mat >());
        set_of_descriptors = boost::shared_ptr< std::vector< cv::Mat > >( new std::vector< cv::Mat >());
        set_of_keypoints = boost::shared_ptr< kpCV_vv >( new kpCV_vv());
        descriptorsGPU = boost::shared_ptr< float_vv >( new float_vv());
        keypointsGPU = boost::shared_ptr< kpGPU_vv >( new kpGPU_vv());
    };
    
    void solveSift();

    void loadImages();
    void enableKeyPoint();		//enable CV KeyPoint
    
    // Get functions
    boost::shared_ptr< kpGPU_vv > getKeypointsGPU() { return keypointsGPU; };
    boost::shared_ptr< float_vv > getDescriptorsGPU() { return descriptorsGPU; };
    
    cv::Mat getImage( int num );
    std::vector<cv::KeyPoint> getKeyPoint( int num );

//     void convertDesGPUtoCV(); 	// TODO, useless until now
//     void convertDataGPUtoCV(); 	// TODO, useless until now
    
};
#endif