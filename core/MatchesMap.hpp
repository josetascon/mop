/**
 * @file MatchesMap.hpp
 * @brief This file has the class MatchesMap used for SIFT matching in datasets. It uses at its core SiftGPU.
 *
 * @author José David Tascón Vidarte
 * @date Aug/02/2013
 */

#ifndef __MATCHESMAP_HPP__
#define __MATCHESMAP_HPP__

// OpenGl Libraries
#include <GL/glut.h>			// GL constants for SiftGPU function

// OpenCV Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

// Boost
#include <boost/shared_ptr.hpp>

// Std Libraries
#include <iostream>
#include <fstream>				// ofstream and/or ifstream

// SiftGPU Library
#include <SiftGPU.h>

// Local Libraries
#include "Debug.hpp"
#include "Common.hpp"
#include "HandleDB.hpp"
#include "CameraPose.hpp"
#include "DepthProjection.hpp"
#include "FeaturesFunctions.hpp"

// ================================================================================================
// ======================================= struct MatchQuery ======================================
// ================================================================================================
struct MatchQuery
{
    std::vector< cv::DMatch > matches;
    int cam_id1;
    int cam_id2;
};


// ================================================================================================
// ======================================== CLASS MatchesMap ======================================
// ================================================================================================
class MatchesMap
{
private:
    int num_images;
    int num_goodmatch;
    int min_nmatch_reliable;
    int actualfeature;
    
    bool continuous;
//     // OPTION 2
//     Eigen::Matrix< std::vector< cv::DMatch> , -1, -1 > globalMatches;
//     Eigen::Matrix< bool, -1, -1 > avalibleMatches;
    typedef std::vector< std::vector<float> > float_vv;
    typedef std::vector< std::vector<SiftGPU::SiftKeypoint> > kpGPU_vv;
    typedef std::vector< std::vector< cv::KeyPoint > > kpCV_vv;
    
    
public:
    //OPTION 1
    std::vector< MatchQuery > globalMatch;
    std::vector<bool> reliableMatch;
    
    MatchesMap() { num_goodmatch = 35; min_nmatch_reliable = 20; continuous = false; };
    MatchesMap(int ngood): num_goodmatch(ngood) { min_nmatch_reliable = 20; continuous = false; };
    MatchesMap(int ngood, int nreliable): num_goodmatch(ngood), min_nmatch_reliable(nreliable) { continuous = false; };

    ~MatchesMap() { };
    
    void solveMatchesPairs_subgroup( boost::shared_ptr< float_vv > descriptorsGPU, int init_image, int final_image );
    void solveMatchesContinuous_subgroup( boost::shared_ptr< float_vv > descriptorsGPU, int init_image, int final_image );
    
    void solveMatchesOneElement_subgroupUp( boost::shared_ptr< float_vv > descriptorsGPU, int element, int final_image );
    void solveMatchesOneElement_subgroupDown( boost::shared_ptr< float_vv > descriptorsGPU, int init_image, int element );
    void solveMatchesOneElement_subgroup( boost::shared_ptr< float_vv > descriptorsGPU, int element, int init_image, int final_image );
    void solveMatchesOneElement( boost::shared_ptr< float_vv > descriptorsGPU, int element );
    
    void solveMatches( boost::shared_ptr< float_vv > descriptorsGPU );
    void solveMatchesContinuous( boost::shared_ptr< float_vv > descriptorsGPU );
    void solveMatchesGroups( boost::shared_ptr< float_vv > descriptorsGPU, int groupsize );
    void solveMatchesGroups( boost::shared_ptr< float_vv > descriptorsGPU, std::vector<int> *cluster );
    
    void robustifyMatches( boost::shared_ptr< kpCV_vv > set_of_keypoints );
    void robustifyMatches( boost::shared_ptr< kpGPU_vv > set_of_keypoints);
    void depthFilter(boost::shared_ptr< kpGPU_vv > set_of_keypoints,
		         std::vector< cv::Mat > *set_of_depth, const int reliable = 20 );
    void depthFilter(boost::shared_ptr< kpGPU_vv > set_of_keypoints,
		         std::vector< std::string > *depth_list, const int reliable = 20 );
    
    void solveDB( HandleDB *mydb, boost::shared_ptr< kpGPU_vv > keypointsGPU );
    void solveDB3D( HandleDB *mydb, boost::shared_ptr< kpGPU_vv > keypointsGPU,
		        std::vector< std::string > *depth_list, Eigen::Matrix3d &calibration );
    
    void setAllContinousOn() { continuous = true; };
    void setAllContinousOff() { continuous = false; };
    
    
    void exportTXT( const char *file_txt, boost::shared_ptr< kpGPU_vv > keypointsGPU );
    void plot( std::vector< cv::Mat > *images, boost::shared_ptr< kpCV_vv > set_of_keypoints );
    
    std::vector<cv::DMatch> match(int nmatch);
};
#endif