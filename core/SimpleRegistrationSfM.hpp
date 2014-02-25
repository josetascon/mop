/**
 * @file SimpleRegistrationSfM.hpp
 * @brief This file has all functions related 
 *
 * @author José David Tascón Vidarte
 * @date Dec/09/2013
 */

#ifndef __SIMPLEREGISTRATIONSFM_HPP__
#define __SIMPLEREGISTRATIONSFM_HPP__

#include "SimpleRegistration.hpp"


// ================================================================================================
// ================================ CLASS SimpleRegistrationSfM ===================================
// ================================================================================================
class SimpleRegistrationSfM: public SimpleRegistration 		// Registration Aligment of Points based on Image Features
{
private:
    bool fallback_sfm;
    int valid_sfm_points;
    Eigen::MatrixXd Structure;
    Eigen::MatrixXd Covariance;
    
public:
    // Constructor
    SimpleRegistrationSfM(int cams, int feats, Eigen::Matrix3d Calib) : SimpleRegistration(cams, feats, Calib)
    { 
        fallback_sfm = true; //enabled by defualt
        valid_sfm_points = 3;
    };
    // Destructor
    ~SimpleRegistrationSfM() { ; };
    
    void setFallBackSfMOn( int min_points )
    {
        fallback_sfm = true;
        valid_sfm_points = min_points;
    }
    
    void setFallBackSfMOff() { fallback_sfm = false; };
    
    void initilizeStructure( boost::shared_ptr< MXb > visibility, boost::shared_ptr< MX_V3d > coordinates, std::string file_depth);
    
    void solvePose( boost::shared_ptr< MXb > visibility, boost::shared_ptr< MX_V3d > coordinates,
		std::vector< std::string > &depth_list, bool optimal = true );
    
};

#endif