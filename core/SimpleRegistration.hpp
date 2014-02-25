/**
 * @file SimpleRegistration.hpp
 * @brief This file has all functions related 
 *
 * @author José David Tascón Vidarte
 * @date Dec/09/2013
 */

#ifndef __SIMPLEREGISTRATION_HPP__
#define __SIMPLEREGISTRATION_HPP__

#define LOCAL_MODEL
// if LOCAL_MODEL variable is ACTIVE
#ifdef LOCAL_MODEL
#define XMODEL(x) { x }
#else
#define XMODEL(x) {}
#endif


// Eigen Libraries
#include <eigen3/Eigen/Dense>

// OpenCV Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

// Boost
#include <boost/shared_ptr.hpp>

// Std Libraries
#include <iostream>
#include <vector>

// Local Libraries
#include "Debug.hpp"
#include "Pose3D.hpp"
#include "PoseICP.hpp"
#include "FeaturesEDM.hpp"


// ================================================================================================
// ================================= CLASS SimpleRegistration =====================================
// ================================================================================================
class SimpleRegistration		// Registration Aligment of Points based on Image Features
{
private:
    bool fallback_icp;
    int valid_min_points;
    int num_cameras;
    int num_features;
    Eigen::Matrix3d Calibration;
    
    std::vector< std::string> *images_rgb;
    std::vector< std::string> *images_depth;
    
    Eigen::MatrixXd X1, X2;
    Eigen::Matrix3d Rot;
    Eigen::Vector3d tr;
    
    void initializeGlobalVariables();
    void initializeCoordinatesOrigin();
    void updateGlobal(int iteration);
    void printGlobal(int iteration);
    
    typedef std::vector< Eigen::Quaternion<double> > Qd_vector;
    typedef std::vector< Eigen::Vector3d > V3d_vector;
    typedef std::vector< Eigen::Matrix3d > M3d_vector;
    typedef std::vector< Eigen::MatrixXd > MXd_vector;
    
    typedef Eigen::Matrix<bool,-1,-1> MXb;
    typedef Eigen::Matrix<Eigen::Vector3d,-1,-1> MX_V3d;
    typedef Eigen::Matrix<Eigen::Vector4d,-1,-1> MX_V4d;
    
//     std::vector< Eigen::MatrixXd > Cameras_RCV;
//     std::vector< Eigen::Matrix3d > Rot_global;
//     std::vector< Eigen::Quaternion<double> > Qn_global;
//     std::vector< Eigen::Vector3d > tr_global;
//     std::vector< Eigen::MatrixXd > Xmodel; // Created to easily access 3D features. ( e.g. when I need to Plot elipsoids )
//     std::vector< Eigen::MatrixXd > Variance;
    boost::shared_ptr< MXd_vector > Cameras_RCV;
    boost::shared_ptr< M3d_vector > Rot_global;
    boost::shared_ptr< Qd_vector > Qn_global;
    boost::shared_ptr< V3d_vector > tr_global;
    boost::shared_ptr< MXd_vector > Xmodel;
    boost::shared_ptr< MXd_vector > Variance;
    
public:
    // Constructor
    SimpleRegistration(int cams, int feats, Eigen::Matrix3d Calib) : num_cameras(cams), num_features(feats), Calibration(Calib) 
    { 
        fallback_icp = false;
        initilizePtrs();
    };
    // Destructor
    ~SimpleRegistration() { ; };
    
    void initilizePtrs()
    {
        Cameras_RCV = boost::shared_ptr< MXd_vector >( new MXd_vector());
        Rot_global = boost::shared_ptr< M3d_vector >( new M3d_vector());
        Qn_global = boost::shared_ptr< Qd_vector >( new Qd_vector());
        tr_global = boost::shared_ptr< V3d_vector >( new V3d_vector());
        Xmodel = boost::shared_ptr< MXd_vector >( new MXd_vector());
        Variance = boost::shared_ptr< MXd_vector >( new MXd_vector());
    };
    
    void setFallBackICPOn( std::vector< std::string> *rgb, std::vector< std::string> *depth, int min_points = 3 ) // Minimal number of points to solve pose
    {
        images_rgb = rgb;
        images_depth = depth;
        fallback_icp = true;
        valid_min_points = min_points;
    }
    
    void setFallBackICPOff() { fallback_icp = false; };
    
    //solve Pose, use continuous matches
    void solvePose( std::vector< MatchQuery > *globalMatch, std::vector< std::vector< cv::KeyPoint > > *set_of_keypoints, 
		std::vector< cv::Mat > *set_of_depth );
    
    void solvePose( boost::shared_ptr< MXb > visibility, boost::shared_ptr< MX_V3d > coordinates,
		std::vector< std::string > &depth_list, bool optimal = true );
    
    void solvePose( boost::shared_ptr< MXb > visibility, boost::shared_ptr< MX_V4d > coordinates, bool optimal = true );
    
    void updateQuaternion();
    
    // Get Functions
    boost::shared_ptr< std::vector< Eigen::Quaternion<double> > > getPtrGlobalQuaternion()
    {
//         return boost::shared_ptr< std::vector< Eigen::Quaternion<double> > >( &Qn_global );
        return Qn_global;
    }
    
    boost::shared_ptr< std::vector< Eigen::Vector3d > > getPtrGlobalTranslation()
    {
//         return boost::shared_ptr< std::vector< Eigen::Vector3d > >( &tr_global );
        return tr_global;
    }
    
    boost::shared_ptr< std::vector< Eigen::MatrixXd > > getPtrXmodel()
    {
//         return boost::shared_ptr< std::vector< Eigen::MatrixXd > > ( &Xmodel );
        return Xmodel;
    }
    
    boost::shared_ptr< std::vector< Eigen::MatrixXd > > getPtrVariance()
    {
//         return boost::shared_ptr< std::vector< Eigen::MatrixXd > > ( &Variance );
        return Variance;
    }
    
    /** Use SimpleRegistration::Ptr as a clean nomenclature to create an smart pointer of SimpleRegistration type.\n 
     * E.g.: \n
     * > SimpleRegistration::Ptr myobject(new SimpleRegistration);
     */
    typedef boost::shared_ptr< SimpleRegistration > Ptr;
};

#endif