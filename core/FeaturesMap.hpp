/**
 * @file FeaturesMap.hpp
 * @brief This file has all functions related with features mapping from database
 *
 * @author José David Tascón Vidarte
 * @date Sep/04/2013
 */

#ifndef __FEATURESMAP_HPP__
#define __FEATURESMAP_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// Boost
#include <boost/shared_ptr.hpp>

// Std Libraries
#include <iostream>
#include <fstream>		// ofstream and/or ifstream
#include <vector>

// Local Libraries
#include "HandleDB.hpp"

// ================================================================================================
// ======================================= CLASS FeaturesMap ======================================
// ================================================================================================
class FeaturesMap
{
private:
    
    bool vis_available;
    bool vis3d_available;
    
    int num_cameras;
    int num_features;
    
    boost::shared_ptr< Eigen::Matrix<bool,-1,-1> > visibility;
    boost::shared_ptr< Eigen::Matrix<Eigen::Vector3d,-1,-1> > coordinates;
    boost::shared_ptr< Eigen::Matrix<Eigen::Vector4d,-1,-1> > coordinates3D;
    
public:
    
    //Constructor
    FeaturesMap()
    { 
        initilizePtrs();
        vis_available = false;
        vis3d_available = false;
    };
    //Destructor
    ~FeaturesMap() { };
    
    void initilizePtrs()
    {
        visibility = boost::shared_ptr< Eigen::Matrix<bool,-1,-1> >( new Eigen::Matrix<bool,-1,-1>() );
        coordinates = boost::shared_ptr< Eigen::Matrix<Eigen::Vector3d,-1,-1> >( new Eigen::Matrix<Eigen::Vector3d,-1,-1>() );
        coordinates3D = boost::shared_ptr< Eigen::Matrix<Eigen::Vector4d,-1,-1> >( new Eigen::Matrix<Eigen::Vector4d,-1,-1>() );
    };
    
    
    void solveVisibility( HandleDB *mydb );
    void solveVisibility3D( HandleDB *mydb );
    void exportTXT(const char *file_txt);
    
    // Get Functions
    int getNumberCameras() { return num_cameras; };
    int getNumberFeatures() { return num_features; };
    
    boost::shared_ptr< Eigen::Matrix<bool,-1,-1> > getVisibility() 
    {
        if( vis3d_available || vis_available ) return visibility;
        else
        {
	  DEBUG_E( ("Visibility matrix from FeatureMap is unavailable, call a \"solve\" function first") ); 
	  exit(-1);
        }
    }
    
    boost::shared_ptr< Eigen::Matrix<Eigen::Vector3d,-1,-1> > getCoordinates()
    {
        if( vis_available ) return coordinates;
        else
        {
	  DEBUG_E( ("Coordinates matrix from FeatureMap is unavailable, call a \"solve\" function first") ); 
	  exit(-1);
        }
    };
    
    boost::shared_ptr< Eigen::Matrix<Eigen::Vector4d,-1,-1> > getCoordinates3D()
        {
        if( vis3d_available ) return coordinates3D;
        else
        {
	  DEBUG_E( ("Coordinates 3D matrix from FeatureMap is unavailable, call a \"solve\" function first") ); 
	  exit(-1);
        }
    };
    
};

#endif