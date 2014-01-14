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
    int num_cameras;
    int num_features;
    
public:
    Eigen::Matrix<bool,-1,-1> visibility;
    Eigen::Matrix<Eigen::Vector3d,-1,-1> coordinates;
    Eigen::Matrix<Eigen::Vector4d,-1,-1> coordinates3D;
    
    //Constructor
    FeaturesMap() { };
    //Destructor
    ~FeaturesMap() { };
    
    void solveVisibility( HandleDB *mydb );
    void solveVisibility3D( HandleDB *mydb );
    void exportTXT(const char *file_txt);
    int cameras() { return num_cameras; };
    int features() { return num_features; };
};

#endif