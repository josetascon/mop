/**
 * @file BALReader.hpp
 * @brief This file has all functions related with import data from BAL (Bundle Adjustment in the Large) files
 *
 * @author José David Tascón Vidarte
 * @date Sep/04/2013
 */

#ifndef __BALREADER_HPP__
#define __BALREADER_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// Std Libraries
#include <iostream>
#include <fstream>		// ofstream and/or ifstream
#include <vector>

// Local Libraries
#include "Common.hpp"

// ================================================================================================
// ======================================== CLASS BALReader =======================================
// ================================================================================================
class BALReader
{
private:
    int num_cameras;
    int num_features;
    int num_observations;
    const char * file_BAL;
    
public:
    //Constructor
    BALReader(const char * file_BAL ): file_BAL(file_BAL) { };
    //Destructor
    ~BALReader() { ; };
    
    void importBAL(Eigen::Matrix<bool,-1,-1> &visibility, Eigen::Matrix<Eigen::Vector3d,-1,-1> &coordinates,
	    std::vector< Eigen::Quaternion<double> > &quaternion, Eigen::MatrixXd &translation_and_intrinsics, Eigen::MatrixXd &structure);
};

#endif