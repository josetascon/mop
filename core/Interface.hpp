/**
 * @file Interface.hpp
 * @brief This file has all functions related with import and export in TXT, XML, GRAPH, or files for PMVS
 *
 * @author José David Tascón Vidarte
 * @date Sep/04/2013
 */

#ifndef __INTERFACE_HPP__
#define __INTERFACE_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// OpenCV Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>		//undistort in WRITEPMVS
#include <opencv2/core/eigen.hpp>

// Boost Libraries
#include <boost/filesystem.hpp>		// Create directories OS independent

// Std Libraries
#include <iostream>
#include <fstream>		// ofstream and/or ifstream
#include <string>
#include <vector>
#include <libgen.h>

// Local Libraries
#include "Common.hpp"

// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================
// @date Sep/16/2013
template <typename T_eig>
void importTXTEigen(const char *filename, Eigen::Matrix<T_eig,-1,-1> &M)
{
    std::string str1;
    int nrow, ncol;
    
    std::ifstream myfile1;
    myfile1.open(filename);
    if (!myfile1.is_open())
    {
        std::cerr << "Error: unable to open file " << filename << "\n";
        exit(0);
    }
    
    myfile1 >> str1;
    myfile1 >> nrow;
    myfile1 >> ncol;
    M = Eigen::Matrix<T_eig,-1,-1>::Zero(nrow,ncol);
    for (register int i = 0; i < nrow; ++i)
    {
        for(register int j = 0; j < ncol; ++j)
        {
	  myfile1 >> M(i,j);
        }
    }
    myfile1.close();
    return;
}

template <typename T_eig>
void exportTXTEigen(const char *filename, Eigen::Matrix<T_eig,-1,-1> &M)
{
    int nrow = M.rows();
    int ncol = M.cols();
    char buf[256];
    // Creating txt file with cameras
    sprintf(buf, "./%s.txt", filename);
    FILE *f = fopen(buf, "w");
    assert(f);
    
    fprintf(f, "%s\n", filename);
    fprintf(f, "%d ", nrow);
    fprintf(f, "%d\n", ncol);
    for (register int i = 0; i < nrow; ++i)
    {
        for(register int j = 0; j < ncol; ++j)
        {
	  fprintf(f, "%0.16e ", M(i,j));
        }
        fprintf(f, "\n");
    }
    
    fclose(f);
    return;
}

template <typename T_eig>
void exportTXTQuaternionVector(const char *filename, std::vector< Eigen::Quaternion<T_eig> > &Qn_global)
{
//     char *filename = (char*)"pose_rot.txt";
    std::ofstream myfile1;
    myfile1.open (filename);
    myfile1.precision(12);

    for(register int it = 0; it < Qn_global.size(); ++it)
    {
        Eigen::Matrix<T_eig,3,3> rr = Qn_global[it].toRotationMatrix();
        myfile1 << rr << "\n";
    }
    myfile1.close();
}

template <typename T_eig>
void exportTXTTranslationVector(const char *filename, std::vector< Eigen::Matrix<T_eig,3,1> > &tr_global)
{
//     char *filename = (char*)"pose_tr.txt";
    std::ofstream myfile1;
    myfile1.open (filename);
    myfile1.precision(12);

    for(register int it = 0; it < tr_global.size(); ++it)
    {
        Eigen::Matrix<T_eig,3,1> tr = tr_global[it];
        myfile1 << tr.transpose() << "\n";
    }
    myfile1.close();
}

template <typename T_eig>
void importTXTQuaternionVector(const char *filename, std::vector< Eigen::Quaternion<T_eig> > &Qn_global)
{
    Qn_global.clear();
    Eigen::Matrix<T_eig,-1,-1> Mdata;
    importTXTEigen( filename, Mdata );
    
    for(register int it = 0; it < Mdata.rows()/3; ++it)
    {
        Eigen::Matrix<T_eig,3,3> rr = Mdata.block(it*3,0,3,3);
        Qn_global.push_back( Eigen::Quaternion<T_eig>(rr) );
    }
}

template <typename T_eig>
void importTXTTranslationVector(const char *filename, std::vector< Eigen::Matrix<T_eig,3,1> > &tr_global)
{
    tr_global.clear();
    Eigen::Matrix<T_eig,-1,-1> Mdata;
    importTXTEigen( filename, Mdata );
    
    for(register int it = 0; it < Mdata.rows(); ++it)
    {
        Eigen::Matrix<T_eig,3,1> vv = Mdata.row(it);
        tr_global.push_back( vv );
    }
}


/**
 * ******************************************************************
 * @brief Read XML File given by filename. The XML file contains a list of images to read. Return the location of images files.
 * 
 * @param filename	 	(input) XML file that contains the list of images
 * @param location		(output) Directory route where are located the images files
 * 
 * @date Jul/12/2013
 */
bool importXMLImageList(const char *file_xml, std::vector< std::string > &files_names, bool clear = true);

void exportXMLImageList(const char *file_xml, std::vector< std::string > &files_names);

void importXMLMultipleImageList( std::vector< std::string > &files_input, std::vector< std::string > &image_list, std::vector<int> &boundaries );

void exportPMVS(const char *output_path, std::vector<std::string> &nameImages, 
	     std::vector< Eigen::MatrixXd > &Cameras, Eigen::Matrix3d Calibration, std::vector< double > distortion);

void undistortImages( const char * output_path, std::vector< std::string > &files_input,
		  Eigen::Matrix3d &Calibration, Eigen::MatrixXd &distortion,
		  const char * file_xml, std::vector< std::string > &undistort_files );

void exportGRAPH( const char *filename, std::vector< Eigen::Quaternion<double> > &Qn_global, 
	       std::vector< Eigen::Vector3d > &tr_global );

#endif