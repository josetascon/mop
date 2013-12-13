/**
 * @file Interface.hpp
 * @brief This file has all functions related with CMVS and PMVS
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

// Std Libraries
#include <iostream>
#include <string>
#include <vector>

// sqlite3 Library
#include <sqlite3.h>

// Local Libraries
#include "Common.hpp"

// ================================================================================================
// ========================================== struc rowDB =========================================
// ================================================================================================
struct rowDB
{
    int feature;
    int camera;
    int idx;
    float coordx;
    float coordy;
};

struct rowDB_3D
{
    int feature;
    int camera;
    int idx;
    float x3d;
    float y3d;
    float z3d;
};

/**
 * ******************************************************************
 * @brief This class has all functions related with database creation/insertion/update for features
 * @author José David Tascón Vidarte
 * @date Jul/15/2013
 */
// ================================================================================================
// ======================================== CLASS HandleDB ========================================
// ================================================================================================
class HandleDB 
{
private:
    char* name_db;
    sqlite3 *db;
    bool db_isopen;
    
public:
    //Constructor
    HandleDB();
    HandleDB( char* nameDB );
    //Destructor
    ~HandleDB();
    
    void openDB();
    void closeDB();
    void createFeaturesTable();
    void createFeaturesTable3D();
    void createIndex1();
    bool isOpen(){ return db_isopen; };
    
    void insertRow(int feature, int camera, int idx, float coordx, float coordy);	//tested
    bool insertUnique(int feature, int camera, int idx, float coordx, float coordy);	//tested
    
    void insertRow3D(int feature, int camera, int idx, float x3d, float y3d, float z3d);
    bool insertUnique3D(int feature, int camera, int idx, float x3d, float y3d, float z3d);
    
    bool searchFeature(int camera, int idx, int *feature);				//tested
    void searchFeature(int camera, std::vector<int> *feature);			//tested
    void searchCamera(int feature, std::vector<int> *camera);			//tested
    void searchRowbyCamera(int camera, std::vector< rowDB > *row_vector);		//tested
    void searchRowbyFeature(int feature, std::vector< rowDB > *row_vector);
    
    void searchRowbyCamera3D(int camera, std::vector< rowDB_3D > *row_vector);
    void searchRowbyFeature3D(int feature, std::vector< rowDB_3D > *row_vector);
    
    int maxFeature();							//tested
    int maxCamera();							//tested
    
    static int callback(void *data, int argc, char **argv, char **azColName);
    static int callback_searchFeature(void *data, int argc, char **argv, char **azColName);
    static int callback_searchVector(void *data, int argc, char **argv, char **azColName);
    static int callback_searchRow(void *data, int argc, char **argv, char **azColName);
    static int callback_max(void *data, int argc, char **argv, char **azColName);
    static int callback_searchRow3D(void *data, int argc, char **argv, char **azColName);
    //     static int callback_search(void *data, int argc, char **argv, char **azColName);
};

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
    void txt(char *file_txt);
    int cameras() { return num_cameras; };
    int features() { return num_features; };
};

// ================================================================================================
// ======================================== CLASS BALProblem ======================================
// ================================================================================================
class BALProblem
{
private:
    int num_cameras;
    int num_features;
    int num_observations;
    const char * file_BAL;
    
public:
    //Constructor
    BALProblem(const char * file_BAL ): file_BAL(file_BAL) { };
    //Destructor
    ~BALProblem() { ; };
    
    void read(Eigen::Matrix<bool,-1,-1> &visibility, Eigen::Matrix<Eigen::Vector3d,-1,-1> &coordinates,
	    std::vector< Eigen::Quaternion<double> > &quaternion, Eigen::MatrixXd &translation_and_intrinsics, Eigen::MatrixXd &structure);
};

// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================
void writePMVS(const char *output_path, std::vector<std::string> &nameImages, 
	     std::vector< Eigen::MatrixXd > &Cameras, Eigen::Matrix3d Calibration, std::vector< double > distortion);

void undistortImages( const char * output_path, std::vector< std::string > &files_input,
		  Eigen::Matrix3d &Calibration, Eigen::MatrixXd &distortion,
		  const char * file_xml, std::vector< std::string > &undistort_files );

void writeGraph( const char *filename, std::vector< Eigen::Quaternion<double> > &Qn_global, 
	       std::vector< Eigen::Vector3d > &tr_global );

#endif