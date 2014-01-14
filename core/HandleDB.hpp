/**
 * @file HandleDB.hpp
 * @brief This file has all functions related database with sqlite3
 *
 * @author José David Tascón Vidarte
 * @date Sep/04/2013
 */

#ifndef __HANDLEDB_HPP__
#define __HANDLEDB_HPP__

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

#endif