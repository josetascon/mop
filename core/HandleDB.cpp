// Created: Sep/04/2013
// Author: José David Tascón Vidarte

#include "HandleDB.hpp"

struct search_int
{
    bool found;
    int value;
};

// ================================================================================================
// =============================== FUNCTIONS of CLASS HandleDB ====================================
// ================================================================================================

// string to char*  || (char*)stringvariable.c_str()
// char* to string  || stringvariable = string(char_variable)
/**
 * ******************************************************************
 * @brief Description: 
 * 
 * @param in		(input)
 * @param out		(output)
 * 
 * @date Jul/15/2013
 */
HandleDB::HandleDB()
{ 
    db_isopen = false;
};

HandleDB::HandleDB( char* nameDB )
{
    name_db = nameDB;
//     openDB();
    db_isopen = false;
//     db_isopen = true;
};

HandleDB::~HandleDB()
{
    if (db_isopen)
    {
        closeDB();
        db_isopen = false;
    }
};

void HandleDB::openDB()
{
    if (db_isopen) return;
    int  rc;
    /* Open database */
    rc = sqlite3_open( name_db, &db);
    if ( rc )
    {
        fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
        exit(0);
    }
    else
    {
        DEBUG_1( fprintf(stdout, "\nOpened database successfully\n"); )
        db_isopen = true;
    }
    return;
}

void HandleDB::closeDB()
{
    int  rc;
    rc = sqlite3_close(db);
    if( rc != SQLITE_OK )
    {
        fprintf(stderr, "SQL error closing db\n");
    }
    else
    {
        DEBUG_1( fprintf(stdout, "Closed database successfully\n"); )
        db_isopen = false;
    }
    return;
}

int HandleDB::callback(void *data, int argc, char **argv, char **azColName)
{
    int i;
    fprintf(stderr, "%s: ", (const char*)data);
    for(i=0; i<argc; i++){
        printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
    }
    printf("\n");
    return 0;
}

void HandleDB::createFeaturesTable()
{
   char *zErrMsg = 0;
   int  rc;
   char *sql;
   int (*fp)(void*, int, char**, char**) = &callback;
   
   /* Create SQL statement */
   std::string command = "create table fcdata(feature int, camera int, idx int, coordx float, coordy float)";
   sql = (char*)command.c_str();

   /* Execute SQL statement */
   rc = sqlite3_exec(db, sql, fp, 0, &zErrMsg);
   if( rc != SQLITE_OK ){
   fprintf(stderr, "SQL error: %s\n", zErrMsg);
      sqlite3_free(zErrMsg);
   }else{
      DEBUG_1( fprintf(stdout, "Table created successfully\n"); )
   }
   return;
}

void HandleDB::createFeaturesTable3D()
{
   char *zErrMsg = 0;
   int  rc;
   char *sql;
   int (*fp)(void*, int, char**, char**) = &callback;
   
   /* Create SQL statement */
   std::string command = "create table fcdata(feature int, camera int, idx int, x3d float, y3d float, z3d float)";
   sql = (char*)command.c_str();

   /* Execute SQL statement */
   rc = sqlite3_exec(db, sql, fp, 0, &zErrMsg);
   if( rc != SQLITE_OK ){
   fprintf(stderr, "SQL error: %s\n", zErrMsg);
      sqlite3_free(zErrMsg);
   }else{
      DEBUG_1( fprintf(stdout, "Table created successfully\n"); )
   }
   return;
}

void HandleDB::createIndex1()
{
   char *zErrMsg = 0;
   int  rc;
   char *sql;
   int (*fp)(void*, int, char**, char**) = &callback;
   
   /* Create SQL statement */
   std::string command = "create index idx1 on fcdata (camera, idx, feature)";
   sql = (char*)command.c_str();

   /* Execute SQL statement */
   rc = sqlite3_exec(db, sql, fp, 0, &zErrMsg);
   if( rc != SQLITE_OK ){
   fprintf(stderr, "SQL error: %s\n", zErrMsg);
      sqlite3_free(zErrMsg);
   }else{
      DEBUG_1( fprintf(stdout, "Index created successfully\n"); )
   }
   return;
}

void HandleDB::insertRow(int feature, int camera, int idx, float coordx, float coordy)
{
    char *zErrMsg = 0;
    int  rc;
    char *sql;
    
    /* SQL statement */
    std::stringstream ss;
    std::string command;
    ss << "insert into fcdata values( "<< feature << ", " << camera << ", " << idx << ", " << coordx << ", " << coordy << " ); ";
    command = ss.str();
    sql = (char*)command.c_str();
    
    /* Execute SQL statement */
    rc = sqlite3_exec(db, sql, NULL, 0, &zErrMsg);
    if( rc != SQLITE_OK ){
        fprintf(stderr, "SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
//         fprintf(stdout, "New feature inseted successfully\n");
    }
    return;
}

bool HandleDB::insertUnique(int feature, int camera, int idx, float coordx, float coordy)
{
    char *zErrMsg = 0;
    int  rc;
    char *sql;
    int search_feature;
    
    bool find;
    find = searchFeature(camera, idx, &search_feature);
    if (find) return false;
    else
    {
        insertRow(feature, camera, idx, coordx, coordy);
        return true;
    }
}

void HandleDB::insertRow3D(int feature, int camera, int idx, float x3d, float y3d, float z3d)
{
    char *zErrMsg = 0;
    int  rc;
    char *sql;
    
    /* SQL statement */
    std::stringstream ss;
    std::string command;
    ss << "insert into fcdata values( "<< feature << ", " << camera << ", " << idx << ", " 
				<< x3d << ", " << y3d << ", " << z3d << " ); ";
    command = ss.str();
    sql = (char*)command.c_str();
    
    /* Execute SQL statement */
    rc = sqlite3_exec(db, sql, NULL, 0, &zErrMsg);
    if( rc != SQLITE_OK ){
        fprintf(stderr, "SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
//         fprintf(stdout, "New feature inseted successfully\n");
    }
    return;
}

bool HandleDB::insertUnique3D(int feature, int camera, int idx, float x3d, float y3d, float z3d)
{
    char *zErrMsg = 0;
    int  rc;
    char *sql;
    int search_feature;
    
    bool find;
    find = searchFeature(camera, idx, &search_feature);
    if (find) return false;
    else
    {
        insertRow3D(feature, camera, idx, x3d, y3d, z3d);
        return true;
    }
}

int HandleDB::callback_searchFeature(void *data, int argc, char **argv, char **azColName)
{
    search_int *pdata = reinterpret_cast<search_int*>(data);
//     for(int i=0; i<argc; i++) printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
    if ( (argc > 0) && (argv[0] != NULL) )
    {
        
        pdata->value = pchar2number<int>(argv[0]);
        pdata->found = true;
    }
    else pdata->found = false;
    return 0;
}

bool HandleDB::searchFeature(int camera, int idx, int *feature)
{
    char *zErrMsg = 0;
    int  rc;
    char *sql;
    search_int val;
    val.found = false;
    val.value = -1;
    search_int *data = &val;

    /* SQL statement */
    std::stringstream ss;
    std::string command;
    ss << "select feature from fcdata where camera="<< camera << " and idx=" << idx << ";";
    command = ss.str();
    sql = (char*)command.c_str();
    
    /* Execute SQL statement */
    rc = sqlite3_exec(db, sql, HandleDB::callback_searchFeature, reinterpret_cast<void*>(data), &zErrMsg);
    if( rc != SQLITE_OK ){
        fprintf(stderr, "SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
//         fprintf(stdout, "Search successfully\n");
    }
    *feature = data->value;
    return data->found;
}

int HandleDB::callback_searchVector(void *data, int argc, char **argv, char **azColName)
{
    std::vector<int> *pdata = reinterpret_cast< std::vector<int>* >(data); //(std::vector<rowDB>*)(data);
//     pdata->resize(argc);
//     for(int i=0; i<argc; i++) printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
    //for(int i=0; i<argc; i++)
    if (argc > 0)
    {
        (*pdata).push_back(pchar2number<int>(argv[0]));
    }
    return 0;
}

void HandleDB::searchFeature(int camera, std::vector<int> *feature)
{
    char *zErrMsg = 0;
    int  rc;
    char *sql;
    feature->clear();
    std::vector<int> *data = feature;
    
    /* SQL statement */
    std::stringstream ss;
    std::string command;
    ss << "select feature from fcdata where camera="<< camera << ";";
    command = ss.str();
    sql = (char*)command.c_str();
    
    /* Execute SQL statement */
    rc = sqlite3_exec(db, sql, HandleDB::callback_searchVector, reinterpret_cast<void*>(data), &zErrMsg);
    if( rc != SQLITE_OK ){
        fprintf(stderr, "SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
        //fprintf(stdout, "Search successfully\n");
    }
    
    return;
}

void HandleDB::searchCamera(int feature, std::vector<int> *camera)
{
    char *zErrMsg = 0;
    int  rc;
    char *sql;
//     std::vector<int> val(1);
    camera->clear();
    std::vector<int> *data = camera;
    
    /* SQL statement */
    std::stringstream ss;
    std::string command;
    ss << "select camera from fcdata where feature="<< feature << ";";
    command = ss.str();
    sql = (char*)command.c_str();
    
    /* Execute SQL statement */
    rc = sqlite3_exec(db, sql, HandleDB::callback_searchVector, reinterpret_cast<void*>(data), &zErrMsg);
    if( rc != SQLITE_OK ){
        fprintf(stderr, "SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
        //fprintf(stdout, "Search successfully\n");
    }
    
    return;
}

int HandleDB::callback_searchRow(void *data, int argc, char **argv, char **azColName)
{
    std::vector<rowDB> *pdata = reinterpret_cast< std::vector<rowDB>* >(data); //(std::vector<rowDB>*)(data);
//     pdata->resize(argc/5);
    rowDB single;
    for(int i=0; i<argc; i+=5)
    {
        single.feature = pchar2number<int>(argv[i]);
        single.camera = pchar2number<int>(argv[i+1]);
        single.idx = pchar2number<int>(argv[i+2]);
        single.coordx = pchar2number<float>(argv[i+3]);
        single.coordy = pchar2number<float>(argv[i+4]);    
    }
    pdata->push_back(single);
    return 0;
}

void HandleDB::searchRowbyCamera(int camera, std::vector< rowDB > *row_vector)
{
    char *zErrMsg = 0;
    int  rc;
    char *sql;
    row_vector->clear();
    std::vector<rowDB> *data = row_vector;
    
    /* SQL statement */
    std::stringstream ss;
    std::string command;
    
    ss << "select * from fcdata where camera="<< camera << ";";
    command = ss.str();
    sql = (char*)command.c_str();
    
    /* Execute SQL statement */
    rc = sqlite3_exec(db, sql, HandleDB::callback_searchRow, reinterpret_cast<void*>(data), &zErrMsg);
    if( rc != SQLITE_OK ){
        fprintf(stderr, "SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
        //fprintf(stdout, "Search successfully\n");
    }
    
    return;
}

void HandleDB::searchRowbyFeature(int feature, std::vector< rowDB > *row_vector)
{
    char *zErrMsg = 0;
    int  rc;
    char *sql;
    row_vector->clear();
    std::vector<rowDB> *data = row_vector;
    
    /* SQL statement */
    std::stringstream ss;
    std::string command;
    
    ss << "select * from fcdata where feature="<< feature << ";";
    command = ss.str();
    sql = (char*)command.c_str();
    
    /* Execute SQL statement */
    rc = sqlite3_exec(db, sql, HandleDB::callback_searchRow, reinterpret_cast<void*>(data), &zErrMsg);
    if( rc != SQLITE_OK ){
        fprintf(stderr, "SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
        //fprintf(stdout, "Search successfully\n");
    }
    
    return;
}

int HandleDB::callback_searchRow3D(void *data, int argc, char **argv, char **azColName)
{
    std::vector<rowDB_3D> *pdata = reinterpret_cast< std::vector<rowDB_3D>* >(data); //(std::vector<rowDB_3D>*)(data);
//     pdata->resize(argc/5);
    rowDB_3D single;
    for(int i=0; i<argc; i+=6)
    {
        single.feature = pchar2number<int>(argv[i]);
        single.camera = pchar2number<int>(argv[i+1]);
        single.idx = pchar2number<int>(argv[i+2]);
        single.x3d = pchar2number<float>(argv[i+3]);
        single.y3d = pchar2number<float>(argv[i+4]);
        single.z3d = pchar2number<float>(argv[i+5]);
    }
    pdata->push_back(single);
    return 0;
}


void HandleDB::searchRowbyCamera3D(int camera, std::vector< rowDB_3D > *row_vector)
{
    char *zErrMsg = 0;
    int  rc;
    char *sql;
    row_vector->clear();
    std::vector<rowDB_3D> *data = row_vector;
    
    /* SQL statement */
    std::stringstream ss;
    std::string command;
    
    ss << "select * from fcdata where camera="<< camera << ";";
    command = ss.str();
    sql = (char*)command.c_str();
    
    /* Execute SQL statement */
    rc = sqlite3_exec(db, sql, HandleDB::callback_searchRow3D, reinterpret_cast<void*>(data), &zErrMsg);
    if( rc != SQLITE_OK ){
        fprintf(stderr, "SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
        //fprintf(stdout, "Search successfully\n");
    }
    
    return;
}

void HandleDB::searchRowbyFeature3D(int feature, std::vector< rowDB_3D > *row_vector)
{
    char *zErrMsg = 0;
    int  rc;
    char *sql;
    row_vector->clear();
    std::vector<rowDB_3D> *data = row_vector;
    
    /* SQL statement */
    std::stringstream ss;
    std::string command;
    
    ss << "select * from fcdata where feature="<< feature << ";";
    command = ss.str();
    sql = (char*)command.c_str();
    
    /* Execute SQL statement */
    rc = sqlite3_exec(db, sql, HandleDB::callback_searchRow3D, reinterpret_cast<void*>(data), &zErrMsg);
    if( rc != SQLITE_OK ){
        fprintf(stderr, "SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
        //fprintf(stdout, "Search successfully\n");
    }
    
    return;
}

int HandleDB::callback_max(void *data, int argc, char **argv, char **azColName)
{
    int *value = reinterpret_cast<int*>(data);
    if (argc>0)
    {
        if (argv[0] != NULL) //std::cout << "\nmaxFeature update;\n" ;
        {
	  *value = pchar2number<int>(argv[0]);
        }
    }
    return 0;
}

int HandleDB::maxFeature()
{
    char *zErrMsg = 0;
    int  rc;
    char *sql;
    int val = -1;
    int *data = &val;
    
    /* SQL statement */
    std::stringstream ss;
    std::string command;
    
    ss << "select max(feature) from fcdata;";
    command = ss.str();
    sql = (char*)command.c_str();
    
    /* Execute SQL statement */
    rc = sqlite3_exec(db, sql, callback_max, reinterpret_cast<void*>(data), &zErrMsg);
    if( rc != SQLITE_OK ){
        fprintf(stderr, "SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
//         fprintf(stdout, "max executed\n");
    }
    
    return val;
}

int HandleDB::maxCamera()
{
    char *zErrMsg = 0;
    int  rc;
    char *sql;
    int val = -1;
    int *data = &val;
    
    /* SQL statement */
    std::stringstream ss;
    std::string command;
    
    ss << "select max(camera) from fcdata;";
    command = ss.str();
    sql = (char*)command.c_str();
    
    /* Execute SQL statement */
    rc = sqlite3_exec(db, sql, callback_max, reinterpret_cast<void*>(data), &zErrMsg);
    if( rc != SQLITE_OK ){
        fprintf(stderr, "SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
//         fprintf(stdout, "max executed\n");
    }
    
    return val;
}