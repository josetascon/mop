// Created: Sep/04/2013
// Author: José David Tascón Vidarte

#include "Interface.hpp"

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
        fprintf(stdout, "\nOpened database successfully\n");
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
        fprintf(stdout, "Closed database successfully\n");
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
      fprintf(stdout, "Table created successfully\n");
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
      fprintf(stdout, "Table created successfully\n");
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
      fprintf(stdout, "Index created successfully\n");
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


// ================================================================================================
// ============================= FUNCTIONS of CLASS FeaturesMap ===================================
// ================================================================================================

void FeaturesMap::solveVisibility( HandleDB *mydb )
{
    num_cameras = mydb->maxCamera() + 1;
    num_features = mydb->maxFeature() + 1;
//     std::cout << "max cameras = " << num_cameras << "\n";
//     std::cout << "max features = " << num_features << "\n";
     
    visibility = Eigen::Matrix<bool,-1,-1>::Zero(num_cameras,num_features);
    coordinates.resize(num_cameras,num_features);
    
    for(register int cam = 0; cam < num_cameras; ++cam)
    {
        std::vector< rowDB > row_vector;
        mydb->searchRowbyCamera( cam, &row_vector);
        for(register int k = 0; k < row_vector.size(); ++k)
        {
	  int a = row_vector[k].camera;
	  int b = row_vector[k].feature;
	  visibility(a,b) = true;
	  coordinates(a,b) = Eigen::Vector3d( row_vector[k].coordx, row_vector[k].coordy, 1.0 );
        }
    }
}

void FeaturesMap::solveVisibility3D( HandleDB *mydb )
{
    num_cameras = mydb->maxCamera() + 1;
    num_features = mydb->maxFeature() + 1;
//     std::cout << "max cameras = " << num_cameras << "\n";
//     std::cout << "max features = " << num_features << "\n";
     
    visibility = Eigen::Matrix<bool,-1,-1>::Zero(num_cameras,num_features);
    coordinates3D.resize(num_cameras,num_features);
    
    for(register int cam = 0; cam < num_cameras; ++cam)
    {
        std::vector< rowDB_3D > row_vector;
        mydb->searchRowbyCamera3D( cam, &row_vector);
//         std::cout << "row size = " << row_vector.size() << "\n";
        for(register int k = 0; k < row_vector.size(); ++k)
        {
	  int a = row_vector[k].camera;
	  int b = row_vector[k].feature;
	  visibility(a,b) = true;
	  coordinates3D(a,b) = Eigen::Vector4d( row_vector[k].x3d, row_vector[k].y3d, row_vector[k].z3d, 1.0 );
        }
    }
}

void FeaturesMap::txt(char *file_txt)
{
    std::ofstream myfile1;
    myfile1.open (file_txt);
    myfile1 << "Visibility [camera x feature]\n";
    myfile1 << visibility.rows() << " " << visibility.cols() << "\n";
    myfile1 << visibility << "\n\n\n";
    myfile1 << "Coordinates\n[(c,f):\t   u\tv\t  1]\n\n";
    for (int j = 0; j < visibility.cols(); j++)
    {
        for (int i = 0; i < visibility.rows(); i++)
        {
	  if ((visibility)(i,j))
	  {
	      myfile1 << "(" << i << "," << j << "):\t" << coordinates(i,j).transpose() << "\n";
	  }
        }
    }
    myfile1.close();
}


// ================================================================================================
// ============================== FUNCTIONS of CLASS BALProblem ===================================
// ================================================================================================
void BALProblem::read(Eigen::Matrix<bool,-1,-1> &visibility, Eigen::Matrix<Eigen::Vector3d,-1,-1> &coordinates,
	    std::vector< Eigen::Quaternion<double> > &quaternion, Eigen::MatrixXd &translation_and_intrinsics, Eigen::MatrixXd &structure)
{
    std::ifstream myfile1;
    myfile1.open(file_BAL);
    if (!myfile1.is_open())
    {
        std::cerr << "Error: unable to open file " << file_BAL << "\n";
        exit(0);
    }
    
    myfile1 >> num_cameras;
    myfile1 >> num_features;
    myfile1 >> num_observations;
    
    visibility = Eigen::Matrix<bool,-1,-1>::Zero(num_cameras,num_features);
    coordinates.resize(num_cameras,num_features);
    translation_and_intrinsics = Eigen::MatrixXd::Zero(6,num_cameras);
    quaternion.resize(num_cameras);
    structure = Eigen::MatrixXd::Ones(4,num_features);
    
    register int camera_idx;
    register int point_idx;
    double coordx;
    double coordy;
    // Reading observations
    for (register int j = 0; j < num_observations; ++j)
    {
        myfile1 >> camera_idx;
        myfile1 >> point_idx;
        myfile1 >> coordx;
        myfile1 >> coordy;
        visibility(camera_idx,point_idx) = true;
        coordinates(camera_idx,point_idx) = Eigen::Vector3d(coordx, coordy, 1.0);
    }
    double qtmp[4];
    double angleaxis[3];
    // Reading cameras
    for (register int j = 0; j < num_cameras; ++j)
    {
        myfile1 >> angleaxis[0];
        myfile1 >> angleaxis[1];
        myfile1 >> angleaxis[2];
        AngleAxisRToQuaternion(&angleaxis[0], &qtmp[0]);
        quaternion[j] = Eigen::Quaternion<double>(qtmp[0],qtmp[1],qtmp[2],qtmp[3]);
        for(register int i = 0; i < 6; ++i) myfile1 >> translation_and_intrinsics(i,j);
    }
    // Reading points
    for (register int j = 0; j < num_features; ++j)
    {
        myfile1 >> structure(0,j);
        myfile1 >> structure(1,j);
        myfile1 >> structure(2,j);
    }
    // Format
//     <num_cameras> <num_points> <num_observations>
//     <camera_index_1> <point_index_1> <x_1> <y_1>
//     ...
//     <camera_index_num_observations> <point_index_num_observations> <x_num_observations> <y_num_observations>
//     <camera_1>
//     ...
//     <camera_num_cameras>
//     <point_1>
//     ...
//     <point_num_points>
}



// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================
void writePMVS(const char *output_path, std::vector<std::string> &nameImages, 
	     std::vector< Eigen::MatrixXd > &Cameras, Eigen::Matrix3d Calibration, std::vector< double > distortion)
{
    int num_cameras = Cameras.size();
    if ( nameImages.size() != num_cameras )
    {
        std::cerr << "\033[1;31m Can't write PMVS. #images is not equal to #cameras \033[0m\n";
        return;
    }
    
    // parameters to imwrite JPEG format
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(100); // from 0 to 100 (the higher is the better). Default value is 95.
    
    char buf[256];
    char *cmd = &buf[0];
    printf("Creating PMVS directories\n");
    sprintf(cmd, "rm -rf %s\n", output_path);		// Delete previous data pmvs
    system(cmd);
    sprintf(cmd, "mkdir -p %s/txt/\n", output_path);
    system(cmd);
    sprintf(cmd, "mkdir -p %s/visualize/\n", output_path);
    system(cmd);
    sprintf(cmd, "mkdir -p %s/models/\n", output_path);
    system(cmd);

    for(int cam = 0; cam < num_cameras; cam++)
    {
        // Creating txt file with cameras
        sprintf(buf, "%s/txt/%08d.txt", output_path, cam);
        FILE *f = fopen(buf, "w");
        assert(f);
        
        fprintf(f, "CONTOUR\n");
        fprintf(f, "%0.8e %0.8e %0.8e %0.8e\n", Cameras[cam](0,0), Cameras[cam](0,1), Cameras[cam](0,2), Cameras[cam](0,3));
        fprintf(f, "%0.8e %0.8e %0.8e %0.8e\n", Cameras[cam](1,0), Cameras[cam](1,1), Cameras[cam](1,2), Cameras[cam](1,3));
        fprintf(f, "%0.8e %0.8e %0.8e %0.8e\n", Cameras[cam](2,0), Cameras[cam](2,1), Cameras[cam](2,2), Cameras[cam](2,3));
        fclose(f);
        
        // Creating Images
        cv::Mat image = cv::imread(nameImages[cam], 1);
        cv::Mat und_img, K; 
        eigen2cv( Calibration, K);
        undistort(image, und_img, K, distortion);
        
        sprintf(cmd, "%s/visualize/%08d.jpg", output_path, cam);
        imwrite(cmd, und_img, compression_params);
    }
    
    // Write the options file for pmvs
    sprintf(buf, "%s/pmvs_options.txt", output_path);
    FILE *f_opt = fopen(buf, "w");

    fprintf(f_opt, "level 0\n");  // Default 1
    fprintf(f_opt, "csize 2\n");
    fprintf(f_opt, "threshold 0.7\n");
    fprintf(f_opt, "wsize 7\n");
    fprintf(f_opt, "minImageNum 3\n");
    fprintf(f_opt, "CPU 8\n");
    fprintf(f_opt, "setEdge 0\n");
    fprintf(f_opt, "useBound 0\n");
    fprintf(f_opt, "useVisData 0\n");	// use vis to improve speed
    fprintf(f_opt, "sequence -1\n");
    fprintf(f_opt, "timages -1 0 %d\n", num_cameras);
    fprintf(f_opt, "oimages -3\n");

    fclose(f_opt);

    return;
}

void undistortImages( const char * output_path, std::vector< std::string > &files_input,
		  Eigen::Matrix3d &Calibration, Eigen::MatrixXd &distortion,
		  const char * file_xml, std::vector< std::string > &undistort_files )
{
    // output_path is the folder
    undistort_files.clear();
    
    // parameters to imwrite PNG format
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(5); // PNG is lossless compression, compression 0 to 9, 
    
    char buf[256];
    char *cmd = &buf[0];
    printf("Creating undistort directories\n");
    sprintf(cmd, "rm -rf %s\n", output_path);		// Delete previous data pmvs
    system(cmd);
    sprintf(cmd, "mkdir -p %s/\n", output_path);
    system(cmd);
    
    cv::Mat K, coeff; 
    eigen2cv( Calibration, K );
    eigen2cv( distortion, coeff );
    
    for(std::vector< std::string >::iterator it = files_input.begin() ; it != files_input.end(); ++it)
    {
        // Loading Images
        
        cv::Mat image = cv::imread( *it, -1);
        cv::Mat und_img;
        undistort(image, und_img, K, coeff);
        
        std::stringstream ss;
        std::string basename = baseFileName( *it );
        ss << output_path << basename;
        std::string cmd = ss.str();
        imwrite(cmd, und_img, compression_params);
        undistort_files.push_back(cmd);
    }
    
    exportXMLImageList( file_xml, undistort_files );
}

void writeGraph( const char *filename, std::vector< Eigen::Quaternion<double> > &Qn_global, 
	       std::vector< Eigen::Vector3d > &tr_global )
{
//     char *file_txt = (char*)"pose.graph";
    std::ofstream myfile1;
    myfile1.open (filename);
    myfile1.precision(12);

    for(int it = 0; it < Qn_global.size(); it++)
    {
        Eigen::Matrix3d rr = Qn_global[it].toRotationMatrix();
//         Eigen::Vector3d tr = tr_global[it];
        Eigen::Vector3d tr = rr.transpose()*(-tr_global[it]); //center of coordinates
        Eigen::Vector3d angles;
        anglesfromRotation( rr, angles, false );
        
        myfile1 << "VERTEX3 ";
        myfile1 << it << " ";
        myfile1 << tr.transpose() << " ";
        myfile1 << angles.transpose();
        myfile1 << "\n";
    }
    
    // Local constraints
    for(int it = 1; it < Qn_global.size(); it++)
    {
        Eigen::Matrix3d rot1 = Qn_global[it-1].toRotationMatrix();
        Eigen::Matrix3d rot2 = Qn_global[it].toRotationMatrix();
//         Eigen::Vector3d tr1 = tr_global[it-1];
//         Eigen::Vector3d tr2 = tr_global[it];
        Eigen::Vector3d tr1 = rot1.transpose()*(-tr_global[it-1]);
        Eigen::Vector3d tr2 = rot2.transpose()*(-tr_global[it]);
        Eigen::Matrix<double,6,1> p_j, p_i, dji;
        Eigen::Vector3d angles1, angles2;
        
        anglesfromRotationZero( rot1, angles1, false );//false for radians units
        anglesfromRotationZero( rot2, angles2, false );//false for radians units
        
        p_i << tr1, angles1;
        p_j << tr2, angles2;
        
        Eigen::Matrix<double,6,6> Ri = Eigen::Matrix<double,6,6>::Identity();
        Ri.block(0,0,3,3) = rot1;
        
        dji = Ri.transpose()*(p_j - p_i);
        
        // Local angles correction
        Eigen::Matrix3d rr = rot2*rot1.transpose();
        anglesfromRotationZero( rr , angles1, false );//false for radians units
        dji.tail(3) = angles1;
        
        myfile1 << "EDGE3 ";
        myfile1 << it - 1 << " ";
        myfile1 << it << " ";
        myfile1 << dji.transpose() << " ";
//         myfile1 << "1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1";
        myfile1 << "\n";
    }
    myfile1.close();
}
