/**
 * @file Common.hpp
 * @brief This file has all usefull functions for data conversion
 *
 * @author José David Tascón Vidarte
 * @date Jun/28/2013
 */

#ifndef __COMMON_HPP__
#define __COMMON_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// OpenCV Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

// Library Boost 
#include <boost/chrono/include.hpp>

// Std Libraries
#include <iostream>
#include <fstream>
#include <string>

#define pi 3.14159265358979323846

// ================================================================================================
// ======================================= CLASS timer_wall =======================================
// ================================================================================================
/**
 * ******************************************************************
 * @brief The class timer_wall is an easy to use timer clock with different units outputs. Based on Boost
 *
 * @author José David Tascón Vidarte
 * @date December/06/2012
 */
class timer_wall
{
private:
      // Time point declaration boost::chrono
      boost::chrono::steady_clock::time_point t_0;
      // Durations declaration boost::chrono
      boost::chrono::steady_clock::duration ns_time_f;
      boost::chrono::microseconds us_time_f;
      boost::chrono::milliseconds ms_time_f;
      boost::chrono::duration<double> s_time_f;
      
public:
      //Constructor
      timer_wall();
      //Destructor
      ~timer_wall();
      
      //void clear_time_clock()
      // Start your your timer and store the initial point to measure
      void start();
      
      // Functions to get the result in different multiplier format
      // Return nanoseconds
      boost::int_least64_t elapsed_ns();
      // Return microseconds
      boost::int_least64_t elapsed_us();
      // Return milliseconds
      boost::int_least64_t elapsed_ms();
      // Return seconds
      double elapsed_s();
};

// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================

// @date July/22/2013
template <typename T>
T sign(T value)
{
    return ( value >= T(0)) ? T(1) : T(-1);
}

// @date July/22/2013
// template Dec/11/2013
template< typename Tp >
Tp pchar2number(char* text)
{
    Tp value;
    std::stringstream ss;	//create a stringstream
    ss << text;		//add number to the stream
    ss >> value;
    return value;		//return a string with the contents of the stream
}

template< typename Tp >
Tp string2number(std::string text)
{
    Tp value;
    std::stringstream ss;	//create a stringstream
    ss << text;		//add number to the stream
    ss >> value;
    return value;		//return a string with the contents of the stream
}

template< typename Tp >
std::string number2string(Tp number)
{
    std::stringstream ss;	//create a stringstream
    ss << number;		//add number to the stream
    return ss.str();	//return a string with the contents of the stream
}


std::string baseFileName (const std::string& str);

template <typename Tf>
void printVector(std::vector<Tf> data)
{
    for (register int j = 0; j < data.size(); ++j) std::cout << data[j] << " ";
    std::cout << "\n";
}

template <typename Tf>
Tf sumVector(std::vector<Tf> data)
{
    Tf sum = 0;
    for (register int j = 0; j < data.size(); ++j) sum += data[j];
    return sum;
}


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

    for(int it = 0; it < Qn_global.size(); it++)
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

    for(int it = 0; it < tr_global.size(); it++)
    {
        Eigen::Matrix<T_eig,3,1> tr = tr_global[it];
        myfile1 << tr.transpose() << "\n";
    }
    myfile1.close();
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
bool importXMLImageList(const char *file_xml, std::vector< std::string > &files_names);

void exportXMLImageList(const char *file_xml, std::vector< std::string > &files_names);




int factorial(int x);
Eigen::MatrixXd pseudoInverse( Eigen::MatrixXd &A);
Eigen::RowVector3d pseudoInverse(  Eigen::Vector3d &A);

template<typename T>
inline void AngleAxisRToQuaternion(const T* angle_axis, T* quaternion) {
  const T& a0 = angle_axis[0];
  const T& a1 = angle_axis[1];
  const T& a2 = angle_axis[2];
  const T theta_squared = a0 * a0 + a1 * a1 + a2 * a2;

  // For points not at the origin, the full conversion is numerically stable.
  if (theta_squared > T(0.0)) {
    const T theta = sqrt(theta_squared);
    const T half_theta = theta * T(0.5);
    const T k = sin(half_theta) / theta;
    quaternion[0] = cos(half_theta);
    quaternion[1] = a0 * k;
    quaternion[2] = a1 * k;
    quaternion[3] = a2 * k;
  } else {
    // At the origin, sqrt() will produce NaN in the derivative since
    // the argument is zero.  By approximating with a Taylor series,
    // and truncating at one term, the value and first derivatives will be
    // computed correctly when Jets are used.
    const T k(0.5);
    quaternion[0] = T(1.0);
    quaternion[1] = a0 * k;
    quaternion[2] = a1 * k;
    quaternion[3] = a2 * k;
  }
}

/**
 * ******************************************************************
 * @brief Description: Euclidean distance of 2D points. Equivalent to sqrt( (x1-x2)^2 + (y1-y2)^2 ).
 * 
 * @param pt1 		(input)
 * @param pt2 		(input)
 * 
 * @date Jul/12/2013
 */
float euclideanDistanceofcvPoints(cv::Point2f pt1, cv::Point2f pt2 );

cv::Mat vector2Mat(cv::InputArray _src);


/**
 * ******************************************************************
 * @brief Description: Converts vector of Point2d to a eigen matrix
 * 
 * @param pts 		(input) std::vector of Point2d
 * @param D2d 		(output) Eigen matrix with size [2 x n]
 * 
 * @date Jun/28/2013
 */
template <typename T_pt, typename T_eig>
void convertPoint2_toEigen(const std::vector< cv::Point_<T_pt> > &pts, Eigen::Matrix<T_eig,-1, -1> &D2d)
{
    Eigen::Matrix<T_eig,-1, -1> M_Data(2,pts.size());
    for (register int i = 0; i < pts.size(); ++i)
    {
        M_Data(0,i) = T_eig( pts[i].x ); //casting
        M_Data(1,i) = T_eig( pts[i].y );
    }
    D2d = M_Data;
};

template <typename T_pt, typename T_eig>
void convertEigentoPoint2_(const Eigen::Matrix<T_eig,-1, -1> &D2d, std::vector< cv::Point_<T_pt> > &pts)
{
    std::vector< cv::Point_<T_pt> > V_Data(D2d.cols());
    for (register int i = 0; i < D2d.cols(); ++i)
    {
        V_Data[i].x = T_pt( D2d(0,i) );
        V_Data[i].y = T_pt( D2d(1,i) );
    }
    pts = V_Data;
};

// void convertPoint2dtoEigen(std::vector<cv::Point2d> &pts, Eigen::MatrixXd &D2d);
// void convertEigentoPoint2d( Eigen::MatrixXd &D2d, std::vector<cv::Point2d> &pts);

/**
 * ******************************************************************
 * @brief Description: Converts vector of Point3d to a eigen matrix
 * 
 * @param pts 		(input) std::vector of Point3d
 * @param D3d 		(output) Eigen matrix with size [3 x n]
 * 
 * @date Jun/28/2013, update to template Sep/18/2013
 */
template <typename T_pt, typename T_eig>
void convertPoint3_toEigen(const std::vector< cv::Point3_<T_pt> > &pts, Eigen::Matrix<T_eig,-1, -1> &D3d)
{
    Eigen::Matrix<T_eig,-1, -1> M_Data(3,pts.size());
    for (register int i = 0; i < pts.size(); ++i)
    {
        M_Data(0,i) = T_eig( pts[i].x );
        M_Data(1,i) = T_eig( pts[i].y );
        M_Data(2,i) = T_eig( pts[i].z );
    }
    D3d = M_Data;
};

template <typename T_pt, typename T_eig>
void convertEigentoPoint3_( const Eigen::Matrix<T_eig,-1, -1> &D3d, std::vector< cv::Point3_<T_pt> > &pts)
{
    std::vector< cv::Point3_<T_pt> > V_Data(D3d.cols());
    for (register int i = 0; i < D3d.cols(); ++i)
    {
        V_Data[i].x = T_pt( D3d(0,i) );
        V_Data[i].y = T_pt( D3d(1,i) );
        V_Data[i].z = T_pt( D3d(2,i) );
    }
    pts = V_Data;
};

// void convertPoint3dtoEigen(std::vector<cv::Point3d> &pts, Eigen::MatrixXd &D3d);
// void convertEigentoPoint3d( Eigen::MatrixXd &D3d, std::vector<cv::Point3d> &pts);

template <typename T_eig>
void convertVector4EtoEigen(const std::vector< Eigen::Matrix<T_eig, 4, 1> > &pts, Eigen::Matrix<T_eig, -1, -1> &D3d)
{
    Eigen::Matrix<T_eig,-1, -1> M_Data(4,pts.size());
    for (register int i = 0; i < pts.size(); ++i)
    {
        M_Data.col(i) = pts[i];
    }
    D3d = M_Data;
};

/**
 * ******************************************************************
 * @brief Description: Converts a Eigen::Quaternion to a std::vector
 * 
 * @param quat_In 		(input) Eigen Quaternion
 * @param quat_Vec 		(output) std::vector 
 * 
 * @date Jun/28/2013, update to template Sep/18/2013
 */
template <typename T_vec, typename T_qt>
void convertQuaterniontoVector(const Eigen::Quaternion<T_qt> &quat_In, std::vector<T_vec> &quat_Vec )
{
    quat_Vec.resize(4,T_vec(0.0));
    quat_Vec[0] = T_vec( quat_In.w() );
    quat_Vec[1] = T_vec( quat_In.x() );
    quat_Vec[2] = T_vec( quat_In.y() );
    quat_Vec[3] = T_vec( quat_In.z() );
};

template <typename T_vec, typename T_qt>
void convertSetQuaterniontoVector(const std::vector< Eigen::Quaternion<T_qt> > &quat_In, std::vector< std::vector<T_vec> > &quat_Vec )
{
    int num_cams = quat_In.size();
    quat_Vec.clear();
    quat_Vec.resize(num_cams);
    
    for (register int cam = 0; cam < num_cams; ++cam) convertQuaterniontoVector( quat_In[cam], quat_Vec[cam] );
};

// void convertQuaterniontoVector( Eigen::Quaternion<double> &quat_In, std::vector<double> &quat_Vec );
// void convertSetQuaterniontoVector( std::vector< Eigen::Quaternion<double> > &quat_In, std::vector< std::vector<double> > &quat_Vec );

/**
 * ******************************************************************
 * @brief Description: Converts a Eigen::Quaternion to a Eigen::Matrix (Vector)
 * 
 * @param quat_In 		(input) Eigen Quaternion
 * @param quat_Vec 		(output) Eigen::Matrix 
 * 
 * @date Sep/29/2013
 */
template <typename T_evec, typename T_qt>
void convertQuaterniontoEigenVector(const Eigen::Quaternion<T_qt> &quat_In, Eigen::Matrix<T_evec,4,1> &quat_Vec )
{
    quat_Vec = Eigen::Matrix<T_evec,4,1>( T_evec(quat_In.w()), T_evec(quat_In.x()), T_evec(quat_In.y()), T_evec(quat_In.z()) );
};

template <typename T_evec, typename T_qt>
void convertSetQuaterniontoEigenVector(const std::vector< Eigen::Quaternion<T_qt> > &quat_In, std::vector< Eigen::Matrix<T_evec,4,1> > &quat_Vec )
{
    int num_cams = quat_In.size();
    quat_Vec.clear();
    quat_Vec.resize(num_cams);
    
    for (register int cam = 0; cam < num_cams; ++cam) convertQuaterniontoEigenVector( quat_In[cam], quat_Vec[cam] );
};

/**
 * ******************************************************************
 * @brief Description: Converts a std::vector to a Eigen::Quaternion
 * 
 * @param qVec 		(input) std::vector
 * @param quat_Out 		(output) Eigen Quaternion
 * 
 * @date Jun/28/2013, update to template Sep/18/2013
 */
template <typename T_vec, typename T_qt>
void convertVectortoQuaternion( const std::vector<T_vec> &qVec, Eigen::Quaternion<T_qt> &quat_Out )
{
    quat_Out = Eigen::Quaternion<T_qt>( T_qt(qVec[0]), T_qt(qVec[1]), T_qt(qVec[2]), T_qt(qVec[3]) );
};

template <typename T_vec, typename T_qt>
void convertSetVectortoQuaternion( const std::vector< std::vector<T_vec> > &qVec, std::vector< Eigen::Quaternion<T_qt> > &quat_Out )
{
    int num_cams = qVec.size();
    quat_Out.clear();
    quat_Out.resize(num_cams);
    
    for (register int cam = 0; cam < num_cams; ++cam) convertVectortoQuaternion( qVec[cam], quat_Out[cam] );
//         quat_Out[cam] = Eigen::Quaternion<double>( qVec[cam][0], qVec[cam][1], qVec[cam][2], qVec[cam][3] );
};

// void convertVectortoQuaternion( std::vector<double> &qVec, Eigen::Quaternion<double> &quat_Out );
// void convertSetVectortoQuaternion( std::vector< std::vector<double> > &qVec, std::vector< Eigen::Quaternion<double> > &quat_Out );

/**
 * ******************************************************************
 * @brief Description: Converts a Eigen::Matrix (Vector) to a Eigen::Quaternion
 * 
 * @param quat_In 		(input) Eigen::Matrix
 * @param quat_Vec 		(output) Eigen Quaternion
 * 
 * @date Sep/29/2013
 */
template <typename T_evec, typename T_qt>
void convertEigenVectortoQuaternion( const Eigen::Matrix<T_evec,4,1> &qVec, Eigen::Quaternion<T_qt> &quat_Out )
{
    quat_Out = Eigen::Quaternion<T_qt>( T_qt(qVec(0)), T_qt(qVec(1)), T_qt(qVec(2)), T_qt(qVec(3)) );
};

template <typename T_evec, typename T_qt>
void convertSetEigenVectortoQuaternion( const std::vector< Eigen::Matrix<T_evec,4,1> > &qVec, std::vector< Eigen::Quaternion<T_qt> > &quat_Out )
{
    int num_cams = qVec.size();
    quat_Out.clear();
    quat_Out.resize(num_cams);
    
    for (register int cam = 0; cam < num_cams; ++cam) convertEigenVectortoQuaternion( qVec[cam], quat_Out[cam] );
};

/**
 * ******************************************************************
 * @brief Description: Converts a general Eigen::Matrix to a std::vector< std::vector > and viceversa
 * 
 * @param matrixE 		(input) Eigen Matrix
 * @param data 		(output) std::vector< std::vector >
 * 
 * @date Jun/28/2013, update to template Sep/18/2013
 */
template <typename T_vec, typename T_eig>
void convertEigentoVector( const Eigen::Matrix<T_eig,-1, -1> &matrixE, std::vector< std::vector<T_vec> > &data )
{
    // Conceive from the needs to change Structure matrix to Vector
    int num_cols = matrixE.cols();
    int num_rows = matrixE.rows();
    data.clear();
    data.resize(num_cols);
    
    for (register int ft = 0; ft < num_cols ; ++ft)
    {
        data[ft].resize(num_rows);
        for (register int x = 0; x < num_rows; ++x)
        {
	  data[ft][x] = T_vec( matrixE(x,ft) );
        }
    }
};

template <typename T_vec, typename T_eig>
void convertVectortoEigen( const std::vector< std::vector<T_vec> > &data, Eigen::Matrix<T_eig,-1, -1> &matrixE )
{
    // Conceive from the needs to change Structure matrix to Vector
    int num_cols = data.size();
    int num_rows = data[0].size();
    Eigen::Matrix<T_eig,-1, -1> MM(num_rows,num_cols);
    
    for (register int ft = 0; ft < num_cols ; ++ft)
    {
        for (register int x = 0; x < num_rows; ++x)
        {
	  MM(x,ft) = T_eig( data[ft][x] );
        }
    }
    matrixE = MM;
};

// void convertEigentoVector( Eigen::MatrixXd &matrixE, std::vector< std::vector<double> > &data );
// void convertVectortoEigen( std::vector< std::vector<double> > &data, Eigen::MatrixXd &matrixE );

/**
 * ******************************************************************
 * @brief Description: Converts eigen matrix to his homogeneous form (adding a last coordinate with 1 for each column)
 * 
 * @param data_inhom 	(input) Eigen matrix with size [rows x cols]
 * @param data_hom 		(output) Eigen matrix with size [(rows+1) x cols)]. Last row all ones
 * 
 * @date Jun/28/2013
 */
void convertHomogeneous(Eigen::MatrixXd &data_inhom, Eigen::MatrixXd &data_hom);

/**
 * ******************************************************************
 * @brief Description: Normalize a Eigen Matrix with homogeneous values (divive each col by his last value)
 * 
 * @param data	 	(input/output) Eigen matrix with size [rows x cols]
 * 
 * @date Jun/28/2013
 */
void normalizeHomogeneous(Eigen::MatrixXd &data);


int countZPositive(Eigen::MatrixXd &data);

/**
 * ******************************************************************
 * @brief Description: Find euler angles from a rotation matrix. R = Rx*Ry*Rz
 * 
 * @param Rot 		(input) Rotation Matrix
 * @param angles		(output) Euler Angles
 * 
 * @date Jul/08/2013
 */
void anglesfromRotation(Eigen::Matrix3d &Rot, Eigen::Vector3d &angles, bool degrees = true);

void anglesfromRotationZero(Eigen::Matrix3d &Rot, Eigen::Vector3d &angles, bool degrees = true);

Eigen::Vector3d radialDistortionCorrection( Eigen::Vector3d &pt2d, Eigen::Matrix3d &kalib, Eigen::VectorXd &distCoeff );

#endif