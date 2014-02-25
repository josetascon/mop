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
#include <string>

// Local Libraries
#include "Debug.hpp"

#ifndef CONSTANT_PI
#define CONSTANT_PI 3.14159265358979323846
#endif

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
      
      // Save a chrono time
      boost::int_least64_t lap_value;
      
public:
      //Constructor
      timer_wall();
      //Destructor
      ~timer_wall();
      
      //void clear_time_clock()
      /** Start your timer and store the initial point to measure **/
      void start();
      
      // Functions to get the result in different multiplier format
      /** Return elapsed time from start time to now in nanoseconds **/
      boost::int_least64_t elapsed_ns();
      /** Return elapsed time from start time to now in microseconds **/
      boost::int_least64_t elapsed_us();
      /** Return elapsed time from start time to now in milliseconds **/
      boost::int_least64_t elapsed_ms();
      /** Return elapsed time from start time to now in seconds **/
      double elapsed_s();
      
      boost::int_least64_t lap();
};

// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================
/**
 * ******************************************************************
 * @brief Template Function to get the sign of a number
 *
 * @author José David Tascón Vidarte
 * @date July/22/2013
 */
template <typename T>
T sign(T value)
{
    return ( value >= T(0)) ? T(1) : T(-1);
}

/**
 * ******************************************************************
 * @brief Template Function to convert a char pointer to a number
 *
 * @author José David Tascón Vidarte
 * @date Dec/11/2013
 */
template< typename Tp >
Tp pchar2number(char* text)
{
    Tp value;
    std::stringstream ss;	//create a stringstream
    ss << text;		//add number to the stream
    ss >> value;
    return value;		//return a string with the contents of the stream
}

/**
 * ******************************************************************
 * @brief Template Function to convert a string to a number
 *
 * @author José David Tascón Vidarte
 * @date Dec/11/2013
 */
template< typename Tp >
Tp string2number(std::string text)
{
    Tp value;
    std::stringstream ss;	//create a stringstream
    ss << text;		//add number to the stream
    ss >> value;
    return value;		//return a string with the contents of the stream
}

/**
 * ******************************************************************
 * @brief Template Function to convert a number to a string
 *
 * @author José David Tascón Vidarte
 * @date Dec/11/2013
 */
template< typename Tp >
std::string number2string(Tp number)
{
    std::stringstream ss;	//create a stringstream
    ss << number;		//add number to the stream
    return ss.str();	//return a string with the contents of the stream
}

/**
 * ******************************************************************
 * @brief Function to find a file name. For instace, the base name of a file string: /home/user/text.txt is text.txt
 *
 * @author José David Tascón Vidarte
 * @date Dec/11/2013
 */
std::string baseFileName (const std::string& str);
std::string extensionFileName (const std::string& str);
bool verifyFileExtension( const char *file_name, std::string& extension, bool stop_execution = true );

/**
 * ******************************************************************
 * @brief Function to print the data with spaces in a std::vector 
 *
 * @author José David Tascón Vidarte
 * @date Dec/11/2013
 */
template <typename Tf>
void printVector(std::vector<Tf> data)
{
    for (register int j = 0; j < data.size(); ++j) std::cout << data[j] << " ";
    std::cout << "\n";
}

/**
 * ******************************************************************
 * @brief Function to sum the data in a std::vector
 *
 * @author José David Tascón Vidarte
 * @date Dec/11/2013
 */
template <typename Tf>
Tf sumVector(std::vector<Tf> data)
{
    Tf sum = 0;
    for (register int j = 0; j < data.size(); ++j) sum += data[j];
    return sum;
}

int factorial(int x);
/**
 * ******************************************************************
 * @brief Function to calculate the Moore–Penrose pseudoinverse of a matrix
 *
 * @author José David Tascón Vidarte
 * @date July/22/2013
 */
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

cv::Mat vector2mat(cv::InputArray _src);


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
void point_vector2eigen(const std::vector< cv::Point_<T_pt> > &pts, Eigen::Matrix<T_eig,-1, -1> &D2d)
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
void eigen2point_vector(const Eigen::Matrix<T_eig,-1, -1> &D2d, std::vector< cv::Point_<T_pt> > &pts)
{
    std::vector< cv::Point_<T_pt> > V_Data(D2d.cols());
    for (register int i = 0; i < D2d.cols(); ++i)
    {
        V_Data[i].x = T_pt( D2d(0,i) );
        V_Data[i].y = T_pt( D2d(1,i) );
    }
    pts = V_Data;
};

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
void point3_vector2eigen(const std::vector< cv::Point3_<T_pt> > &pts, Eigen::Matrix<T_eig,-1, -1> &D3d)
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
void eigen2point3_vector( const Eigen::Matrix<T_eig,-1, -1> &D3d, std::vector< cv::Point3_<T_pt> > &pts)
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

template <typename T_eig>
void eigen_vector2eigen(const std::vector< Eigen::Matrix<T_eig, 4, 1> > &pts, Eigen::Matrix<T_eig, -1, -1> &D3d)
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
void quaternion2vector(const Eigen::Quaternion<T_qt> &quat_In, std::vector<T_vec> &quat_Vec )
{
    quat_Vec.resize(4,T_vec(0.0));
    quat_Vec[0] = T_vec( quat_In.w() );
    quat_Vec[1] = T_vec( quat_In.x() );
    quat_Vec[2] = T_vec( quat_In.y() );
    quat_Vec[3] = T_vec( quat_In.z() );
};

template <typename T_vec, typename T_qt>
void quaternion_vector2vector_vector(const std::vector< Eigen::Quaternion<T_qt> > &quat_In, std::vector< std::vector<T_vec> > &quat_Vec )
{
    int num_cams = quat_In.size();
    quat_Vec.clear();
    quat_Vec.resize(num_cams);
    
    for (register int cam = 0; cam < num_cams; ++cam) quaternion2vector( quat_In[cam], quat_Vec[cam] );
};

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
void quaternion2eigen(const Eigen::Quaternion<T_qt> &quat_In, Eigen::Matrix<T_evec,4,1> &quat_Vec )
{
    quat_Vec = Eigen::Matrix<T_evec,4,1>( T_evec(quat_In.w()), T_evec(quat_In.x()), T_evec(quat_In.y()), T_evec(quat_In.z()) );
};

template <typename T_evec, typename T_qt>
void quaternion_vector2eigen_vector(const std::vector< Eigen::Quaternion<T_qt> > &quat_In, std::vector< Eigen::Matrix<T_evec,4,1> > &quat_Vec )
{
    int num_cams = quat_In.size();
    quat_Vec.clear();
    quat_Vec.resize(num_cams);
    
    for (register int cam = 0; cam < num_cams; ++cam) quaternion2eigen( quat_In[cam], quat_Vec[cam] );
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
void vector2quaternion( const std::vector<T_vec> &qVec, Eigen::Quaternion<T_qt> &quat_Out )
{
    quat_Out = Eigen::Quaternion<T_qt>( T_qt(qVec[0]), T_qt(qVec[1]), T_qt(qVec[2]), T_qt(qVec[3]) );
};

template <typename T_vec, typename T_qt>
void vector_vector2quaternion_vector( const std::vector< std::vector<T_vec> > &qVec, std::vector< Eigen::Quaternion<T_qt> > &quat_Out )
{
    int num_cams = qVec.size();
    quat_Out.clear();
    quat_Out.resize(num_cams);
    
    for (register int cam = 0; cam < num_cams; ++cam) vector2quaternion( qVec[cam], quat_Out[cam] );
};

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
void eigen2quaternion( const Eigen::Matrix<T_evec,4,1> &qVec, Eigen::Quaternion<T_qt> &quat_Out )
{
    quat_Out = Eigen::Quaternion<T_qt>( T_qt(qVec(0)), T_qt(qVec(1)), T_qt(qVec(2)), T_qt(qVec(3)) );
};

template <typename T_evec, typename T_qt>
void eigen_vector2quaternion_vector( const std::vector< Eigen::Matrix<T_evec,4,1> > &qVec, std::vector< Eigen::Quaternion<T_qt> > &quat_Out )
{
    int num_cams = qVec.size();
    quat_Out.clear();
    quat_Out.resize(num_cams);
    
    for (register int cam = 0; cam < num_cams; ++cam) eigen2quaternion( qVec[cam], quat_Out[cam] );
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
void eigen2vector_vector( const Eigen::Matrix<T_eig,-1, -1> &matrixE, std::vector< std::vector<T_vec> > &data )
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
void vector_vector2eigen( const std::vector< std::vector<T_vec> > &data, Eigen::Matrix<T_eig,-1, -1> &matrixE )
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

/**
 * ******************************************************************
 * @brief Description: Converts eigen matrix to his homogeneous form (adding a last coordinate with 1 for each column)
 * 
 * @param data_inhom 	(input) Eigen matrix with size [rows x cols]
 * @param data_hom 		(output) Eigen matrix with size [(rows+1) x cols)]. Last row all ones
 * 
 * @date Jun/28/2013
 */
template< typename Teig >
void homogeneous(Eigen::Matrix<Teig,-1,-1> &data_inhom, Eigen::Matrix<Teig,-1,-1> &data_hom)
{
    Eigen::Matrix<Teig,-1,-1> Out(data_inhom.rows()+1,data_inhom.cols());
    Eigen::Matrix<Teig,1,-1> Un = Eigen::Matrix<Teig,1,-1>::Ones(data_inhom.cols()); // RowVectorX
    Out << data_inhom, Un;
    data_hom = Out;
}

/**
 * ******************************************************************
 * @brief Description: Normalize a Eigen Matrix with homogeneous values (divive each col by his last value)
 * 
 * @param data	 	(input/output) Eigen matrix with size [rows x cols]
 * 
 * @date Jun/28/2013
 */
template< typename Teig >
void normalizeHomogeneous(Eigen::Matrix<Teig,-1,-1> &data)
{
    // Divide each col by his last value
    for (register int i = 0; i < data.cols(); ++i) data.col(i) /= fabs(data(data.rows()-1,i));
}

// Designed to count positive in z axis for 3d points
template < typename Teig >
int countPositivesInRow(Eigen::Matrix<Teig,-1,-1> &data, int row)
{
    int count = 0;
    for (int k = 0; k < data.cols(); k++)
        if (data(row,k)> 0.0) count++; 
        
    return count;
}

/**
 * ******************************************************************
 * @brief Description: Find euler angles from a rotation matrix. R = Rx*Ry*Rz
 * 
 * @param Rot 		(input) Rotation Matrix
 * @param angles		(output) Euler Angles
 * 
 * @date Jul/08/2013
 */
template< typename Tp >
void rotation2angles(Eigen::Matrix< Tp, 3, 3 > &Rot, Eigen::Matrix< Tp, 3, 1 > &angles, bool degrees = true)
{
    Tp anglex = 0.0;
    Tp angley = 0.0;
    Tp anglez = 0.0;
    
    anglex = atan2( Rot(1,2), Rot(2,2) );
    angley = atan2( -Rot(0,2), sqrt( pow( Rot(0,0), 2.0 ) + pow( Rot(0,1), 2.0 ) ) );
    
    Tp s1 = sin(anglex); Tp c1 = cos(anglex);
    anglez = atan2( s1*Rot(2,0) - c1*Rot(1,0) , c1*Rot(1,1) - s1*Rot(2,1) );
    
    // radians to degrees convertion
    if(degrees)
        angles = Eigen::Matrix< Tp, 3, 1 >( -anglex*180/CONSTANT_PI, -angley*180/CONSTANT_PI, -anglez*180/CONSTANT_PI );
    else 
        angles = Eigen::Matrix< Tp, 3, 1 >( -anglex, -angley, -anglez );
    return;
}

template< typename Tp >
void rotation2angles_DetectZero(Eigen::Matrix< Tp, 3, 3 > &Rot, Eigen::Matrix< Tp, 3, 1 > &angles, bool degrees = true)
{
    Tp anglex = 0.0;
    Tp angley = 0.0;
    Tp anglez = 0.0;
    Eigen::Matrix< Tp, 3, 3 > RR = Rot;
    
    for (register int i = 0; i < 3; ++i) 
        for (register int j = 0; j < 3; ++j) 
	  if ( std::abs(RR(i,j)) < 1e-10 ) RR(i,j) = 0.0;
    
    rotation2angles<Tp>( RR, angles, degrees);
    return;
}

template< typename Tp >
Eigen::Matrix< Tp, 4, 4 > transformationMatrix( Eigen::Quaternion< Tp > &quaternion, Eigen::Matrix< Tp, 3, 1 > &translation)
{
    Eigen::Matrix< Tp, 3, 3 > rr = quaternion.toRotationMatrix();
    Eigen::Matrix< Tp, 4, 4 > ttmm;
    ttmm << rr, translation, 0.0, 0.0, 0.0, 1.0;
    return ttmm;
}

Eigen::Vector3d radialDistortionCorrection( Eigen::Vector3d &pt2d, Eigen::Matrix3d &kalib, Eigen::VectorXd &distCoeff );

#endif