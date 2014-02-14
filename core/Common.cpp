// Created: Jun/28/2013
// Author: José David Tascón Vidarte

#include "Common.hpp"

// ================================================================================================
// =============================== FUNCTIONS of CLASS timer_wall ==================================
// ================================================================================================
// Constructor
timer_wall::timer_wall() 
{
    ;
}

// Destructor
timer_wall::~timer_wall() 
{
    ;
}

//void clear_time_clock()
// Start your your timer and store the initial point to measure
void timer_wall::start()
{
    t_0 = boost::chrono::steady_clock::now();
}

// Functions to get the result in different multiplier format
boost::int_least64_t timer_wall::elapsed_ns()
{
    ns_time_f = (boost::chrono::steady_clock::now() - t_0);
    lap_value = ns_time_f.count();
    return ns_time_f.count();
}

boost::int_least64_t timer_wall::elapsed_us()
{
    us_time_f = boost::chrono::duration_cast<boost::chrono::microseconds>(boost::chrono::steady_clock::now() - t_0);
    lap_value = us_time_f.count();
    return us_time_f.count();
}

boost::int_least64_t timer_wall::elapsed_ms()
{
    ms_time_f = boost::chrono::duration_cast<boost::chrono::milliseconds>(boost::chrono::steady_clock::now() - t_0);
    lap_value = ms_time_f.count();
    return ms_time_f.count();
}

double timer_wall::elapsed_s()
{
    s_time_f = (boost::chrono::steady_clock::now() - t_0);
    lap_value = (boost::int_least64_t) s_time_f.count(); //casting
    return s_time_f.count();
}

boost::int_least64_t timer_wall::lap()
{
    return lap_value;
}
    
    



// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================

std::string baseFileName (const std::string& str)
{
//   std::cout << "Splitting: " << str << '\n';
  unsigned found = str.find_last_of("/\\");
//   std::cout << " path: " << str.substr(0,found) << '\n';
//   std::cout << " file: " << str.substr(found+1) << '\n';
  return str.substr(found+1);
}

int factorial(int x)
{
    return (x == 1 ? x : x * factorial(x - 1));
}

Eigen::MatrixXd pseudoInverse( Eigen::MatrixXd &A)
{
    Eigen::MatrixXd B = (A.transpose()*A).inverse()*A.transpose();
    return B;
}

Eigen::RowVector3d pseudoInverse(  Eigen::Vector3d &A)
{
    Eigen::RowVector3d B = ( (A.transpose()*A).inverse() )*A.transpose();
    return B;
}

float euclideanDistanceofcvPoints(cv::Point2f pt1, cv::Point2f pt2 )
{
    cv::Point2f result = pt1 - pt2;
    return sqrt(pow(result.x,2.0) + pow(result.y,2.0));
}

cv::Mat vector2mat(cv::InputArray _src)
{
    return _src.getMat();
}

Eigen::Vector3d radialDistortionCorrection( Eigen::Vector3d &pt2d, Eigen::Matrix3d &kalib, Eigen::VectorXd &distCoeff )
{
    double fx = kalib(0,0);
    double fy = kalib(1,1);
    double cx = kalib(0,2);
    double cy = kalib(1,2);
    
    pt2d(0) = (pt2d(0)-cx)/fx;
    pt2d(1) = (pt2d(1)-cy)/fy; //// CORRECT Y AXIS
    
    int num_coeff = distCoeff.size();
    //double k1 = 0.0, k2 = 0, p1 = 0.0, p2 = 0.0, k3 = 0.0, k4 = 0.0, k5 = 0.0, k6 = 0.0;
    Eigen::VectorXd dC = Eigen::VectorXd::Zero(8);
    for (int k = 0; k < num_coeff; k++) dC(k) = distCoeff(k);
    
    double r2 = pt2d(0)*pt2d(0) + pt2d(1)*pt2d(1);
    
    double rf = (1 + dC(0)*r2 + dC(1)*r2*r2 + dC(4)*r2*r2*r2)/(1 + dC(5)*r2 + dC(6)*r2*r2 + dC(7)*r2*r2*r2);
    double tanfx = (dC(2)*(r2 + 2*pt2d(0)*pt2d(0))) + 2*dC(3)*pt2d(0)*pt2d(1);
    double tanfy = (dC(2)*(r2 + 2*pt2d(1)*pt2d(1))) + 2*dC(3)*pt2d(0)*pt2d(1);
    
    double xnew = pt2d(0)*rf + tanfx;
    double ynew = pt2d(1)*rf + tanfy;
    
    Eigen::Vector3d out;
    
    out(0) = xnew*fx + cx; 
    out(1) = ynew*fy + cy;
    out(2) = 1.0;
    return out;
}

// void radialDistortionCorrectionSet( Eigen::MatrixXd &pt2d, Eigen::Matrix3d &kalib, Eigen::VectorXd &distCoeff )
// {
//     Eigen::MatrixXd
//     
//     for(int i=0; i < p1hom.cols(); i++)
//     {
//         radialDistortionCorrection( p1hom.col(i), Keigen, distC_Eig );
//     
//     
// }