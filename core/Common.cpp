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
    return ns_time_f.count();
}

boost::int_least64_t timer_wall::elapsed_us()
{
    us_time_f = boost::chrono::duration_cast<boost::chrono::microseconds>(boost::chrono::steady_clock::now() - t_0);
    return us_time_f.count();
}

boost::int_least64_t timer_wall::elapsed_ms()
{
    ms_time_f = boost::chrono::duration_cast<boost::chrono::milliseconds>(boost::chrono::steady_clock::now() - t_0);
    return ms_time_f.count();
}

double timer_wall::elapsed_s()
{
    s_time_f = (boost::chrono::steady_clock::now() - t_0);
    return s_time_f.count();
}



// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================

std::string int2string(int number)
{
    std::stringstream ss;	//create a stringstream
    ss << number;		//add number to the stream
    return ss.str();	//return a string with the contents of the stream
}

int pchar2int(char* number)
{
    int value;
    std::stringstream ss;	//create a stringstream
    ss << number;		//add number to the stream
    ss >> value;
    return value;		//return a string with the contents of the stream
}

float pchar2float(char* number)
{
    float value;
    std::stringstream ss;	//create a stringstream
    ss << number;		//add number to the stream
    ss >> value;
    return value;
}

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

cv::Mat vector2Mat(cv::InputArray _src)
{
    return _src.getMat();
}

void convertHomogeneous(Eigen::MatrixXd &data_inhom, Eigen::MatrixXd &data_hom)
{
    Eigen::MatrixXd Out(data_inhom.rows()+1,data_inhom.cols());
    Eigen::RowVectorXd Un = Eigen::RowVectorXd::Ones(data_inhom.cols());
    Out << data_inhom, Un;
    data_hom = Out;
}

void normalizeHomogeneous(Eigen::MatrixXd &data)
{
    // Divide each col by his last value
    for (int i = 0; i < data.cols(); i++) data.col(i) /= fabs(data(data.rows()-1,i));
}

int countZPositive(Eigen::MatrixXd &data)
{
    int count = 0;
    for (int k = 0; k < data.cols(); k++)
        if (data(2,k)> 0.0) count++; // Z axis NEGATIVE DUE TO left hand
        
    return count;
}

bool readStringList( const std::string& filename, std::vector<std::string>& location )		//Read XML Files
{
    location.resize(0);
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    cv::FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != cv::FileNode::SEQ )
        return false;
    cv::FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        location.push_back((std::string)*it);
    return true;
}

void anglesfromRotation(Eigen::Matrix3d &Rot, Eigen::Vector3d &angles, bool degrees)
{
    double anglex = 0.0;
    double angley = 0.0;
    double anglez = 0.0;
    
    anglex = atan2( Rot(1,2), Rot(2,2) );
    angley = atan2( -Rot(0,2), sqrt( pow( Rot(0,0), 2.0 ) + pow( Rot(0,1), 2.0 ) ) );
    
    double s1 = sin(anglex); double c1 = cos(anglex);
    anglez = atan2( s1*Rot(2,0) - c1*Rot(1,0) , c1*Rot(1,1) - s1*Rot(2,1) );
    
    // radians to degrees convertion
    if(degrees)
        angles = Eigen::Vector3d( -anglex*180/pi, -angley*180/pi, -anglez*180/pi );
    else 
        angles = Eigen::Vector3d( -anglex, -angley, -anglez );
//     angles(0) = -anglex*180/pi;
//     angles(1) = -angley*180/pi;
//     angles(2) = -anglez*180/pi;
    return;
}

void anglesfromRotationZero(Eigen::Matrix3d &Rot, Eigen::Vector3d &angles, bool degrees)
{
    double anglex = 0.0;
    double angley = 0.0;
    double anglez = 0.0;
    Eigen::Matrix3d RR = Rot;
    
    for (register int i = 0; i < 3; ++i) 
        for (register int j = 0; j < 3; ++j) 
	  if ( std::abs(RR(i,j)) < 1e-10 ) RR(i,j) = 0.0;
    
    anglex = atan2( RR(1,2), RR(2,2) );
    angley = atan2( -RR(0,2), sqrt( pow( RR(0,0), 2.0 ) + pow( RR(0,1), 2.0 ) ) );
    
    double s1 = sin(anglex); double c1 = cos(anglex);
    anglez = atan2( s1*RR(2,0) - c1*RR(1,0) , c1*RR(1,1) - s1*RR(2,1) );
    
    // radians to degrees convertion
    if(degrees)
        angles = Eigen::Vector3d( -anglex*180/pi, -angley*180/pi, -anglez*180/pi );
    else 
        angles = Eigen::Vector3d( -anglex, -angley, -anglez );
    
    
    return;
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