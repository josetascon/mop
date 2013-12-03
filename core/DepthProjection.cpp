// Created: May/08/2013
// Author: José David Tascón Vidarte

#include "DepthProjection.hpp"

// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================

void removeBadPoints(std::vector< cv::Point2d > &image_point, cv::Mat &measurez, bool close_range)
{
    // ==================================== If only close range data is desired ====================================
    // If close_range Bad points aren't stored (equals to 0, > 3 meters, or < 0.5 meters).
    int cnt = -1; //Debug
    if (close_range)
    {
        for ( int i = 0; i < image_point.size(); i++)
        {
	  cnt++; //Debug
	  int u = (int)image_point[i].x;
	  int v = (int)image_point[i].y;
	  if (measurez.at<short>(v,u) == 0 || measurez.at<short>(v,u) > 15000.0 || measurez.at<short>(v,u) < 0.5*5000.0)
	  {
	      // Debug
	      // std::cout << "Remove point " << i << '\n';
	      image_point.erase( image_point.begin() + i);
	      i--;
	      continue;
	  }
        }
    }
    else
    {
        for ( int i = 0; i < image_point.size(); i++)
        {
	  int u = (int)image_point[i].x;
	  int v = (int)image_point[i].y;
	  if (measurez.at<short>(v,u) == 0)
	  {
	      image_point.erase( image_point.begin() + i);
	      i--;
	      continue;
	  }
	  
        }
    }
}

std::vector<int> removeBadPointsDual(std::vector< cv::Point2d > &image_point1, std::vector< cv::Point2d > &image_point2,
						 cv::Mat &measurez1, cv::Mat &measurez2, bool close_range)
{
    // ==================================== If only close range data is desired ====================================
    // If close_range Bad points aren't stored (equals to 0, > 3 meters, or < 0.5 meters).
    int cnt = -1; //Debug
    std::vector<int> good(image_point1.size(),1);
    if (close_range)
    {
        for ( int i = 0; i < image_point1.size(); i++)
        {
	  cnt++; //Debug
	  int u1 = (int)image_point1[i].x;
	  int v1 = (int)image_point1[i].y;
	  int u2 = (int)image_point2[i].x;
	  int v2 = (int)image_point2[i].y;
	  if (measurez1.at<short>(v1,u1) > 15000.0 || measurez1.at<short>(v1,u1) < 0.4*5000.0
	      || measurez2.at<short>(v2,u2) > 15000.0 || measurez2.at<short>(v2,u2) < 0.4*5000.0)
	  {
	      // Debug
// 	      std::cout << "Remove point " << cnt << '\n';
	      good[cnt] = 0;
	      image_point1.erase( image_point1.begin() + i);
	      image_point2.erase( image_point2.begin() + i);
	      i--;
	      continue;
	  }
        }
    }
    else
    {
        for ( int i = 0; i < image_point1.size(); i++)
        {
	  cnt++;
	  int u1 = (int)image_point1[i].x;
	  int v1 = (int)image_point1[i].y;
	  int u2 = (int)image_point2[i].x;
	  int v2 = (int)image_point2[i].y;
	  if (measurez1.at<short>(v1,u1) <= 0 || measurez2.at<short>(v2,u2) <= 0)
	  {
	      good[cnt] = 0;
	      image_point1.erase( image_point1.begin() + i);
	      image_point2.erase( image_point2.begin() + i);
	      i--;
	      continue;
	  }
        }
    }
    return good;
}

Eigen::Matrix<int,-1, 1> removeBadPointsDual(std::vector< Eigen::Vector3d > &image_point1, std::vector< Eigen::Vector3d > &image_point2,
				        cv::Mat &measurez1, cv::Mat &measurez2, Eigen::MatrixXd &x1, Eigen::MatrixXd &x2, 
				        bool close_range)
{
    // ==================================== If only close range data is desired ====================================
    // If close_range Bad points aren't stored (equals to 0, > 3 meters, or < 0.4 meters).
    int num_points = image_point1.size();
    Eigen::Matrix<int,-1, 1> good = Eigen::Matrix<int,-1, 1>::Zero(num_points);
    x1 = Eigen::MatrixXd::Ones(3,num_points);
    x2 = Eigen::MatrixXd::Ones(3,num_points);
    int count = 0;
    if (close_range)
    {
        for (register int i = 0; i < num_points; ++i)
        {
	  int u1 = (int)image_point1[i](0);
	  int v1 = (int)image_point1[i](1);
	  int u2 = (int)image_point2[i](0);
	  int v2 = (int)image_point2[i](1);
	  if (measurez1.at<short>(v1,u1) < 15000.0 && measurez1.at<short>(v1,u1) > 0.4*5000.0
	      && measurez2.at<short>(v2,u2) < 15000.0 && measurez2.at<short>(v2,u2) > 0.4*5000.0)
	  {
	      good(i) = true;
	      x1.col(count) = Eigen::Vector3d(image_point1[i]);
	      x2.col(count) = Eigen::Vector3d(image_point2[i]);
	      count++;
	  }
        }
    }
    else
    {
        for ( int i = 0; i < num_points; i++)
        {
	  int u1 = (int)image_point1[i](0);
	  int v1 = (int)image_point1[i](1);
	  int u2 = (int)image_point2[i](0);
	  int v2 = (int)image_point2[i](1);
	  if (measurez1.at<short>(v1,u1) > 0 && measurez2.at<short>(v2,u2) > 0)
	  {
	      good(i) = true;
	      x1.col(count) = Eigen::Vector3d(image_point1[i]);
	      x2.col(count) = Eigen::Vector3d(image_point2[i]);
	      count++;
	  }
        }
    }
    x1.conservativeResize(3,count);
    x2.conservativeResize(3,count);
    return good;
}


cv::Point3d projection(cv::Point2d &image_point, cv::Mat &measurez, double &cx, double &cy, double &fx, double &fy)
{
    double u = image_point.x;
    double v = image_point.y;
    double factor = 5000.0; 			// for the 16-bit PNG files 5000 equivalent to 1m. Use 50.0 to change scale to cm
    cv::Point3d world_point;
    
    // ====================================        World Point Calculation      ====================================
//     world_point.z = measurez.at<short>((int) round(v),(int) round(u)) / factor;
    world_point.z = measurez.at<short>((int) (v),(int) (u)) / factor;
    world_point.x = (u - cx) * world_point.z / fx;
    world_point.y = (v - cy) * world_point.z / fy; 			//I DON'T Change the world Y-axis
    // Debug
    //std::cout << ": 2D_Point = " << image_point <<  " \t||\t3D_Point = " << world_point << '\n';
    return world_point;
}

cv::Point3f colorExtraction(cv::Point2d &image_point, cv::Mat &color_image)
{
    uchar *p_rgb = color_image.data;
    int u = (int)image_point.x;
    int v = (int)image_point.y;
    cv::Point3d color_data;
    // ====================================               Color                 ====================================
    int blue = v*color_image.cols*3 + u*3;
    int green = blue + 1;
    int red = blue + 2;
    color_data.x = ((float) p_rgb[red])/255.0;
    color_data.y = ((float) p_rgb[green])/255.0;
    color_data.z = ((float) p_rgb[blue])/255.0;
    
    return color_data;
}

float colorExtraction(int x, int y, cv::Mat &color_image)
{
    uchar *p_rgb = color_image.data;
    // ====================================               Color                 ====================================
    int blue = y*color_image.cols*3 + x*3;
    int green = blue + 1;
    int red = blue + 2;
    uint32_t rgb = ((uint32_t)p_rgb[red] << 16 | (uint32_t)p_rgb[green] << 8 | (uint32_t)p_rgb[blue]);
    return *reinterpret_cast<float*>(&rgb);;
}

float colorExtraction(Eigen::Vector2i &image_point, cv::Mat &color_image)
{
    return colorExtraction(image_point(0), image_point(1), color_image);
}

void calc3Dfrom2D(std::vector<cv::KeyPoint> &corners, cv::Mat &depth, cv::Mat &kalibration,
						std::vector<cv::Point3d> &world_point3d)
{
    cv::Mat_<double> parametersk = cv::Mat_<double>(kalibration);
    double fx = parametersk(0,0);
    double fy = parametersk(1,1);
    double cx = parametersk(0,2);
    double cy = parametersk(1,2);
    
    world_point3d.clear();
    
    for ( int i = 0; i < corners.size(); i++)
    {
        // Debug
        //std::cout << "it " << i;
        cv::Point2d point_aux(corners[i].pt.x, corners[i].pt.y );
        world_point3d.push_back( projection( point_aux, depth, cx, cy, fx, fy) );
    }
}

void calc3Dfrom2D(std::vector<cv::Point2d> &corners, cv::Mat &depth, cv::Mat &kalibration,
						std::vector<cv::Point3d> &world_point3d)
{
    cv::Mat_<double> parametersk = cv::Mat_<double>(kalibration);
    double fx = parametersk(0,0);
    double fy = parametersk(1,1);
    double cx = parametersk(0,2);
    double cy = parametersk(1,2);
    
    world_point3d.clear();
    
    for ( int i = 0; i < corners.size(); i++)
    {
        // Debug
        //std::cout << "it " << i;
        world_point3d.push_back( projection(corners[i], depth, cx, cy, fx, fy) );
    }
}

void calc3DandRGBfrom2D(std::vector<cv::KeyPoint> &corners, cv::Mat &depth, cv::Mat &kalibration, cv::Mat &image, 
							std::vector<cv::Point3d> &world_point3d, std::vector< cv::Point3f > &color)
{
    cv::Mat_<double> parametersk = cv::Mat_<double>(kalibration);
    double fx = parametersk(0,0);
    double fy = parametersk(1,1);
    double cx = parametersk(0,2);
    double cy = parametersk(1,2);
    
    world_point3d.clear();
    color.clear();
    
    for ( int i = 0; i < corners.size(); i++)
    {
        // Debug
        //std::cout << "it " << i;
        cv::Point2d point_aux(corners[i].pt.x, corners[i].pt.y );
        world_point3d.push_back( projection(point_aux, depth, cx, cy, fx, fy) );
        color.push_back( colorExtraction(point_aux, image) );
    }
}

void calc3DandRGBfrom2D(std::vector<cv::Point2d> &corners, cv::Mat &depth, cv::Mat &kalibration, cv::Mat &image, 
							std::vector<cv::Point3d> &world_point3d, std::vector< cv::Point3f > &color)
{
    cv::Mat_<double> parametersk = cv::Mat_<double>(kalibration);
    double fx = parametersk(0,0);
    double fy = parametersk(1,1);
    double cx = parametersk(0,2);
    double cy = parametersk(1,2);
    
    world_point3d.clear();
    color.clear();
    
    for ( int i = 0; i < corners.size(); i++)
    {
        world_point3d.push_back( projection(corners[i], depth, cx, cy, fx, fy) );
        color.push_back( colorExtraction(corners[i], image) );
    }
}

void varianceDepth(double &z, double &sigma)
{
    // [Nguyen,Izadi_2012] model
    sigma = 0.0012 + 0.0019*(pow(z-0.4,2.0));
}

void varianceDepth(Eigen::RowVectorXd &z, Eigen::RowVectorXd &sigma)
{
    // [Nguyen,Izadi_2012] model
    Eigen::RowVectorXd n;
    n = z - 0.4*(Eigen::RowVectorXd::Ones(z.size()));
    sigma = 0.0012*(Eigen::RowVectorXd::Ones(z.size())) + 0.0019*(n.cwiseProduct(n));
}

void varianceKinectSet( Eigen::MatrixXd &X, Eigen::Matrix3d &K, Eigen::MatrixXd &W, double vx, double vy )
{
    // Forward Propagation Variance
    // sx, sy: variance in image; default 1 pixel
    W = Eigen::MatrixXd::Zero(X.rows(),X.cols());
    Eigen::RowVectorXd ones_row = Eigen::RowVectorXd::Ones(X.cols());
    // Depth variance, z axis variance
    Eigen::RowVectorXd zz = X.row(2);
    Eigen::RowVectorXd sz;
    varianceDepth(zz, sz);
    W.row(2) = sz.cwiseProduct(sz);
//     std::cout << "depth =\n" << sz << '\n';
    // Squares
    Eigen::RowVectorXd sx = vx*ones_row;
    Eigen::RowVectorXd sy = vy*ones_row;
    Eigen::RowVectorXd sx2 = sx.cwiseProduct(sx);
    Eigen::RowVectorXd sy2 = sy.cwiseProduct(sy);
    Eigen::RowVectorXd sz2 = W.row(2);
    
    double fx = K(0,0);
    double fy = K(1,1);
    double x0 = K(0,2);
    double y0 = K(1,2);
    double c1 = pow(1/fx,2.0);
    double c2 = pow(-x0/fx,2.0);
    double c3 = pow(1/fy,2.0);
    double c4 = pow(-y0/fy,2.0);
    // Variance of x and y with forward propagation
    W.row(0) = c1*sx2.cwiseProduct(sz2) + c2*sz2;
    W.row(1) = c3*sy2.cwiseProduct(sz2) + c4*sz2;
}