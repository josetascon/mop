// Created: Nov/04/2013
// Author: José David Tascón Vidarte

#include "InterfacePCL.hpp"


// ====================================================================================================================================
// =====================================================  FUNCTIONS PCL ===============================================================
// ====================================================================================================================================

template< typename Tpoint >
void projection( int &u, int &v, float &depth, float intrinsic[4], Tpoint &structure )
{
    float xyz[3];
    int uv[2] = { u, v };
    projectiongeneral( uv, depth, intrinsic, xyz);
    structure.x = xyz[0];  structure.y = xyz[1];  structure.z = xyz[2];
}

uint32_t extractColor( int &u, int &v, uchar *p_rgb, int &columns )
{
    RGBValue color;
    color.Alpha = 0;
    
    int image_idx = v*columns*3 + u*3;
    color.Blue  = p_rgb[image_idx];
    color.Green = p_rgb[image_idx + 1];
    color.Red   = p_rgb[image_idx + 2];
    return color.long_value;
}

void cv2PointCloud(cv::Mat &depth, Eigen::Matrix3d &calibration, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_xyz)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
    cloud->height = depth.size().height;
    cloud->width = depth.size().width;
    cloud->points.resize (cloud->height * cloud->width);
    
    float intrinsic[4] = { (float)calibration(0,2), (float)calibration(1,2), (float)calibration(0,0), (float)calibration(1,1) };
    float bad_point = std::numeric_limits<float>::quiet_NaN ();
    register int depth_idx = 0;
    
    for (int v = 0; v < cloud->height; ++v)
    {
        for (register int u = 0; u < cloud->width; ++u, ++depth_idx)
        {
	  pcl::PointXYZ& pt = cloud->points[depth_idx];
	  float measurez = depth.at<short>(v,u);
	  if( !boundarykinect( measurez ) ) pt.x = pt.y = pt.z = bad_point; 	// Invalid measurement, outside the boundaries
	  else projection( u, v, measurez, intrinsic, pt );		// Valid point projected
	  // 	  printf("Depth (%i,%i): %f\n", u, v, pt.z);
        }
    }
    cloud->sensor_origin_.setZero ();
    cloud->sensor_orientation_.setIdentity ();
    cloud->is_dense = false;
    cloud_xyz = cloud;
}

void cv2PointCloud(cv::Mat &image, cv::Mat &depth, Eigen::Matrix3d &calibration, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGBA>);
    cloud->height = depth.size().height;
    cloud->width = depth.size().width;
    cloud->points.resize (cloud->height * cloud->width);
    
    float intrinsic[4] = { (float)calibration(0,2), (float)calibration(1,2), (float)calibration(0,0), (float)calibration(1,1) };
    float bad_point = std::numeric_limits<float>::quiet_NaN ();
    
    uchar *p_rgb = image.data;
    int image_cols = image.size().width;
    register int depth_idx = 0;
    
    for (int v = 0; v < cloud->height; ++v)
    {
        for (register int u = 0; u < cloud->width; ++u, ++depth_idx)
        {
	  pcl::PointXYZRGBA& pt = cloud->points[depth_idx];
	  float measurez = depth.at<short>(v,u);
	  if( !boundarykinect( measurez ) ) pt.x = pt.y = pt.z = bad_point; 	// Invalid measurement, outside the boundaries
	  else 
	  {
	      projection( u, v, measurez, intrinsic, pt );
	      pt.rgba = extractColor( u, v, p_rgb, image_cols );
	  }
        }
    }
    cloud->sensor_origin_.setZero ();
    cloud->sensor_orientation_.setIdentity ();
    cloud->is_dense = false;
    cloud_out = cloud;
}

void cv2PointCloudDense(cv::Mat &image, cv::Mat &depth, Eigen::Matrix3d &calibration, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGBA>);
    
    float intrinsic[4] = { (float)calibration(0,2), (float)calibration(1,2), (float)calibration(0,0), (float)calibration(1,1) };
    float bad_point = std::numeric_limits<float>::quiet_NaN ();
    
    uchar *p_rgb = image.data;
    int image_cols = image.size().width;
    
    for (register int v = 0; v < depth.size().height; ++v)
    {
        for (register int u = 0; u < depth.size().width; ++u)
        {
	  pcl::PointXYZRGBA pt;
	  float measurez = depth.at<short>(v,u);
	  if ( boundarykinect( measurez ) )			// Check for invalid measurements
	  {
	      projection( u, v, measurez, intrinsic, pt );
	      pt.rgba = extractColor( u, v, p_rgb, image_cols );
	      cloud->push_back(pt);
	  }
        }
    }
    cloud->sensor_origin_.setZero ();
    cloud->sensor_orientation_.setIdentity ();
    cloud->is_dense = true;
    DEBUG_3( std::cout << "Cloud size = " << cloud->width*cloud->height << "\n"; )
    cloud_out = cloud;
}

void cv2PointCloudDense(cv::Mat &image, cv::Mat &depth, Eigen::Matrix3d &calibration,
	         pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out, boost::shared_ptr< Eigen::MatrixXd > &covariance)
{
    cv2PointCloudDense(image, depth, calibration, cloud_out);
    computeCovariance(cloud_out, calibration, covariance);
}

void computeCovariance(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, Eigen::Matrix3d &calibration, 
		   boost::shared_ptr< Eigen::MatrixXd > &covariance)
{
    boost::shared_ptr< Eigen::MatrixXd > Cov (new Eigen::MatrixXd);
    int sizeC = cloud->width;
    Eigen::MatrixXd X3 = Eigen::MatrixXd::Zero(3,sizeC);
    //     std::cout << "width " << cloud->width << "\n";
    //     std::cout << "height " << cloud->height << "\n";
    for (register int k = 0; k < sizeC; ++k)
    {
        pcl::PointXYZRGBA pt = cloud->points[k];
        //         X3(0,k) = pt.x;
        //         X3(1,k) = pt.y;
        X3(2,k) = pt.z;		// Only Z is needed for varianceKinectSet function
    }
    //     std::cout << "X " << X3.transpose() << "\n";
    varianceKinectSet( X3, calibration, *Cov);
    covariance = Cov;
}

void cv2PointCloud_Pose( cv::Mat &image, cv::Mat &depth, Eigen::Matrix3d &calibration, 
		Eigen::Quaternion<double> &Qn, Eigen::Vector3d &tr,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud )
		//boost::shared_ptr< Eigen::MatrixXd > &covariance, bool covarian )
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc01;
    cv2PointCloudDense(image, depth, calibration, pc01);
    Eigen::Vector3d center = tr;
    Eigen::Matrix3d rot = Qn.toRotationMatrix();
    center = -rot.transpose()*center;
    pc01->sensor_origin_ = Eigen::Vector4f(center(0),center(1),center(2), 0.0);
    Eigen::Quaternion<float> q1 = Qn.template cast<float>();
    pc01->sensor_orientation_ = q1.conjugate();
    cloud = pc01;
}

void cv2PointCloud_Pose( cv::Mat &image, cv::Mat &depth, Eigen::Matrix3d &calibration, 
		Eigen::Quaternion<double> &Qn, Eigen::Vector3d &tr,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
		boost::shared_ptr< Eigen::MatrixXd > &covariance )
{
    cv2PointCloud_Pose( image, depth, calibration, Qn, tr, cloud );
    computeCovariance(cloud, calibration, covariance);
}

void cv2PointCloudSet(std::vector<cv::Mat> &image, std::vector<cv::Mat> &depth, Eigen::Matrix3d &calibration, 
		std::vector<Eigen::Quaternion<double> > &Qn, std::vector< Eigen::Vector3d > &tr,
		std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &set_cloud)
{
    int num_cameras = image.size();
    set_cloud.clear();
    set_cloud.resize(num_cameras);
    
    for (register int k = 0; k < num_cameras ; ++k) 
        cv2PointCloud_Pose( image[k], depth[k], calibration, Qn[k], tr[k], set_cloud[k] );
}

void cv2PointCloudSet(std::vector<cv::Mat> &image, std::vector<cv::Mat> &depth, Eigen::Matrix3d &calibration, 
		std::vector<Eigen::Quaternion<double> > &Qn, std::vector< Eigen::Vector3d > &tr,
		std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &set_cloud,
		std::vector< boost::shared_ptr< Eigen::MatrixXd > > &set_covariance)
{
    int num_cameras = image.size();
    set_cloud.clear();
    set_cloud.resize(num_cameras);
    set_covariance.clear();
    set_covariance.resize(num_cameras);
    
    for (register int k = 0; k < num_cameras ; ++k)
        cv2PointCloud_Pose( image[k], depth[k], calibration, Qn[k], tr[k], set_cloud[k], set_covariance[k] );
}

void cv2PointCloudSet(std::vector<std::string> &image_list, std::vector<std::string> &depth_list, Eigen::Matrix3d &calibration, 
		std::vector<Eigen::Quaternion<double> > &Qn, std::vector< Eigen::Vector3d > &tr,
		std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &set_cloud,
		std::vector< boost::shared_ptr< Eigen::MatrixXd > > &set_covariance)
{
    int num_cameras = image_list.size();
    set_cloud.clear();
    set_cloud.resize(num_cameras);
    set_covariance.clear();
    set_covariance.resize(num_cameras);
    DEBUG_1( std::cout << "\n================================ Point Cloud, Creating Point Clouds from Images ==================================\n"; )
    for (register int k = 0; k < num_cameras ; ++k)
    {
        cv::Mat image = cv::imread(image_list[k], 1);
        cv::Mat depth = cv::imread(depth_list[k], -1);
        cv2PointCloud_Pose( image, depth, calibration, Qn[k], tr[k], set_cloud[k], set_covariance[k] );
        DEBUG_1( std::string str = "Cloud size = %i, \tImage %i : ";
	  str.append( basename(strdup(image_list[k].c_str())) );
	  str.append("\n");
	  printf(str.c_str(),set_cloud[k]->width*set_cloud[k]->height, k);
        )
    }
}

void sparse2dense( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_sparse, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_dense)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    if (!cloud_sparse->is_dense)
    {
        int height = cloud_sparse->height;
        int width = cloud_sparse->width;
        
        for (register int v = 0; v < height; ++v)
        {
	  for (register int u = 0; u < width; ++u)
	  {
	      float value = cloud_sparse->at(u,v).x;
	      if( isnan(value) )
	      {
		continue;
	      }
	      else
	      {
		cloud->push_back( cloud_sparse->at(u,v) );
		//printf("Point (%i,%i): [%f,%f,%f]\n", u, v, cloud_sparse->at(u,v).x, cloud_sparse->at(u,v).y, cloud_sparse->at(u,v).z);
	      }
	  }
        }
        
        cloud->sensor_origin_ = cloud_sparse->sensor_origin_;
        cloud->sensor_orientation_ = cloud_sparse->sensor_orientation_;
        cloud->is_dense = true;
        DEBUG_2( std::cout << "Sparse Cloud size = " << cloud_sparse->width*cloud_sparse->height << "\n"; )
        DEBUG_2( std::cout << "Dense Cloud size = " << cloud->width*cloud->height << "\n"; )
    }
    else cloud = cloud_sparse;
    cloud->is_dense = true;
    cloud_dense = cloud;
}

void set2unique( std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &data, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGBA>);
    int num_pc = data.size();
    DEBUG_1( std::cout << "\n================================ Point Cloud, Joint Cloud Model ==================================\n"; )
    for (register int k = 0; k < num_pc; ++k)
    {
        for (pcl::PointCloud< pcl::PointXYZRGBA >::iterator it = data[k]->begin() ; it != data[k]->end(); ++it)
	  //         int num_points = data[k]->width * data[k]->height;
	  //         std::cout << "# points: "<< num_points <<"\n";
	  //         for (register int i = 0 ; i < num_points; ++i)
        {
	  pcl::PointXYZRGBA pt1;
	  pt1 = *it;
	  // 	  pt1 = (*data[k])[i];
	  Eigen::Vector3f vv(pt1.x, pt1.y, pt1.z);
	  Eigen::Vector4f cc = data[k]->sensor_origin_;
	  Eigen::Quaternion<float> qq = data[k]->sensor_orientation_;
	  // 	  qq = qq.conjugate();					// reverse rotation
	  Eigen::Matrix3f rot = qq.toRotationMatrix();
	  Eigen::Vector3f tt = cc.head(3);				// recover translation 
	  Eigen::Vector3f pp = rot*vv + tt;
	  // Update real value of x,y & z
	  pt1.x = pp(0);
	  pt1.y = pp(1);
	  pt1.z = pp(2);
	  cloud->push_back( pt1 );
        }
    }
    DEBUG_3( std::cout << "End merging clouds\n"; )
    cloud->sensor_origin_.setZero ();
    cloud->sensor_orientation_ = Eigen::Quaternion<float>::Identity();
    cloud->is_dense = true;
    DEBUG_1( std::cout << "Final Point Cloud size = " << cloud->width*cloud->height << "\n"; )
    cloud_out = cloud;
}

// void mergeClouds( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pc1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pc2,
// 	      boost::shared_ptr< Eigen::MatrixXd > &covariance1, boost::shared_ptr< Eigen::MatrixXd > &covariance2, 
// 	      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out, boost::shared_ptr< Eigen::MatrixXd > &covariance_out )
// {
//     timer_wall timer1;
//     timer1.start();
//     int num_pts1 = pc1->width;
//     int num_pts2 = pc2->width;
//     //     int num_pts2 = 20000;
//     pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
//     kdtree.setInputCloud (pc1);
//     int k = 1; // 4;
//     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc_out (new pcl::PointCloud <pcl::PointXYZRGBA>);
//     pcl::copyPointCloud( *pc1, *pc_out );
//     
//     Eigen::Vector4f cc = pc2->sensor_origin_;
//     Eigen::Quaternion<float> qq = pc2->sensor_orientation_;
//     Eigen::Matrix3f rot = qq.toRotationMatrix();
//     Eigen::Vector3f tt = cc.head(3);				// recover translation
//     
//     boost::shared_ptr< Eigen::MatrixXd > Cout (new Eigen::MatrixXd);
//     boost::shared_ptr< Eigen::MatrixXd > Cout_reduced (new Eigen::MatrixXd);
//     *Cout = Eigen::MatrixXd::Zero(3, num_pts1 + num_pts2 );
//     Cout->block(0,0,3,num_pts1) = *covariance1;
//     
// //     Eigen::VectorXd dist = Eigen::VectorXd::Zero( pc2->width );
// //     std::ofstream myfile1;
// //     myfile1.open ("merge.txt");
//     
// //     RGBValue color;
// //     color.Alpha = 0;
// //     color.Red = 0;
// //     color.Blue = 255;
// //     color.Green = 0;
//     int count = num_pts1;
//     
//     for (register int i = 0; i < num_pts2; ++i)
//     {
//         std::vector<int> idx_search(k);
//         std::vector<float> sq_distance(k);
//         pcl::PointXYZRGBA search_point = pc2->points[i];
//         
//         Eigen::Vector3f vv = Eigen::Vector3f( search_point.x, search_point.y, search_point.z );
//         Eigen::Vector3f pv = rot*vv + tt;				// pv  = p tilde
//         search_point.x = pv(0); 
//         search_point.y = pv(1);
//         search_point.z = pv(2);
// //         std::cout << "Point to search (" << pv.transpose() << "):\n";
// //         myfile1 << "Point to search (" << pv.transpose() << "):\n";
//         
//         if ( kdtree.nearestKSearch (search_point, k, idx_search, sq_distance) > 0 )
//         {
// 	  
// // 	  if (sqrt(sq_distance[0]) > 0.005 )
// //     	  {
// //     	      pc_out->push_back( search_point );
// //     	  }
// 	  pcl::PointXYZRGBA near_point = pc1->points[ idx_search[0] ];
// 	  Eigen::Vector3f pa = Eigen::Vector3f( near_point.x, near_point.y, near_point.z );
// 	  Eigen::Vector3d tmp1 = covariance1->col( idx_search[0] );
// 	  Eigen::Vector3f tmp2 = Eigen::Vector3f( tmp1(0), tmp1(1), tmp1(2) );
// 	  Eigen::Matrix3f Ca = Eigen::Matrix3f( tmp2.asDiagonal() );
// 	  
// 	  tmp1 = covariance2->col( i );
// 	  tmp2 = Eigen::Vector3f( tmp1(0), tmp1(1), tmp1(2) );
// 	  Eigen::Matrix3f Ctmp = Eigen::Matrix3f( tmp2.asDiagonal() );
// 	  Eigen::Matrix3f Cv = rot*Ctmp*rot.transpose();
// 	  
// 	  Eigen::Matrix3f CenInv = (Ca.inverse() + Cv.inverse());
// 	  Eigen::Matrix3f Cen = CenInv.inverse();
// 	  Eigen::Vector3f pen = pa + Cen*Cv.inverse()*(pv - pa);
// 	  Eigen::Vector3f p11 = pen - pa;
// 	  Eigen::Vector3f p22 = pen - pv;
// 	  float d1 = p11.transpose()*CenInv*p11;
// 	  float d2 = p22.transpose()*CenInv*p22;
// 	  d1 = sqrt(d1);
// 	  d2 = sqrt(d2);
// 	  
// // 	  std::cout << "Id search: " << idx_search[0] << "\n";
// // 	  std::cout << "covariance cols: " << covariance1->cols() << "\n";
// // 	  std::cout << "covariance1: " << Ca << "\n";
// // 	  std::cout << "covariance2: " << Cv << "\n";
// // 	  std::cout << "pen: " << pen.transpose() << "\n";
// 	  
// // 	  myfile1 << "Closest point (" << pa.transpose() << ")\n";
// // 	  myfile1 << "Estimated point (" << pen.transpose() << ")\n";
// // 	  myfile1 << "distance d1 = " << d1 << "\n";
// // 	  myfile1 << "distance d2 = " << d2 << "\n\n";
// 	  
// 
// 	  if ( d1 < 3 && d2 < 3 )
// 	  {
// // 	      near_point.x = pen(0);
// // 	      near_point.y = pen(1);
// // 	      near_point.z = pen(2);
// // 	      near_point.rgba = color.long_value;
// 	      pc_out->points[ idx_search[0] ] = near_point;
// 	      Eigen::Vector3f diagC = Cen.diagonal();
// 	      Cout->col( idx_search[0] ) = Eigen::Vector3d( diagC(0), diagC(1), diagC(2) );
// 	  }
// 	  else
// 	  {
// 	      pc_out->push_back( search_point );
// 	      Cout->col(count) = covariance2->col(i);
// 	      count++;	
// 	  }
//         }
//     }
//     
//     *Cout_reduced = Cout->block(0,0,3,count);
//     covariance_out = Cout_reduced;
// //     myfile1.close();
//     
//     //         std::cout << "Point to search (" << pv.transpose() << "):\n";
//     //         if ( kdtree.nearestKSearch (search_point, k, idx_search, sq_distance) > 0 )
//     //         {
//     // 	  for (size_t j = 0; j < idx_search.size (); ++j)
//     // 	  std::cout << "    " << pc1->points[ idx_search[j] ].x 
//     // 		    << " "  << pc1->points[ idx_search[j] ].y 
//     // 		    << " "  << pc1->points[ idx_search[j] ].z 
//     // 		    << " (squared distance: " << sq_distance[j] << ")" << std::endl;
//     
//     
//     // 	  dist(i) = sqrt(sq_distance[0]);
//     // 	  if (sqrt(sq_distance[0]) > 0.005 )
//     // 	  {
//     // 	      pc_out->push_back( search_point );
//     // 	  }
//     //         }
//     //     Eigen::RowVectorXd uu = dist.colwise().mean();
//     //     Eigen::RowVectorXd var = ( (dist.rowwise() - uu).colwise().squaredNorm() )/(dist.rows() - 1);
//     //     std::cout << "Merge clouds Stats\n";
//     //     std::cout << "Mean of distances = " << uu << "\n";
//     //     std::cout << "Std of distances = " << var.cwiseSqrt() << "\n";
//     
//     DEBUG_2( std::cout << "Merged cloud size = " << pc_out->width*pc_out->height << "\n"; )
//     DEBUG_2( std::cout << "Elapsed time to merge: " << timer1.elapsed_s() << " [s]\n"; )
//     cloud_out = pc_out;
// }
// 
// void mergeCloudSet( std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &set_cloud, 
// 		std::vector< boost::shared_ptr< Eigen::MatrixXd > > &set_covariance,
// 		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &model)
// {
//     int size_clouds = set_cloud.size();
//     int cumulative_points = 0;
//     
//     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr initial_cloud, result_cloud;
//     boost::shared_ptr< Eigen::MatrixXd > cov_init, cov_final;
//     
//     initial_cloud = set_cloud[0];
//     cov_init = set_covariance[0];
//     cumulative_points += set_cloud[0]->width*set_cloud[0]->height;
//     DEBUG_1( std::cout << "\n================================ Point Cloud, Merge Cloud Model (Kyostila et al. 2013) ==================================\n"; )
//     for (register int i = 1; i < size_clouds; ++i)
//     {
//         cumulative_points += set_cloud[i]->width*set_cloud[i]->height;
//         mergeClouds( initial_cloud, set_cloud[i], cov_init, set_covariance[i], result_cloud, cov_final );
//         initial_cloud = result_cloud;
//         cov_init = cov_final;
//     }
//     
//     int final_points = result_cloud->width*result_cloud->height;
//     DEBUG_1( std::cout << "Merged cloud size = " << final_points << "\n"; )
//     DEBUG_1( printf( "Saving: [%i / %i] => %f%%\n\n", final_points, cumulative_points, ((float)final_points/(float)cumulative_points)*100.0); )
//     model = result_cloud;
// }

// void poseICP( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 )
// {
//     pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
//     icp.setInputCloud(cloud_in);
//     icp.setInputTarget(cloud_out);
//     pcl::PointCloud<pcl::PointXYZ> Final;
//     icp.align(Final);
//     std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//     icp.getFitnessScore() << std::endl;
//     std::cout << icp.getFinalTransformation() << std::endl;
// }