// Created: Feb/14/2014
// Author: José David Tascón Vidarte

#include "MergeClouds.hpp"

// ================================================================================================
// ============================= FUNCTIONS of CLASS MergeClouds ===================================
// ================================================================================================
void MergeClouds::loadDenseCloud( std::string &image, std::string &depth, 
			  Eigen::Quaternion<double> &Qn, Eigen::Vector3d &tr,
			  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out,
			  boost::shared_ptr< Eigen::MatrixXd > &covariance)
{
    cv::Mat rgb = cv::imread(image, -1);
    cv::Mat range = cv::imread(depth, -1);
    cv2PointCloud_Pose( rgb, range, Calibration, Qn, tr, cloud_out, covariance );
}


void MergeClouds::mergeTwo_PCL(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pc1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pc2,
	      boost::shared_ptr< Eigen::MatrixXd > &variance1, boost::shared_ptr< Eigen::MatrixXd > &variance2, 
	      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out, boost::shared_ptr< Eigen::MatrixXd > &covariance_out )
{
    mergeClouds( pc1, pc2, variance1, variance2, cloud_out, covariance_out );
}

void MergeClouds::mergeSet_PCL( std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &set_cloud, 
		std::vector< boost::shared_ptr< Eigen::MatrixXd > > &set_covariance,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &model)
{
    mergeCloudSet( set_cloud, set_covariance, model);
}

void MergeClouds::mergeTwo( std::string &image1, std::string &depth1, std::string &image2, std::string &depth2, 
		        Eigen::Quaternion<double> &qn, Eigen::Vector3d &tr )
{
    Eigen::Quaternion<double> qinit = Eigen::Quaternion<double>::Identity();
    Eigen::Vector3d tinit = Eigen::Vector3d::Zero();
    loadDenseCloud( image1, depth1, qinit, tinit, cloud1, covariance1 );
    loadDenseCloud( image2, depth2, qn, tr, cloud2, covariance2 );
}

void MergeClouds::mergeSet( std::vector< Eigen::Quaternion<double> > &Qn_global, std::vector< Eigen::Vector3d > &tr_global )
{
    DEBUG_1( std::cout << "\n================================ Point Cloud, Merge Cloud Model (Kyostila et al. 2013) ==================================\n"; )
    
    int cumulative_points = 0;
    loadDenseCloud( (*image_list)[0], (*depth_list)[0], Qn_global[0], tr_global[0], cloud1, covariance1 );
    cumulative_points += cloud1->width*cloud1->height;
    for ( register int k = 1; k < image_list->size(); ++k )
    {
        loadDenseCloud( (*image_list)[k], (*depth_list)[k], Qn_global[k], tr_global[k], cloud2, covariance2 );
        cumulative_points += cloud2->width*cloud2->height;
        mergeTwo_PCL( cloud1, cloud2, covariance1, covariance2, model_cloud, model_covariance );
        cloud1 = model_cloud;
        covariance1 = model_covariance;
    }
    int final_points = model_cloud->width*model_cloud->height;
    DEBUG_1( std::cout << "Merged cloud size = " << final_points << "\n"; )
    DEBUG_1( printf( "Saving: [%i / %i] => %f%%\n\n", final_points, cumulative_points, ((float)final_points/(float)cumulative_points)*100.0); )
}


// ====================================================================================================================================
// ===================================================  Additional FUNCTIONS ==========================================================
// ====================================================================================================================================
void mergeClouds( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pc1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &pc2,
	      boost::shared_ptr< Eigen::MatrixXd > &covariance1, boost::shared_ptr< Eigen::MatrixXd > &covariance2, 
	      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out, boost::shared_ptr< Eigen::MatrixXd > &covariance_out )
{
    timer_wall timer1;
    timer1.start();
    int num_pts1 = pc1->width;
    int num_pts2 = pc2->width;
    //     int num_pts2 = 20000;
    pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
    kdtree.setInputCloud (pc1);
    int k = 1; // 4;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc_out (new pcl::PointCloud <pcl::PointXYZRGBA>);
    pcl::copyPointCloud( *pc1, *pc_out );
    
    Eigen::Vector4f cc = pc2->sensor_origin_;
    Eigen::Quaternion<float> qq = pc2->sensor_orientation_;
    Eigen::Matrix3f rot = qq.toRotationMatrix();
    Eigen::Vector3f tt = cc.head(3);				// recover translation
    
    boost::shared_ptr< Eigen::MatrixXd > Cout (new Eigen::MatrixXd);
    boost::shared_ptr< Eigen::MatrixXd > Cout_reduced (new Eigen::MatrixXd);
    *Cout = Eigen::MatrixXd::Zero(3, num_pts1 + num_pts2 );
    Cout->block(0,0,3,num_pts1) = *covariance1;
    
//     Eigen::VectorXd dist = Eigen::VectorXd::Zero( pc2->width );
//     std::ofstream myfile1;
//     myfile1.open ("merge.txt");
    
//     RGBValue color;
//     color.Alpha = 0;
//     color.Red = 0;
//     color.Blue = 255;
//     color.Green = 0;
    int count = num_pts1;
    
    for (register int i = 0; i < num_pts2; ++i)
    {
        std::vector<int> idx_search(k);
        std::vector<float> sq_distance(k);
        pcl::PointXYZRGBA search_point = pc2->points[i];
        
        Eigen::Vector3f vv = Eigen::Vector3f( search_point.x, search_point.y, search_point.z );
        Eigen::Vector3f pv = rot*vv + tt;				// pv  = p tilde
        search_point.x = pv(0); 
        search_point.y = pv(1);
        search_point.z = pv(2);
//         std::cout << "Point to search (" << pv.transpose() << "):\n";
//         myfile1 << "Point to search (" << pv.transpose() << "):\n";
        
        if ( kdtree.nearestKSearch (search_point, k, idx_search, sq_distance) > 0 )
        {
	  
// 	  if (sqrt(sq_distance[0]) > 0.005 )
//     	  {
//     	      pc_out->push_back( search_point );
//     	  }
	  pcl::PointXYZRGBA near_point = pc1->points[ idx_search[0] ];
	  Eigen::Vector3f pa = Eigen::Vector3f( near_point.x, near_point.y, near_point.z );
	  Eigen::Vector3d tmp1 = covariance1->col( idx_search[0] );
	  Eigen::Vector3f tmp2 = Eigen::Vector3f( tmp1(0), tmp1(1), tmp1(2) );
	  Eigen::Matrix3f Ca = Eigen::Matrix3f( tmp2.asDiagonal() );
	  
	  tmp1 = covariance2->col( i );
	  tmp2 = Eigen::Vector3f( tmp1(0), tmp1(1), tmp1(2) );
	  Eigen::Matrix3f Ctmp = Eigen::Matrix3f( tmp2.asDiagonal() );
	  Eigen::Matrix3f Cv = rot*Ctmp*rot.transpose();
	  
	  Eigen::Matrix3f CenInv = (Ca.inverse() + Cv.inverse());
	  Eigen::Matrix3f Cen = CenInv.inverse();
	  Eigen::Vector3f pen = pa + Cen*Cv.inverse()*(pv - pa);
	  Eigen::Vector3f p11 = pen - pa;
	  Eigen::Vector3f p22 = pen - pv;
	  float d1 = p11.transpose()*CenInv*p11;
	  float d2 = p22.transpose()*CenInv*p22;
	  d1 = sqrt(d1);
	  d2 = sqrt(d2);
	  
// 	  std::cout << "Id search: " << idx_search[0] << "\n";
// 	  std::cout << "covariance cols: " << covariance1->cols() << "\n";
// 	  std::cout << "covariance1: " << Ca << "\n";
// 	  std::cout << "covariance2: " << Cv << "\n";
// 	  std::cout << "pen: " << pen.transpose() << "\n";
	  
// 	  myfile1 << "Closest point (" << pa.transpose() << ")\n";
// 	  myfile1 << "Estimated point (" << pen.transpose() << ")\n";
// 	  myfile1 << "distance d1 = " << d1 << "\n";
// 	  myfile1 << "distance d2 = " << d2 << "\n\n";
	  

	  if ( d1 < 3 && d2 < 3 )
	  {
// 	      near_point.x = pen(0);
// 	      near_point.y = pen(1);
// 	      near_point.z = pen(2);
// 	      near_point.rgba = color.long_value;
	      pc_out->points[ idx_search[0] ] = near_point;
	      Eigen::Vector3f diagC = Cen.diagonal();
	      Cout->col( idx_search[0] ) = Eigen::Vector3d( diagC(0), diagC(1), diagC(2) );
	  }
	  else
	  {
	      pc_out->push_back( search_point );
	      Cout->col(count) = covariance2->col(i);
	      count++;	
	  }
        }
    }
    
    *Cout_reduced = Cout->block(0,0,3,count);
    covariance_out = Cout_reduced;
//     myfile1.close();
    
    //         std::cout << "Point to search (" << pv.transpose() << "):\n";
    //         if ( kdtree.nearestKSearch (search_point, k, idx_search, sq_distance) > 0 )
    //         {
    // 	  for (size_t j = 0; j < idx_search.size (); ++j)
    // 	  std::cout << "    " << pc1->points[ idx_search[j] ].x 
    // 		    << " "  << pc1->points[ idx_search[j] ].y 
    // 		    << " "  << pc1->points[ idx_search[j] ].z 
    // 		    << " (squared distance: " << sq_distance[j] << ")" << std::endl;
    
    
    // 	  dist(i) = sqrt(sq_distance[0]);
    // 	  if (sqrt(sq_distance[0]) > 0.005 )
    // 	  {
    // 	      pc_out->push_back( search_point );
    // 	  }
    //         }
    //     Eigen::RowVectorXd uu = dist.colwise().mean();
    //     Eigen::RowVectorXd var = ( (dist.rowwise() - uu).colwise().squaredNorm() )/(dist.rows() - 1);
    //     std::cout << "Merge clouds Stats\n";
    //     std::cout << "Mean of distances = " << uu << "\n";
    //     std::cout << "Std of distances = " << var.cwiseSqrt() << "\n";
    
    DEBUG_2( std::cout << "Merged cloud size = " << pc_out->width*pc_out->height << "\n"; )
    DEBUG_2( std::cout << "Elapsed time to merge: " << timer1.elapsed_s() << " [s]\n"; )
    cloud_out = pc_out;
}

void mergeCloudSet( std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &set_cloud, 
		std::vector< boost::shared_ptr< Eigen::MatrixXd > > &set_covariance,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &model)
{
    int size_clouds = set_cloud.size();
    int cumulative_points = 0;
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr initial_cloud, result_cloud;
    boost::shared_ptr< Eigen::MatrixXd > cov_init, cov_final;
    
    initial_cloud = set_cloud[0];
    cov_init = set_covariance[0];
    cumulative_points += set_cloud[0]->width*set_cloud[0]->height;
    DEBUG_1( std::cout << "\n================================ Point Cloud, Merge Cloud Model (Kyostila et al. 2013) ==================================\n"; )
    for (register int i = 1; i < size_clouds; ++i)
    {
        cumulative_points += set_cloud[i]->width*set_cloud[i]->height;
        mergeClouds( initial_cloud, set_cloud[i], cov_init, set_covariance[i], result_cloud, cov_final );
        initial_cloud = result_cloud;
        cov_init = cov_final;
    }
    
    int final_points = result_cloud->width*result_cloud->height;
    DEBUG_1( std::cout << "Merged cloud size = " << final_points << "\n"; )
    DEBUG_1( printf( "Saving: [%i / %i] => %f%%\n\n", final_points, cumulative_points, ((float)final_points/(float)cumulative_points)*100.0); )
    model = result_cloud;
}