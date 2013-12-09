// Nov 20 2013
// José David Tascón V.

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// PCL Libraries
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Std Libraries
#include <iostream>
#include <random>
#include <string>
#include <cmath>

// Local libraries
#include "Common.hpp"
#include "Interface.hpp"
#include "InterfacePCL.hpp"
#include "DepthProjection.hpp"
#include "Registration.hpp"
#include "Optimizer.hpp"
#include "Plot.hpp"

// using namespace std;
// using namespace Eigen;

void help()
{ 
	std::cout << " Usage: ./synthetic_range -f <#features> -c <#cameras>\n"
				"\tOptions:\n" 
				"\t[-f]\t Number of features per image to generate (default = 50)\n" 
				"\t[-c]\t Number of cameras to generate (default = 36)\n"
				"\t[-w]\t Width of images (default = 640)\n"
				"\t[-h]\t Height of images (default = 480)\n"
				"\t[-d]\t Frontal distance of three-dimensional points to cameras (default = 1.5)\n"
				"\t[-r]\t Ratio of camera circular rigid (default = 1.0)\n"
				"\t[-a]\t Run statistical analysis\n"
				"\t[-o]\t Output file name for statistical analysis (default = stats_synthetic_ba.txt) \n"
	<< std::endl;
}

// ================================================= MAIN ==============================================
int main(int argc, char* argv[])
{
    // ========================================== Varible declaration ==========================================
    bool analysis = false;
    int experiments = 1;
    double noise_std = 0.3;
    double max_noise = 0.4;
    double ratio = 1.0;
    double frontal_distance = 1.5;
    int num_features;
    int fxc = 50;
    int num_cameras = 36;
    int image_width = 640;
    int image_height = 480;
    std::string output_file = "stats_synthetic_range.txt";
    
    // ================ Check input ==============
    if( argc < 2 )
    {
        help();
        return 0;
    }
    else
    {
        for(int i = 1; i < argc; i++ )
        {
	  const char* s = argv[i];
	  
	  if( strcmp( s, "-c" ) == 0 )
	  {
	      i++;
	      num_cameras = pchar2int(argv[i]);
	  }
	  else if( strcmp( s, "-f" ) == 0 )
	  {
	      i++;
	      fxc = pchar2int(argv[i]);
	  }
	  else if( strcmp( s, "-w" ) == 0 )
	  {
	      i++;
	      image_width = pchar2int(argv[i]);
	  }
	  else if( strcmp( s, "-h" ) == 0 )
	  {
	      i++;
	      image_height = pchar2int(argv[i]);
	  }
	  else if( strcmp( s, "-a" ) == 0 )
	  {
	      analysis = true;
	  }
	  else if( strcmp( s, "-o" ) == 0 )
	  {
	      i++;
	      output_file = argv[i];
	  }
	  else if( strcmp( s, "-d" ) == 0 )
	  {
	      i++;
	      frontal_distance = pchar2float(argv[i]);
	  }
	  else if( strcmp( s, "-r" ) == 0 )
	  {
	      i++;
	      ratio = pchar2float(argv[i]);
	  }
	  else if( strcmp( s, "-h" ) == 0 )
	  {
	      help();
	      return 0;
	  }
	  else
	  {
	      help();
	      return fprintf( stderr, "Unknown option %s\n", s ), -1;
	  }
        }
        
    }
    
    
    if ( analysis )
    {
        experiments = 21; //nominal 21
    }
    
    int config = 1; //experiment configuration
    Eigen::MatrixXd details(config,4); // num_cameras, fxc, num_features, ratio
    Eigen::MatrixXd data_lin_final(experiments,4);
    Eigen::MatrixXd data_opt_final(experiments,4);
    //Drift
    std::vector< Eigen::MatrixXd > data_drift_lin(experiments);
    std::vector< Eigen::MatrixXd > data_drift_opt(experiments);
    
    int exp = 0;
//     int count_std = 0;
    
//     for ( ;noise_std < max_noise; noise_std += 0.1, count_std++)
//     {
        for ( int exp = 0; exp < experiments; exp++ )
        {
	  
	  num_features = fxc*num_cameras;
	  
	  // CONFIG DETAILS
	  details(0,0) = num_cameras;
	  details(0,1) = fxc;
	  details(0,2) = num_features;
	  details(0,3) = ratio;
	  
	  timer_wall timer1;
	  std::vector< Eigen::Quaternion<double> > qu_synthetic(num_cameras + 1);
	  std::vector< Eigen::Matrix3d > rot_synthetic(num_cameras + 1);
	  std::vector< Eigen::Vector3d > cam_center(num_cameras + 1);
	  std::vector< Eigen::Vector3d > tr_synthetic(num_cameras + 1);
	  std::vector< Eigen::MatrixXd > camera_synthetic(num_cameras + 1);
	  boost::shared_ptr< std::vector< Eigen::MatrixXd > > image_synthetic (new std::vector< Eigen::MatrixXd >(num_cameras + 1));
	  Eigen::MatrixXd structure(4,num_features);
	  Eigen::MatrixXd st_synthetic(4,num_features);
	  Eigen::MatrixXd st_synthetic_noisy(4,num_features);
	  
	  boost::shared_ptr< Eigen::Matrix<bool,-1,-1> > visibility (new Eigen::Matrix<bool,-1,-1> (num_cameras + 1,num_features) );
	  boost::shared_ptr< Eigen::Matrix< Eigen::Vector4d,-1,-1> > coordinates 
	  (new Eigen::Matrix< Eigen::Vector4d,-1,-1> (num_cameras + 1,num_features) );
	  boost::shared_ptr< Eigen::Matrix< Eigen::Vector4d,-1,-1> > coordinates_noise 
	  (new Eigen::Matrix< Eigen::Vector4d,-1,-1> (num_cameras + 1,num_features) );
	  
	  Eigen::Matrix3d K;
	  double wd2 = image_width/2.0;
	  double hd2 = image_height/2.0;
	  double f = 600.0;
	  K << f, 0.0, wd2, 0.0, f, hd2, 0.0, 0.0, 1.0;		// Camera Matrix (intrinsics)
	  // Random generators
	  std::random_device rd;
	  std::mt19937_64 gen(rd());
	  std::uniform_real_distribution<double> uniform(0.0,1.0);	// min and max
	  std::normal_distribution<double> gaussian(0.0,1.0); 	// mean 0, std 1.0
	  
	  // ========================================== END Varible declaration ==========================================
	  
	  // ========================================== Cameras init ==========================================
	  // First camera
	  std::cout << "Synthetic Run of a Range Alignment system:\n";
// 	  std::cout << "Experiment: " << exp << "; Noise std: " << noise_std << "\n";
	  std::cout << "Init Cameras ... \t\t";
	  
	  qu_synthetic[0] = Eigen::Quaternion<double>::Identity();
	  rot_synthetic[0] = Eigen::Matrix3d::Identity();
	  cam_center[0] = Eigen::Vector3d(0.0, 0.0, 1.0*ratio);		// Circular translation
	  tr_synthetic[0] = qu_synthetic[0]*(-1.0*cam_center[0]);
	  camera_synthetic[0] = buildProjectionMatrix( K, rot_synthetic[0], tr_synthetic[0] );
	  st_synthetic = Eigen::MatrixXd::Ones(4,num_features);
	  (*visibility) = Eigen::Matrix<bool,-1,-1>::Zero(num_cameras + 1,num_features);
	  // Generate rotation and translation
	  for(register int cam = 1; cam < num_cameras; ++cam)
	  {
	      double ax = 0.0;
	      double ay = -(cam)*(2*pi/num_cameras);
	      double az = 0.0;
	      Eigen::Matrix3d Rtmp;
	      Rtmp = Eigen::AngleAxisd(ax, Eigen::Vector3d::UnitX()) * 
		    Eigen::AngleAxisd(ay, Eigen::Vector3d::UnitY()) * 
		    Eigen::AngleAxisd(az, Eigen::Vector3d::UnitZ());
	      qu_synthetic[cam] = Eigen::Quaternion<double>(Rtmp);
	      rot_synthetic[cam] = Rtmp;
	      cam_center[cam] = Eigen::Vector3d(ratio*std::sin(-ay), 0.0, ratio*std::cos(-ay));
	      tr_synthetic[cam] = -Rtmp*cam_center[cam];
	      camera_synthetic[cam] = buildProjectionMatrix( K, Rtmp, tr_synthetic[cam] );
	  }
	  
	  // Loop Close Camera  ||  Last camera equal to First
	  qu_synthetic[num_cameras] = Eigen::Quaternion<double>::Identity();
	  rot_synthetic[num_cameras] = Eigen::Matrix3d::Identity();;
	  cam_center[num_cameras] = Eigen::Vector3d(0.0, 0.0, 1.0*ratio);
	  tr_synthetic[num_cameras] = qu_synthetic[num_cameras]*(-1.0*cam_center[num_cameras]);;
	  camera_synthetic[num_cameras] = buildProjectionMatrix( K, rot_synthetic[num_cameras], tr_synthetic[num_cameras] );
	      
	  std::cout << "[OK]\n";
	  // ======================================== END Cameras init ========================================
	  
	  // ========================================== Random points init ==========================================
	  // Generate points in front of each camera
	  std::cout << "Init 3D Points ... \t\t";
	  for(register int cam = 0; cam < num_cameras; ++cam)
	  {
	      Eigen::MatrixXd X(3,fxc);
	      for(register int j = 0; j < fxc; ++j)
	      {
		X(0,j) = 4.0*uniform(gen) - 2.0;
		X(1,j) = 4.0*uniform(gen) - 2.0;
		X(2,j) = 1.5*uniform(gen) + 1.5; // Depth noise
	      }
	      Eigen::Matrix3d Rtmp = rot_synthetic[cam];
	      Eigen::MatrixXd tmp = (Rtmp.transpose()*X).colwise() + cam_center[cam];
	      st_synthetic.block( 0, cam*fxc, 3, fxc ) = tmp;
	      // Debug
        //         printf("Col %d - %d; \tRow %d - %d;\n", cam*fxc, (cam+1)*fxc, 0, 3);
        //         std::cout << "Structure\n" << st_synthetic.block( 0, cam*fxc, 3, fxc ) << "\n";
	  }
	  std::cout << "[OK]\n";
	  // ========================================== END Random points init ==========================================
	  
	  // ========================================== Synthetic Image Data ==========================================
	  // Project structure to images. The visibility matrix says which points are in which view. 
	  // 
	  std::cout << "Projections, 2D Points ... \t\t";
	  for(register int cam = 0; cam < num_cameras; ++cam)
	  {
	      Eigen::MatrixXd tmp = st_synthetic;//.block( 0, cam*fxc, 4, fxc );
	      (*image_synthetic)[cam] = camera_synthetic[cam]*tmp;
	      normalizeHomogeneous( (*image_synthetic)[cam] );
	      Eigen::MatrixXd W;
	      Eigen::MatrixXd Xn;
	      for(register int ft = 0; ft < num_features; ++ft)
	      {
		Eigen::Vector3d ff = (*image_synthetic)[cam].col(ft);
		if ( (ff(0) < image_width) && (ff(0) > 0.0) && (ff(1) < image_height) && (ff(1) > 0.0) )
		{
		    (*visibility)(cam,ft) = true;
		    Eigen::Vector4d vv = st_synthetic.col(ft);
		    Eigen::Vector3d pp = rot_synthetic[cam]*vv.head(3) + tr_synthetic[cam];
		    vv.head(3) = pp;
		    (*coordinates)(cam,ft) = vv;
		    
		    // Noise model
		    Xn = vv;
		    varianceKinectSet( Xn, K, W );
		    vv(0) += std::sqrt(W(0)) * gaussian(gen);
		    vv(1) += std::sqrt(W(1)) * gaussian(gen);
		    vv(2) += std::sqrt(W(2)) * gaussian(gen);
		    (*coordinates_noise)(cam,ft) = vv;
		}
	      }
	  }
	  image_synthetic.reset();
	  
	  // Loop Close Camera  ||  Last camera equal to First
	  for(register int ft = 0; ft < num_features; ++ft)
	  {
	      (*visibility)(num_cameras,ft) = (*visibility)(0,ft);
	      (*coordinates)(num_cameras,ft) = (*coordinates)(0,ft);
	      (*coordinates_noise)(num_cameras,ft) = (*coordinates_noise)(0,ft);
	  }
	  
	  std::cout << "[OK]\n";
            printf("Visibility Matrix [%d x %d]\n\n",visibility->rows(),visibility->cols());
// 	  std::cout << "Visibility =\n" << visibility << "\n";
	  // ======================================== END Synthetic Image Data ========================================
	  
	  // ========================================== Range Pose ==========================================
	  
	  SimpleRegistration sr01( num_cameras + 1, num_features, K );
	  timer1.start();
	  sr01.solvePose( visibility.get(), coordinates.get(), true ); // true for optimal
	  std::cout << "Elapsed time to solve Pose: " << timer1.elapsed_s() << " [s]\n";
	  
	  SimpleRegistration sr02( num_cameras + 1, num_features, K );
	  timer1.start();
	  sr02.solvePose( visibility.get(), coordinates_noise.get(), false ); // false for linear
	  std::cout << "Elapsed time to solve Pose: " << timer1.elapsed_s() << " [s]\n";
	  
	  visibility.reset();
	  coordinates.reset();
	  coordinates_noise.reset();
	  
	  // ======================================== END Range Pose ========================================
	  
	  
	  // ========================================== Error Calculation ==========================================
	  // Final error, Final - First camera
	  Eigen::Vector3d cf, angle_opt, angle_lin;
	  cf = sr01.Qn_global[num_cameras].conjugate()*(-sr01.tr_global[num_cameras]);	// center final - cinit (0)
	  double opt_euclidean_error = cf.norm();
	  cf = sr02.Qn_global[num_cameras].conjugate()*(-sr02.tr_global[num_cameras]);	// center final - cinit (0)
	  double lin_euclidean_error = cf.norm();
	  
	  Eigen::Matrix3d rot = sr01.Qn_global[num_cameras].toRotationMatrix();
	  anglesfromRotationZero( rot, angle_opt );
	  rot = sr02.Qn_global[num_cameras].toRotationMatrix();
	  anglesfromRotationZero( rot, angle_lin );
	  
	  std::cout << "\n================================ Error Stats ==================================\n";
	  std::cout << "Euclidean distance for optimal method = " << opt_euclidean_error << " [m]\n";
	  std::cout << "Angle difference for optimal method = " << angle_opt.transpose() << " (degrees)\n";
	  
	  std::cout << "Euclidean distance for linear method = " << lin_euclidean_error << " [m]\n";
	  std::cout << "Angle difference for linear method = " << angle_lin.transpose() << " (degrees)\n";
	  
	  data_lin_final(exp, 0) = lin_euclidean_error;
	  data_opt_final(exp, 0) = opt_euclidean_error;
	  data_lin_final.row(exp).tail(3) = angle_lin.transpose();
	  data_opt_final.row(exp).tail(3) = angle_opt.transpose();
	  
// 	  std::cout << "Matrix for linear method = " << data_lin_final << "\n";
	  
	  // Drift every camera, each experiment
	  data_drift_lin[exp] = Eigen::MatrixXd::Zero(num_cameras+1,4);
	  data_drift_opt[exp] = Eigen::MatrixXd::Zero(num_cameras+1,4);
	  
	  for (register int cam = 0; cam < num_cameras + 1; ++cam)
	  {
	      Eigen::Vector3d center = qu_synthetic[cam].conjugate()*(-tr_synthetic[cam]);
	      Eigen::Vector3d center_opt = sr01.Qn_global[cam].conjugate()*(-sr01.tr_global[cam]);
	      Eigen::Vector3d center_lin = sr02.Qn_global[cam].conjugate()*(-sr02.tr_global[cam]);
	      
	      double drift_dist_opt = (center - center_opt).norm();
	      double drift_dist_lin = (center - center_lin).norm();
	      
	      // Angles calculation
	      Eigen::Vector3d angle, angle_opt, angle_lin;
	      Eigen::Matrix3d rot;
	      
	      rot = qu_synthetic[cam].toRotationMatrix();
	      anglesfromRotationZero( rot, angle );
	      rot = sr01.Qn_global[cam].toRotationMatrix();
	      anglesfromRotationZero( rot, angle_opt );
	      rot = sr02.Qn_global[cam].toRotationMatrix();
	      anglesfromRotationZero( rot, angle_lin );
	      
	      // Angle error is just the the substraction
	      angle_opt -= angle;
	      angle_lin -= angle;
	      
	      // Saving the data
	      data_drift_opt[exp](cam, 0) = drift_dist_opt;
	      data_drift_lin[exp](cam, 0) = drift_dist_lin;
	      data_drift_opt[exp].row(cam).tail(3) = angle_opt.transpose();
	      data_drift_lin[exp].row(cam).tail(3) = angle_lin.transpose();
	  }
	  
	  
	  
	  // Residual Error (measured - estimated); Calculated in GloblaOptimizer
// // 	  double residual = reprojectionErrorCalculation( &visibility, &coordinates_noise, &intrinsics_param, 
// // 						&opt01.quaternion, &opt01.translation, &opt01.structure );
// // 	  std::cout << "Residual error from function: " << residual << "\n";
// 	  double residual_initial, residual_final, time;
// 	  opt02.stats(residual_initial, residual_final, time);
// 	  
// 	  // Estimation Error (true - estimated)
// 	  double estimation = reprojectionErrorCalculation( &visibility, &coordinates, &intrinsics_param, 
// 						  &opt01.quaternion, &opt01.translation, &opt01.structure );
// 	  std::cout << "Estimation error: " << estimation << "\n";
// 	  
// 	  data_residual_initial(exp, count_std) = residual_initial;
// 	  data_residual_final(exp, count_std) = residual_final;
// 	  data_estimation(exp, count_std) = estimation;
// 	  data_time(exp, count_std) = time;
	  // ======================================== END Error Calculation ========================================
	  
	  // ============================================ Plot Data ============================================
	  
// 	  writeGraph( (char*)"synthetic_opt.graph", sr01.Qn_global, sr01.tr_global ); 
// 	  writeGraph( (char*)"synthetic_lin.graph", sr02.Qn_global, sr02.tr_global );
// 	  writeTextFileVQ((char*)"syn_rot.txt", qu_synthetic);
// 	  writeTextFileVT((char*)"syn_tr.txt", cam_center);
// 	  writeTextFileVQ((char*)"syn_rot_opt.txt", sr01.Qn_global);
// 	  writeTextFileVT((char*)"syn_tr_opt.txt", sr01.tr_global);
// 	  writeTextFileVQ((char*)"syn_rot_lin.txt", sr02.Qn_global);
// 	  writeTextFileVT((char*)"syn_tr_lin.txt", sr02.tr_global);
	  
	  if ( !analysis)
	  {
	      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	      Eigen::Vector4d offset = Eigen::Vector4d::Zero();
	      offset.head(3) = tr_synthetic[0];
	      Eigen::MatrixXd WorldPts = st_synthetic.colwise() + offset;
	      
	      
	      eigen2PointCloud( WorldPts, cloud );
	      
	      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	      viewer = visualizeCloud(cloud);
	      visualizeCameras(viewer, sr01.Qn_global, sr01.tr_global);
	      visualizeCameras(viewer, sr02.Qn_global, sr02.tr_global);
	      
	      while (!viewer->wasStopped ())
	      {
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	      }
	  }
	  
	  // ========================================== END Plot Data ==========================================
        }
//     }
    
    if (analysis)
    {
        ofstream myfile1;
        myfile1.open (output_file.c_str());
        myfile1 << "Synthetic Run of a Structure from Motion system\n";
        myfile1 << "Details [num_cameras x num_features] = [" << details(0) << " x " << details(1) << "]\n";
        myfile1 << "Ratio in circle motion = " << details(3) << "\n";
        myfile1 << "#Experiments(rows): " << experiments << "\n\n";
        myfile1 << "Final distances with linear method:\n" << data_lin_final << "\n\n";
        myfile1 << "Final distances with optimal method:\n" << data_opt_final << "\n\n";
        myfile1 << "Drift linear data:"  << "\n";
        for (register int exp = 0; exp < data_drift_lin.size(); ++exp)
        {
	  myfile1 << "Experiment " << exp << ":\n";
	  myfile1 << data_drift_lin[exp] << "\n";
        }
        myfile1 << "\nDrift optimal data:"  << "\n";
        for (register int exp = 0; exp < data_drift_opt.size(); ++exp)
        {
	  myfile1 << "Experiment " << exp << ":\n";
	  myfile1 << data_drift_opt[exp] << "\n";
        }
        myfile1.close();
    }
    
    exit(0);
}    