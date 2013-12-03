// Octubre 7 2013
// José David Tascón V.

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// Std Libraries
#include <iostream>
#include <random>
#include <string>
#include <cmath>

// Local libraries
#include "Common.hpp"
#include "MultipleCamera.hpp"
#include "Optimizer.hpp"
#include "Plot.hpp"

// using namespace std;
// using namespace Eigen;

void help()
{ 
	std::cout << " Usage: ./synthetic -f <#features> -c <#cameras>\n"
				"\tOptions:\n" 
				"\t[-f]\t Number of features per image to generate (default = 50)\n" 
				"\t[-c]\t Number of cameras to generate (default = 24)\n"
				"\t[-w]\t Width of images (default = 640)\n"
				"\t[-h]\t Height of images (default = 480)\n"
				"\t[-n]\t Standard Deviation for Gaussian Noise (zero mean) in image points (default = 0.3)\n"
				"\t[-d]\t Frontal distance of three-dimensional points to cameras (default = 7.0)\n"
				"\t[-r]\t Ratio of camera circular rigid (default = 0.5)\n"
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
    double ratio = 1.5;
    double frontal_distance = 7.0;
    int num_features;
    int fxc = 50;
    int num_cameras = 24;
    int image_width = 640;
    int image_height = 480;
    std::string output_file = "stats_synthetic_ba.txt";
    
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
	  else if( strcmp( s, "-n" ) == 0 )
	  {
	      i++;
	      noise_std = pchar2float(argv[i]);
	      max_noise = noise_std + 0.1;
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
	  else
	  {
	      help();
	      return fprintf( stderr, "Unknown option %s\n", s ), -1;
	  }
        }
        
    }
    num_features = fxc*num_cameras;
    
    if ( analysis )
    {
        experiments = 21; //nominal 21
        noise_std = 0.1;
        max_noise = 1.01; //nominal 1.1
    }
    
    Eigen::MatrixXd data_residual_initial(experiments,10);
    Eigen::MatrixXd data_residual_final(experiments,10);
    Eigen::MatrixXd data_estimation(experiments,10);
    Eigen::MatrixXd data_time(experiments,10);
    int count_std = 0;
    
    for ( ;noise_std < max_noise; noise_std += 0.1, count_std++)
    {
        for ( int exp = 0; exp < experiments; exp++ )
        {
	  timer_wall timer1;
	  std::vector< Eigen::Quaternion<double> > quaternion(num_cameras);
	  std::vector< Eigen::Quaternion<double> > qu_synthetic(num_cameras);
	  std::vector< Eigen::Matrix3d > rot_synthetic(num_cameras);
	  std::vector< Eigen::Vector3d > cam_center(num_cameras);
	  std::vector< Eigen::Vector3d > translation(num_cameras);
	  std::vector< Eigen::Vector3d > tr_synthetic(num_cameras);
	  std::vector< Eigen::MatrixXd > camera_synthetic(num_cameras);
	  std::vector< Eigen::MatrixXd > image_synthetic(num_cameras);
	  std::vector< Eigen::MatrixXd > image_noise_synthetic(num_cameras);
	  Eigen::MatrixXd structure(4,num_features);
	  Eigen::MatrixXd st_synthetic(4,num_features);
	  
	  Eigen::Matrix<bool,-1,-1> visibility(num_cameras,num_features);
	  Eigen::Matrix< Eigen::Vector3d,-1,-1> coordinates(num_cameras,num_features);
	  Eigen::Matrix< Eigen::Vector3d,-1,-1> coordinates_noise(num_cameras,num_features);
	  
	  Eigen::Matrix3d K;
	  double wd2 = image_width/2.0;
	  double hd2 = image_height/2.0;
	  double f = 600.0;
	  K << f, 0.0, wd2, 0.0, f, hd2, 0.0, 0.0, 1.0;		// Camera Matrix (intrinsics)
	  // Random generators
	  std::random_device rd;
	  std::mt19937_64 gen(rd());
	  std::uniform_real_distribution<double> uniform(0.0,1.0);
	  std::normal_distribution<double> gaussian(0.0,1.0);
	  
	  // ========================================== END Varible declaration ==========================================
	  
	  // ========================================== Cameras init ==========================================
	  // First camera
	  std::cout << "Synthetic Run of a Structure from Motion system:\n";
	  std::cout << "Experiment: " << exp << "; Noise std: " << noise_std << "\n";
	  std::cout << "Init Cameras ... \t\t";
	  
	  qu_synthetic[0] = Eigen::Quaternion<double>::Identity();
	  rot_synthetic[0] = Eigen::Matrix3d::Identity();
	  cam_center[0] = Eigen::Vector3d(0.0, 0.0, 1.0*ratio);		// Circular translation
	  tr_synthetic[0] = qu_synthetic[0]*(-1.0*cam_center[0]);
	  camera_synthetic[0] = buildProjectionMatrix( K, rot_synthetic[0], tr_synthetic[0] );
	  st_synthetic = Eigen::MatrixXd::Ones(4,num_features);
	  visibility = Eigen::Matrix<bool,-1,-1>::Zero(num_cameras,num_features);
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
		X(2,j) = uniform(gen);
	      }
	      Eigen::Matrix3d Rtmp = rot_synthetic[cam];
	      Eigen::MatrixXd tmp = (Rtmp.transpose()*X).colwise() + frontal_distance*cam_center[cam];
	      st_synthetic.block( 0, cam*fxc, 3, fxc ) = tmp;
	      // Debug
        //         printf("Col %d - %d; \tRow %d - %d;\n", cam*fxc, (cam+1)*fxc, 0, 3);
        //         std::cout << "Structure\n" << st_synthetic.block( 0, cam*fxc, 3, fxc ) << "\n";
	  }
	  std::cout << "[OK]\n";
	  // ========================================== END Random points init ==========================================
	  
	  // ========================================== Synthetic Image Data ==========================================
	  // Project structure to images
	  std::cout << "Projections, 2D Points ... \t\t";
	  for(register int cam = 0; cam < num_cameras; ++cam)
	  {
	      Eigen::MatrixXd tmp = st_synthetic;//.block( 0, cam*fxc, 4, fxc );
	      image_synthetic[cam] = camera_synthetic[cam]*tmp;
	      normalizeHomogeneous( image_synthetic[cam] );
	      image_noise_synthetic[cam] = image_synthetic[cam];
	      // Adding the noise to images
	      for(register int ft = 0; ft < num_features; ++ft)
	      {
		image_noise_synthetic[cam](0,ft) += noise_std*gaussian(gen);
		image_noise_synthetic[cam](1,ft) += noise_std*gaussian(gen);
	      }
	      for(register int ft = 0; ft < num_features; ++ft)
	      {
		Eigen::Vector3d ff = image_synthetic[cam].col(ft);
		Eigen::Vector3d mm = image_noise_synthetic[cam].col(ft);
		if ( (mm(0) < image_width) && (mm(0) > 0.0) && (mm(1) < image_height) && (mm(1) > 0.0) )
		{
		    visibility(cam,ft) = true;
		    coordinates_noise(cam,ft) = mm;
		    coordinates(cam,ft) = ff;
		}
	      }
	  }
	  std::cout << "[OK]\n";
            printf("Visibility Matrix [%d x %d]\n\n",visibility.rows(),visibility.cols());
// 	  std::cout << "Visibility =\n" << visibility << "\n";
	  // ======================================== END Synthetic Image Data ========================================
	  
	  // ========================================== Structure from Motion ==========================================
	  double intrinsics[4] = { K(0,0), K(1,1), K(0,2), K(1,2) };
	  std::vector< double > intrinsics_param(&intrinsics[0], &intrinsics[4]);
	  std::vector< double > coefficients(5,0.0);
	  
	  IncrementalBA opt01( &visibility, &coordinates_noise );
	  opt01.setIntrinsics( &intrinsics_param );
	  opt01.setDistortion( &coefficients );
	  opt01.runC();
	  
	  GlobalOptimizer opt02;
	  opt02.setParameters( &visibility, &coordinates_noise, &opt01.quaternion, &opt01.translation, &opt01.structure );
	  opt02.setIntrinsics( &intrinsics_param );
	  opt02.setDistortion( &coefficients );
	  opt02.runBA();
        //     std::cout << "Incrementel BA time: "<< timer1.elapsed_s() << " [s]\n"; 
	  opt01.updateCamera();
	  
	  std::vector< Eigen::Matrix3d > rotation(opt01.quaternion.size());
	  for (register int i = 0; i < rotation.size(); ++i) rotation[i] = opt01.quaternion[i].toRotationMatrix();
	  // ======================================== END Structure from Motion ========================================
	  
	  
	  // ========================================== Error Calculation ==========================================
	  // Residual Error (measured - estimated); Calculated in GloblaOptimizer
// 	  double residual = reprojectionErrorCalculation( &visibility, &coordinates_noise, &intrinsics_param, 
// 						&opt01.quaternion, &opt01.translation, &opt01.structure );
// 	  std::cout << "Residual error from function: " << residual << "\n";
	  double residual_initial, residual_final, time;
	  opt02.stats(residual_initial, residual_final, time);
	  
	  // Estimation Error (true - estimated)
	  double estimation = reprojectionErrorCalculation( &visibility, &coordinates, &intrinsics_param, 
						  &opt01.quaternion, &opt01.translation, &opt01.structure );
	  std::cout << "Estimation error: " << estimation << "\n";
	  
	  data_residual_initial(exp, count_std) = residual_initial;
	  data_residual_final(exp, count_std) = residual_final;
	  data_estimation(exp, count_std) = estimation;
	  data_time(exp, count_std) = time;
	  // ======================================== END Error Calculation ========================================
	  
	  // ============================================ Plot Data ============================================
	  if ( !analysis)
	  {
        //         //Structure in one vector (Global)
        //         std::vector< std::vector<cv::Point3d> > WP;
        //         std::vector<cv::Point3d> pts;
        //         convertEigentoPoint3_( st_synthetic , pts );
        //         WP.push_back(pts);
        //         // Data to plot
        //         PlotGL::viewer.setRotation( &rot_synthetic );
        //         PlotGL::viewer.setTranslation( &tr_synthetic );
        //         PlotGL::viewer.setStructure( &WP );
        //     //     PlotGL::viewer.setColor( &Color );
	      
	      std::vector< std::vector<cv::Point3d> > WP;
	      std::vector<cv::Point3d> pts;
	      convertEigentoPoint3_( opt01.structure , pts );
	      WP.push_back(pts);
	      // Data to plot
	      PlotGL::viewer.setRotation( &rotation );
	      PlotGL::viewer.setTranslation( &opt01.translation );
	      PlotGL::viewer.setStructure( &WP );
	      //     PlotGL::viewer.setColor( &Color );
	  
	      PlotGL::viewer.run(argc, argv, true);
	  }
	  // ========================================== END Plot Data ==========================================
        }
    }
    
    if (analysis)
    {
        ofstream myfile1;
        myfile1.open (output_file.c_str());
        myfile1 << "Synthetic Run of a Structure from Motion system\n";
        myfile1 << "Set size [num_cameras x num_features] = [" << num_cameras << " x " << num_features << "]\n";
        myfile1 << "Camera size = [" << image_width << " x " << image_height << "]\n";
        myfile1 << "Evalutation -> #Noise(columns): " << count_std;
        myfile1 << ", {std from 0.1 to 1.1}\t #Experiments(rows): " << experiments << "\n\n"; 
        myfile1 << "Residual Initial:\n" << data_residual_initial << "\n\n";
        myfile1 << "Time:\n" << data_time << "\n\n";
        myfile1 << "Residual Final:\n" << data_residual_final << "\n\n";
        myfile1 << "Estimation Final:\n" << data_estimation << "\n\n";
        myfile1.close();
    }
    
    exit(0);
}    