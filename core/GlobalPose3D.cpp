// Created: Jan/31/2013
// Author: José David Tascón Vidarte

#include "GlobalPose3D.hpp"

void GlobalPose3D::solve( boost::shared_ptr< MXb > visibility, boost::shared_ptr< MX_V4d > coordinates,
		     Eigen::Matrix3d *Calibration,
		     boost::shared_ptr< Qd_vector > Qn_global, boost::shared_ptr< V3d_vector > tr_global)
{
    //TODO CHANGE coordinates3d to vector3d to save memory, is ba style (projective geometry) and unnecesary here
    num_cameras = visibility->rows();
    num_features = visibility->cols();
    Structure = Eigen::MatrixXd::Zero(3,num_features);
    Covariance = Eigen::MatrixXd::Zero(3,num_features);
    
    DEBUG_1( std::cout << "\n================================ Global Pose Optimization ==================================\n"; )
    for(register int k = 0; k < num_cameras; k++) // pose 1-2
    {   
        Eigen::Matrix3d R = (*Qn_global)[k].toRotationMatrix();
        Eigen::Vector3d t = (*tr_global)[k];
        
        for (register int ft = 0; ft < num_features; ft++)
        {
	  if ( (*visibility)(k,ft) )
	  {
	      Eigen::Vector3d pa, va_tmp, pv, vv_temp;
	      
	      // Point1: actual
	      pa = Structure.col(ft);
	      va_tmp = Covariance.col(ft);
	      
	      // Point2: viewed
	      Eigen::Vector4d temp = (*coordinates)(k,ft);
	      pv = temp.head(3);
	      varianceKinect( pv, *Calibration, vv_temp); 	// Use varianceKinect for variance2
	      
	      LinearEstimator<double> blue( pa, va_tmp, pv, vv_temp, R, t );
	      blue.estimate();
	      
	      //Update
	      Structure.col(ft) = blue.getPoint();
	      Covariance.col(ft) = blue.getVariance();
	  }
        }
    }
    
    DEBUG_3( std::cout << "Structure:\n" << Structure << "\n"; )
    
    // Form hera, a global pose+structure to be optimized
    OptimizeG3D gopt;
    gopt.setParameters( &Structure, &Covariance, visibility.get(), coordinates.get(), Qn_global.get(), tr_global.get() );
    gopt.pose_Covariance();
    
}