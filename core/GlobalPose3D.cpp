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
	      Eigen::Vector4d temp = (*coordinates)(k,ft);
	      Eigen::Vector3d pt = temp.head(3);
	      
	      Eigen::Vector3d pen, pa, pv;
	      Eigen::Matrix3d Cen, Ca, Cv;
	      
	      // use varianceKinect
	      Eigen::Vector3d v1temp;
	      Eigen::Matrix3d Ctmp;
	      varianceKinect( pt, *Calibration, v1temp);
	      Ctmp = Eigen::Matrix3d( v1temp.asDiagonal() );
	      DEBUG_3( std::cout << "pt = " << pt.transpose() << "\n"; )
	      DEBUG_3( std::cout << "Cov = " << v1temp.transpose() << "\n"; )
	      
	      // Put point and covariance to reference frame
	      pv = R.transpose()*(pt - t);
	      Cv = R*Ctmp*R.transpose();
	      
	      // BLUE estimator
	      Eigen::Vector3d v2temp = Covariance.col(ft);
	      Ca = Eigen::Matrix3d( v2temp.asDiagonal() );
	      pa = Structure.col(ft);
	      if ( v2temp(0) != 0) Cen = (Ca.inverse() + Cv.inverse()).inverse();
	      else Cen = Cv;
	      pen = pa + Cen*Cv.inverse()*(pv - pa);
	      
	      DEBUG_3( std::cout << "pen = " << pen.transpose() << "\n"; )
	      DEBUG_3( std::cout << "Cen = " << (Cen.diagonal()).transpose() << "\n"; )
	      
	      //Update
	      Structure.col(ft) = pen;
	      Covariance.col(ft) = Cen.diagonal();
	  }
        }
    }
    
    DEBUG_3( std::cout << "Structure:\n" << Structure << "\n"; )
    
    // Form hera, a global pose+structure to be optimized
    OptimizeG3D gopt;
    gopt.setParameters( &Structure, &Covariance, visibility.get(), coordinates.get(), Qn_global.get(), tr_global.get() );
    gopt.pose_Covariance();
    
}