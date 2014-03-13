// Jul/31/2012
// Author: José David Tascón Vidarte

#include "GlobalOptimizerSfM.hpp"

// ================================================================================================
// ============================= FUNCTIONS of CLASS GlobalOptimizerSfM ============================
// ================================================================================================

/**
 * ******************************************************************
 * @function Constructor
 */
GlobalOptimizerSfM::GlobalOptimizerSfM() 
{
    ;
}

/**
 * ******************************************************************
 * @function Destructor
 */
GlobalOptimizerSfM::~GlobalOptimizerSfM() 
{
	;
}

// Assignation methods
void GlobalOptimizerSfM::setVisibility( Eigen::Matrix<bool,-1,-1> *view_matrix ){ visibility = view_matrix; };
void GlobalOptimizerSfM::setCoordinates( Eigen::Matrix<Eigen::Vector3d,-1,-1> *coord ){ coordinates = coord; };
void GlobalOptimizerSfM::setTranslation( std::vector< Eigen::Vector3d > *tr ){ translation = tr; };
// void GlobalOptimizerSfM::setRotation( std::vector< Eigen:Matrix3d > *rot){ rotation = rot; };
// void GlobalOptimizerSfM::setRotation( std::vector< std::vector<double> > *rot ){ rotation = rot; };
// void GlobalOptimizerSfM::setQuaternion( std::vector< std::vector<double> > *quat ){ quaternion = quat; };
void GlobalOptimizerSfM::setQuaternion( std::vector< Eigen::Quaternion<double> > *quat )
{ 
    quaternion = quat; 
    quaternion_vector2vector_vector( *quat, qv_internal ); // QUATERNION to Vector
};
// void GlobalOptimizerSfM::setStructure( std::vector< std::vector<double> > *st ){ structure = st; };
void GlobalOptimizerSfM::setStructure( Eigen::MatrixXd *st ){ structure = st; };
void GlobalOptimizerSfM::setIntrinsics( std::vector<double> *internal_param ){ intrinsics = internal_param; };
void GlobalOptimizerSfM::setIntrinsics( Eigen::Matrix3d *K )
{
    double tmp[4] = { (*K)(0,0), (*K)(1,1), (*K)(0,2), (*K)(1,2) }; // [fx,fy,cx,cy]
    calibration = std::vector< double >(&tmp[0], &tmp[4]);
    intrinsics = &calibration; 
};
void GlobalOptimizerSfM::setDistortion( std::vector<double> *coefficients){ distortion = coefficients; };

/**
 * ******************************************************************
 * @function setParameters
 */
void GlobalOptimizerSfM::setParameters( Eigen::Matrix<bool,-1,-1> *view_matrix,
	       Eigen::Matrix<Eigen::Vector3d,-1,-1> *coord,
	       std::vector< Eigen::Quaternion<double> > *quat, std::vector< Eigen::Vector3d > *tr,
	       Eigen::MatrixXd *st  )
{
    setVisibility( view_matrix );
    setCoordinates( coord );
    setTranslation( tr );
    setQuaternion( quat );
    setStructure( st );
}
// void GlobalOptimizerSfM::setParameters( Eigen::Matrix<bool,-1,-1> *view_matrix,
// 	       Eigen::Matrix<Eigen::Vector3d,-1,-1> *coord,
// 	       std::vector< std::vector<double> > *quat, std::vector< Eigen::Vector3d > *tr,
// 	       std::vector< std::vector<double> > *st  )
// {
//     setVisibility( view_matrix );
//     setCoordinates( coord );
//     setTranslation( tr );
//     setQuaternion( quat );
//     setStructure( st );
// }

void GlobalOptimizerSfM::update()
{
    vector_vector2quaternion_vector( qv_internal, *quaternion );
//     translation->at(1).normalize();
}

/**
 * ******************************************************************
 * @function runBA
 * @brief Apply Bundle Adjustment to all parameter (cameras and structure)
 */
void GlobalOptimizerSfM::runBA()//char *argv0)
{
    int num_cams = visibility->rows();
    int num_features = visibility->cols();
    int step = structure->rows();
    double cost = 0.0;
    //     google::InitGoogleLogging(argv0);
    Problem problem;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    //ceres::LossFunction* loss_function = FLAGS_robustify ? new ceres::HuberLoss(1.0) : NULL;
    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    options.num_threads = 8;
    options.max_num_iterations = 80;
    
    for (register int ft = 0; ft < num_features ; ++ft)
    {
        if( (*visibility)(0,ft) == true && (*structure)(3,ft) )
        {
	  CostFunction* cost_function0 = new AutoDiffCostFunction<RE_constKDQT_S, 2, 3>( 
	      new RE_constKDQT_S( (*coordinates)(0,ft)(0), (*coordinates)(0,ft)(1), intrinsics->data(), distortion->data(), 
			      qv_internal[0].data(), translation->at(0).data() ));
	  problem.AddResidualBlock(cost_function0, loss_function, (structure->data()+step*ft) );
        }
//         if( (*visibility)(1,ft) == true && (*structure)(3,ft) )
//         {
// 	  CostFunction* cost_function1 = new AutoDiffCostFunction<RE_constKD_normT_QTS, 2, 4, 3, 3>( 
// 	      new RE_constKD_normT_QTS( (*coordinates)(1,ft)(0), (*coordinates)(1,ft)(1), intrinsics->data(), distortion->data() ));
// 	  problem.AddResidualBlock(cost_function1, loss_function, qv_internal[1].data(), translation->at(1).data(), (structure->data()+step*ft) );
//         }
    }
    for (register int cam = 1; cam < num_cams; ++cam)
    {
        for (register int ft = 0; ft < num_features ; ++ft)
        {
	  if( (*visibility)(cam,ft) == true && (*structure)(3,ft) )
	  {
	      CostFunction* cost_function = new AutoDiffCostFunction<RE_constKD_QTS, 2, 4, 3, 3>( 
		new RE_constKD_QTS( (*coordinates)(cam,ft)(0), (*coordinates)(cam,ft)(1), intrinsics->data(), distortion->data() ));
	      problem.AddResidualBlock(cost_function, loss_function, qv_internal[cam].data(), translation->at(cam).data(), (structure->data()+step*ft) );
	  }
        }
    }
    
    cost = 0.0;
    problem.Evaluate(Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
    initial_reprojection_error = std::sqrt(double(cost/num_features));
    /// Needs to be corrected to the number of observations, 
    // I can use   || int Problem::NumResiduals() || to know that number easily (having account the amount of residuals)
    printf("Initial RMS Reprojection Error is : %0.8f\n", initial_reprojection_error);
    
    Solver::Summary summary;
    timer_wall timer1;
    timer1.start();
    Solve(options, &problem, &summary);
    time = timer1.elapsed_s();
    std::cout << summary.BriefReport() << "\n";
    
    cost = 0.0;
    problem.Evaluate(Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
    final_reprojection_error = std::sqrt(double(cost/num_features));
    printf("Final RMS Reprojection Error is : %0.8f\n", final_reprojection_error);
    printf("Time to solve %f [s]\n\n", time);
    
    update(); // update quaternion; normaliza translation 1
    return;
}

void GlobalOptimizerSfM::stats(double &initial_error, double &final_error, double &solver_time)
{
    initial_error = initial_reprojection_error;
    final_error = final_reprojection_error;
    solver_time = time;
}


// ================================================================================================
// ====================================== ADD FUNCTION ============================================
// ================================================================================================

double reprojectionErrorCalculation(Eigen::Matrix<bool,-1,-1> *visibility, Eigen::Matrix<Eigen::Vector3d,-1,-1> *coordinates,
		    std::vector<double> *intrinsics, std::vector< Eigen::Quaternion<double> > *quaternion, 
		    std::vector< Eigen::Vector3d > *translation, Eigen::MatrixXd *structure)
{
    int num_cams = visibility->rows();
    int num_features = visibility->cols();
    int step = structure->rows();
    std::vector< Eigen::Vector4d > q_vector;
    quaternion_vector2eigen_vector( *quaternion, q_vector );
    
    Problem problem;
    
    for (register int cam = 0; cam < num_cams; ++cam)
    {
        for (register int ft = 0; ft < num_features ; ++ft)
        {
	  if( (*visibility)(cam,ft) == true && (*structure)(3,ft) )
	  {
    	      CostFunction* cost_function = new AutoDiffCostFunction<RE_constK_QTS, 2, 4, 3, 3>( 
		new RE_constK_QTS( (*coordinates)(cam,ft)(0), (*coordinates)(cam,ft)(1), intrinsics->data()));
	      problem.AddResidualBlock(cost_function, NULL, q_vector[cam].data(), translation->at(cam).data(), (structure->data()+step*ft) );
	  }
        }
    }
    
    double cost = 0;
    problem.Evaluate(Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
    return std::sqrt(double(cost/num_features));
}
    

// ============================================================================
// ================================= AUXILIAR =================================
// 		Use to create .... in your code
// ============================================================================

// /**
//  * ******************************************************************
//  * @functions ....
//  */