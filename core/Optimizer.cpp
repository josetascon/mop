// Jul/31/2012
// Author: José David Tascón Vidarte

#include "Optimizer.hpp"

// ================================================================================================
// ============================== FUNCTIONS of CLASS GlobalOptimizer===============================
// ================================================================================================

/**
 * ******************************************************************
 * @function Constructor
 */
GlobalOptimizer::GlobalOptimizer() 
{
    ;
}

/**
 * ******************************************************************
 * @function Destructor
 */
GlobalOptimizer::~GlobalOptimizer() 
{
	;
}

// Assignation methods
void GlobalOptimizer::setVisibility( Eigen::Matrix<bool,-1,-1> *view_matrix ){ visibility = view_matrix; };
void GlobalOptimizer::setCoordinates( Eigen::Matrix<Eigen::Vector3d,-1,-1> *coord ){ coordinates = coord; };
void GlobalOptimizer::setTranslation( std::vector< Eigen::Vector3d > *tr ){ translation = tr; };
// void GlobalOptimizer::setRotation( std::vector< Eigen:Matrix3d > *rot){ rotation = rot; };
// void GlobalOptimizer::setRotation( std::vector< std::vector<double> > *rot ){ rotation = rot; };
// void GlobalOptimizer::setQuaternion( std::vector< std::vector<double> > *quat ){ quaternion = quat; };
void GlobalOptimizer::setQuaternion( std::vector< Eigen::Quaternion<double> > *quat )
{ 
    quaternion = quat; 
    quaternion_vector2vector_vector( *quat, qv_internal ); // QUATERNION to Vector
};
// void GlobalOptimizer::setStructure( std::vector< std::vector<double> > *st ){ structure = st; };
void GlobalOptimizer::setStructure( Eigen::MatrixXd *st ){ structure = st; };
void GlobalOptimizer::setIntrinsics( std::vector<double> *internal_param ){ intrinsics = internal_param; };
void GlobalOptimizer::setDistortion( std::vector<double> *coefficients){ distortion = coefficients; };

/**
 * ******************************************************************
 * @function setParameters
 */
void GlobalOptimizer::setParameters( Eigen::Matrix<bool,-1,-1> *view_matrix,
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
// void GlobalOptimizer::setParameters( Eigen::Matrix<bool,-1,-1> *view_matrix,
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

void GlobalOptimizer::update()
{
    vector_vector2quaternion_vector( qv_internal, *quaternion );
//     (*translation)[1].normalize();
}

/**
 * ******************************************************************
 * @function runBA
 * @brief Apply Bundle Adjustment to all parameter (cameras and structure)
 */
void GlobalOptimizer::runBA()//char *argv0)
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
			      qv_internal[0].data(), (*translation)[0].data() ));
	  problem.AddResidualBlock(cost_function0, loss_function, (structure->data()+step*ft) );
        }
//         if( (*visibility)(1,ft) == true && (*structure)(3,ft) )
//         {
// 	  CostFunction* cost_function1 = new AutoDiffCostFunction<RE_constKD_normT_QTS, 2, 4, 3, 3>( 
// 	      new RE_constKD_normT_QTS( (*coordinates)(1,ft)(0), (*coordinates)(1,ft)(1), intrinsics->data(), distortion->data() ));
// 	  problem.AddResidualBlock(cost_function1, loss_function, qv_internal[1].data(), (*translation)[1].data(), (structure->data()+step*ft) );
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
	      problem.AddResidualBlock(cost_function, loss_function, qv_internal[cam].data(), (*translation)[cam].data(), (structure->data()+step*ft) );
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

void GlobalOptimizer::stats(double &initial_error, double &final_error, double &solver_time)
{
    initial_error = initial_reprojection_error;
    final_error = final_reprojection_error;
    solver_time = time;
}


// ================================================================================================
// ================================ FUNCTIONS of CLASS OptimizeG3D=================================
// ================================================================================================
void OptimizeG3D::pose_Covariance()
{
    DEBUG_1( std::cout << "Global, 3D Pose Optimization\t...\n"; )

    if ( Variance->cols() != Pts->cols() )
    {
        std::cout << "Warning: Aligment of 3D Points do not run, Standard Deviation matrix unavailable or deficient\n";
        return;
    }
    
    int num_cams = visibility->rows();
    int num_features = visibility->cols();
    int step = Pts->rows();
    double cost = 0.0;
    
    Problem problem;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    
    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false; // true for step values
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    options.num_threads = 8;
    options.max_num_iterations = 40;
    
    DEBUG_2( std::cout << "Total cams: " << num_cams << "\n"; )
    DEBUG_2( std::cout << "Total features: " << num_features << "\n"; )
    DEBUG_2( std::cout << "Quaternions internal: " << qv_internal.size() << "\n"; )
    DEBUG_2( std::cout << "Quaternions internal doubles: " << qv_internal[0].size() << "\n"; )
    DEBUG_2( printf("Pts: [%i x %i]\n", Pts->rows(), Pts->cols()); )
    
    for (register int cam = 0; cam < num_cams; ++cam)
    {
        for (register int ft = 0; ft < num_features ; ++ft)
        {
	  Eigen::Vector3d vv = Variance->col(ft); // Check if variance vector (corresponding to feature ft) is not zero
	  if( (*visibility)(cam,ft) && !(vv.isZero()) )
	  {
	      CostFunction* cost_function = new AutoDiffCostFunction<RE3D_QTS_Cov, 3, 4, 3, 3>( 
		        new RE3D_QTS_Cov( ((*coordinates)(cam,ft).data()), (Variance->data()+step*ft)));
	      problem.AddResidualBlock(cost_function, loss_function, qv_internal[cam].data(), (*trs)[cam].data(), (Pts->data()+step*ft) );
// 	      CostFunction* cost_function = new AutoDiffCostFunction<RE3D_constS_QT_Cov, 3, 4, 3>( 
// 		        new RE3D_constS_QT_Cov( (Pts->data()+step*ft), ((*coordinates)(cam,ft).data()), (Variance->data()+step*ft)));
// 	      problem.AddResidualBlock(cost_function, loss_function, qv_internal[cam].data(), (*trs)[cam].data() );
	  }
        }
        DEBUG_3( std::cout << "Camera: " << cam << "\n"; )
    }
    cost = 0.0;
    problem.Evaluate(Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
    DEBUG_1( std::cout << "Initial Mahalanobis Distance Error is : " << std::sqrt(double(cost/num_features)) << "\n"; )
    
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    DEBUG_1( std::cout << summary.BriefReport() << "\n"; )
    
    cost = 0.0;
    problem.Evaluate(Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
    DEBUG_1( std::cout << "Final Mahalanobis Distance Error is : " << std::sqrt(double(cost/num_features)) << "\n"; )
    updateQuaternion();
}

/**
 * ******************************************************************
 * @function localBA
 * @brief Apply Bundle Adjustment locally to neighbors defined by local_frames (cameras and structure)
 */
// void Optimizer::localBA(int local_frames)
// {
//     int num_cams = visibility->rows();
//     int num_features = visibility->cols();
//     int step = structure->rows();
// //     google::InitGoogleLogging(argv0);
//     
// //     Problem problem;
// //     ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
//     Solver::Options options;
//     options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//     options.minimizer_progress_to_stdout = false;
//     options.gradient_tolerance = 1e-16;
//     options.function_tolerance = 1e-16;
//     options.num_threads = 8;
//     options.max_num_iterations = 20;
//     
//     Eigen::Matrix3d Calibration;
//     Calibration << (*intrinsics)[0], 0.0, (*intrinsics)[2], 0.0, (*intrinsics)[1], (*intrinsics)[3], 0.0, 0.0, 1.0;
//     
//     // Local pointers
//     Eigen::MatrixXd * const ptr_structure = structure;
//     std::vector< std::vector<double> > * const ptr_qv_internal = &qv_internal;
//     std::vector< Eigen::Vector3d > * const ptr_translation = translation;
//     
//     for (int cam = 0; cam < num_cams; cam += local_frames-2)
//     {
//         Problem problem;
//         ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
//         std::cout << "Initial camera is : " << cam << "\n";
//         
//         for (int loc = 0; (loc < local_frames) && (cam+loc < num_cams) ; loc++) //|| (cam+loc < num_cams)
//         {
// 	  std::cout << "iteration: " << cam+loc << "\n";
// 	  if( loc < 2 ) // preserve previous translation and orientations
// 	  {
// 	      for (int ft = 0; ft < num_features ; ft++)
// 	      {
// 		if ((*visibility)(cam+loc,ft) == true)
// 		{
// 		    CostFunction* cost_function0 = new AutoDiffCostFunction<RE_constKDQT_S, 2, 3>( 
// 		        new RE_constKDQT_S( (*coordinates)(cam+loc,ft)(0), (*coordinates)(cam+loc,ft)(1), intrinsics->data(), distortion->data(), 
// 				        (*ptr_qv_internal)[cam+loc].data(), (*ptr_translation)[cam+loc].data() ));
// 		    problem.AddResidualBlock(cost_function0, loss_function, (ptr_structure->data()+step*ft) );
// 		}
// 	      }
// 	  }
// 	  else
// 	  {
// 	      Eigen::Vector3d tr_tmp1, tr_tmp2;
// 	      Eigen::Quaternion<double> qt_tmp1, qt_tmp2;
// 	      tr_tmp2 = (*ptr_translation)[cam+loc];
// 	      tr_tmp1 = (*ptr_translation)[cam+loc-1];
// 	      vector2quaternion( (*ptr_qv_internal)[cam+loc], qt_tmp2);
// 	      vector2quaternion( (*ptr_qv_internal)[cam+loc-1], qt_tmp1);
// 	      (*ptr_translation)[cam+loc] = qt_tmp2*tr_tmp1 + tr_tmp2;
// 	      qt_tmp2 = qt_tmp2*qt_tmp1;
// 	      quaternion2vector( qt_tmp2 , (*ptr_qv_internal)[cam+loc]);
// 	      std::vector< Eigen::MatrixXd > Cameras12(2);
// 	      Eigen::Matrix3d r = qt_tmp1.toRotationMatrix();
// 	      Cameras12[0] = buildProjectionMatrix( Calibration, r, (*ptr_translation)[cam+loc-1] );
// 	      r = qt_tmp2.toRotationMatrix();
// 	      Cameras12[1] = buildProjectionMatrix( Calibration, r, (*ptr_translation)[cam+loc] );
// 	      solveStructureInitial( (*visibility), (*coordinates), (*ptr_structure), Cameras12, cam+loc-1, cam+loc );
// 	      
// 	      for (int ft = 0; ft < num_features ; ft++)
// 	      {
// 		if ((*visibility)(cam+loc,ft) == true)
// 		{	  
// 		    CostFunction* cost_function1 = new AutoDiffCostFunction<RE_constKD_QTS, 2, 4, 3, 3>( 
// 		        new RE_constKD_QTS( (*coordinates)(cam+loc,ft)(0), (*coordinates)(cam+loc,ft)(1), intrinsics->data(), distortion->data() ));
// 		    problem.AddResidualBlock(cost_function1, loss_function, (*ptr_qv_internal)[cam+loc].data(), 
// 				         (*ptr_translation)[cam+loc].data(), (ptr_structure->data()+step*ft) );
// 		}
// 	      }
// 	  }
//         }
//         
//         std::cout << "Start opt\n"; 
//         Solver::Summary summary;
//         Solve(options, &problem, &summary);
//         std::cout << summary.BriefReport() << "\n";
//         
//         double cost = 0;
//         problem.Evaluate(Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
//         std::cout << "RMS Reprojection Error is : " << std::sqrt(double(cost/num_features)) << "\n\n";
//         std::cout << "End opt\n";
//         
//     }
//     update(); // update quaternion
// }


/**
 * ******************************************************************
 * @function incrementalBA
 * @brief Apply Bundle Adjustment to all parameter (cameras and structure)
 */
// void Optimizer::incrementalBA()//char *argv0)
// {
//     ;
// }




// ================================================================================================
// ============================== FUNCTIONS of CLASS LocalOptimizer================================
// ================================================================================================
/**
 * ******************************************************************
 * @function Constructor
 */
LocalOptimizer::LocalOptimizer() 
{
    ;
}

/**
 * ******************************************************************
 * @function Destructor
 */
LocalOptimizer::~LocalOptimizer() 
{
    ;
}

void LocalOptimizer::setParameters(Eigen::MatrixXd *obs1, Eigen::MatrixXd *obs2, 
		   Eigen::Matrix3d *Rot, Eigen::Vector3d *tr)
{
    observation1 = obs1;
    observation2 = obs2;
    translation = tr;
    
    // rotation conditioning
    rotation = Rot;
    Eigen::Quaternion<double> qout(*Rot);
    quaternion2vector<double,double>( qout, quaternionV);
}

void LocalOptimizer::setParameters3Dto3D(Eigen::MatrixXd *obs1, Eigen::MatrixXd *obs2, 
		   Eigen::Matrix3d *Rot, Eigen::Vector3d *tr)
{
    setParameters(obs1, obs2, Rot, tr);
}


void LocalOptimizer::setParameters3Dto3D(Eigen::MatrixXd *obs1, Eigen::MatrixXd *obs2, 
				 Eigen::Matrix3d *Rot, Eigen::Vector3d *tr, 
				 Eigen::MatrixXd *var1, Eigen::MatrixXd *var2)
{
    setParameters(obs1, obs2, Rot, tr);
    variance1 = var1;
    variance2 = var2;
}

void LocalOptimizer::setParameters3Dto2D(Eigen::MatrixXd *obs1, Eigen::MatrixXd *obs2, 
		   Eigen::Matrix3d *Rot, Eigen::Vector3d *tr, Eigen::MatrixXd *st )
{
    setParameters( obs1, obs2, Rot, tr);
    // structure conditioning
    structure = st;
//     convertEigentoVector( *st, structureV );
}

void LocalOptimizer::updatePose3Dto2D()
{
    update();
    translation->normalize();
    // update Structure data
//     convertVectortoEigen( structureV, *structure );
}

void LocalOptimizer::update()
{
    // update rotation data
    Eigen::Quaternion<double> qout;
    vector2quaternion<double,double>(quaternionV, qout);
    qout.normalize();
    *rotation = qout.toRotationMatrix();
}

void LocalOptimizer::pose3Dto2D()
{
    DEBUG_2( std::cout << "Two view Optimization:\n"; )
    
    Problem problem;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    
    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false; //true
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    options.num_threads = 8;
    options.max_num_iterations = 20;
    int step = structure->rows();
    int num_features = observation1->cols();
    double quaternionI[4] = {1.0,0.0,0.0,0.0};
    double trI[3] = {0.0,0.0,0.0};
//     std::cout << "Observation size : " << observation1->cols() << "\n";
//     for (register int ft = 0; ft < num_features ; ++ft) // Function with only intrinsics
//     {
//         CostFunction* cost_function1 = new AutoDiffCostFunction<RE_constKQT_S, 2, 3>( 
// 		new RE_constKQT_S( (*observation1)(0,ft), (*observation1)(1,ft), intrinsics->data(), &quaternionI, &trI  ));
//         problem.AddResidualBlock(cost_function1, loss_function, (structure->data()+step*ft) );
//         
//         CostFunction* cost_function2 = new AutoDiffCostFunction<RE_constK_QTS, 2, 4, 3, 3>( 
// 		new RE_constK_QTS( (*observation2)(0,ft), (*observation2)(1,ft), intrinsics->data() ));
//         problem.AddResidualBlock(cost_function2, loss_function, quaternionV.data(), translation->data(), (structure->data()+step*ft) );
//     }
    
    for (register int ft = 0; ft < num_features ; ++ft)
    {
        CostFunction* cost_function1 = new AutoDiffCostFunction<RE_constKDQT_S, 2, 3>( 
		new RE_constKDQT_S( (*observation1)(0,ft), (*observation1)(1,ft), intrinsics->data(), distortion->data(), &quaternionI[0], &trI[0] ));
        problem.AddResidualBlock(cost_function1, loss_function, (structure->data()+step*ft) );
        
        CostFunction* cost_function2 = new AutoDiffCostFunction<RE_constKD_normT_QTS, 2, 4, 3, 3>( 
		new RE_constKD_normT_QTS( (*observation2)(0,ft), (*observation2)(1,ft), intrinsics->data(), distortion->data() ));
        problem.AddResidualBlock(cost_function2, loss_function, quaternionV.data(), translation->data(), (structure->data()+step*ft) );
    }
    
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    DEBUG_2( std::cout << summary.BriefReport() << "\n"; )

    double cost = 0;
    problem.Evaluate(Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
    DEBUG_2( std::cout << "RMS Reprojection Error is : " << std::sqrt(double(cost/num_features)) << "\n\n"; )

    updatePose3Dto2D();
//     cost = 0;
//     problem.Evaluate(Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
//     std::cout << "Evaluation: " << cost << "\n";
}

void LocalOptimizer::pose3Dto3D()
{
    DEBUG_2( std::cout << "Aligment of 3D Points, Pose Optimization\t...\n"; )
    
    Problem problem;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    
    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    options.num_threads = 8;
    options.max_num_iterations = 20;
    
    int num_features = observation1->cols();
    
    for (register int ft = 0; ft < num_features ; ++ft)
    {
        CostFunction* cost_function = new AutoDiffCostFunction<RE3D, 6, 4, 3>( 
		new RE3D( (observation1->data()+3*ft), (observation2->data()+3*ft) ));
        problem.AddResidualBlock(cost_function, loss_function, quaternionV.data(), translation->data() );
    }
    
//     ReprojectionError3D(double *pt1, double *pt2) //pt2 = q*pt1*~q + t ; ~q*(p2 - t1)*q = pt1
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    DEBUG_2( std::cout << summary.BriefReport() << "\n"; )
    
    double cost = 0.0;
    problem.Evaluate(Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
    double reprojection_error = std::sqrt(double(cost/num_features));
    DEBUG_2( printf("RMS Reprojection Error is : %0.8f\n", reprojection_error); )
    
    update();
}

void LocalOptimizer::pose3Dto3D_Covariance()
{
    DEBUG_2( std::cout << "Aligment of 3D Points, Pose Optimization\t...\n"; )
    
    if ( variance1->cols() != observation1->cols() )
    {
        std::cout << "Warning: Aligment of 3D Points do not run, Standard Deviation matrix unavailable or deficient\n";
        return;
    }
    
    Problem problem;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    
    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    options.num_threads = 8;
    options.max_num_iterations = 30;
    
    int num_features = observation1->cols();
    
    for (register int ft = 0; ft < num_features ; ++ft)
    {
        CostFunction* cost_function = new AutoDiffCostFunction<RE3D_Covariance, 6, 4, 3>( 
		new RE3D_Covariance( (observation1->data()+3*ft), (observation2->data()+3*ft), (variance1->data()+3*ft), (variance2->data()+3*ft)  ));
        problem.AddResidualBlock(cost_function, loss_function, quaternionV.data(), translation->data() );
    }
//     ReprojectionError3D(double *pt1, double *pt2) //pt2 = q*pt1*~q + t ; ~q*(p2 - t1)*q = pt1
    double cost = 0;
    problem.Evaluate(Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
    DEBUG_2( std::cout << "Initial Mahalanobis Distance Error is : " << std::sqrt(double(cost/num_features)) << "\n"; )
    
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    DEBUG_2( std::cout << summary.BriefReport() << "\n"; )
    
    cost = 0;
    problem.Evaluate(Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
    DEBUG_2( std::cout << "Final Mahalanobis Distance Error is : " << std::sqrt(double(cost/num_features)) << "\n"; )
    update();
}

// ================================================================================================
// ============================ FUNCTIONS of CLASS PartialIncremental =============================
// ================================================================================================
void PartialIncremental::update(int constant_cam, int num_cams)
{
    for (register int cam = constant_cam; cam < num_cams; ++cam)
    {
        eigen2quaternion(q_vector[cam],(*quaternion)[cam]);
        (*quaternion)[cam].normalize();
    }
}

void PartialIncremental::run(int constant_cam, int num_cams)
{
    int num_features = visibility->cols();
    int step = structure->rows();
    quaternion_vector2eigen_vector( *quaternion, q_vector );
    
    Problem problem;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    options.num_threads = 8;
    options.max_num_iterations = 10;
    
    for (register int cam = 0; cam < constant_cam; ++cam)
    {
        for (register int ft = 0; ft < num_features ; ++ft)
        {
	  if( (*visibility)(cam,ft) == true && (*structure)(3,ft) )
	  {
	      CostFunction* cost_function0 = new AutoDiffCostFunction<RE_constKDQT_S, 2, 3>( 
	      new RE_constKDQT_S( (*coordinates)(cam,ft)(0), (*coordinates)(cam,ft)(1), intrinsics->data(), distortion->data(), 
			      q_vector[cam].data(), (*translation)[cam].data() ));
	      problem.AddResidualBlock(cost_function0, loss_function, (structure->data()+step*ft) );
	  }
        }
    }
    for (register int cam = constant_cam; cam < num_cams; ++cam)
    {
        for (register int ft = 0; ft < num_features ; ++ft)
        {
	  if( (*visibility)(cam,ft) == true && (*structure)(3,ft) )
	  {
    	      CostFunction* cost_function = new AutoDiffCostFunction<RE_constKD_QTS, 2, 4, 3, 3>( 
		new RE_constKD_QTS( (*coordinates)(cam,ft)(0), (*coordinates)(cam,ft)(1), intrinsics->data(), distortion->data() ));
	      problem.AddResidualBlock(cost_function, loss_function, q_vector[cam].data(), (*translation)[cam].data(), (structure->data()+step*ft) );
	  }
        }
    }
    
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    
    double cost = 0;
    problem.Evaluate(Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
    std::cout << "RMS Reprojection Error is : " << std::sqrt(double(cost/num_features)) << "\n\n";
    
    update(constant_cam, num_cams); // update quaternion and normalize
    return;
}


// ================================================================================================
// =============================== FUNCTIONS of CLASS BALOptimizer ================================
// ================================================================================================
void BALOptimizer::runBAL()
{
    int num_cams = visibility->rows();
    int num_features = visibility->cols();
    int step_tr = translation_and_intrinsics->rows();
    int step_st = structure->rows();
    double cost;
    quaternion_vector2eigen_vector( *quaternion, q_vector );
    
    Problem problem;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    
    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    options.num_threads = 8;
    options.max_num_iterations = 50;
    
    for (register int cam = 0; cam < num_cams; ++cam)
    {
        for (register int ft = 0; ft < num_features ; ++ft)
        {
	  if( (*visibility)(cam,ft) == true )
	  {
	      CostFunction* cost_function = new AutoDiffCostFunction<Snavely_RE_KDQTS, 2, 4, 6, 3>( 
		new Snavely_RE_KDQTS( (*coordinates)(cam,ft)(0), (*coordinates)(cam,ft)(1)) );
	      problem.AddResidualBlock(cost_function, loss_function, q_vector[cam].data(), 
				(translation_and_intrinsics->data()+step_tr*cam), (structure->data()+step_st*ft) );
	  }
        }
    }
    
    cost = 0;
    problem.Evaluate(Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
    std::cout << "Initial RMS Reprojection Error is : " << std::sqrt(double(cost/num_features)) << "\n";
    
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    
    cost = 0;
    problem.Evaluate(Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
    std::cout << "RMS Reprojection Error is : " << std::sqrt(double(cost/num_features)) << "\n\n";
    
    update(); // update quaternion; normaliza translation 1
    return;
}

void BALOptimizer::update()
{
    for (register int cam = 0; cam < q_vector.size(); ++cam)
    {
        eigen2quaternion(q_vector[cam],(*quaternion)[cam]);
        (*quaternion)[cam].normalize();
    }
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
	      problem.AddResidualBlock(cost_function, NULL, q_vector[cam].data(), (*translation)[cam].data(), (structure->data()+step*ft) );
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