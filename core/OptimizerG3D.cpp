// Jul/31/2012
// Author: José David Tascón Vidarte

#include "OptimizerG3D.hpp"

// ================================================================================================
// =============================== FUNCTIONS of CLASS OptimizerG3D ================================
// ================================================================================================
void OptimizerG3D::pose_Covariance()
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
    options.max_num_iterations = 50;
    
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
	      problem.AddResidualBlock(cost_function, loss_function, qv_internal[cam].data(), trs->at(cam).data(), (Pts->data()+step*ft) );
// 	      CostFunction* cost_function = new AutoDiffCostFunction<RE3D_constS_QT_Cov, 3, 4, 3>( 
// 		        new RE3D_constS_QT_Cov( (Pts->data()+step*ft), ((*coordinates)(cam,ft).data()), (Variance->data()+step*ft)));
// 	      problem.AddResidualBlock(cost_function, loss_function, qv_internal[cam].data(), trs->at(cam).data() );
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

void OptimizerG3D::pose_LSQ()
{
    DEBUG_1( std::cout << "Global, 3D Pose Optimization (LSQ)\t...\n"; )
    
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
    options.max_num_iterations = 50;
    
    DEBUG_2( std::cout << "Total cams: " << num_cams << "\n"; )
    DEBUG_2( std::cout << "Total features: " << num_features << "\n"; )
    DEBUG_2( std::cout << "Quaternions internal: " << qv_internal.size() << "\n"; )
    DEBUG_2( std::cout << "Quaternions internal doubles: " << qv_internal[0].size() << "\n"; )
    DEBUG_2( printf("Pts: [%i x %i]\n", Pts->rows(), Pts->cols()); )
    
    for (register int cam = 0; cam < num_cams; ++cam)
    {
        for (register int ft = 0; ft < num_features ; ++ft)
        {
	  Eigen::Vector4d vv = (*coordinates)(cam,ft); // Check if variance vector (corresponding to feature ft) is not zero
	  if( (*visibility)(cam,ft) && !(vv.isZero()) )
	  {
	      CostFunction* cost_function = new AutoDiffCostFunction<RE3D_QTS, 3, 4, 3, 3>( 
		        new RE3D_QTS( (*coordinates)(cam,ft).data()));
	      problem.AddResidualBlock(cost_function, loss_function, qv_internal[cam].data(), trs->at(cam).data(), (Pts->data()+step*ft) );
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