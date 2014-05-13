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
    
    num_cams = visibility->rows();
    num_features = visibility->cols();
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
    
    num_observations = 0;
    for (register int cam = 0; cam < num_cams; ++cam)
    {
        for (register int ft = 0; ft < num_features ; ++ft)
        {
	  Eigen::Vector3d vv = Variance->col(ft); // Check if variance vector (corresponding to feature ft) is not zero
	  if( (*visibility)(cam,ft) && !(vv.isZero()) )
	  {
	      num_observations++;
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
    cost = reprojectionError(problem);
    DEBUG_1( printf("Visibility Matrix [Cameras x Features] -> Observations: [%d x %d] -> %d\n", num_cams, num_features, num_observations); )
    DEBUG_1( std::cout << "Initial Mahalanobis Distance Error is : " << cost << "\n"; )
    DEBUG_1( std::cout << "Initial Euclidean Distance Error is : " << euclideanReprojectionError() << "\n"; )
    
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    DEBUG_1( std::cout << summary.BriefReport() << "\n"; )
    std::vector< IterationSummary > it_summary = summary.iterations;
    if (export_txt) exportFileReport( it_summary );
    
    cost = reprojectionError(problem);
    DEBUG_1( std::cout << "Final Mahalanobis Distance Error is : " << cost << "\n"; )
    DEBUG_1( std::cout << "Final Euclidean Distance Error is : " << euclideanReprojectionError() << "\n"; )
    updateQuaternion();
}

void OptimizerG3D::pose_LSQ()
{
    DEBUG_1( std::cout << "Global, 3D Pose Optimization (LSQ)\t...\n"; )
    
    num_cams = visibility->rows();
    num_features = visibility->cols();
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
    
    num_observations = 0;
    for (register int cam = 0; cam < num_cams; ++cam)
    {
        for (register int ft = 0; ft < num_features ; ++ft)
        {
	  Eigen::Vector4d vv = (*coordinates)(cam,ft); // Check if variance vector (corresponding to feature ft) is not zero
	  if( (*visibility)(cam,ft) && !(vv.isZero()) )
	  {
	      num_observations++;
	      CostFunction* cost_function = new AutoDiffCostFunction<RE3D_QTS, 3, 4, 3, 3>( 
		        new RE3D_QTS( (*coordinates)(cam,ft).data()));
	      problem.AddResidualBlock(cost_function, loss_function, qv_internal[cam].data(), trs->at(cam).data(), (Pts->data()+step*ft) );
	  }
        }
        DEBUG_3( std::cout << "Camera: " << cam << "\n"; )
    }
    cost = reprojectionError(problem);
    DEBUG_1( printf("Visibility Matrix [Cameras x Features] -> Observations: [%d x %d] -> %d\n", num_cams, num_features, num_observations); )
    DEBUG_1( std::cout << "Initial Euclidean Distance Error is : " << cost << "\n"; )
    
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    DEBUG_1( std::cout << summary.BriefReport() << "\n"; )
//     if (export_txt) exportFileReport( summary.FullReport() );
    
    cost = reprojectionError(problem);
    DEBUG_1( std::cout << "Final Euclidean Distance Error is : " << cost << "\n"; )
    updateQuaternion();
}

double OptimizerG3D::reprojectionError(Problem &problem)
{
    double cost = 0.0;
    problem.Evaluate(Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
    return std::sqrt(double(cost/num_observations));
}

double OptimizerG3D::mahalanobisReprojectionError()
{
    double cost = 0.0;
    int step = Pts->rows();
    Problem problem;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    
    num_observations = 0;
    for (register int cam = 0; cam < num_cams; ++cam)
    {
        for (register int ft = 0; ft < num_features ; ++ft)
        {
	  Eigen::Vector3d vv = Variance->col(ft); // Check if variance vector (corresponding to feature ft) is not zero
	  if( (*visibility)(cam,ft) && !(vv.isZero()) )
	  {
	      num_observations++;
	      CostFunction* cost_function = new AutoDiffCostFunction<RE3D_QTS_Cov, 3, 4, 3, 3>( 
		        new RE3D_QTS_Cov( ((*coordinates)(cam,ft).data()), (Variance->data()+step*ft)));
	      problem.AddResidualBlock(cost_function, loss_function, qv_internal[cam].data(), trs->at(cam).data(), (Pts->data()+step*ft) );
	  }
        }
    }
    problem.Evaluate(Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
    return std::sqrt(double(cost/num_observations));
}

double OptimizerG3D::euclideanReprojectionError()
{
    double cost = 0.0;
    int step = Pts->rows();
    Problem problem;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    
    num_observations = 0;
    for (register int cam = 0; cam < num_cams; ++cam)
    {
        for (register int ft = 0; ft < num_features ; ++ft)
        {
	  Eigen::Vector4d vv = (*coordinates)(cam,ft); // Check if variance vector (corresponding to feature ft) is not zero
	  if( (*visibility)(cam,ft) && !(vv.isZero()) )
	  {
	      num_observations++;
	      CostFunction* cost_function = new AutoDiffCostFunction<RE3D_QTS, 3, 4, 3, 3>( 
		        new RE3D_QTS( (*coordinates)(cam,ft).data()));
	      problem.AddResidualBlock(cost_function, loss_function, qv_internal[cam].data(), trs->at(cam).data(), (Pts->data()+step*ft) );
	  }
        }
    }
    problem.Evaluate(Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
    return std::sqrt(double(cost/num_observations));
}

void OptimizerG3D::exportFileReport(std::vector< IterationSummary > &it_summary)
{
    std::ofstream myfile1;
    myfile1.open (filename);
    myfile1 << num_cams << "\t" << num_features << "\t" << num_observations << "\n";
    for (register int i = 0; i < it_summary.size(); ++i)
    {
        myfile1 << std::scientific;
        myfile1 << it_summary.at(i).iteration << "\t" << it_summary.at(i).cost << "\t" << it_summary.at(i).step_is_valid << "\n";
    }
    myfile1.close();
}