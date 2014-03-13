// Jul/31/2012
// Author: José David Tascón Vidarte

#include "IncrementalOptimizer.hpp"

// ================================================================================================
// ============================ FUNCTIONS of CLASS IncrementalOptimizer =============================
// ================================================================================================
void IncrementalOptimizer::update(int constant_cam, int num_cams)
{
    for (register int cam = constant_cam; cam < num_cams; ++cam)
    {
        eigen2quaternion(q_vector[cam],quaternion->at(cam));
        quaternion->at(cam).normalize();
    }
}

void IncrementalOptimizer::run(int constant_cam, int num_cams)
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
    
    double tmp[4] = { (*intrinsics)(0,0), (*intrinsics)(1,1), (*intrinsics)(0,2), (*intrinsics)(1,2) }; // [fx,fy,cx,cy]
    std::vector< double > calibration(&tmp[0], &tmp[4]);
    std::vector< double > *intrinsics = &calibration;
    
    for (register int cam = 0; cam < constant_cam; ++cam)
    {
        for (register int ft = 0; ft < num_features ; ++ft)
        {
	  if( (*visibility)(cam,ft) == true && (*structure)(3,ft) )
	  {
	      CostFunction* cost_function0 = new AutoDiffCostFunction<RE_constKDQT_S, 2, 3>( 
	      new RE_constKDQT_S( (*coordinates)(cam,ft)(0), (*coordinates)(cam,ft)(1), intrinsics->data(), distortion->data(), 
			      q_vector[cam].data(), translation->at(cam).data() ));
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
	      problem.AddResidualBlock(cost_function, loss_function, q_vector[cam].data(), translation->at(cam).data(), (structure->data()+step*ft) );
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