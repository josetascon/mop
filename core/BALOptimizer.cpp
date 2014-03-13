// Jul/31/2012
// Author: José David Tascón Vidarte

#include "BALOptimizer.hpp"

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
        eigen2quaternion(q_vector[cam],quaternion->at(cam));
        quaternion->at(cam).normalize();
    }
}






    
    

// ============================================================================
// ================================= AUXILIAR =================================
// 		Use to create .... in your code
// ============================================================================

// /**
//  * ******************************************************************
//  * @functions ....
//  */