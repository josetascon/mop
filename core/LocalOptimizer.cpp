// Jul/31/2012
// Author: José David Tascón Vidarte

#include "LocalOptimizer.hpp"

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

void LocalOptimizer::poseCamera()
{
    // HERE observation1 are image points [3 x n] and observation2 3d points [4 x n]
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
    int step = observation2->rows();
    int num_features = observation1->cols();
    
    for (register int ft = 0; ft < num_features ; ++ft)
    {
        CostFunction* cost_function2 = new AutoDiffCostFunction<RE_Camera, 3, 4, 3>( 
		new RE_Camera( (*observation1)(0,ft), (*observation1)(1,ft), intrinsics->data(), (observation2->data()+step*ft)  ));
        problem.AddResidualBlock(cost_function2, loss_function, quaternionV.data(), translation->data() );
    }
    
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    DEBUG_2( std::cout << summary.BriefReport() << "\n"; )

    double cost = 0;
    problem.Evaluate(Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
    DEBUG_2( std::cout << "RMS Reprojection Error is : " << std::sqrt(double(cost/num_features)) << "\n\n"; )
    update();
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
    

// ============================================================================
// ================================= AUXILIAR =================================
// 		Use to create .... in your code
// ============================================================================

// /**
//  * ******************************************************************
//  * @functions ....
//  */