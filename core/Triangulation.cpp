// Created: Jun/28/2013
// Author: José David Tascón Vidarte

#include "Triangulation.hpp"

// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================

Eigen::Vector4d linearTriangulation( Eigen::Vector3d &x1, Eigen::Vector3d &x2, Eigen::MatrixXd &P1, Eigen::MatrixXd &P2 )
{
    // Triangulation method needs that the last coordinate of x1 and x2 is equal to one (1)
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4,4);
    A.row(0) = x1(0)*P1.row(2) - P1.row(0);
    A.row(1) = x1(1)*P1.row(2) - P1.row(1);
    A.row(2) = x2(0)*P2.row(2) - P2.row(0);
    A.row(3) = x2(1)*P2.row(2) - P2.row(1);
    
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeFullV);
    Eigen::MatrixXd VV = svd.matrixV();
    
    // Debug:
    //std::cout << "vector x1 = " << x1(0) << '\n';
    //std::cout << "Matrix A = " << A << '\n';
    //std::cout << "Matrix V = " << VV << '\n';
    return VV.col(3)/VV.col(3)(3);
}

//MULTIPLE CAMERA TRIANGULATION
Eigen::Vector4d linearTriangulation( std::vector<Eigen::Vector3d> &xdata, std::vector<Eigen::MatrixXd> &P )
{
    int num_elements = P.size();
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_elements*2, 4 );
    
    for (int k = 0; k < num_elements; k++)
    {
        A.row(2*k) = xdata[k](0)*P[k].row(2) - P[k].row(0);
        A.row(2*k+1) = xdata[k](1)*P[k].row(2) - P[k].row(1);
        // EXTEND ROWS WITH OTHER CAMERA MATRICES TO TRIANGULATE ADITIONAL VIEWS of POINTs in ONE RESULT
    }
    
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeFullV);
    Eigen::MatrixXd VV = svd.matrixV();
    
    // Debug:
    //std::cout << "vector x1 = " << x1(0) << '\n';
    //std::cout << "Matrix A = " << A << '\n';
    //std::cout << "Matrix V = " << VV << '\n';
    return VV.col(3)/VV.col(3)(3);
}

Eigen::MatrixXd linearTriangulationSet( Eigen::MatrixXd &x1, Eigen::MatrixXd &x2, Eigen::MatrixXd &P1, Eigen::MatrixXd &P2 )
{
    int n = x1.cols();
    Eigen::MatrixXd Xdata(4,n);
    
    for (int i = 0; i < n; i++)
    {
        Eigen::Vector3d xx1 = x1.col(i);
        Eigen::Vector3d xx2 = x2.col(i);
        Xdata.col(i) = linearTriangulation( xx1, xx2, P1, P2 );
    }
    //Xdata.transposeInPlace();
    
    return Xdata; 
}

Eigen::MatrixXd normalizeTMatrix( Eigen::MatrixXd &data)
{
    int n = data.cols();
    double x_mean = data.row(0).mean();
    double y_mean = data.row(1).mean();
    double scale_norm = 1.0;
    Eigen::MatrixXd Xd_norm;
    Eigen::Matrix3d Ttrans, Tscale;
    Eigen::VectorXd v_norms(n);
    
    Ttrans << 1, 0, -x_mean, 0, 1, -y_mean, 0, 0, 1;
    Xd_norm = Ttrans*data;
    Xd_norm.row(2) = Eigen::RowVectorXd::Zero(n);
    
    //timer_wall timer1;
    //timer1.start_time_clock();
    //v_norms = Xd_norm.colwise().stableNorm(); // Optional 
    for (int i = 0; i < n; i++)
    {
        v_norms(i) = Xd_norm.col(i).stableNorm();
    }
    //std::cout << "Norm of each column time = " << timer1.elapsed_time_clock_us() << " microseconds\n";
    scale_norm = sqrt(2)/v_norms.mean();
    Tscale << scale_norm, 0, 0, 0, scale_norm, 0, 0, 0, 1;
    
    // Debug:
    //std::cout << "Ttrans:\n" << Ttrans << '\n';
    //std::cout << "Xd_norm:\n" << Xd_norm << '\n';
    //std::cout << "v_norms:\n" << v_norms << '\n';
    return Tscale*Ttrans;
}

Eigen::MatrixXd linearTriangulationNormalized( Eigen::MatrixXd &x1, Eigen::MatrixXd &x2, Eigen::MatrixXd &P1, Eigen::MatrixXd &P2 )
{
    int n = x1.cols();
    Eigen::MatrixXd Xdata(4,n);
    
    Eigen::Matrix3d Tn1 = normalizeTMatrix( x1 );
    Eigen::Matrix3d Tn2 = normalizeTMatrix( x2 );
    
    Eigen::MatrixXd x1_norm = Tn1*x1;
    Eigen::MatrixXd x2_norm = Tn2*x2;
    
    Eigen::MatrixXd P1_ts = Tn1*P1;
    Eigen::MatrixXd P2_ts = Tn2*P2;
    
    for (int i = 0; i < n; i++)
    {
        Eigen::Vector3d xx1 = x1_norm.col(i);
        Eigen::Vector3d xx2 = x2_norm.col(i);
        Xdata.col(i) = linearTriangulation( xx1, xx2, P1_ts, P2_ts );
    }
    //Xdata.transposeInPlace();
    
    return Xdata; 
}