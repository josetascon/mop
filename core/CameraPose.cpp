// Created: May/12/2013
// Author: José David Tascón Vidarte

#include "CameraPose.hpp"

// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================

bool determinantCorrection(Eigen::Matrix3d &Rot)
{
    double det1 = Rot.determinant();
    if( det1 < -0.9999999 && det1 > -1.0000001 )
    {
        // When a rotation matrix is a reflection because its determinant is -1
        Rot.col(2) = -1.0*Rot.col(2); // Corrects the third column sign
        std::cout << "Rotation matrix has det = -1, correct procedure to achieve det = 1\n";
        return true;
    }
    return false;
}

bool checkCoherentRotation(Eigen::Matrix3d &Rot)
{
    double det1 = Rot.determinant();
    if(fabsf( det1 )-1.0 > 1e-07)
    {
        std::cerr << "det(R) != +-1.0, this is not a rotation matrix" << '\n';
        return false;
    }
    return true;
}

Eigen::MatrixXd buildProjectionMatrix( Eigen::Matrix3d &kalibration, Eigen::Matrix3d &Rotation, Eigen::Vector3d &translation)
{
    Eigen::MatrixXd PP(3,4);
    PP << Rotation, translation;
    PP = kalibration*PP;
    return PP;
}

void getFundamentalandRemoveOutliers(std::vector< cv::Point2d > &pts1, std::vector< cv::Point2d > &pts2, std::vector< cv::DMatch > &matches,
					cv::Mat &Fundamental)
{
    std::vector< cv::Point2d > pts1_tmp = pts1;
    std::vector< cv::Point2d > pts2_tmp = pts2;
    std::vector< cv::DMatch > new_matches;
    std::vector<uchar> status(pts1.size());
    
    //Debug
    // 	for(int i=0; i < pts1_tmp.size(); i++)
    // 	{
    // 		std::cout << "Point of image1, " << i << ": " << pts1_tmp[i];
    // 		std::cout << "\t||\tPoint of image2, " << i << ": " << pts2_tmp[i] << '\n';
    // 	}
    // ==================== Fundamental Matrix computation ====================
    double minVal,maxVal;
    cv::minMaxIdx(pts1_tmp,&minVal,&maxVal);
    Fundamental = findFundamentalMat(pts1_tmp, pts2_tmp, cv::FM_RANSAC, 0.006 * maxVal, 0.99, status); //threshold from [Snavely07 4.1]
    
    // ==================== status vector store inliers as indexes of matches ====================
    if ( cv::countNonZero(status) != 0 )
    {
        pts1.clear();
        pts2.clear();
        for (int i=0; i<status.size(); i++) 
        {
	  if (status[i]) 
	  {
	      pts1.push_back(pts1_tmp[i]);
	      pts2.push_back(pts2_tmp[i]);
	      new_matches.push_back(matches[i]);
	  }
        }
        // Debug
//         std::cout << "F keeping " << cv::countNonZero(status) << " / " << status.size() << '\n';
//         std::cout << matches.size() << " matches before, " << new_matches.size() << " new matches after Fundamental Matrix\n";
        matches.clear();
        matches = new_matches; //keep only those points who survived the fundamental matrix
    }
    return;
}

int robustMatchesfromFundamental(std::vector< cv::Point2d > &pts1, std::vector< cv::Point2d > &pts2, std::vector<cv::DMatch> &matches, cv::Mat &Fund, int max_iter)
{
    std::vector< cv::Point2d > pts1_tmp = pts1;
    std::vector< cv::Point2d > pts2_tmp = pts2;
    std::vector< cv::DMatch > new_matches = matches;
    cv::Mat FF;
    int inliers, inliers_before;
    
    for (int i = 0; i < max_iter; i++)
    {
        inliers_before = pts1_tmp.size();
        getFundamentalandRemoveOutliers(pts1_tmp, pts2_tmp, matches, FF);
        inliers = pts1_tmp.size();;
        if (inliers == inliers_before) break;
    }
    Fund = FF;
    return inliers;
}

void poseFundamental( cv::Mat &Fundamental, cv::Mat &kalibration,
		        cv::Mat &Essential, cv::Mat &Rot1, cv::Mat &Rot2, cv::Mat &translation1, cv::Mat &translation2)
{
    const cv::Mat W = (cv::Mat_<double>(3,3) << 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    const cv::Mat WT = (cv::Mat_<double>(3,3) << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    
    cv::SVD singular_vd;
    cv::Mat D, U, VT;
    
    Essential = kalibration.t()*(Fundamental*kalibration);
    Essential = Essential*(1/Essential.at<double>(2,2));
    singular_vd.compute(Essential, D, U, VT);
    
    Rot1 = U*(W*VT);
    Rot2 = U*(WT*VT);
    
    translation1 = U.col(2);
    translation2 = -1.0*U.col(2);
    
    return;
}

void fundamental2essential( Eigen::Matrix3d &Fundamental, Eigen::Matrix3d &kalibration,
		        Eigen::Matrix3d &Essential)
{
    Essential = kalibration.transpose()*(Fundamental*kalibration);
    Essential = Essential/Essential(2,2);
    
    return;
}

void poseEssential( Eigen::Matrix3d &Essential, Eigen::Matrix3d &Rot1, Eigen::Matrix3d &Rot2, 
		    Eigen::Vector3d &ttr1, Eigen::Vector3d &ttr2)
{
    Eigen::Matrix3d W;
    W << 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix3d WT;
    WT << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    //ttr1 = Eigen::Vector3d::Zero(); ttr2 = Eigen::Vector3d::Zero();
    
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Essential, Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::MatrixXd VV = svd.matrixV();
    Eigen::MatrixXd UU = svd.matrixU();
    
    Rot1 = UU*(W*VV.transpose());
    Rot2 = UU*(WT*VV.transpose());
    
    ttr1 = UU.col(2);
    ttr2 = -1.0*UU.col(2);
    
    return;
}

Eigen::MatrixXd linearCamera( Eigen::MatrixXd &xpt, Eigen::MatrixXd &Xpt)
{
    int n = xpt.cols();
    if (n < 6)
    {
        DEBUG_E( ("Camera Matrix can't be estimated in linearCamera(). Minimun required points are 6") ); 
        exit(-1);
    }
    
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2*n,12);
    
    for (register int k = 0; k < n; ++k)
    {
        Eigen::RowVectorXd St(4);
        St << Xpt(0,k), Xpt(1,k), Xpt(2,k), Xpt(3,k);
        A.row(2*k) << Eigen::RowVectorXd::Zero(4), -xpt(2,k)*St, xpt(1,k)*St;
        A.row(2*k+1) << xpt(2,k)*St, Eigen::RowVectorXd::Zero(4), -xpt(0,k)*St;
    }
//     std::cout << "A:\n " << A <<'\n';
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV | Eigen::ComputeThinU);
    Eigen::MatrixXd VV = svd.matrixV();
    Eigen::MatrixXd P(3,4);
    P << VV(0,11), VV(1,11), VV(2,11), VV(3,11), 
        VV(4,11), VV(5,11), VV(6,11), VV(7,11), 
        VV(8,11), VV(9,11), VV(10,11), VV(11,11);
    P = P/P(2,3);
    return P;
}