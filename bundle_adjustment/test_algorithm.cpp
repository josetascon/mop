// Created: May/15/2013
// Author: José David Tascón Vidarte

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// // OpenCV Libraries
#include <opencv2/opencv.hpp>

// Std Libraries
#include <iostream>
#include <string>
#include <cmath>

// Local libraries
#include "CameraPose.hpp"
#include "Common.hpp"
#include "Interface.hpp"

#define pi 3.14159265358979323846

using namespace cv;
using namespace std;
using namespace Eigen;

// ========================================== MAIN ==========================================
int main(int argc, char* argv[])
{
     
    double cx = 328.59418282611989; // optical center x
    double cy = 265.23529733631955; // optical center y
    double fx = 520.28075824744883; // focal length x
    double fy = 517.35099060486289; // focal length y
    
    Eigen::Matrix3d Keigen;
    Keigen << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;		// Camera Matrix (intrinsics)
    
    Eigen::Matrix3d F1;
    F1 << -1.5634e-05,  -1.6716e-04,   2.5756e-02, 1.6651e-04,  -8.8477e-06, -9.0707e-02, -1.8446e-02, 8.9162e-02, 1.0000e+00;
    
    Matrix3d Essential, Rot1, Rot2;
    Vector3d tr1, tr2;
    
    // test quaternion allocator
    Quaternion<double> q1(0.1,0.3,0.2,1.3);
    Quaternion<double> q2(0.6,0.5,0.4,0.7);
    Quaternion<double> q3(0.73,0.8122,0.33,0.111);
    
    Vector4d qmmm = q2.coeffs();
    double a[4];
    std::cout << qmmm;

    vector<Quaternion<double> > Qin(3);
    vector< vector<double> > Q_vec;
    Qin[0] = q1; Qin[1] = q2; Qin[2] = q3;
    
    convertSetQuaterniontoVector( Qin, Q_vec );
    convertSetVectortoQuaternion( Q_vec, Qin);
    
    
    
    cout << "\n\n++++++++++++++++++++++++++++++++++++++++++++\n";
    cout << "++++   Rotation and Translation ++++\n";
    cout << "++++++++++++++++++++++++++++++++++++++++++++\n";
    
    essentialfromFundamental(F1, Keigen, Essential);
    posefromEssential( Essential, Rot1, Rot2, tr1, tr2 );
    
    cout << "Rot1:\n " << Rot1 <<'\n';
    cout << "Rot2:\n " << Rot2 <<'\n';
    cout << "tr1:\n " << tr1.transpose() <<'\n';
    cout << "tr2:\n " << tr2.transpose() <<'\n';
    cout << "pseudo inverse:\n" << pseudoInverse(tr1) <<"\n";
    cout << "Camera Rot2 & t1:\n" << buildProjectionMatrix( Keigen, Rot2, tr1) << '\n';
    
    for (int i = 0; i < 3; i++)
    {
        std::cout << "Equivalent quaternion Cam " << i << ":\n" << Qin[i].w() << " " << Qin[i].vec().transpose() << '\n';
        printf("%f , %f , %f , %f \n", Q_vec[i][0], Q_vec[i][1], Q_vec[i][2], Q_vec[i][3]);
    }
    
    cout << "\n\n++++++++++++++++++++++++++++++++++++++++++++\n";
    cout << "++++   Testing pointer to data in Eigen and vector ++++\n";
    cout << "++++++++++++++++++++++++++++++++++++++++++++\n";
    
    std::vector< std::vector<int> > vector01(2);
    vector01[0].resize(2);
    vector01[1].resize(2);
    vector01[0][0] = 1;
    vector01[0][1] = 2;
    vector01[1][0] = 3;
    vector01[1][1] = 4;
    
    for (int i = 0; i < 4; i++)
    {
        int *p = vector01[0].data();
        printf( "Value pointer std::vector< std::vector<int> > #%i\n", p[i]); // POINTER TO vector<vector> NO continuos data.
    }
    
    Eigen::MatrixXi M1(2,2);
    M1(0) = 1;
    M1(1) = 2;
    M1(2) = 3;
    M1(3) = 4;
    std::cout << "Matrix:\n" << M1 << "\n";
    for (int i = 0; i < 4; i++)
    {
        int *p = M1.data();
        printf( "Value pointerin Eigen Matrix #%i\n", p[i]); // POINTER TO vector<vector> NO continuos data.
    }
    
//     cout << "\n\n++++++++++++++++++++++++++++++++++++++++++++\n";
//     cout << "++++   Reading Eigen Matrix from File ++++\n";
//     cout << "++++++++++++++++++++++++++++++++++++++++++++\n";
//     
//     Eigen::MatrixXd B;
// //     Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> B;
//     readTextFileEigen("./data.txt", B);
//     std::cout << "Data:\n" << B << "\n";
//     
//     writeTextFileEigen("output01", B);
//     
//     // Test template convertPoint2_toEigen
//     std::vector< cv::Point_<float>, std::allocator<cv::Point_<float> > > aa(2);
// //     std::vector<cv::Point2f> aa(2);
//     aa[0].x = 1.0, aa[1].x = 3.0;
//     
// //     Eigen::MatrixXd AA;
//     Eigen::Matrix<double,-1, -1, 0, -1, -1> AA;
//     
//     convertPoint2_toEigen<float,double>(aa,AA);
//     
//     std::cout << "Matrix:\n" << AA << "\n";
    
    
    
    
    
    
        // ========================================== Test FeaturesEDM ==========================================
    /*
    SiftED myfeat(imageList_rgb);
    myfeat.solveSift();
    myfeat.loadImages();
    myfeat.enableKeyPoint();
    
    cv::Mat image_key1, image_key2;
    
    drawKeypoints(myfeat.image(0), myfeat.KeyPoint(0), image_key1, Scalar(0,0,255));//, 4);
    drawKeypoints(myfeat.image(2), myfeat.KeyPoint(2), image_key2, Scalar(0,0,255));//, 4);
    
    imshow( "im1", image_key1 );
    imshow( "im2", image_key2 );
    
//     std::vector< std::vector<float> > desc = myfeat.get_descriptorsGPU();
//     std::vector< std::vector<SiftGPU::SiftKeypoint> > keys = myfeat.get_keypointsGPU();
    
    MatchesMap my_mmap(25,18);
    my_mmap.solveMatches(&myfeat.descriptorsGPU);
    
    std::vector<cv::DMatch> match00 = my_mmap.match(8);
    
    std::cout << "TOTAL MATCHES = " << match00.size() << '\n';
    drawMatches( myfeat.image(0), myfeat.KeyPoint(0), myfeat.image(2), myfeat.KeyPoint(2),
		match00, draw_match, Scalar::all(-1), Scalar::all(-1),
		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	  imshow( "MATCHES X", draw_match );
    
    cvWaitKey(0);
    */
    // ========================================== END Test FeaturesEDM ==========================================
    
    
    
    // ========================================== Test Database ==========================================
    /*
    string nameDB = "features01.db";
    HandleDB mydb( (char*)nameDB.c_str() );
    
    if (!mydb.isOpen()) exit(0);
    
//     mydb.insertUnique( 0, 0, 25, 12.2, 11.1);
//     mydb.insertUnique( 0, 1, 3, 52.5, 81.8);
//     mydb.insertUnique( 1, 1, 13, 43.4, 71.7);
//     mydb.insertUnique( 1, 2, 43, 134.4, 222.7);
//     mydb.insertUnique( 2, 2, 14, 77.8, 2.9);
    
    int featureA;
    bool find=false;
    find = mydb.searchFeature( 0, 25, &featureA);
    if (find) std::cout << "Correct search, feature # = "<< featureA << "\n";
    else std::cout << "Bad search\n";
    std::cout << "Max feature value = " << mydb.maxFeature() << '\n';
    
    std::vector<int> feature;
    mydb.searchFeature(2, &feature);
    std::cout << "Feature vector size = " << feature.size() << '\n';
    for (int i=0; i < feature.size(); i++) printf("feature = %i\n",feature[i]);
    
    std::vector<int> camera;
    mydb.searchCamera(0, &camera );
    for (int i=0; i < camera.size(); i++) printf("camera = %i\n",camera[i]);
//     void searchRowbyCamera(int camera, std::vector< rowDB > *row_vector);
    
    std::vector< rowDB > row_vector;
    mydb.searchRowbyCamera(2, &row_vector);
    for (int i=0; i < row_vector.size(); i++) printf("idx = %i\n",row_vector[i].idx);
    
    mydb.closeDB();
    */
    // ========================================== END Test Database ==========================================
    
    cout << "\n\n++++++++++++++++++++++++++++++++++++++++++++\n";
    cout << "++++   Test linear Camera ++++\n";
    cout << "++++++++++++++++++++++++++++++++++++++++++++\n";
    Eigen::Matrix3d K;
    K << 500.0, 0.0 , 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0;
    
    Matrix3d RR;
    RR = AngleAxisd(10.0*pi/180, Vector3d::UnitX()) * AngleAxisd(20.0*pi/180, Vector3d::UnitY()) * AngleAxisd(-5.0*pi/180, Vector3d::UnitZ());
    Vector3d tt(2.0, -0.5, 1.0);
    
    MatrixXd Psyn = buildProjectionMatrix(K, RR, tt);
    
    MatrixXd St_inh = Eigen::MatrixXd::Random(3,10);
    MatrixXd St;
    convertHomogeneous(St_inh, St);
    MatrixXd xpt = Psyn*St;
//     cout << "St:\n " << St.transpose() <<'\n';
//     cout << "xpt:\n " << xpt.transpose() <<'\n';
    MatrixXd Prcv = linearCamera( xpt, St);
    
    std::cout << "Camera Matrix Original:\n" << Psyn << "\n";
    std::cout << "Camera Matrix Recover:\n" << Prcv << "\n";
    
    
    
    
    cout << "\n\n++++++++++++++++++++++++++++++++++++++++++++\n";
    cout << "++++   Reading BAL from File ++++\n";
    cout << "++++++++++++++++++++++++++++++++++++++++++++\n";
    
    string file1 = "/home/jose/Desktop/Workspace/R3D/Ceres/basics/build/problem-49-7776-pre.txt";
    BALProblem bal(file1.c_str());
    Eigen::Matrix<bool,Dynamic,Dynamic> visibility;
    Eigen::Matrix<Eigen::Vector3d,Dynamic,Dynamic> coordinates;
    std::vector< Eigen::Quaternion<double> > quaternion;
    Eigen::MatrixXd tr_and_int;
    Eigen::MatrixXd structure;
    
    bal.read(visibility,coordinates,quaternion,tr_and_int,structure); 
    
    
//     tr_and_int.transposeInPlace();
//     writeTextFileEigen("output01", tr_and_int);
//     std::cout << "Translation and Intrinsics saved to output01.txt\n";
    structure.transposeInPlace();
    writeTextFileEigen("output01", structure);
    std::cout << "Structure saved to output01.txt\n";
    
    
    exit(0);
}