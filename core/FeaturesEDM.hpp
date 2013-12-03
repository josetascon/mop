/**
 * @file FeaturesEDM.hpp
 * @brief This file has all functions related with Features extraction/description/matching
 *
 * @author José David Tascón Vidarte
 * @date Aug/02/2013
 */

#ifndef __FEATURESEDM_HPP__
#define __FEATURESEDM_HPP__

// OpenGl Libraries
#include <GL/glut.h>			// GL constants for SiftGPU function

// OpenCV Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>

// Std Libraries
#include <iostream>

// SiftGPU Library
#include <SiftGPU.h>

// Local Libraries
#include "Common.hpp"
#include "Interface.hpp"
#include "CameraPose.hpp"
#include "DepthProjection.hpp"

// ================================================================================================
// ========================================== CLASS SiftED ========================================
// ================================================================================================
class SiftED
{
private:
//     bool verbose;
    int num_images;
    std::vector< std::string > nameImages;
    
    //CV INFO
    std::vector< cv::Mat > set_of_descriptors;

public:
    std::vector< cv::Mat > set_of_images;
    std::vector< std::vector< cv::KeyPoint > > set_of_keypoints;
    
    std::vector< std::vector<SiftGPU::SiftKeypoint> > keypointsGPU;
    std::vector< std::vector<float> > descriptorsGPU;
    
    //Constructor
    SiftED( std::vector< std::string > filenameIms );
    //Destructor
    ~SiftED();

    void solveSift();

    void loadImages();
    void enableKeyPoint();		//enable CV KeyPoint
    cv::Mat image( int num );
    std::vector<cv::KeyPoint> KeyPoint( int num );

    void convertDesGPUtoCV(); 	// TODO, useless until now
    void convertDataGPUtoCV(); 	// TODO, useless until now
    
//     std::vector< std::vector<float> > get_descriptorsGPU();
//     std::vector< std::vector<SiftGPU::SiftKeypoint> > get_keypointsGPU();
};


// ================================================================================================
// ======================================= struct MatchQuery ======================================
// ================================================================================================
struct MatchQuery
{
    std::vector< cv::DMatch > matches;
    int cam_id1;
    int cam_id2;
};


// ================================================================================================
// ======================================== CLASS MatchesMap ======================================
// ================================================================================================
class MatchesMap
{
private:
    int num_images;
    int num_goodmatch;
    int min_nmatch_reliable;
    int actualfeature;
//     // OPTION 2
//     Eigen::Matrix< std::vector< cv::DMatch> , -1, -1 > globalMatches;
//     Eigen::Matrix< bool, -1, -1 > avalibleMatches;
    
public:
    //OPTION 1
    std::vector< MatchQuery > globalMatch;
    std::vector<bool> reliableMatch;
    
    MatchesMap() { num_goodmatch = 35; min_nmatch_reliable = 20; };
    MatchesMap(int ngood): num_goodmatch(ngood) { min_nmatch_reliable = 20; };
    MatchesMap(int ngood, int nreliable): num_goodmatch(ngood), min_nmatch_reliable(nreliable) { };

    ~MatchesMap() { };
    
    void solveMatches( std::vector< std::vector<float> > *descriptorsGPU );
    void solveMatchesContinuous( std::vector< std::vector<float> > *descriptorsGPU );
    void robustifyMatches( std::vector< std::vector< cv::KeyPoint > > *set_of_keypoints );
    void robustifyMatches(std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints);
    void depthFilter(std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints,
		         std::vector< cv::Mat > *set_of_depth, const int reliable = 20 );
    void depthFilter(std::vector< std::vector< SiftGPU::SiftKeypoint > > *set_of_keypoints,
		         std::vector< std::string > *depth_list, const int reliable = 20 );
    
    void solveDB( HandleDB *mydb, std::vector< std::vector<SiftGPU::SiftKeypoint> > *keypointsGPU );
    void solveDB3D( HandleDB *mydb, std::vector< std::vector<SiftGPU::SiftKeypoint> > *keypointsGPU, 
		 std::vector< std::string > *depth_list, Eigen::Matrix3d &calibration );
    void txt( char *file_txt, std::vector< std::vector<SiftGPU::SiftKeypoint> > *keypointsGPU );
    void plot( std::vector< cv::Mat > *images, std::vector< std::vector< cv::KeyPoint > > *set_of_keypoints );
    
    std::vector<cv::DMatch> match(int nmatch);
};


// ================================================================================================
// ====================================== CLASS SimpleSiftGPU =====================================
// ================================================================================================
class SimpleSiftGPU
{
private:
    int num_matches;
    SiftGPU sift;
    SiftMatchGPU matcher;
    
public:
    std::vector< cv::DMatch > matches;
    
    SimpleSiftGPU(int num_match);
    ~SimpleSiftGPU();
    
    void solveFeatures( cv::Mat &Image, std::vector<SiftGPU::SiftKeypoint> &keypoints, std::vector<float> &descriptors );
    void solveFeatures( cv::Mat &Image, std::vector<cv::KeyPoint> &keypoints, std::vector<float> &descriptors );
    void solveFeatures( std::string nameImage, std::vector<SiftGPU::SiftKeypoint> &keypoints, std::vector<float> &descriptors );
    
    int solveMatches( std::vector<SiftGPU::SiftKeypoint> &keypoints1, std::vector<SiftGPU::SiftKeypoint> &keypoints2,
			   std::vector<float> &descriptors1, std::vector<float> &descriptors2, 
			     std::vector<cv::Point2d> &pt1, std::vector<cv::Point2d> &pt2 );
    
    int solveMatches( std::vector<SiftGPU::SiftKeypoint> &keyGPU1, std::vector<SiftGPU::SiftKeypoint> &keyGPU2,
			   std::vector<float> &descriptors1, std::vector<float> &descriptors2, 
			   std::vector<cv::Point2d> &pt1, std::vector<cv::Point2d> &pt2,
			   std::vector<cv::KeyPoint> &cvkeypoints1, std::vector<cv::KeyPoint> &cvkeypoints2 );
    
    int solveMatches( std::vector<cv::KeyPoint> &cvkeypoints1, std::vector<cv::KeyPoint> &cvkeypoints2,
			   std::vector<float> &descriptors1, std::vector<float> &descriptors2, 
			   std::vector<cv::Point2d> &pt1, std::vector<cv::Point2d> &pt2 );
};

// ================================================================================================
// ======================================= CLASS ImagesFilter =====================================
// ================================================================================================
class ImagesFilter
{
private:
    int num_images_in;
    int num_images_out;
    std::vector< std::string > filenames;
    std::vector< bool > valid;
    
public:
    ImagesFilter( std::vector< std::string > filenames ): filenames(filenames) 
    { num_images_in = filenames.size(); valid.resize(num_images_in, 0); };
    
    void solveImages();
};
    

// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================
/**
 * ******************************************************************
 * @brief Description: Transform a vector set of KeyPoint to Point2d 
 * 
 * @param kps		(input)
 * @param ps		(output)
 * 
 * @date May/10/2013
 */
void keyPointstoPoints(const std::vector< cv::KeyPoint > &kps, std::vector< cv::Point2d > &ps);

/**
 * ******************************************************************
 * @brief Description: Transform a vector set of Point2d to KeyPoint
 * 
 * @param ps 		(input)
 * @param kps 		(output)
 * 
 * @date May/10/2013
 */
void pointstoKeyPoints(const std::vector< cv::Point2d > &ps, std::vector< cv::KeyPoint > &kps);

/**
 * ******************************************************************
 * @brief Description: From a set of keypoints keep just the desired given by matches.
 * The output of points is in Eigen::MatrixXd with 3xn size format.
 * 
 * @param goodmatches	(input)
 * @param keypoints1	(input)
 * @param keypoints2	(input)
 * @param pts1		(output)
 * @param pts2		(output)
 * 
 * @date May/10/2013
 */
void eigenfromMatches(std::vector< cv::DMatch > &matches,
		  std::vector< cv::KeyPoint > &keypoints1, std::vector< cv::KeyPoint > &keypoints2,
		  Eigen::MatrixXd &pts1, Eigen::MatrixXd &pts2);

void eigenfromMatches(std::vector< cv::DMatch > &matches,
		  std::vector< SiftGPU::SiftKeypoint > &keypoints1, std::vector< SiftGPU::SiftKeypoint > &keypoints2,
		  Eigen::MatrixXd &pts1, Eigen::MatrixXd &pts2);

void eigenfromSiftKeypoint(std::vector< SiftGPU::SiftKeypoint > &keys, Eigen::MatrixXf &pts);

// NON EXISTENT
// void keepPointsfromGoodMatches( std::vector< cv::DMatch > &goodmatches, 
// 			  std::vector< cv::KeyPoint > &keypoints1, std::vector< cv::KeyPoint > &keypoints2,
// 			  std::vector< cv::Point2d > &pts1, std::vector< cv::Point2d > &pts2);


void removeRepeatedKeyPoints( std::vector< cv::KeyPoint > &keypoints );
void eliminateRepeatedMatches( std::vector< cv::DMatch > &matches );

/**
 * ******************************************************************
 * @brief Description: Extract SIFT features from images.
 * The image is divided in a grid tryng to bring a stable distribution of features
 * 
 * @param image		(input)
 * @param keypoints		(output)
 * @param num_features 	Maximun number of features allowed in full image. Default value 100.
 * @param grid_rows 	Number of rows in the image grid. Default value 4
 * @param grid_cols 	Number of columns in the image grid. Default value 4
 * 
 * @date May/10/2013
 */
void gridSiftFeatureDetect ( cv::Mat &image, std::vector< cv::KeyPoint > &keypoints, 
		         int num_features = 100, int grid_rows = 4, int grid_cols = 4);

/**
 * ******************************************************************
 * @brief (SET VERSION) Description: Extract features and descriptors from a SET of images. Internal usage of gridSiftFeatureDetect function
 * 
 * @param set_of_images	(input)
 * @param set_of_keypoints	(output)
 * @param set_of_descriptors	(output)
 * @param num_features 	Maximun number of features allowed in full image. Default value 100.
 * @param grid_rows 	Number of rows in the image grid. Default value 4
 * @param grid_cols 	Number of columns in the image grid. Default value 4
 * 
 * @date May/10/2013
 */
void featureDetectandDescript(std::vector< cv::Mat > &set_of_images, 
			std::vector< std::vector< cv::KeyPoint > > &set_of_keypoints,
			std::vector< cv::Mat > &set_of_descriptors,
			int num_features = 100, int grid_rows = 4, int grid_cols = 4 );

/**
 * ******************************************************************
 * @brief (ONE IMAGE VERSION) Description: Extract features and descriptors from one image. Internal usage of gridSiftFeatureDetect function
 * 
 * @param image		(input)
 * @param keypoints		(output)
 * @param descriptors	(output)
 * @param num_features 	Maximun number of features allowed in full image. Default value 100.
 * @param grid_rows 	Number of rows in the image grid. Default value 4
 * @param grid_cols 	Number of columns in the image grid. Default value 4
 * 
 * @date May/10/2013
 */
void featureDetectandDescript(cv::Mat &image, 
			  std::vector< cv::KeyPoint > &keypoints,
			  cv::Mat &descriptors,
			  int num_features = 100, int grid_rows = 4, int grid_cols = 4 );


void extractPointsfromMatches( std::vector< cv::DMatch > &goodmatches, 
			  std::vector< cv::KeyPoint > &keypoints1, std::vector< cv::KeyPoint > &keypoints2,
			  std::vector< cv::Point2d > &pts1, std::vector< cv::Point2d > &pts2);

void extractPointsfromMatches( std::vector< cv::DMatch > &goodmatches, 
			  std::vector< SiftGPU::SiftKeypoint > &keypoints1, std::vector< SiftGPU::SiftKeypoint > &keypoints2,
			  std::vector< cv::Point2d > &pts1, std::vector< cv::Point2d > &pts2);

/**
 * ******************************************************************
 * @brief Description: Extract all distances from matches and bring the range of them (minimun and maximum)
 * 
 * @param matches		(input)
 * @param distances		(output) Vector of double with distances between matches
 * @param min		(output) Minimun distance value in matches
 * @param max		(output) Maximun distance value in matches
 * 
 * @date May/10/2013
 */
void distanceRangeofMatches( std::vector< cv::DMatch > &matches, std::vector< double > &distances, double &min, double &max);

/**
 * ******************************************************************
 * @brief Description: Struct used to sort matches(std::sort)  through their distance parameter
 * 
 * @date May/10/2013
 */
struct SortObject
{
	std::vector<double> value;
	bool operator() (int i,int j) { return (value[i]<value[j]);}
};

/**
 * ******************************************************************
 * @brief Description: Sort distances of matches and returns lesser ones as goodmatches.
 * The desired best number of good matches is defined by num_goodmatch.
 * 
 * @param matches		(input)
 * @param distances		(input)
 * @param num_goodmatch	(input)
 * @param goodmatches	(output)
 * 
 * @date May/10/2013
 */
void goodMatchesfromDistances(std::vector< cv::DMatch > &matches, std::vector< double > &distances, int num_goodmatch,
			std::vector< cv::DMatch > &goodmatches);

void goodMatcheswithEuclidean( std::vector< cv::DMatch > &matches,
			 std::vector< cv::KeyPoint > &keypoints1, std::vector< cv::KeyPoint > &keypoints2,
			 float thold, int num_ecm = 12);

void solveFeatures( cv::Mat &image1, cv::Mat &image2,
		std::vector< cv::KeyPoint > &keypoints1 , std::vector< cv::KeyPoint > &keypoints2,
		cv::Mat &descriptors1, cv::Mat &descriptors2, std::vector< cv::DMatch > &goodmatches,
		int num_features = 200, int num_goodmatch = 35,
		bool debug = false, bool draw = false);

void solveFeaturesEuclidean( cv::Mat &image1, cv::Mat &image2,
		std::vector< cv::KeyPoint > &keypoints1 , std::vector< cv::KeyPoint > &keypoints2,
		cv::Mat &descriptors1, cv::Mat &descriptors2, std::vector< cv::DMatch > &goodmatches,
		int num_features = 200, int num_goodmatch = 35,
		bool debug = false, bool draw = false);

//@date July/22/2013
void solveFeaturesGPU( std::string nameImage1, std::string nameImage2,
		std::vector< cv::KeyPoint > &keypoints1 , std::vector< cv::KeyPoint > &keypoints2,
		std::vector< cv::DMatch > &matches,
		int num_goodmatch = 50,
		bool debug = false, bool draw = false);

void globalContinuousFeatures( std::vector< cv::Mat > &images, std::vector< std::vector< cv::KeyPoint > > &keypoints,
			 std::vector< cv::Mat > &descriptors, std::vector< std::vector< cv::DMatch > > &matches,
		          int num_features = 200, int num_goodmatch = 35, bool debug = false, bool draw = false);

void tagContinuosFeatures( std::vector< std::vector< cv::KeyPoint > > &keypoints, std::vector< std::vector< cv::DMatch > > &goodmatches, 
	        Eigen::Matrix<bool,-1,-1> &visibility, Eigen::Matrix<Eigen::Vector3d,-1,-1> &coordinates,
	        Eigen::MatrixXi &idxKPMatrix, std::vector< Eigen::Matrix<int,2,-1> > &idxKPVector);


#endif