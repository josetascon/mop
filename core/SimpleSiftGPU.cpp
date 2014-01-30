// Created: Aug/02/2013
// File: Jan/13/2014
// Author: José David Tascón Vidarte

#include "SimpleSiftGPU.hpp"

// ================================================================================================
// ============================= FUNCTIONS of CLASS SimpleSiftGPU =================================
// ================================================================================================
SimpleSiftGPU::SimpleSiftGPU(int num_match): matcher(num_match), num_matches(num_match)
{
    //processing parameters first
//     char * s_argv[] = {(char*)"-fo", (char*)"-1", (char*)"-v", (char*)"1"};
//     sift.ParseParam(4, s_argv);
    
    char * s_argv[] = {(char*)"-fo", (char*)"-1", (char*)"-v", (char*)"0", (char*)"-tc2", (char*)"1000" }; // tc2 limit max features
    // Change vebose level with -v 1
    sift.ParseParam(6, s_argv);
    int support = sift.CreateContextGL();
    if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED)
    {
        fprintf(stderr, "Can't open OpenGL context for SiftGPU\n");
        return;
    }
};

SimpleSiftGPU::~SimpleSiftGPU() { };

void SimpleSiftGPU::solveFeatures( cv::Mat &Image, std::vector<SiftGPU::SiftKeypoint> &keypoints, std::vector<float> &descriptors )
{
    unsigned char *p_rgb = Image.data;
    sift.RunSIFT(Image.size().width, Image.size().height, p_rgb, GL_RGB, GL_UNSIGNED_BYTE);
//     sift.RunSIFT(nameImage.c_str());
    int num = sift.GetFeatureNum();//get feature count
    descriptors.resize(128*num);
    keypoints.resize(num);
    sift.GetFeatureVector(&keypoints[0], &descriptors[0]);
}

void SimpleSiftGPU::solveFeatures( cv::Mat &Image, std::vector<cv::KeyPoint> &keypoints, std::vector<float> &descriptors )
{
    std::vector<SiftGPU::SiftKeypoint> keyGPU;
    SimpleSiftGPU::solveFeatures( Image, keyGPU, descriptors );
    keypoints.clear();
    for (register int i = 0; i < keyGPU.size(); ++i) keypoints.push_back( cv::KeyPoint( cv::Point2f(keyGPU[i].x, keyGPU[i].y) , keyGPU[i].s, keyGPU[i].o ) );
}

void SimpleSiftGPU::solveFeatures( std::string nameImage, std::vector<SiftGPU::SiftKeypoint> &keypoints, std::vector<float> &descriptors )
{
    sift.RunSIFT(nameImage.c_str());
    int num = sift.GetFeatureNum();//get feature count
    descriptors.resize(128*num);
    keypoints.resize(num);
    sift.GetFeatureVector(&keypoints[0], &descriptors[0]);
}

int SimpleSiftGPU::solveMatches( std::vector<SiftGPU::SiftKeypoint> &keypoints1, std::vector<SiftGPU::SiftKeypoint> &keypoints2,
			   std::vector<float> &descriptors1, std::vector<float> &descriptors2, 
			     std::vector<cv::Point2d> &pt1, std::vector<cv::Point2d> &pt2 )
{
    if(matcher.VerifyContextGL() == 0)
    {
        fprintf(stderr, "Can't open OpenGL context for Matcher in SiftGPU\n");
        return -1;
    }
    //Set two sets of descriptor data to the matcher
    matcher.SetDescriptors(0, descriptors1.size(), &descriptors1[0]);
    matcher.SetDescriptors(1, descriptors2.size(), &descriptors2[0]);
    //Match and read back result to input buffer
    int match_buf[num_matches][2];
    int nmatch = matcher.GetSiftMatch(num_matches, match_buf);
    
    pt1.clear();
    pt2.clear();
    matches.clear();
    for (register int i = 0; i < nmatch ; ++i)
    {
        pt1.push_back(cv::Point2d(keypoints1[match_buf[i][0]].x,keypoints1[match_buf[i][0]].y));
        pt2.push_back(cv::Point2d(keypoints2[match_buf[i][1]].x,keypoints2[match_buf[i][1]].y));
        matches.push_back(cv::DMatch( match_buf[i][0], match_buf[i][1], i ));
    }
    return nmatch;
}

int SimpleSiftGPU::solveMatches( std::vector<SiftGPU::SiftKeypoint> &keyGPU1, std::vector<SiftGPU::SiftKeypoint> &keyGPU2,
			   std::vector<float> &descriptors1, std::vector<float> &descriptors2, 
			   std::vector<cv::Point2d> &pt1, std::vector<cv::Point2d> &pt2,
			   std::vector<cv::KeyPoint> &cvkeypoints1, std::vector<cv::KeyPoint> &cvkeypoints2 )
{
    if(matcher.VerifyContextGL() == 0)
    {
        fprintf(stderr, "Can't open OpenGL context for Matcher in SiftGPU\n");
        return -1;
    }
    //Set two sets of descriptor data to the matcher
    matcher.SetDescriptors(0, descriptors1.size(), &descriptors1[0]);
    matcher.SetDescriptors(1, descriptors2.size(), &descriptors2[0]);
    //Match and read back result to input buffer
    int match_buf[num_matches][2];
    int nmatch = matcher.GetSiftMatch(num_matches, match_buf);
    
    pt1.clear();
    pt2.clear();
    cvkeypoints1.clear();
    cvkeypoints2.clear();
    matches.clear();
    for (register int i = 0; i < nmatch ; ++i) // Only valid keypoints are returned, Matches have features id in order
    {
        cvkeypoints1.push_back( cv::KeyPoint(cv::Point2f(keyGPU1[match_buf[i][0]].x,keyGPU1[match_buf[i][0]].y),
			  keyGPU1[match_buf[i][0]].s, keyGPU1[match_buf[i][0]].o ));
        cvkeypoints2.push_back( cv::KeyPoint(cv::Point2f(keyGPU2[match_buf[i][1]].x,keyGPU2[match_buf[i][1]].y),
			  keyGPU2[match_buf[i][1]].s, keyGPU2[match_buf[i][1]].o ));
        pt1.push_back(cv::Point2d(keyGPU1[match_buf[i][0]].x,keyGPU1[match_buf[i][0]].y));
        pt2.push_back(cv::Point2d(keyGPU2[match_buf[i][1]].x,keyGPU2[match_buf[i][1]].y));
        matches.push_back(cv::DMatch( i, i, i ));
    }
    return nmatch;
}

int SimpleSiftGPU::solveMatches( std::vector<cv::KeyPoint> &cvkeypoints1, std::vector<cv::KeyPoint> &cvkeypoints2,
			   std::vector<float> &descriptors1, std::vector<float> &descriptors2, 
			   std::vector<cv::Point2d> &pt1, std::vector<cv::Point2d> &pt2 )
{
    if(matcher.VerifyContextGL() == 0)
    {
        fprintf(stderr, "Can't open OpenGL context for Matcher in SiftGPU\n");
        return -1;
    }
    //Set two sets of descriptor data to the matcher
    matcher.SetDescriptors(0, descriptors1.size(), &descriptors1[0]);
    matcher.SetDescriptors(1, descriptors2.size(), &descriptors2[0]);
    //Match and read back result to input buffer
    int match_buf[num_matches][2];
    int nmatch = matcher.GetSiftMatch(num_matches, match_buf);
    
    pt1.clear();
    pt2.clear();
    matches.clear();
    for (register int i = 0; i < nmatch ; ++i) // Use cvkeypoints to get points as std::vector<cv::Point2d>
    {
        pt1.push_back(cv::Point2d(cvkeypoints1[match_buf[i][0]].pt.x,cvkeypoints1[match_buf[i][0]].pt.y));
        pt2.push_back(cv::Point2d(cvkeypoints2[match_buf[i][1]].pt.x,cvkeypoints2[match_buf[i][1]].pt.y));
        matches.push_back(cv::DMatch( match_buf[i][0], match_buf[i][1], i ));			// matches saved with original feature id
    }
    return nmatch;
}

// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================
void solveFeaturesGPU( std::string nameImage1, std::string nameImage2,
		std::vector< cv::KeyPoint > &keypoints1 , std::vector< cv::KeyPoint > &keypoints2,
		std::vector< cv::DMatch > &matches,
		int num_goodmatch,
		bool debug, bool draw)
{
    //create a SiftGPU instance
    SiftGPU sift;
    //processing parameters first
    char * s_argv[] = {(char*)"-fo", (char*)"-1", (char*)"-v", (char*)"1"};
    //-fo -1, starting from -1 octave //-v 1,
    //only print out # feature and overall time
    sift.ParseParam(4, s_argv);
    
    //Initialize GLUT
//     glutInit(&argc, argv);
//     glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
//     glutInitWindowSize(640, 480);
//         //Create the window
//     glutCreateWindow("Image");
    
    //create an OpenGL context for computation
    int support = sift.CreateContextGL();
    //call VerfifyContexGL instead if using your own GL context
    //int support = sift.VerifyContextGL();
    if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED)
    {
        fprintf(stderr, "Can't open OpenGL context for SiftGPU\n");
        return;
    }
    
    sift.RunSIFT(nameImage1.c_str()); //process an image
    
    int num1 = sift.GetFeatureNum();//get feature count
    //allocate memory for readback
    std::vector<float> descriptors1(128*num1);
    std::vector<SiftGPU::SiftKeypoint> keys1(num1);
    //read back keypoints and normalized descritpros
    sift.GetFeatureVector(&keys1[0], &descriptors1[0]); //specify NULL if you don’t need keypoints or descriptors
    
    sift.RunSIFT(nameImage2.c_str());
    int num2 = sift.GetFeatureNum();//get feature count
    std::vector<float> descriptors2(128*num2);
    std::vector<SiftGPU::SiftKeypoint> keys2(num2);
    sift.GetFeatureVector(&keys2[0], &descriptors2[0]);
    
    if (debug)
    {
        std::cout << "Number of keypoints Image 1: " << keys1.size() << '\n';
        std::cout << "Number of keypoints Image 2: " << keys2.size() << '\n';
    }
    
    //specify the naximum number of features to match
    SiftMatchGPU matcher(num_goodmatch);
    //You can call SetMaxSift anytime to change this limit
    //You can call SetLanguage to select shader language
    //between GLSL/CUDA before initialization
    //Verify current OpenGL Context and do initialization
    if(matcher.VerifyContextGL() == 0)
    {
        fprintf(stderr, "Can't open OpenGL context for Matcher in SiftGPU\n");
        return;
    }
    //Set two sets of descriptor data to the matcher
    matcher.SetDescriptors(0, num1, &descriptors1[0]);
    matcher.SetDescriptors(1, num2, &descriptors2[0]);
    //Match and read back result to input buffer
    int match_buf[num_goodmatch][2];
    int nmatch = matcher.GetSiftMatch(num_goodmatch, match_buf);
    // You can also use homography and/or fundamental matrix to
    // guide the putative matching
    // Check function SiftMatchGPU::GetGuidedMatch
    // For more details of the above functions, check
    // SimpleSIFT.cpp and SiftGPU.h in the code package.
    
    keypoints1.clear();
    keypoints2.clear();
    matches.clear();
    
    if (debug)
    {
        for (register int i = 0; i < nmatch ; ++i)
        {
	  printf("Point of image1, id#%i : (%f,%f)" , match_buf[i][0], keys1[match_buf[i][0]].x, keys1[match_buf[i][0]].y);
	  printf("\t||\tPoint of image2, id#%i : (%f,%f)\n" , match_buf[i][1], keys2[match_buf[i][1]].x, keys2[match_buf[i][1]].y);
        }
    }
    
    for (register int i = 0; i < nmatch ; ++i)
    {
        keypoints1.push_back(cv::KeyPoint(cv::Point2f(keys1[match_buf[i][0]].x,keys1[match_buf[i][0]].y),
			  keys1[match_buf[i][0]].s, keys1[match_buf[i][0]].o ));
        keypoints2.push_back(cv::KeyPoint(cv::Point2f(keys2[match_buf[i][1]].x,keys2[match_buf[i][1]].y),
			  keys2[match_buf[i][1]].s, keys2[match_buf[i][1]].o ));
        matches.push_back(cv::DMatch( match_buf[i][0], match_buf[i][1], i ));
    }
    
    if (draw)
    {
        cv::Mat image1 = cv::imread(nameImage1);
        cv::Mat image2 = cv::imread(nameImage2);
        cv::Mat image_key1, image_key2;
    
        drawKeypoints(image1, keypoints1, image_key1, cv::Scalar(0,0,255));//, 4);
        drawKeypoints(image2, keypoints2, image_key2, cv::Scalar(0,0,255));//, 4);
    
        imshow( "im1", image_key1 );
        imshow( "im2", image_key2 );
        
        cvWaitKey(0);
        
        cv::Mat draw_match;
        drawMatches( image1, keypoints1, image2, keypoints2,
		matches, draw_match, cv::Scalar::all(-1), cv::Scalar::all(-1),
		std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        imshow( "MATCHES", draw_match );
        cvWaitKey(0);
    }
    
    return;
}