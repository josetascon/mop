// Created: Jan/13/2013
// Author: José David Tascón Vidarte

#include "FrameSelector.hpp"

// ================================================================================================
// ============================== FUNCTIONS of CLASS FrameSelector ================================
// ================================================================================================
void FrameSelector::solve()
{
    int matches;
    
//     SiftED feat_handler( filenames );
//     feat_handler.solveSift();
    boost::shared_ptr< std::vector<SiftGPU::SiftKeypoint> > base_key, previous_key, actual_key;
    boost::shared_ptr< std::vector<float> > base_dsc, previous_dsc, actual_dsc;
    std::vector<cv::Point2d> pt1, pt2;
    
    SimpleSiftGPU sift( 400 );
    sift.solveFeatures( filenames[0], *base_key, *base_dsc );
        
    for(int i = 1; i < num_images_in ; ++i) 
    {
        sift.solveFeatures( filenames[i], *actual_key, *actual_dsc );
        matches = sift.solveMatches( *base_key, *actual_key, *base_dsc, *actual_dsc, pt1, pt2 );
        if (matches > min_matches )
        {
	  
// 	  pt1,pt2
        }
	  
    }
    ///WORK HERE
    
    
}