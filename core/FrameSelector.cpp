// Created: Jan/13/2013
// Author: José David Tascón Vidarte

#include "FrameSelector.hpp"

// ================================================================================================
// ============================== FUNCTIONS of CLASS FrameSelector ================================
// ================================================================================================
void FrameSelector::solve()
{
    int matches;
    int base_frame = 0;
    bool next_frame = false;
    
//     SiftED feat_handler( filenames );
//     feat_handler.solveSift();
    
    boost::shared_ptr< std::vector<SiftGPU::SiftKeypoint> > base_key (new std::vector<SiftGPU::SiftKeypoint>);
    boost::shared_ptr< std::vector<SiftGPU::SiftKeypoint> > actual_key (new std::vector<SiftGPU::SiftKeypoint>);
    boost::shared_ptr< std::vector<SiftGPU::SiftKeypoint> > previous_key (new std::vector<SiftGPU::SiftKeypoint>);
    
    boost::shared_ptr< std::vector<float> > base_dsc (new std::vector<float>);
    boost::shared_ptr< std::vector<float> > actual_dsc (new std::vector<float>);
    boost::shared_ptr< std::vector<float> > previous_dsc (new std::vector<float>);
    std::vector<cv::Point2d> pt1, pt2;
    
    SimpleSiftGPU sift( 400 );
    sift.solveFeatures( filenames[0], *base_key, *base_dsc );
//     std::cout << "Find features in " << filenames[0] << "\n";
    valid[0] = true;
    num_images_out++;
    
    for(int i = 1; i < num_images_in ; ++i) 
    {
        matches = 0;
        sift.solveFeatures( filenames[i], *actual_key, *actual_dsc );
        matches = sift.solveMatches( *base_key, *actual_key, *base_dsc, *actual_dsc, pt1, pt2 );
        
        printf("MATCH: %04i -> %04i:\t#matches = %i\n", base_frame, i, matches);
        
        if ( !next_frame && (matches < min_matches + window ))
        {
	  base_frame = i;
	  base_key.swap(actual_key);
	  base_dsc.swap(actual_dsc);
	  actual_key->clear();
	  actual_dsc->clear();
	  valid[base_frame] = true;
	  continue;
        }
        
        if (matches > min_matches + window )
        {
	  next_frame = true; // mark when pass through adjacent frame
	  previous_key.swap(actual_key);
	  previous_dsc.swap(actual_dsc);
	  actual_key->clear();
	  actual_dsc->clear();
        }
        else
        {
	  i--;
	  base_frame = i;
	  base_key.swap(previous_key);
	  base_dsc.swap(previous_dsc);
	  previous_key->clear();
	  previous_dsc->clear();
	  
	  valid[base_frame] = true;
	  num_images_out++;
	  next_frame = false;
        }
    }
    // Debug
    printVector( valid );
}

void FrameSelector::solvewithDepth( std::vector< std::string > &depth_files )
{
    int matches;
    int base_frame = 0;
    bool next_frame = false;
    
//     SiftED feat_handler( filenames );
//     feat_handler.solveSift();
    
    boost::shared_ptr< std::vector<SiftGPU::SiftKeypoint> > base_key (new std::vector<SiftGPU::SiftKeypoint>);
    boost::shared_ptr< std::vector<SiftGPU::SiftKeypoint> > actual_key (new std::vector<SiftGPU::SiftKeypoint>);
    boost::shared_ptr< std::vector<SiftGPU::SiftKeypoint> > previous_key (new std::vector<SiftGPU::SiftKeypoint>);
    
    boost::shared_ptr< std::vector<float> > base_dsc (new std::vector<float>);
    boost::shared_ptr< std::vector<float> > actual_dsc (new std::vector<float>);
    boost::shared_ptr< std::vector<float> > previous_dsc (new std::vector<float>);
    std::vector<cv::Point2d> pt1, pt2;
    
    SimpleSiftGPU sift( 400 );
    sift.solveFeatures( filenames[0], *base_key, *base_dsc );
//     std::cout << "Find features in " << filenames[0] << "\n";
    valid[0] = true;
    num_images_out++;
    
    cv::Mat base_image = cv::imread( depth_files[0], -1);
    cv::Mat actual_image;
    
    for(int i = 1; i < num_images_in ; ++i) 
    {
        matches = 0;
        sift.solveFeatures( filenames[i], *actual_key, *actual_dsc );
        matches = sift.solveMatches( *base_key, *actual_key, *base_dsc, *actual_dsc, pt1, pt2 );
        actual_image = cv::imread( depth_files[i], -1);
        removeBadPointsDual( pt1, pt2, base_image, actual_image );
        matches = pt1.size();
        
        printf("MATCH: %04i -> %04i:\t#matches = %i\n", base_frame, i, matches);
        
        if ( !next_frame && (matches < min_matches + window ))
        {
	  base_frame = i;
	  base_key.swap(actual_key);
	  base_dsc.swap(actual_dsc);
	  actual_key->clear();
	  actual_dsc->clear();
	  valid[base_frame] = true;
	  base_image = cv::imread( depth_files[base_frame], -1);
	  continue;
        }
        
        if (matches > min_matches + window )
        {
	  next_frame = true; // mark when pass through adjacent frame
	  previous_key.swap(actual_key);
	  previous_dsc.swap(actual_dsc);
	  actual_key->clear();
	  actual_dsc->clear();
        }
        else
        {
	  i--;
	  base_frame = i;
	  base_key.swap(previous_key);
	  base_dsc.swap(previous_dsc);
	  previous_key->clear();
	  previous_dsc->clear();
	  
	  valid[base_frame] = true;
	  base_image = cv::imread( depth_files[base_frame], -1);
	  num_images_out++;
	  next_frame = false;
        }
    }
    // Debug
    printVector( valid );
    
}

void FrameSelector::exportXML( const char* filename, std::vector< std::string > &filenames )
{
    std::vector< std::string > export_files;
    for (register int i = 0; i < filenames.size(); ++i)
        if (valid[i]) export_files.push_back( filenames[i]);
    
    exportXMLImageList( filename, export_files );
}