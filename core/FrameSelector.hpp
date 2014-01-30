/**
 * @file FrameSelector.hpp
 * @brief This file has the FrameSelector class.
 *
 * @author José David Tascón Vidarte
 * @date Jan/13/2013
 */

#ifndef __FRAMESELECTOR_HPP__
#define __FRAMESELECTOR_HPP__

// Boost Libraries
#include <boost/shared_ptr.hpp>

// Std Libraries
#include <iostream>

// SiftGPU Library
#include <SiftGPU.h>

// Local Libraries
#include "Common.hpp"
#include "Interface.hpp"
#include "SimpleSiftGPU.hpp"
#include "DepthProjection.hpp"


// ================================================================================================
// ======================================= CLASS FrameSelector ====================================
// ================================================================================================
class FrameSelector
{
private:
    int num_images_in;
    int num_images_out;
    int min_matches;
    int window;
    std::vector< std::string > filenames;
    std::vector< bool > valid;
    
public:
    // Constructor
    FrameSelector( std::vector< std::string > filenames ): filenames(filenames) 
    {
        min_matches = 40;
        window = 30;
        num_images_in = filenames.size();
        valid.resize(num_images_in, 0);
    };
    
    FrameSelector( std::vector< std::string > filenames, int min_matches ): filenames(filenames), min_matches(min_matches)
    {
        window = 30;
        num_images_in = filenames.size();
        valid.resize(num_images_in, 0);
    };
    
    FrameSelector( std::vector< std::string > filenames, int min_matches, int window ): filenames(filenames), min_matches(min_matches), window(window)
    {
        num_images_in = filenames.size();
        valid.resize(num_images_in, 0);
    };
    
    void solve();
    void solvewithDepth( std::vector< std::string > &depth_files );
    
    void getValid( std::vector< bool > &valid_images ) { valid_images = valid; };
    void exportXML( const char* filename, std::vector< std::string > &filenames );
};

#endif