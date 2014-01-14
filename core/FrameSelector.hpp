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
#include "SimpleSiftGPU.hpp"


// ================================================================================================
// ======================================= CLASS FrameSelector ====================================
// ================================================================================================
class FrameSelector
{
private:
    int num_images_in;
    int num_images_out;
    int min_matches;
    std::vector< std::string > filenames;
    std::vector< bool > valid;
    
public:
    FrameSelector( std::vector< std::string > filenames ): filenames(filenames) 
    {
        min_matches = 40;
        num_images_in = filenames.size();
        valid.resize(num_images_in, 0);
    };
    
    void solve();
};

#endif