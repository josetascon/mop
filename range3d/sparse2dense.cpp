// Oct/30/2013
// Author: José David Tascón Vidarte

// libc
#include <iostream>
#include <cmath>

// PCL Libraries
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>


void help(char* arg0)
{ 
    std::cout << "\033[1;33m Usage: " << arg0 << " -i <pcd input> -o <pcd output> \033[0m\n"
	  "\tOptions:\n" 
	  "\t[-i]\t Filename of input pcd\n"
	  "\t[-o]\t Filename of output pcd\n"
	  "\t[-d]\t Debug information\n"
    << std::endl;
}

int main(int argc, char* argv[])
{
    const char* filename_input;
    const char* filename_output;
    bool debug = false;
    if( argc < 2 )
    {
        help(argv[0]);
        return 0;
    }
    else
    {
        for(int i = 1; i < argc; i++ )
        {
	  const char* s = argv[i];
	  
	  if( strcmp( s, "-i" ) == 0 )
	  {
	      i++;
	      filename_input = argv[i];
	  }
	  else if( strcmp( s, "-o" ) == 0 )
	  {
	      i++;
	      filename_output = argv[i];
	  }
	  else if( strcmp( s, "-d" ) == 0 )
	  {
	      debug = true;
	  }
	  else
	  {
	      help(argv[0]);
	      return fprintf( stderr, "Unknown option %s\n", s ), -1;
	  }
        }
    }
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_dense(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (filename_input, *cloud_in) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read input file\n");
        return (-1);
    }
    
    if (!cloud_in->is_dense)
    {
        int height = cloud_in->height;
        int width = cloud_in->width;
        
        for (register int v = 0; v < height; ++v)
        {
	  for (register int u = 0; u < width; ++u)
	  {
	      float value = cloud_in->at(u,v).x;
	      if( isnan(value) )
	      {
		continue;
	      }
	      else
	      {
		cloud_dense->push_back( cloud_in->at(u,v) );
		if(debug) printf("Point (%i,%i): [%f,%f,%f]\n", u, v, cloud_in->at(u,v).x, cloud_in->at(u,v).y, cloud_in->at(u,v).z);
	      }
	  }
        }
        
        cloud_dense->sensor_origin_.setZero ();
        cloud_dense->sensor_orientation_.w () = 1.0f;
        cloud_dense->sensor_orientation_.x () = 0.0f;
        cloud_dense->sensor_orientation_.y () = 0.0f;
        cloud_dense->sensor_orientation_.z () = 0.0f;
        cloud_dense->is_dense = true;
        std::cout << "Sparse Cloud size = " << cloud_in->width*cloud_in->height << "\n";
        std::cout << "Dense Cloud size = " << cloud_dense->width*cloud_dense->height << "\n";
    }
    
    pcl::io::savePCDFileASCII (filename_output, *cloud_dense);
    std::cout << "Dense cloud saved as: " << filename_output << "\n";
    return 0;
}