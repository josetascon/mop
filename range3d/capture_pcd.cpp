// Sep/09/2013
// Author: José David Tascón Vidarte

// libc
#include <iostream>

// PCL Libraries
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

// // OpenCV Libraries
// #include <opencv2/opencv.hpp>
// #include <opencv2/highgui/highgui.hpp>
// 
// // OpenMP
// #include <omp.h>

int count_pcd = 0;
char* filename = (char*)"out";
pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr pc;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    if (event.getKeySym() == "s" && event.keyDown())
    {
        char buf[256];
        char *cmd = &buf[0];
        sprintf(cmd, "./%s_%04d.pcd\0", filename, count_pcd);
        std::string str01(cmd, std::find(cmd, cmd + 128, '\0'));
        pcl::io::savePCDFileASCII (str01, *pc);
        std::cout << "Point Cloud Captured: " << str01 << "\n";
        count_pcd++;
    }
    return;
}
    
class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") { viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);}
    
    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
        if (!viewer.wasStopped())
        {
	  viewer.showCloud (cloud);
	  pc = cloud;
        }

    }
    
    void run ()
    {
        pcl::Grabber* interface = new pcl::OpenNIGrabber();
        
        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
        boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
        
        interface->registerCallback (f);
        interface->start ();
        
        while (!viewer.wasStopped())
        {
	  sleep (1);
        }        
        interface->stop ();
    }

    pcl::visualization::CloudViewer viewer;
};

void help(char* arg0)
{ 
    std::cout << " Usage: " << arg0 << " -f <filename prefix>\n"
	  "\tOptions:\n" 
	  "\t[-f]\t Filename prefix (default = 'out')\n" 
    << std::endl;
}

int main(int argc, char* argv[])
{
    std::cout << "Press 's' key to capture a pcd\n";
    
    if( argc > 4 )
    {
        help(argv[0]);
        return 0;
    }
    else
    {
        for(int i = 1; i < argc; i++ )
        {
	  const char* s = argv[i];
	  
	  if( strcmp( s, "-f" ) == 0 )
	  {
	      i++;
	      filename = argv[i];
	  }
	  else
	  {
	      help(argv[0]);
	      return fprintf( stderr, "Unknown option %s\n", s ), -1;
	  }
        }
    }
    
    SimpleOpenNIViewer v;
    v.run ();
    
    return 0;
}