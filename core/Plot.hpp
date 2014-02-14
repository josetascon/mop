/**
 * @file Plot.hpp
 * @brief Plot, class with functions, data, & handlers used to plot in OpenGL (glut library) or with VTK + PCLVisualizer
 *
 * @author José David Tascón Vidarte
 * @date February/21/2012
 */

#ifndef __PLOT_HPP__
#define __PLOT_HPP__

// OpenGl Libraries
#include <GL/glut.h>

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// OpenCV Libraries
#include <opencv2/opencv.hpp>		// TODO REMOVE dependency, implies a change in PlotGL class

// PCL Libraries
#include <pcl/visualization/cloud_viewer.h>

// VTK Libraries
#include <vtkVersion.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtkDataSet.h>
#include <vtkImageActor.h>
#include <vtkActor.h>
#include <vtkPolyData.h>
#include <vtkContourFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkRendererCollection.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include <vtkMutableUndirectedGraph.h>
#include <vtkGraphLayoutView.h>
#include <vtkCircularLayoutStrategy.h>
#include <vtkDataSetAttributes.h>

#include <vtkCellArray.h>
#include <vtkIntArray.h>
#include <vtkFloatArray.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkLine.h>
#include <vtkPlane.h>
#include <vtkDiskSource.h>
#include <vtkLineSource.h>
#include <vtkSphereSource.h>
#include <vtkTextSource.h>
#include <vtkParametricFunctionSource.h>
#include <vtkParametricEllipsoid.h>

#include <vtkImageReader.h>
#include <vtkPNGReader.h>
#include <vtkImageData.h>
#include <vtkInteractorStyleImage.h>

// Std Libraries
#include <iostream>
#include <utility>			//std::pair
#include <string>
#include <random>

// Local libraries
#include "Common.hpp"
// #include "CameraPose.hpp"

// Color Definition
#define _BLACK 0.0f,0.0f,0.0f
#define _WHITE 1.0f,1.0f,1.0f
#define _RED 1.0f,0.0f,0.0f
#define _GREEN 0.0f,1.0f,0.0f
#define _BLUE 0.0f,0.0f,1.0f

// ================================================================================================
// ======================================= CLASS PlotGL ===========================================
// ================================================================================================
class PlotGL
{
public:
    //Constructor
    PlotGL();
    //Destructor
    ~PlotGL();
    
    static PlotGL viewer;
    void run(int argc, char **argv, bool init);
    
    // Assignation methods
    void setRotation(std::vector< Eigen::Matrix3d > *rot);
    void setTranslation( std::vector< Eigen::Vector3d > *tr );
    void setStructure( std::vector< std::vector<cv::Point3d> > *WP);
    void setColor( std::vector< std::vector<cv::Point3f> > *CLR );
    
private:
    // Private DATA
    bool background;
    bool enablecolorsofpoints;
    bool enablegrid;
    bool shrink_axis;
    bool plot_camera;
    
    float _scale_view[3];
    float _translation_view[3];
    float _angle_view[3];
    std::vector< float > colors_cam;
    
    // Variables
    std::vector< Eigen::Matrix3d > *rotation;
    std::vector< Eigen::Vector3d > *translation;
    std::vector< Eigen::Vector3d > camera_center;	//internal variable
    std::vector< std::vector<cv::Point3d> > *World_Point; // world point design from camera view
    std::vector< std::vector<cv::Point3f> > *Color;
    
    // Draw objects
    void initRendering();
    void drawAxis();
    void drawGrid(int sq_min, int sq_max, int ypos);
    void drawNumberCamera(int value);
    void drawPinCamera();
    void drawCameras();
    void drawPoints();
    
    //handleFunctions
    void handleDrawScene();
    void handleResize(int w, int h);
    void handleKeypress(unsigned char key, int x, int y);
    void handleUpdate(int value);
    
    static void wrapper_handleDrawScence() { viewer.handleDrawScene(); };
    static void wrapper_handleResize(int w, int h) { viewer.handleResize(w,h); };
    static void wrapper_handleKeypress(unsigned char key, int x, int y) { viewer.handleKeypress(key,x,y); };
    static void wrapper_handleUpdate(int value) { viewer.handleUpdate(value); };
    
};


// ================================================================================================
// ================================= Some Visualize Functions =====================================
// ================================================================================================

void visualizeSphereInWindow();

boost::shared_ptr<pcl::visualization::PCLVisualizer> 
visualizeCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);

boost::shared_ptr<pcl::visualization::PCLVisualizer> 
visualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

boost::shared_ptr<pcl::visualization::PCLVisualizer> 
visualizeCloudSet( std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &set_cloud );

void visualizeCameras(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, 
	      std::vector< Eigen::Quaternion<double> > &quaternion, std::vector< Eigen::Vector3d > &translation, bool black = false );

void visualizeCameras(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, std::vector<std::string> &filename_images,
	      std::vector< Eigen::Quaternion<double> > &quaternion, std::vector< Eigen::Vector3d > &translation );

void visualizeNoise(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, 
		std::vector< Eigen::MatrixXd > &Xmodel, std::vector< Eigen::MatrixXd > &Variance,
		std::vector< Eigen::Quaternion<double> > &quaternion, std::vector< Eigen::Vector3d > &translation, double amplitude = 200.0);

void visualizeGraph( int num_vertex, int num_edges , std::vector<float> &weights, std::vector< std::pair<int,int> > &edges_pairs );

void visualizeTrack( boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, 
	      std::vector< Eigen::Quaternion<double> > &quaternion, std::vector< Eigen::Vector3d > &translation, 
	      Eigen::Vector3d color = Eigen::Vector3d( 0.0, 0.6, 0.0 ), double line_width = 2.0);



// ================================================================================================
// ================================= Functions Visualizer PCL =====================================
// ================================================================================================
void addActorToRenderCollection(const vtkSmartPointer<vtkRendererCollection> &collection, const vtkSmartPointer<vtkActor> &actor, int viewport = 0);
void addActorToRenderCollection(const vtkSmartPointer<vtkRendererCollection> &collection, const vtkSmartPointer<vtkImageActor> &actor, int viewport = 0);
vtkSmartPointer<vtkActor> actorLine(Eigen::Vector3d init_pt, Eigen::Vector3d end_pt, double line_width = 1.0);
vtkSmartPointer<vtkActor> actorSphere(Eigen::Vector3d center, double radius);
vtkSmartPointer<vtkActor> actorCircleR(const Eigen::Vector3d center, const Eigen::Vector3d orientation, double radius);
vtkSmartPointer<vtkActor> actorEllipsoid(const Eigen::Vector3d center, const Eigen::Vector3d radius);
vtkSmartPointer<vtkActor> actorEllipsoidContour(const Eigen::Vector3d center, const Eigen::Vector3d radius, int num_contours = 15, int normal = 0);
vtkSmartPointer<vtkActor> plotCamera( const Eigen::Vector3d center, const Eigen::Vector3d orientation, 
			        double proportion = 1.33333, double vertix = 0.15, double depth = 2.0);
vtkSmartPointer<vtkActor> plotText( const std::string text,  const Eigen::Vector3d center, 
				  const Eigen::Vector3d orientation, const Eigen::Vector3f color, float fontsize = 1.0);
vtkSmartPointer<vtkImageActor> plotCameraImage(const char* filename, const Eigen::Vector3d center, const Eigen::Vector3d orientation, 
				  double &proportion, double vertix = 0.15, double depth = 2.0);

#endif