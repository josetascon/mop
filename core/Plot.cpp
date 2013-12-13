// February/19/2012
// Modified May/07/2013
// Author: José David Tascón Vidarte

#include "Plot.hpp"

PlotGL PlotGL::viewer;
// ================================================================================================
// ================================= FUNCTIONS of CLASS PlotGL=====================================
// ================================================================================================

/**
 * ******************************************************************
 * @function Constructor
 */
PlotGL::PlotGL() 
{
    // Initialize with construction
    background = false;
    enablecolorsofpoints = false;
    enablegrid = false;
    shrink_axis = true;
    plot_camera = true;
    
    _scale_view[0] =  _scale_view[1] = _scale_view[2] = 1.0f;
    _translation_view[0] = _translation_view[1] = _translation_view[2] = 0.0f; 
    _angle_view[0] = 195.0f;
    _angle_view[1] = 30.f;
    _angle_view[2] = 0.0f;
}

/**
 * ******************************************************************
 * @function Destructor
 */
PlotGL::~PlotGL() 
{
    ;
}

// Assignation methods
/**
 * ******************************************************************
 * @function setRotation
 */
void PlotGL::setRotation(std::vector< Eigen::Matrix3d > *rot)
{
    rotation = rot;
}

/**
 * ******************************************************************
 * @function setTranslation
 */
void PlotGL::setTranslation( std::vector< Eigen::Vector3d > *tr )
{
    translation = tr;
    camera_center.resize(tr->size());
}

/**
 * ******************************************************************
 * @function setStructure
 */
void PlotGL::setStructure( std::vector< std::vector<cv::Point3d> > *WP)
{
    World_Point = WP;
}

/**
 * ******************************************************************
 * @function setColor
 */
void PlotGL:: setColor( std::vector< std::vector<cv::Point3f> > *CLR )
{
    Color = CLR;
}

/**
 * ******************************************************************
 * @function initRendering
 */
// void initRendering()
void PlotGL::initRendering() 						//Initializes 3D rendering
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL); 			//Enable color
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f); 	//Change the background to black
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	// Generating random color to use in plot (it runs once)
	colors_cam.resize(3*(rotation->size()));
	//cout << "Size colors: " << colors_cam.size() << '\n';
	for (register int k = 0; k < colors_cam.size(); ++k)
	{
		colors_cam[k]= (rand() % 100) / 100.0;
		//cout << "color " << k << " = " << colors_cam[k] << '\n';
	}
}

/**
 * ******************************************************************
 * @function drawAxis
 */
// void drawAxis()
void PlotGL::drawAxis()
{
    float axis_lenght = 100.0f;
    if (shrink_axis) axis_lenght = 1.0f;
    glPushMatrix(); 					//PLOT Axis
    glBegin(GL_LINES); 					// X axis
    glColor3f( _RED );
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(axis_lenght, 0.0f, 0.0f);
    glEnd();
    glBegin(GL_LINES); 					// Y axis
    glColor3f( _GREEN );
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, axis_lenght, 0.0f);
    glEnd();
    glBegin(GL_LINES); 					// Z axis
    glColor3f( _BLUE );
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, axis_lenght);
    glEnd();
    glPopMatrix();
    
    glPushMatrix(); 					//Draw Principal World Point. Center of Coordinates
    glPointSize(3.0f);
    glBegin(GL_POINTS); 				//Vertex position and color of points
    glColor3f(_BLACK);	//Black Color
    glVertex3f(0.0f, 0.0f, 0.0f);
    glEnd();
    glPopMatrix();
}

void PlotGL::drawGrid(int sq_min, int sq_max, int ypos)
{
    glColor3f(.3,.3,.3);
    glBegin(GL_QUADS);
    glVertex3f( sq_min, ypos + 0.001, sq_min);
    glVertex3f( sq_min, ypos + 0.001, sq_max);
    glVertex3f( sq_max, ypos + 0.001, sq_max);
    glVertex3f( sq_max, ypos + 0.001, sq_min);
    glEnd();

    glBegin(GL_LINES);
    for (register int i=sq_min; i<=sq_max; ++i) {
        if (i==0) { glColor3f(.2,.2,.6); } else { glColor3f(.5,.5,.5); };
        glVertex3f(i,ypos,sq_min);
        glVertex3f(i,ypos,sq_max);
        if (i==0) { glColor3f(.6,.2,.2); } else { glColor3f(.5,.5,.5); };
        glVertex3f(sq_min,ypos,i);
        glVertex3f(sq_max,ypos,i);
    };
    glEnd();
}

/**
 * ******************************************************************
 * @function drawNumberCamera
 */
void PlotGL::drawNumberCamera(int value)
{
    float scale = 1.0f/750.0f;
    glPushMatrix();
    if (background) glColor3f(0.0f, 0.0f, 0.0f); // Black
    else glColor3f(1.0f, 1.0f, 1.0f); // White
    
    glRotatef(180, 1.0f, 0.0f, 0.0f);
    glScalef(scale, scale, scale);
    glLineWidth(2.0f);
    std::string s1 = number2string(value);
    const char *ps1 = s1.c_str();
    for(register int i = 0; i < s1.length(); ++i) glutStrokeCharacter(GLUT_STROKE_ROMAN, (int)ps1[i] );
    glPopMatrix();
}

/**
 * ******************************************************************
 * @function drawPinCamera
 */
// void drawPinCamera()
void PlotGL::drawPinCamera()
{
    float vertix = 0.15;
    float depth = 2.0;
    float proportion = 1.3333;
    glPushMatrix(); 						//PLOT Camera
    glLineWidth(2.0f);
    
    glBegin(GL_LINE_LOOP);					// PLOT square
    glVertex3f(proportion*vertix, vertix, depth*vertix);
    glVertex3f(proportion*vertix, -vertix, depth*vertix);
    glVertex3f(-proportion*vertix, -vertix, depth*vertix);
    glVertex3f(-proportion*vertix, vertix, depth*vertix);
    glEnd();
    
    glBegin(GL_LINES);						// Plot lines from center to square
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(proportion*vertix, vertix, depth*vertix);
    
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(proportion*vertix, -vertix, depth*vertix);
    
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(-proportion*vertix, -vertix, depth*vertix);
    
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(-proportion*vertix, vertix, depth*vertix);
    glEnd();
    
    glPopMatrix();
}

/**
 * ******************************************************************
 * @function drawCameras
 */
// void drawCameras()
void PlotGL::drawCameras()
{
    for (register int k = 0; k < rotation->size(); ++k)
    {
        glPushMatrix();
        Eigen::Vector3d angles;
        anglesfromRotation((*rotation)[k], angles);
        camera_center[k] = (-(*rotation)[k].transpose())*((*translation)[k]);
        glTranslatef( camera_center[k](0) , camera_center[k](1), camera_center[k](2));
//         glTranslatef((*translation)[k](0) , (*translation)[k](1), (*translation)[k](2)); //(orig sign +)
        glRotatef(-angles(0), 1.0f, 0.0f, 0.0f);
        glRotatef(-angles(1), 0.0f, 1.0f, 0.0f);
        glRotatef(-angles(2), 0.0f, 0.0f, 1.0f);
        
        // 		std::cout << "Translation Values: " << translation[k] << '\n';
        // 		std::cout << "Rotation Values: " << angles << '\n';
        // **** Random Color
        glColor3f(colors_cam[3*k],colors_cam[3*k+1],colors_cam[3*k+2]);
        drawPinCamera();	// Draw camera shape
        drawNumberCamera(k);	// Draw camera number
        glPopMatrix();
    }
}

/**
 * ******************************************************************
 * @function drawPoints
 */
// void drawPoints()
void PlotGL::drawPoints()
{
    for (register int k = 0; k < World_Point->size(); ++k)
    {
        glPushMatrix();
        glPointSize(2.0f); // size of points
        /// ICP need each orientation
        Eigen::Vector3d angles;
        anglesfromRotation((*rotation)[k], angles);
        glTranslatef( camera_center[k](0) , camera_center[k](1), camera_center[k](2));
        glRotatef(-angles(0), 1.0f, 0.0f, 0.0f);
        glRotatef(-angles(1), 0.0f, 1.0f, 0.0f);
        glRotatef(-angles(2), 0.0f, 0.0f, 1.0f);
        /// ICP
        
        // 		std::cout << "Translation Values: " << translation[k] << '\n';
        // 		std::cout << "Rotation Values: " << angles << '\n';
        
        // **** Random Color
        glColor3f(colors_cam[3*k],colors_cam[3*k+1],colors_cam[3*k+2]);
//         drawCamera();
        // **** COLOR ARRAY from RGB Images
        if (enablecolorsofpoints)
        {
	  glEnableClientState (GL_COLOR_ARRAY);
	  glColorPointer (3, GL_FLOAT, 0, (*Color)[k].data());
        }
        else glDisableClientState(GL_COLOR_ARRAY);
        
        glEnableClientState (GL_VERTEX_ARRAY);
        glVertexPointer (3, GL_DOUBLE, 0, (*World_Point)[k].data());
        glBegin(GL_POINTS);
        for (register int i = 0; i < (*World_Point)[k].size(); ++i)
        {
	  glArrayElement (i);
        }
        glEnd();
        //glDisableClientState(GL_VERTEX_ARRAY);
        glPopMatrix();
    }
}

// ================================================================================================
// ============================ Handlers Functions of Class PlotGL ================================
// ================================================================================================
/**
 * ******************************************************************
 * @function handleDrawScene
 */
void PlotGL::handleDrawScene()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (background) glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//glClearColor(0.7f, 0.9f, 1.0f, 1.0f); 	//Change the background to sky blue
    else glClearColor(0.0f, 0.0f, 0.0f, 1.0f); 	//Change the background to black
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    // Translation
    // Alternative use gluLookat (0.0, 0.0, 0.0, 0.0, 0.0, -5.0, 0.0, 1.0, 0.0); 
    // NEEDS APROPIATE LIBRARY
    glTranslatef(0.0f, 0.0f, -5.0f);
    glTranslatef(_translation_view[0], _translation_view[1], _translation_view[2]);
    
    // Scale (Zoom In, Zoom out)
    glScalef(_scale_view[0], _scale_view[1], _scale_view[2]);
//     glScalef(sx, sy, sz);
    
    //glRotate*(a, x, y, z)
    glRotatef(_angle_view[0], 1.0f, 0.0f, 0.0f);
    glRotatef(_angle_view[1], 0.0f, 1.0f, 0.0f);
    glRotatef(_angle_view[2], 0.0f, 0.0f, 1.0f);
    
    // add code here
    
    drawAxis();
    if (plot_camera) drawCameras();
    drawPoints();
    if (enablegrid) drawGrid(-30, 30, 4);
    
    // ....
    
    glutSwapBuffers();
    //glFlush();
}

/**
 * ******************************************************************
 * @function handleKeypress
 */
void PlotGL::handleKeypress(unsigned char key, int x, int y)
{
    switch (key) // Keyboard control
    {
        case 'r':						// RESET to initial view
	  _scale_view[0] =  _scale_view[1] = _scale_view[2] = 1.0f;
	  _translation_view[0] = _translation_view[1] = _translation_view[2] = 0.0f; 
	  _angle_view[0] = 195.0f; 	//Degrees
	  _angle_view[1] = 30.0f;
	  _angle_view[2] = 0.0f;
	  break;

        case 'b':						// Enable/Disable background color
	  background = !background;
	  break;
	  
        case 'c':						// Enable/Disable color of points
	  enablecolorsofpoints = !enablecolorsofpoints;
	  break;
	  
        case 'g':						// Enable/Disable grid
	  enablegrid = !enablegrid;
	  break;
	  
        case 'x':						// Enable/Disable large axis
	  shrink_axis = !shrink_axis;
	  break;
        case 'p':						// Enable/Disable camera plotting
	  plot_camera = !plot_camera;
	  break;
	  
        case '+':						// Zoom IN
	   _scale_view[0] *= 2.0f;
	   _scale_view[1] =  _scale_view[2] = _scale_view[0];
	  break;
	  
        case '-':						// Zoom OUT
	  _scale_view[0] /= 2.0f;
	  _scale_view[1] =  _scale_view[2] = _scale_view[0];
	  break;
	  
        case 'w':						// Move trough Axis z +
	  _translation_view[2] += 0.5f;
	  break;
	  
        case 's':						// Move trough Axis z -
	  _translation_view[2] -= 0.5f;
	  break;
	  
        case 'e':						// Move trough Axis y +
	  _translation_view[1] += 0.5f;
	  break;
	  
        case 'q':						// Move trough Axis y -
	  _translation_view[1] -= 0.5f;
	  break;
	  
        case 'a':						// Move trough Axis x +
	  _translation_view[0] += 0.5f;
	  break;
	  
        case 'd':						// Move trough Axis x -
	  _translation_view[0] -= 0.5f;
	  break;
	  
        case '1':						// Rotate +15° Axis x (pitch as defined)
	  _angle_view[0] += 15.0f;
        break;
        
        case '2':						// Rotate -15° Axis x (pitch as defined)
	  _angle_view[0] -= 15.0f;
	  break;
	  
        case '4':						// Rotate +15° Axis y (yaw as defined)
	  _angle_view[1] += 15.0f;
	  break;
	  
        case '5':						// Rotate -15° Axis y (yaw as defined)
	  _angle_view[1] -= 15.0f;
	  break;
	  
        case '7':						// Rotate +15° Axis z (roll as defined)
	  _angle_view[2] += 15.0f;
	  break;
	  
        case '8':						// Rotate -15° Axis z (roll as defined)
	  _angle_view[2] -= 15.0f;
	  break;
	  
        case 27: //Escape key END visualization
	  exit(0);
    }
}

/**
 * ******************************************************************
 * @function handleResize
 */ 
void PlotGL::handleResize(int w, int h) 			//Called when the window is resized
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (double)w / (double)h, 1.0, 200.0);
}

/**
 * ******************************************************************
 * @function handleUpdate
 */
void PlotGL::handleUpdate(int value)
{
    glutPostRedisplay();
    glutTimerFunc(25, wrapper_handleUpdate, 0);
//     glutTimerFunc(25, handleUpdateInternal, 0);
}

void PlotGL::run(int argc, char **argv, bool init)
{
    //Initialize GLUT
    if (init) glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(640, 480);
    
    //Create the window
    glutCreateWindow("3D Reconstruction");
    initRendering();

    //Set handler functions
    glutDisplayFunc( &wrapper_handleDrawScence );	//Consider glViewport to include two image models in one Window (subdivide a single window)
    glutReshapeFunc( &wrapper_handleResize );
    glutKeyboardFunc( &wrapper_handleKeypress );
    //glutMouseFunc(void (*func)(int button, int state, int x, int y));    
    glutTimerFunc(25, &wrapper_handleUpdate, 0); //Add a timer

    glutMainLoop();
}

// void handleUpdateInternal(int value)
// {
// 	glutPostRedisplay();
// 	glutTimerFunc(25, handleUpdateInternal, 0);
// }

// ================================================================================================
// ==================================== END CLASS PlotGL ==========================================
// ================================================================================================

// ================================================================================================
// ================================= Some Visualize Functions =====================================
// ================================================================================================

void visualizeSphereInWindow()
{
    // =================================== Sphere in a New Window ===================================
    vtkSmartPointer<vtkSphereSource> sphereSource = 
    vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetPhiResolution(20);
    sphereSource->SetThetaResolution(20);
    sphereSource->Update();
    
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(sphereSource->GetOutputPort());
    mapper->ScalarVisibilityOff();
    vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
    sphereActor->SetMapper(mapper);
    
    // Create a renderer, render window, and interactor
    vtkSmartPointer<vtkRenderer> renderer1 = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow1 = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow1->SetSize(400,200);
    
    renderWindow1->AddRenderer(renderer1);
    renderer1->AddActor(sphereActor);
    
    renderWindow1->SetWindowName("Sphere");
    renderWindow1->SetPosition(100,0);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor1 = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor1->SetRenderWindow(renderWindow1);
    // Render and interact
    renderWindow1->Render();
    renderWindowInteractor1->Start();
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> 
visualizeCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    
    // Plot a single cloud
    std::string str1 = "cloud_0";
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, str1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, str1);
    viewer->addCoordinateSystem (0.3);
    viewer->initCameraParameters();
    return viewer;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> 
visualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    
    // Plot a single cloud
    std::string str1 = "cloud_0";
    viewer->addPointCloud<pcl::PointXYZ> (cloud, str1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, str1);
    viewer->addCoordinateSystem (0.3);
    viewer->initCameraParameters();
    return viewer;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> 
visualizeCloudSet( std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > &set_cloud )
{
    int num_cameras = set_cloud.size();
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    std::vector<std::string> handle_cloud(num_cameras);
    for(register int k = 0; k < num_cameras; ++k)
    {
        char buf[256];
        sprintf(buf, "cloud%0.4d",k);
        std::string str1 = std::string(buf);
        handle_cloud[k] = str1;
        
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(set_cloud[k]);
        viewer->addPointCloud<pcl::PointXYZRGBA> (set_cloud[k], rgb, str1);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, str1);
        viewer->addCoordinateSystem (0.3);
        viewer->initCameraParameters ();
    }
    return viewer;
}

void visualizeCameras(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, 
	      std::vector< Eigen::Quaternion<double> > &quaternion, std::vector< Eigen::Vector3d > &translation )
{
    int num_cameras = quaternion.size();
    // ============================================ PLOT CAMERAS ============================================ 
    Eigen::Vector3f white_color(1.0,1.0,1.0);
    // Color
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_real_distribution<double> uniform(0.0,1.0);	// min and max
    Eigen::Vector3d color = Eigen::Vector3d( uniform(gen), uniform(gen), uniform(gen) );
//     std::cout << "Camera color: [ " << color.transpose() << " ]\n";
    for(register int k = 0; k < num_cameras; ++k)
    {
        Eigen::Matrix3d rot = quaternion[k].toRotationMatrix();
        Eigen::Vector3d orientation, center;
        Eigen::Matrix3d rot_inv = rot.transpose();
        anglesfromRotationZero(rot_inv, orientation); // TODO NO use this. use transformation matrix in plotcamera (create function)
        center = -rot_inv*translation[k];
        double ratio = 1.333333;
        vtkSmartPointer<vtkActor> act02 = plotCamera(center, orientation, ratio, 0.05, 1.5);
        act02->GetProperty()->SetColor( color(0), color(1), color(2) );
        
        std::stringstream ss;
        ss << k;
        vtkSmartPointer<vtkActor> act03 = plotText( ss.str(), center, orientation, white_color, 2.0);
        
        addActorToRenderCollection( viewer->getRendererCollection(), act02 );
        addActorToRenderCollection( viewer->getRendererCollection(), act03 );
    }
}

void visualizeCameras(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, std::vector<std::string> &filename_images,
	      std::vector< Eigen::Quaternion<double> > &quaternion, std::vector< Eigen::Vector3d > &translation )
{
    // Visualize cameras with images
    int num_cameras = filename_images.size();
    // ============================================ PLOT CAMERAS ============================================ 
    for(register int k = 0; k < num_cameras; ++k)
    {
        Eigen::Matrix3d rot = quaternion[k].toRotationMatrix();
        Eigen::Vector3d orientation, center;
        Eigen::Matrix3d rot_inv = rot.transpose();
        anglesfromRotation(rot_inv, orientation);
        center = -rot_inv*translation[k];
        double ratio = 0;
        vtkSmartPointer<vtkImageActor> act01 = plotCameraImage(filename_images[k].c_str(), center, orientation, ratio, 0.05, 1.5);
        vtkSmartPointer<vtkActor> act02 = plotCamera(center, orientation, ratio, 0.05, 1.5);
        
        addActorToRenderCollection( viewer->getRendererCollection(), act01 );
        addActorToRenderCollection( viewer->getRendererCollection(), act02 );
    }
}

void visualizeNoise(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, 
		std::vector< Eigen::MatrixXd > &Xmodel, std::vector< Eigen::MatrixXd > &Variance,
		std::vector< Eigen::Quaternion<double> > &quaternion, std::vector< Eigen::Vector3d > &translation, double amplitude)
{
   // ============================================ PLOT NOISE IN MODEL
    // Pose from cloud_set
//     Eigen::Vector4f cc = set_cloud[1]->sensor_origin_;
//     Eigen::Quaternion<float> qq = set_cloud[1]->sensor_orientation_;
//     Eigen::Matrix3f rot = qq.toRotationMatrix();
//     Eigen::Vector3f tt = cc.head(3);				// recover translation 
    
    // Random generators
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_real_distribution<double> uniform(0.0,1.0);
    int size_model = Xmodel.size();
    
    for(register int it = 0; it < size_model; ++it)
    {
        Eigen::Quaternion<float> qq = quaternion[it].template cast<float>();
        Eigen::Matrix3f rot = qq.toRotationMatrix();
        rot.transposeInPlace();
        Eigen::Vector3f tt = -rot*translation[it].template cast<float>();
        float red = uniform(gen), green = uniform(gen), blue = uniform(gen); 
        
        for(register int k=0; k < Xmodel[it].cols(); ++k)
        {
	  Eigen::Vector3d devk = Variance[it].col(k);
	  devk.cwiseSqrt();
	  Eigen::Vector3f vv = Xmodel[it].col(k).template cast<float>();
	  Eigen::Vector3f pp = rot*vv + tt;
	  vtkSmartPointer<vtkActor> act01 = actorEllipsoid(Eigen::Vector3d(pp(0),pp(1),pp(2)), devk*amplitude); //amplify variance to visualize
	  act01->GetProperty()->SetColor( red, green, blue );
	  act01->GetProperty()->SetOpacity(0.3);
	  addActorToRenderCollection( viewer->getRendererCollection(), act01 );
        }
    }
}

void visualizeGraph( int num_vertex, int num_edges , std::vector<float> &weights, std::vector< std::pair<int,int> > &edges_pairs )
{
    std::vector< vtkIdType > vtid(num_vertex);
    
    // =================================== GRAPH DECLARATION ===================================
    vtkSmartPointer<vtkMutableUndirectedGraph> g = vtkSmartPointer<vtkMutableUndirectedGraph>::New();
    
    vtkSmartPointer<vtkFloatArray> ids = vtkSmartPointer<vtkFloatArray>::New();
    ids->SetNumberOfComponents(1);
    ids->SetName("VertexIDs");
    
    vtkSmartPointer<vtkFloatArray> w_array = vtkSmartPointer<vtkFloatArray>::New();
    w_array->SetNumberOfComponents(1);
    w_array->SetName("Weights");
    
    for ( register int k = 0; k < num_vertex; k++ ) 
    {
        ids->InsertNextValue(k);			// Set the vertex ids
        vtid[k] = g->AddVertex();
    }
    
    for ( register int j = 0; j < num_edges; j++ ) 
    {
        w_array->InsertNextValue( weights[j] );					// Set the edge weights
        g->AddEdge( vtid[ edges_pairs[j].first ], vtid[ edges_pairs[j].second ] );	
    }
    
    // Add the edge weight and ids arrays to the graph
    g->GetEdgeData()->AddArray(w_array);
    g->GetVertexData()->AddArray(ids);
    
    // =================================== END GRAPH DECLARATION ===================================
    
    std::cout << std::cout << "\n================================ GRAPH Visualization ==================================\n";
    std::cout << "Number of vertices: " << g->GetNumberOfVertices() << std::endl;
    std::cout << "Number of edges: " << g->GetNumberOfEdges() << std::endl;
    
//     std::cout << "Number of Weights: "  
//     << vtkFloatArray::SafeDownCast(g->GetEdgeData()->GetArray("Weights"))->GetNumberOfTuples()
//     << std::endl;
//     
//     for(vtkIdType i = 0; i < w_array->GetNumberOfTuples(); i++)
//     {
//         double w = w_array->GetValue(i);
//         std::cout << "Weight " << i << " : " << w << std::endl;
//     }
        
    vtkSmartPointer<vtkGraphLayoutView> graphLayoutView = vtkSmartPointer<vtkGraphLayoutView>::New();
    graphLayoutView->AddRepresentationFromInput(g);
    graphLayoutView->SetColorVertices(true);
    graphLayoutView->SetVertexColorArrayName("VertexDegree");
    graphLayoutView->SetVertexLabelVisibility(true);
    graphLayoutView->SetVertexLabelArrayName("VertexIDs");
    graphLayoutView->SetEdgeLabelVisibility(true);
    graphLayoutView->SetEdgeLabelArrayName("Weights");
//     graphLayoutView->SetLayoutStrategyToCircular();
//     graphLayoutView->SetLayoutStrategyToSpanTree();
//     graphLayoutView->SetLayoutStrategyToSimple2D();
    graphLayoutView->ResetCamera();
    graphLayoutView->UpdateLayout();
    graphLayoutView->Render();
    
    // Create a renderer, render window, and interactor
    vtkSmartPointer<vtkRenderer> renderer2 = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow2 = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow2->SetSize(600,450);
    
    renderWindow2->AddRenderer(renderer2);
    graphLayoutView->SetRenderWindow(renderWindow2);
//     graphLayoutView->GetInteractor()->Start();
    
    renderWindow2->SetWindowName("Connectivity Graph");
    renderWindow2->SetPosition(800,0);
    
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor2 = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor2->SetRenderWindow(renderWindow2);
    
    // Render and interact
    renderWindow2->Render();
    renderWindowInteractor2->Start();
}


// ================================================================================================
// ================================= Functions Visualizer PCL =====================================
// ================================================================================================
void addActorToRenderCollection(const vtkSmartPointer<vtkRendererCollection> &collection, const vtkSmartPointer<vtkActor> &actor, int viewport)
{
    collection->InitTraversal ();
    vtkRenderer* renderer = NULL;
    int i = 0;
    while ((renderer = collection->GetNextItem ()) != NULL)
    {
        // Should we add the actor to all renderers?
        if (viewport == 0)
        {
	  renderer->AddActor (actor);
	  //      renderer->Render ();
        }
        else if (viewport == i)               // add the actor only to the specified viewport
        {
	  renderer->AddActor (actor);
	  //      renderer->Render ();
        }
        ++i;
    }
}

void addActorToRenderCollection(const vtkSmartPointer<vtkRendererCollection> &collection, const vtkSmartPointer<vtkImageActor> &actor, int viewport)
{
    collection->InitTraversal ();
    vtkRenderer* renderer = NULL;
    int i = 0;
    while ((renderer = collection->GetNextItem ()) != NULL)
    {
        // Should we add the actor to all renderers?
        if (viewport == 0)
        {
	  renderer->AddActor (actor);
	  //      renderer->Render ();
        }
        else if (viewport == i)               // add the actor only to the specified viewport
        {
	  renderer->AddActor (actor);
	  //      renderer->Render ();
        }
        ++i;
    }
}

vtkSmartPointer<vtkActor> actorSphere(Eigen::Vector3d center, double radius)
{
    vtkSmartPointer<vtkSphereSource> sphereSource = 
    vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetPhiResolution(20);
    sphereSource->SetThetaResolution(20);
    sphereSource->SetCenter(center(0), center(1), center(2));
    sphereSource->SetRadius(radius);
    sphereSource->Update();
    
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(sphereSource->GetOutputPort());
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    return actor;
}

vtkSmartPointer<vtkActor> actorCircleR(const Eigen::Vector3d center, const Eigen::Vector3d orientation, double radius)
{   
    vtkSmartPointer<vtkDiskSource> disk = vtkSmartPointer<vtkDiskSource>::New ();
    disk->SetCircumferentialResolution (100);
    disk->SetInnerRadius (radius - 0.001);
    disk->SetOuterRadius (radius + 0.001);
    disk->SetCircumferentialResolution (20);
    
    // Set the circle origin
    vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New ();
    t->Identity (); 
    t->Translate (center(0), center(1), center(2));
    t->RotateZ(orientation(2));
    t->RotateY(orientation(1));
    t->RotateX(orientation(0));
    
    vtkSmartPointer<vtkTransformPolyDataFilter> tf = vtkSmartPointer<vtkTransformPolyDataFilter>::New ();
    tf->SetTransform (t);
    tf->SetInputConnection (disk->GetOutputPort ());
//     vtkSmartPointer<vtkDataSet> data = tf->GetOutput();
    
    // Create an Actor
//     vtkSmartPointer<vtkLODActor> actor;
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(tf->GetOutputPort());
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetRepresentationToWireframe ();
    actor->GetProperty()->SetLighting (false);
    return actor;
}

vtkSmartPointer<vtkActor> actorEllipsoid(const Eigen::Vector3d center, const Eigen::Vector3d radius)
{  
    vtkSmartPointer<vtkParametricEllipsoid> parametricObject = vtkSmartPointer<vtkParametricEllipsoid>::New();
    parametricObject->SetXRadius(radius(0));
    parametricObject->SetYRadius(radius(1));
    parametricObject->SetZRadius(radius(2));
    
    vtkSmartPointer<vtkParametricFunctionSource> parametricFunctionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
    parametricFunctionSource->SetParametricFunction(parametricObject);
    parametricFunctionSource->Update();
    
    // Transform, translation to center
    vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New ();
    t->Identity (); 
    t->Translate (center(0), center(1), center(2));
    vtkSmartPointer<vtkTransformPolyDataFilter> tf = vtkSmartPointer<vtkTransformPolyDataFilter>::New ();
    tf->SetTransform (t);
    tf->SetInputConnection (parametricFunctionSource->GetOutputPort ());
    
    // Visualize
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(tf->GetOutputPort());
    
    // Create an actor for the contours
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    return actor;
}

vtkSmartPointer<vtkActor> actorEllipsoidContour(const Eigen::Vector3d center, const Eigen::Vector3d radius, int num_contours, int normal)
{  
    vtkSmartPointer<vtkParametricEllipsoid> parametricObject = vtkSmartPointer<vtkParametricEllipsoid>::New();
    parametricObject->SetXRadius(radius(0));
    parametricObject->SetYRadius(radius(1));
    parametricObject->SetZRadius(radius(2));
    
    vtkSmartPointer<vtkParametricFunctionSource> parametricFunctionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
    parametricFunctionSource->SetParametricFunction(parametricObject);
    parametricFunctionSource->Update();
    
    // Transform, translation to center
    vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New ();
    t->Identity (); 
    t->Translate (center(0), center(1), center(2));
    vtkSmartPointer<vtkTransformPolyDataFilter> tf = vtkSmartPointer<vtkTransformPolyDataFilter>::New ();
    tf->SetTransform (t);
    tf->SetInputConnection (parametricFunctionSource->GetOutputPort());
    tf->Update();
    
    tf->GetOutput()->ComputeBounds();
    double bounds[6];
    tf->GetOutput()->GetBounds(bounds);
//     printf("Bound x: (%f , %f)\n", bounds[0], bounds[1]);
//     printf("Bound y: (%f , %f)\n", bounds[2], bounds[3]);
//     printf("Bound z: (%f , %f)\n", bounds[4], bounds[5]);
    
    vtkSmartPointer<vtkPlane> plane = vtkSmartPointer<vtkPlane>::New();
    plane->SetOrigin((bounds[1] + bounds[0]) / 2.0,
		 (bounds[3] + bounds[2]) / 2.0,
		 (bounds[5] + bounds[4]) / 2.0);
    if(normal == 0) plane->SetNormal(1,0,0);
    else if(normal == 1) plane->SetNormal(0,1,0);
    else plane->SetNormal(0,0,1);
    
    // Create Scalars
    vtkSmartPointer<vtkDoubleArray> scalars = vtkSmartPointer<vtkDoubleArray>::New();
    int numberOfPoints = tf->GetOutput()->GetNumberOfPoints();
    scalars->SetNumberOfTuples(numberOfPoints);
    vtkPoints *pts = tf->GetOutput()->GetPoints();
    for (int i = 0; i < numberOfPoints; ++i)
    {
        double point[3];
        pts->GetPoint(i, point);
        scalars->SetTuple1(i, plane->EvaluateFunction(point));
    }
    tf->GetOutput()->GetPointData()->SetScalars(scalars);
    tf->GetOutput()->GetPointData()->GetScalars()->GetRange();
    
    vtkSmartPointer<vtkContourFilter> contour = vtkSmartPointer<vtkContourFilter>::New();
    contour->SetInputConnection(tf->GetOutputPort());
    contour->ComputeScalarsOff();
    contour->ComputeNormalsOff();
    contour->GenerateValues(num_contours,
		        .99 * tf->GetOutput()->GetPointData()->GetScalars()->GetRange()[0],
		        .99 * tf->GetOutput()->GetPointData()->GetScalars()->GetRange()[1]);
    contour->Update();
//     contour->Print(std::cout);
    
    // Visualize
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(contour->GetOutputPort());
    mapper->ScalarVisibilityOff();
    
    // Create an actor for the contours
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetLineWidth(2);
    actor->GetProperty()->SetLighting (false);
    return actor;
}

vtkSmartPointer<vtkActor> plotCamera( const Eigen::Vector3d center, const Eigen::Vector3d orientation, 
			        double proportion, double vertix, double depth)
{
//     double vertix = 0.15;
//     double depth = 2.0;
//     double proportion = 1.3333;
    
    // Create five points. 
    double origin[3] = {0.0, 0.0, 0.0};
    double p0[3] = {proportion*vertix, vertix, depth*vertix};
    double p1[3] = {proportion*vertix, -vertix, depth*vertix};
    double p2[3] = {-proportion*vertix, -vertix, depth*vertix};
    double p3[3] = {-proportion*vertix, vertix, depth*vertix};
    
    // Create a vtkPoints object and store the points in it
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(origin);
    points->InsertNextPoint(p0);
    points->InsertNextPoint(p1);
    points->InsertNextPoint(p2);
    points->InsertNextPoint(p3);
    
    // Create a cell array to store the lines in and add the lines to it
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    
    for (int i = 1; i < 5; i++)
    {
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0,0); 		// i is the number of the point in points. This plot lines from origin to i
        line->GetPointIds()->SetId(1,i); 		// i is the number of the point in points. This plot lines from origin to i
        lines->InsertNextCell(line);
        
        vtkSmartPointer<vtkLine> line2 = vtkSmartPointer<vtkLine>::New();
//         line2->GetPointIds()->SetNumberOfIds(2); 		// two id to be inserted, 0 and 1
        line2->GetPointIds()->SetId(0,i-1); 		// i is the number of the point in points, as the first point of the line
        line2->GetPointIds()->SetId(1,i); 		// i+1 is the number of the point in points, as the second point of the line
        lines->InsertNextCell(line2);
    }
    
    vtkSmartPointer<vtkLine> line3 = vtkSmartPointer<vtkLine>::New();
    line3->GetPointIds()->SetNumberOfIds(2);
    line3->GetPointIds()->SetId(0,4);
    line3->GetPointIds()->SetId(1,1);
    lines->InsertNextCell(line3);
    
    // Create a polydata to store everything in
    vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
    linesPolyData->SetPoints(points);			// Add the points to the dataset
    linesPolyData->SetLines(lines);			// Add the lines to the dataset
    
    // Transfrom the polydata (move)
    vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New ();
    t->Identity (); 
    t->Translate (center(0), center(1), center(2));
    t->RotateZ(orientation(2));
    t->RotateY(orientation(1));
    t->RotateX(orientation(0));
    
    vtkSmartPointer<vtkTransformPolyDataFilter> tf = vtkSmartPointer<vtkTransformPolyDataFilter>::New ();
    tf->SetTransform (t);
    tf->SetInput(linesPolyData);
    tf->Update();
    
    // Visualize
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(tf->GetOutputPort());
    mapper->ScalarVisibilityOff();
    
    // Create an actor for the contours
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(0.2,0.8,0.8);
    actor->GetProperty()->SetLineWidth(2);
//     actor->GetProperty()->SetLighting (false);
    return actor;
}

vtkSmartPointer<vtkActor> plotText( const std::string text,  const Eigen::Vector3d center, 
				  const Eigen::Vector3d orientation, const Eigen::Vector3f color, float fontsize)
{
    vtkSmartPointer<vtkTextSource> text_poly = vtkSmartPointer<vtkTextSource>::New();
    text_poly->SetText ( text.c_str() );
    text_poly->BackingOff();
    text_poly->Update();
    
    // Transfrom the polydata (move)
    vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New ();
    t->Identity (); 
    t->Translate (center(0), center(1), center(2));
    t->Scale( fontsize*0.005, fontsize*0.005, fontsize*0.005 );
//     t->RotateZ(orientation(2));		// Rotate the text in the Z	 axis
    t->RotateZ(orientation(2) - 180);
//     t->RotateY(orientation(1));
    t->RotateY(orientation(1) - 180); 		// Rotate the text in the Y axis
    t->RotateX(orientation(0));
    
    vtkSmartPointer<vtkTransformPolyDataFilter> tf = vtkSmartPointer<vtkTransformPolyDataFilter>::New ();
    tf->SetTransform (t);
    tf->SetInput(text_poly->GetOutput());
    tf->Update();
    
    // Visualize
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(tf->GetOutputPort());
    mapper->ScalarVisibilityOff();
    
    // Create an actor for the contours
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor( color(0), color(1), color(2) );
//     actor->GetProperty()->SetLineWidth(2);
    actor->GetProperty()->SetLighting (false);
    return actor;
    
}

vtkSmartPointer<vtkImageActor> plotCameraImage(const char* filename, const Eigen::Vector3d center, const Eigen::Vector3d orientation, 
				  double &proportion, double vertix, double depth )
{
    vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New();
    reader->SetFileName(filename);
    reader->SetDataScalarTypeToUnsignedChar();
    reader->Update();
    
    // Create an actor
    vtkSmartPointer<vtkImageActor> actorImage = vtkSmartPointer<vtkImageActor>::New();
    actorImage->SetInput(reader->GetOutput());
    double bounds[6];
    actorImage->GetBounds(bounds);
    
    //Proportion parameter works as input IF is provided and > 0, ELSE it is read from image width / height
    if (proportion <= 0) proportion = bounds[1]/bounds[3];
    double origin[3] = {-proportion*vertix, vertix, depth*vertix};
    
    // Transfrom the polydata (move)
    vtkSmartPointer<vtkTransform> P1 = vtkSmartPointer<vtkTransform>::New ();
    P1->Identity (); 
    P1->Translate (origin);
    P1->RotateX(180.0);
    P1->Scale((2*proportion*vertix)/bounds[1], (2*vertix)/bounds[3], 1.0 );
    
    vtkSmartPointer<vtkTransform> P2 = vtkSmartPointer<vtkTransform>::New ();
    P2->Identity (); 
    P2->Translate (center(0), center(1), center(2));
    P2->RotateZ(orientation(2));
    P2->RotateY(orientation(1));
    P2->RotateX(orientation(0));
    
    vtkSmartPointer<vtkMatrix4x4> PT = vtkSmartPointer<vtkMatrix4x4>::New();
    vtkSmartPointer<vtkMatrix4x4> oper = vtkSmartPointer<vtkMatrix4x4>::New();
    oper->Multiply4x4(P2->GetMatrix(), P1->GetMatrix(), PT);
    actorImage->SetUserMatrix (PT);
    
    return actorImage;
}

// Plot objects EXAMPLES
    // REMEMBER: viewer IS Defined AS:       boost::shared_ptr<pcl::visualization::PCLVisualizer> 
// //     vtkSmartPointer<vtkActor> act01 = actorEllipsoidContour(Eigen::Vector3d(1.0,0.0,0.0), Eigen::Vector3d(0.5,0.3,0.2), 15, 1);
//     vtkSmartPointer<vtkActor> act01 = actorEllipsoid(Eigen::Vector3d(0.628490732, -0.421111397, 1.777), Eigen::Vector3d(0.1,0.1,0.2));
//     vtkSmartPointer<vtkActor> act02 = actorEllipsoid(Eigen::Vector3d(1.1,0.0,0.0), Eigen::Vector3d(0.2,0.1,0.4));
// //     vtkSmartPointer<vtkActor> act01 = actorCircleR(Eigen::Vector3d(1.0,0.0,0.0), Eigen::Vector3d(0.0,0.0,0.0), 0.3);
// //     vtkSmartPointer<vtkActor> act01 = actorSphere(Eigen::Vector3d(1.0,2.0,0.0), 0.1);
//     act01->GetProperty()->SetColor(0.0,0.8,0.0);
//     act02->GetProperty()->SetColor(0.0,0.0,0.8);
//     act01->GetProperty()->SetOpacity(0.3);
//     act02->GetProperty()->SetOpacity(0.3);
// //     act02->GetProperty()->SetColor(0,0.8,0.8);
//     addActorToRenderCollection( viewer->getRendererCollection(), act01 );
//     addActorToRenderCollection( viewer->getRendererCollection(), act02 );
    
