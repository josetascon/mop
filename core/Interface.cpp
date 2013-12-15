// Created: Sep/04/2013
// Author: José David Tascón Vidarte

#include "Interface.hpp"

// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================
void exportXMLImageList(const char *file_xml, std::vector< std::string > &files_names)
{
    std::ofstream myfile1;
    myfile1.open (file_xml);
    myfile1 << "<?xml version=\"1.0\"?>" << "\n";
    myfile1 << "<opencv_storage>" << "\n";
    myfile1 << "<images>" << "\n";
    for (std::vector< std::string >::iterator it = files_names.begin() ; it != files_names.end(); ++it)
    {
        myfile1 << *it << "\n";
    }
    myfile1 << "</images>" << "\n"; 
    myfile1 << "</opencv_storage>" << "\n";
    myfile1.close();
}

bool importXMLImageList(const char *file_xml, std::vector< std::string > &files_names)		//Read XML Files
{
    files_names.clear();
    cv::FileStorage fs(file_xml, cv::FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    cv::FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != cv::FileNode::SEQ )
        return false;
    cv::FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        files_names.push_back((std::string)*it);
    return true;
}


void exportPMVS(const char *output_path, std::vector<std::string> &nameImages, 
	     std::vector< Eigen::MatrixXd > &Cameras, Eigen::Matrix3d Calibration, std::vector< double > distortion)
{
    int num_cameras = Cameras.size();
    if ( nameImages.size() != num_cameras )
    {
        std::cerr << "\033[1;31m Can't write PMVS. #images is not equal to #cameras \033[0m\n";
        return;
    }
    
    // parameters to imwrite JPEG format
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(100); // from 0 to 100 (the higher is the better). Default value is 95.
    
    char buf[256];
    char *cmd = &buf[0];
    printf("Creating PMVS directories\n");
    sprintf(cmd, "rm -rf %s\n", output_path);		// Delete previous data pmvs
    system(cmd);
    sprintf(cmd, "mkdir -p %s/txt/\n", output_path);
    system(cmd);
    sprintf(cmd, "mkdir -p %s/visualize/\n", output_path);
    system(cmd);
    sprintf(cmd, "mkdir -p %s/models/\n", output_path);
    system(cmd);

    for(int cam = 0; cam < num_cameras; cam++)
    {
        // Creating txt file with cameras
        sprintf(buf, "%s/txt/%08d.txt", output_path, cam);
        FILE *f = fopen(buf, "w");
        assert(f);
        
        fprintf(f, "CONTOUR\n");
        fprintf(f, "%0.8e %0.8e %0.8e %0.8e\n", Cameras[cam](0,0), Cameras[cam](0,1), Cameras[cam](0,2), Cameras[cam](0,3));
        fprintf(f, "%0.8e %0.8e %0.8e %0.8e\n", Cameras[cam](1,0), Cameras[cam](1,1), Cameras[cam](1,2), Cameras[cam](1,3));
        fprintf(f, "%0.8e %0.8e %0.8e %0.8e\n", Cameras[cam](2,0), Cameras[cam](2,1), Cameras[cam](2,2), Cameras[cam](2,3));
        fclose(f);
        
        // Creating Images
        cv::Mat image = cv::imread(nameImages[cam], 1);
        cv::Mat und_img, K; 
        eigen2cv( Calibration, K);
        undistort(image, und_img, K, distortion);
        
        sprintf(cmd, "%s/visualize/%08d.jpg", output_path, cam);
        imwrite(cmd, und_img, compression_params);
    }
    
    // Write the options file for pmvs
    sprintf(buf, "%s/pmvs_options.txt", output_path);
    FILE *f_opt = fopen(buf, "w");

    fprintf(f_opt, "level 0\n");  // Default 1
    fprintf(f_opt, "csize 2\n");
    fprintf(f_opt, "threshold 0.7\n");
    fprintf(f_opt, "wsize 7\n");
    fprintf(f_opt, "minImageNum 3\n");
    fprintf(f_opt, "CPU 8\n");
    fprintf(f_opt, "setEdge 0\n");
    fprintf(f_opt, "useBound 0\n");
    fprintf(f_opt, "useVisData 0\n");	// use vis to improve speed
    fprintf(f_opt, "sequence -1\n");
    fprintf(f_opt, "timages -1 0 %d\n", num_cameras);
    fprintf(f_opt, "oimages -3\n");

    fclose(f_opt);

    return;
}

void undistortImages( const char * output_path, std::vector< std::string > &files_input,
		  Eigen::Matrix3d &Calibration, Eigen::MatrixXd &distortion,
		  const char * file_xml, std::vector< std::string > &undistort_files )
{
    // output_path is the folder
    undistort_files.clear();
    
    // parameters to imwrite PNG format
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(5); // PNG is lossless compression, compression 0 to 9, 
    
    char buf[256];
    char *cmd = &buf[0];
    printf("Creating undistort directories\n");
    sprintf(cmd, "rm -rf %s\n", output_path);		// Delete previous data pmvs
    system(cmd);
    sprintf(cmd, "mkdir -p %s/\n", output_path);
    system(cmd);
    
    cv::Mat K, coeff; 
    eigen2cv( Calibration, K );
    eigen2cv( distortion, coeff );
    
    for(std::vector< std::string >::iterator it = files_input.begin() ; it != files_input.end(); ++it)
    {
        // Loading Images
        
        cv::Mat image = cv::imread( *it, -1);
        cv::Mat und_img;
        undistort(image, und_img, K, coeff);
        
        std::stringstream ss;
        std::string basename = baseFileName( *it );
        ss << output_path << basename;
        std::string cmd = ss.str();
        imwrite(cmd, und_img, compression_params);
        undistort_files.push_back(cmd);
    }
    
    exportXMLImageList( file_xml, undistort_files );
}

void exportGRAPH( const char *filename, std::vector< Eigen::Quaternion<double> > &Qn_global, 
	       std::vector< Eigen::Vector3d > &tr_global )
{
//     char *file_txt = (char*)"pose.graph";
    std::ofstream myfile1;
    myfile1.open (filename);
    myfile1.precision(12);

    for(int it = 0; it < Qn_global.size(); it++)
    {
        Eigen::Matrix3d rr = Qn_global[it].toRotationMatrix();
//         Eigen::Vector3d tr = tr_global[it];
        Eigen::Vector3d tr = rr.transpose()*(-tr_global[it]); //center of coordinates
        Eigen::Vector3d angles;
        rotation2angles( rr, angles, false );		// false = radians units
        
        myfile1 << "VERTEX3 ";
        myfile1 << it << " ";
        myfile1 << tr.transpose() << " ";
        myfile1 << angles.transpose();
        myfile1 << "\n";
    }
    
    // Local constraints
    for(int it = 1; it < Qn_global.size(); it++)
    {
        Eigen::Matrix3d rot1 = Qn_global[it-1].toRotationMatrix();
        Eigen::Matrix3d rot2 = Qn_global[it].toRotationMatrix();
//         Eigen::Vector3d tr1 = tr_global[it-1];
//         Eigen::Vector3d tr2 = tr_global[it];
        Eigen::Vector3d tr1 = rot1.transpose()*(-tr_global[it-1]);
        Eigen::Vector3d tr2 = rot2.transpose()*(-tr_global[it]);
        Eigen::Matrix<double,6,1> p_j, p_i, dji;
        Eigen::Vector3d angles1, angles2;
        
        rotation2angles_DetectZero( rot1, angles1, false );//false for radians units
        rotation2angles_DetectZero( rot2, angles2, false );//false for radians units
        
        p_i << tr1, angles1;
        p_j << tr2, angles2;
        
        Eigen::Matrix<double,6,6> Ri = Eigen::Matrix<double,6,6>::Identity();
        Ri.block(0,0,3,3) = rot1;
        
        dji = Ri.transpose()*(p_j - p_i);
        
        // Local angles correction
        Eigen::Matrix3d rr = rot2*rot1.transpose();
        rotation2angles_DetectZero( rr , angles1, false );//false for radians units
        dji.tail(3) = angles1;
        
        myfile1 << "EDGE3 ";
        myfile1 << it - 1 << " ";
        myfile1 << it << " ";
        myfile1 << dji.transpose() << " ";
//         myfile1 << "1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1";
        myfile1 << "\n";
    }
    myfile1.close();
}
