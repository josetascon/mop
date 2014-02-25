// Created: Sep/04/2013
// Author: José David Tascón Vidarte

#include "FeaturesMap.hpp"

// ================================================================================================
// ============================= FUNCTIONS of CLASS FeaturesMap ===================================
// ================================================================================================

void FeaturesMap::solveVisibility( HandleDB *mydb )
{
    num_cameras = mydb->maxCamera() + 1;
    num_features = mydb->maxFeature() + 1;
//     std::cout << "max cameras = " << num_cameras << "\n";
//     std::cout << "max features = " << num_features << "\n";
     
//     visibility = Eigen::Matrix<bool,-1,-1>::Zero(num_cameras,num_features);
    visibility->resize(num_cameras,num_features);
    visibility->setZero();
    coordinates->resize(num_cameras,num_features);
    
    for(register int cam = 0; cam < num_cameras; ++cam)
    {
        std::vector< rowDB > row_vector;
        mydb->searchRowbyCamera( cam, &row_vector);
        for(register int k = 0; k < row_vector.size(); ++k)
        {
	  int a = row_vector[k].camera;
	  int b = row_vector[k].feature;
	  (*visibility)(a,b) = true;
	  (*coordinates)(a,b) = Eigen::Vector3d( row_vector[k].coordx, row_vector[k].coordy, 1.0 );
        }
    }
    vis_available = true;
}

void FeaturesMap::solveVisibility3D( HandleDB *mydb )
{
    num_cameras = mydb->maxCamera() + 1;
    num_features = mydb->maxFeature() + 1;
//     std::cout << "max cameras = " << num_cameras << "\n";
//     std::cout << "max features = " << num_features << "\n";
    
//     visibility = Eigen::Matrix<bool,-1,-1>::Zero(num_cameras,num_features);
    visibility->resize(num_cameras,num_features);
    visibility->setZero();
    coordinates3D->resize(num_cameras,num_features);
    
    for(register int cam = 0; cam < num_cameras; ++cam)
    {
        std::vector< rowDB_3D > row_vector;
        mydb->searchRowbyCamera3D( cam, &row_vector);
//         std::cout << "row size = " << row_vector.size() << "\n";
        for(register int k = 0; k < row_vector.size(); ++k)
        {
	  int a = row_vector[k].camera;
	  int b = row_vector[k].feature;
	  (*visibility)(a,b) = true;
	  (*coordinates3D)(a,b) = Eigen::Vector4d( row_vector[k].x3d, row_vector[k].y3d, row_vector[k].z3d, 1.0 );
        }
    }
    vis3d_available = true;
}

void FeaturesMap::exportTXT(const char *file_txt)
{
    std::ofstream myfile1;
    myfile1.open (file_txt);
    myfile1 << "Visibility [camera x feature]\n";
    myfile1 << visibility->rows() << " " << visibility->cols() << "\n";
    myfile1 << visibility << "\n\n\n";
    myfile1 << "Coordinates\n[(c,f):\t   u\tv\t  1]\n\n";
    for (int j = 0; j < visibility->cols(); j++)
    {
        for (int i = 0; i < visibility->rows(); i++)
        {
	  if ((*visibility)(i,j))
	  {
	      myfile1 << "(" << i << "," << j << "):\t" << (*coordinates)(i,j).transpose() << "\n";
	  }
        }
    }
    myfile1.close();
}