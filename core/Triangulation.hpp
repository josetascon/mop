/**
 * @file Triangulation.hpp
 * @brief This file has all functions related with Projective and Epipolar Geometry
 *
 * @author José David Tascón Vidarte
 * @date Jun/28/2013
 */

#ifndef __TRIANGULATION_HPP__
#define __TRIANGULATION_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// Std Libraries
#include <iostream>
#include <vector>

// ====================================================================================================================================
// =======================================================  FUNCTIONS  ================================================================
// ====================================================================================================================================
/**
 * ******************************************************************
 * @brief Decription: Linear triangulation algorithm from HZ_2004, It yields one 3D point from two related features (2D) at different camera location.
 * Use of SVD as minimization
 * 
 * @param x1 		(input) First image homogeneous coordinate of feature
 * @param x2 		(input) Second image homogeneous coordinate of feature
 * @param P1 		(input) First image camera matrix
 * @param P2 		(input) Second image camera matrix
 * 
 * @return (Vector4d) homogeneous value of 3D triangulated point
 * 
 * @date Jul/4/2013
 */
Eigen::Vector4d linearTriangulation( Eigen::Vector3d &x1, Eigen::Vector3d &x2, Eigen::MatrixXd &P1, Eigen::MatrixXd &P2 );

// Multiple View
//@date Jul/16/2013
Eigen::Vector4d linearTriangulation( std::vector< Eigen::Vector3d > &xdata, std::vector< Eigen::MatrixXd > &P );

/**
 * ******************************************************************
 * @brief Decription: Linear triangulation algorithm from HZ_2004, It yields a set of 3D point from two related features (2D) sets at different camera location.
 * Use of SVD as minimization
 * 
 * @param x1 		(input) First image homogeneous coordinate of features set (size [3 x n])
 * @param x2 		(input) Second image homogeneous coordinate of features set (size [3 x n])
 * @param P1 		(input) First image camera matrix
 * @param P2 		(input) Second image camera matrix
 * 
 * @return (MatrixXd) homogeneous value of 3D triangulated points with size [4 x n]
 * 
 * @date Jul/4/2013
 */
Eigen::MatrixXd linearTriangulationSet( Eigen::MatrixXd &x1, Eigen::MatrixXd &x2, Eigen::MatrixXd &P1, Eigen::MatrixXd &P2 );

/**
 * ******************************************************************
 * @brief Decription: Find a normalization matrix to multiply the homogeneous data [3 x n] of features coordinates
 * 
 * @param data 		(input) Set of n features coordinates in a matrix (size [3 x n], 2D homogeneous)
 * @return (MatrixXd) Matrix [3 x 3] that produces an appropiated normalization to data
 * 
 * @date Jul/4/2013
 */
Eigen::MatrixXd normalizeTMatrix( Eigen::MatrixXd &data);

/**
 * ******************************************************************
 * @brief Decription: Linear triangulation algorithm with normalization from HZ_2004 , It yields a set of 3D point from two related features (2D) sets at different camera location.
 * Use of SVD as minimization. Improves linearTriangulationSet function because the normalization (internally executed) of x1 and x2.
 * 
 * @param x1 		(input) First image homogeneous coordinate of features set (size [3 x n])
 * @param x2 		(input) Second image homogeneous coordinate of features set (size [3 x n])
 * @param P1 		(input) First image camera matrix
 * @param P2 		(input) Second image camera matrix
 * 
 * @return (MatrixXd) homogeneous value of 3D triangulated points with size [4 x n]
 * 
 * @date Jul/4/2013
 */
Eigen::MatrixXd linearTriangulationNormalized( Eigen::MatrixXd &x1, Eigen::MatrixXd &x2, Eigen::MatrixXd &P1, Eigen::MatrixXd &P2 );

#endif