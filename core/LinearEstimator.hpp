/**
 * @file LinearEstimator.hpp
 * @brief This file has the best linear unbiased estimator (BLUE) implementation for points in 3D
 *
 * @author José David Tascón Vidarte
 * @date Feb/25/2014
 */

#ifndef __LINEARESTIMATOR_HPP__
#define __LINEARESTIMATOR_HPP__

// Eigen Libraries
#include <eigen3/Eigen/Dense>

// Std Libraries
#include <iostream>

// Local Libraries
#include "Debug.hpp"

// ================================================================================================
// ===================================== CLASS LinearEstimator ====================================
// ================================================================================================
template< typename tp >
class LinearEstimator
{
private:
    typedef Eigen::Matrix< tp, 3, 1 > V3;
    typedef Eigen::Matrix< tp, 3, 3 > M3;
    V3 Xpoint;
    V3 Covariance;
    
    V3 X1, X2;
    V3 Variance1, Variance2;
    
    M3 Rot;
    V3 tr;
    
public:
    // Constructor
    LinearEstimator( V3 &point1, V3 &variance1, V3 &point2, V3 &variance2, M3 &rotation, V3 &translation)
    {
        setPoints( point1, point2 );
        setVariances( variance1, variance2 );
        setPose( rotation, translation );
    }
    // Destructor
    ~LinearEstimator() { };
    
    void estimate()
    {
        DEBUG_3( std::cout << "point actual = " << X1.transpose() << "\n"; )
        DEBUG_3( std::cout << "covar actual = " << Variance1.transpose() << "\n"; )
        DEBUG_3( std::cout << "point viewed = " << X2.transpose() << "\n"; )
        DEBUG_3( std::cout << "covar viewed = " << Variance2.transpose() << "\n"; )
        
        // Temporal Variables
        V3 x1ref, x2ref;
        M3 C1ref, C2ref, C2tmp, Cout;
        
        x1ref = X1;				// Copy to non-modify X1
        C1ref = M3( Variance1.asDiagonal() );	// transform variance vector to matrix
        C2tmp = M3( Variance2.asDiagonal() );
        
        // Put point and covariance to reference frame
        x2ref = Rot.transpose()*(X2 - tr);
        C2ref = Rot*C2tmp*Rot.transpose();
        
        // BLUE estimator
        if ( x1ref.isZero(1e-5) ) Cout = C2ref;	// If x1ref doesn't have value then Xpoint = x2ref
        else Cout = (C1ref.inverse() + C2ref.inverse()).inverse(); 
        Xpoint = x1ref + Cout*C2ref.inverse()*(x2ref - x1ref);
        Covariance = Cout.diagonal();
        
        DEBUG_3( std::cout << "point estimation = " << Xpoint.transpose() << "\n"; )
        DEBUG_3( std::cout << "covar estimation = " << Covariance.transpose() << "\n"; )
    }
    
    // Set Functions
    void setPoints( V3 &point1, V3 &point2 ){ X1 = point1; X2 = point2; };
    void setVariances( V3 variance1, V3 &variance2 ){ Variance1 = variance1; Variance2 = variance2; };
    void setPose( M3 &rotation, V3 &translation ){ Rot = rotation; tr = translation; };
    
    // Get Functions
    V3 getPoint(){ return Xpoint; };
    V3 getVariance(){ return Covariance; };   
};



#endif