/**
 * @file reprojection_error.hpp
 * @brief reprojection_error, functions with cost for ceres reprojection error
 *
 * @author José David Tascón Vidarte
 * @date Jul/31/2012
 */

#ifndef __REPROJECTION_ERROR_HPP__
#define __REPROJECTION_ERROR_HPP__

#include "ceres/rotation.h"

// Reprojection error model with Quaternion and avoiding radial distortion
// Radial distortion is previously corrected to images point
// Use quaternion as rotation component
// CONSTANT 4 intrinsics variables
// Total parameters adjusted 11, Motion:7 + Structure: 4.
// 4 quaternion (rotation) variables, 3 translation variables , 
// 4 Three-dimensional point variables
struct RE_constK_QTS
{   
    RE_constK_QTS(double observed_x, double observed_y, double *intrinsics)
    : observed_x(observed_x), observed_y(observed_y), intrinsics(intrinsics) {}
    
    template <typename T>  // Allow optimization of parameters
    bool operator()(const T* const quaternion, const T* const translation, const T* const point, T* residuals) const 
    {
        const T fx = T(intrinsics[0]);
        const T fy = T(intrinsics[1]);
        const T cx = T(intrinsics[2]);
        const T cy = T(intrinsics[3]);
        // Use a quaternion rotation that doesn't assume the quaternion is
        // normalized, since one of the ways to run the bundler is to let Ceres
        // optimize all 4 quaternion parameters unconstrained.
        T p[3];
        
        ceres::QuaternionRotatePoint(quaternion, point, p);
        
        p[0] += translation[0];//*point[3];
        p[1] += translation[1];//*point[3];
        p[2] += translation[2];//*point[3];
        
        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = p[0] / p[2]; // - p[0] / p[2];
        T yp = p[1] / p[2]; // - p[1] / p[2];
        
        // Apply second and fourth order radial distortion.
        //     T r2 = xp*xp + yp*yp;
        //     T distortion = T(1.0) + r2  * (l1 + l2  * r2);
        
        // Compute final projected point position.
        T predicted_x = fx * xp + cx;
        T predicted_y = fy * yp + cy;
        
        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        
        return true;
    }
    
    double observed_x;
    double observed_y;
    double *intrinsics;
};

struct RE_constKQT_S
{   
    RE_constKQT_S(double observed_x, double observed_y, double *intrinsics, double *quaternion, double *translation)
    : observed_x(observed_x), observed_y(observed_y), intrinsics(intrinsics), quaternion(quaternion), translation(translation) {}
    
    template <typename T> // Constrained optimization for quaternion and translation, allowed parameter structure
    bool operator()(const T* const point, T* residuals) const 
    {
        const T fx = T(intrinsics[0]);
        const T fy = T(intrinsics[1]);
        const T cx = T(intrinsics[2]);
        const T cy = T(intrinsics[3]);
        // Use a quaternion rotation that doesn't assume the quaternion is
        // normalized, since one of the ways to run the bundler is to let Ceres
        // optimize all 4 quaternion parameters unconstrained.
        T p[3];
        T quat[4] = {T(quaternion[0]),T(quaternion[1]),T(quaternion[2]),T(quaternion[3])};
        T tr[3] = {T(translation[0]),T(translation[1]),T(translation[2])};
        
        ceres::QuaternionRotatePoint(quat, point, p);
        
        p[0] += tr[0];//*point[3];
        p[1] += tr[1];//*point[3];
        p[2] += tr[2];//*point[3];
        
        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = p[0] / p[2]; // - p[0] / p[2];
        T yp = p[1] / p[2]; // - p[1] / p[2];
        
        // Apply second and fourth order radial distortion.
        //     T r2 = xp*xp + yp*yp;
        //     T distortion = T(1.0) + r2  * (l1 + l2  * r2);
        
        // Compute final projected point position.
        T predicted_x = fx * xp + cx;
        T predicted_y = fy * yp + cy;
        
        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        
        return true;
    }
    
    double observed_x;
    double observed_y;
    double *intrinsics;
    double *quaternion;
    double *translation;
};

struct RE_constKS_QT
{   
    RE_constKS_QT(double observed_x, double observed_y, double *intrinsics, double *point)
    : observed_x(observed_x), observed_y(observed_y), intrinsics(intrinsics), point(point) {}
    
    template <typename T>  // Allow optimization of parameters
    bool operator()(const T* const quaternion, const T* const translation, T* residuals) const 
    {
        const T fx = T(intrinsics[0]);
        const T fy = T(intrinsics[1]);
        const T cx = T(intrinsics[2]);
        const T cy = T(intrinsics[3]);
        // Use a quaternion rotation that doesn't assume the quaternion is
        // normalized, since one of the ways to run the bundler is to let Ceres
        // optimize all 4 quaternion parameters unconstrained.
        T p[3];
        T pts[3] = {T(point[0]),T(point[1]),T(point[2])};
        
        ceres::QuaternionRotatePoint(quaternion, pts, p);
        
        p[0] += translation[0];//*point[3];
        p[1] += translation[1];//*point[3];
        p[2] += translation[2];//*point[3];
        
        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = p[0] / p[2]; // - p[0] / p[2];
        T yp = p[1] / p[2]; // - p[1] / p[2];
        
        // Apply second and fourth order radial distortion.
        //     T r2 = xp*xp + yp*yp;
        //     T distortion = T(1.0) + r2  * (l1 + l2  * r2);
        
        // Compute final projected point position.
        T predicted_x = fx * xp + cx;
        T predicted_y = fy * yp + cy;
        
        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        
        return true;
    }
    
    double observed_x;
    double observed_y;
    double *intrinsics;
    double *point;
};

struct RE_constKD_QTS
{   
    RE_constKD_QTS(double observed_x, double observed_y, double *intrinsics, double *distortion)
    : observed_x(observed_x), observed_y(observed_y), intrinsics(intrinsics), distortion(distortion) {}
    
    template <typename T>  // Allow optimization of parameters
    bool operator()(const T* const quaternion, const T* const translation, const T* const point, T* residuals) const
    {
        
        const T fx = T(intrinsics[0]);
        const T fy = T(intrinsics[1]);
        const T cx = T(intrinsics[2]);
        const T cy = T(intrinsics[3]);
        // Use a quaternion rotation that doesn't assume the quaternion is
        // normalized, since one of the ways to run the bundler is to let Ceres
        // optimize all 4 quaternion parameters unconstrained.
        T p[3];
        ceres::QuaternionRotatePoint(quaternion, point, p);
        
        p[0] += translation[0];//*point[3];
        p[1] += translation[1];//*point[3];
        p[2] += translation[2];//*point[3];
        
        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = p[0] / p[2]; // - p[0] / p[2];
        T yp = p[1] / p[2]; // - p[1] / p[2];
        
        // Apply radial distortion.
        const T k1 = T(distortion[0]);
        const T k2 = T(distortion[1]);
        const T p1 = T(distortion[2]);
        const T p2 = T(distortion[3]);
        const T k3 = T(distortion[4]);
        
        T r2 = xp*xp + yp*yp;
        T xpd = xp*( T(1.0) + r2  * (k1 + k2*r2 + k3*r2*r2) ) + T(2.0)*p1*xp*yp + p2*(r2 + T(2.0)*xp*xp);
        T ypd = yp*( T(1.0) + r2  * (k1 + k2*r2 + k3*r2*r2) ) + T(2.0)*p2*xp*yp + p1*(r2 + T(2.0)*yp*yp);
        
        // Compute final projected point position.
        T predicted_x = fx * xpd + cx;
        T predicted_y = fy * ypd + cy;
        
        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        
        return true;
    }
    
    double observed_x;
    double observed_y;
    double *intrinsics;
    double *distortion;
};

struct RE_constKDq_QTS
{   
    RE_constKDq_QTS(double observed_x, double observed_y, double *intrinsics, double *distortion, double *qt_before)
    : observed_x(observed_x), observed_y(observed_y), intrinsics(intrinsics), distortion(distortion), qt_before(qt_before) {}
    
    template <typename T>  // Allow optimization of parameters
    bool operator()(const T* const quaternion, const T* const translation, const T* const point, T* residuals) const
    {
        
        const T fx = T(intrinsics[0]);
        const T fy = T(intrinsics[1]);
        const T cx = T(intrinsics[2]);
        const T cy = T(intrinsics[3]);
        // Use a quaternion rotation that doesn't assume the quaternion is
        // normalized, since one of the ways to run the bundler is to let Ceres
        // optimize all 4 quaternion parameters unconstrained.
        T p[3];
        T w[4] = {T(qt_before[0]),T(qt_before[1]),T(qt_before[2]),T(qt_before[3])};
        T zw[4];
        ceres::QuaternionProduct(quaternion, w, zw); // zw = z*w
        ceres::QuaternionRotatePoint(zw, point, p);
        
        p[0] += translation[0];//*point[3];
        p[1] += translation[1];//*point[3];
        p[2] += translation[2];//*point[3];
        
        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = p[0] / p[2]; // - p[0] / p[2];
        T yp = p[1] / p[2]; // - p[1] / p[2];
        
        // Apply radial distortion.
        const T k1 = T(distortion[0]);
        const T k2 = T(distortion[1]);
        const T p1 = T(distortion[2]);
        const T p2 = T(distortion[3]);
        const T k3 = T(distortion[4]);
        
        T r2 = xp*xp + yp*yp;
        T xpd = xp*( T(1.0) + r2  * (k1 + k2*r2 + k3*r2*r2) ) + T(2.0)*p1*xp*yp + p2*(r2 + T(2.0)*xp*xp);
        T ypd = yp*( T(1.0) + r2  * (k1 + k2*r2 + k3*r2*r2) ) + T(2.0)*p2*xp*yp + p1*(r2 + T(2.0)*yp*yp);
        
        // Compute final projected point position.
        T predicted_x = fx * xpd + cx;
        T predicted_y = fy * ypd + cy;
        
        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        
        return true;
    }
    
    double observed_x;
    double observed_y;
    double *intrinsics;
    double *distortion;
    double *qt_before;
};

struct RE_constKDQT_S
{
    RE_constKDQT_S(double observed_x, double observed_y, double *intrinsics, double *distortion, double *quaternion, double *translation)
    : observed_x(observed_x), observed_y(observed_y), intrinsics(intrinsics), distortion(distortion), quaternion(quaternion), translation(translation) {}
    
    template <typename T>  // Allow optimization of parameters
    bool operator()(const T* const point, T* residuals) const
    {
        
        const T fx = T(intrinsics[0]);
        const T fy = T(intrinsics[1]);
        const T cx = T(intrinsics[2]);
        const T cy = T(intrinsics[3]);
        // Use a quaternion rotation that doesn't assume the quaternion is
        // normalized, since one of the ways to run the bundler is to let Ceres
        // optimize all 4 quaternion parameters unconstrained.
        T p[3];
        T quat[4] = {T(quaternion[0]),T(quaternion[1]),T(quaternion[2]),T(quaternion[3])};
        T tr[3] = {T(translation[0]),T(translation[1]),T(translation[2])};
        
        ceres::QuaternionRotatePoint(quat, point, p);
        
        p[0] += tr[0];//*point[3];
        p[1] += tr[1];//*point[3];
        p[2] += tr[2];//*point[3];
        
        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = p[0] / p[2]; // - p[0] / p[2];
        T yp = p[1] / p[2]; // - p[1] / p[2];
        
        // Apply radial distortion.
        const T k1 = T(distortion[0]);
        const T k2 = T(distortion[1]);
        const T p1 = T(distortion[2]);
        const T p2 = T(distortion[3]);
        const T k3 = T(distortion[4]);
        
        T r2 = xp*xp + yp*yp;
        T xpd = xp*( T(1.0) + r2  * (k1 + k2*r2 + k3*r2*r2) ) + T(2.0)*p1*xp*yp + p2*(r2 + T(2.0)*xp*xp);
        T ypd = yp*( T(1.0) + r2  * (k1 + k2*r2 + k3*r2*r2) ) + T(2.0)*p2*xp*yp + p1*(r2 + T(2.0)*yp*yp);
        
        // Compute final projected point position.
        T predicted_x = fx * xpd + cx;
        T predicted_y = fy * ypd + cy;
        
        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        
        return true;
    }
    
    double observed_x;
    double observed_y;
    double *intrinsics;
    double *distortion;
    double *quaternion;
    double *translation;
};

struct RE_constKD_normT_QTS
{   
    RE_constKD_normT_QTS(double observed_x, double observed_y, double *intrinsics, double *distortion)
    : observed_x(observed_x), observed_y(observed_y), intrinsics(intrinsics), distortion(distortion) {}
    
    template <typename T>  // Allow optimization of parameters
    bool operator()(const T* const quaternion, const T* const translation, const T* const point, T* residuals) const
    {
        const T fx = T(intrinsics[0]);
        const T fy = T(intrinsics[1]);
        const T cx = T(intrinsics[2]);
        const T cy = T(intrinsics[3]);
        // Use a quaternion rotation that doesn't assume the quaternion is
        // normalized, since one of the ways to run the bundler is to let Ceres
        // optimize all 4 quaternion parameters unconstrained.
        T p[3];
        const T normt = T(1) / sqrt(translation[0] * translation[0] + translation[1] * translation[1] + translation[2] * translation[2]);        
        ceres::QuaternionRotatePoint(quaternion, point, p);
        
        p[0] += translation[0] * normt;//*point[3];
        p[1] += translation[1] * normt;//*point[3];
        p[2] += translation[2] * normt;//*point[3];
        
        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = p[0] / p[2]; // - p[0] / p[2];
        T yp = p[1] / p[2]; // - p[1] / p[2];
        
        // Apply radial distortion.
        const T k1 = T(distortion[0]);
        const T k2 = T(distortion[1]);
        const T p1 = T(distortion[2]);
        const T p2 = T(distortion[3]);
        const T k3 = T(distortion[4]);
        
        T r2 = xp*xp + yp*yp;
        T xpd = xp*( T(1.0) + r2  * (k1 + k2*r2 + k3*r2*r2) ) + T(2.0)*p1*xp*yp + p2*(r2 + T(2.0)*xp*xp);
        T ypd = yp*( T(1.0) + r2  * (k1 + k2*r2 + k3*r2*r2) ) + T(2.0)*p2*xp*yp + p1*(r2 + T(2.0)*yp*yp);
        
        // Compute final projected point position.
        T predicted_x = fx * xpd + cx;
        T predicted_y = fy * ypd + cy;
        
        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        
        return true;
    }
    
    double observed_x;
    double observed_y;
    double *intrinsics;
    double *distortion;
};
/*
struct RE_constKDQ_normT_TS
{   
    RE_constKDQ_normT_TS(double observed_x, double observed_y, double *intrinsics, double *distortion, double *quaternion)
    : observed_x(observed_x), observed_y(observed_y), intrinsics(intrinsics), distortion(distortion), quaternion(quaternion) {}
    
    template <typename T>  // Allow optimization of parameters
    bool operator()(const T* const translation, const T* const point, T* residuals) const
    {
        const T fx = T(intrinsics[0]);
        const T fy = T(intrinsics[1]);
        const T cx = T(intrinsics[2]);
        const T cy = T(intrinsics[3]);
        // Use a quaternion rotation that doesn't assume the quaternion is
        // normalized, since one of the ways to run the bundler is to let Ceres
        // optimize all 4 quaternion parameters unconstrained.
        T p[3];
        const T quat[4] = {T(quaternion[0]),T(quaternion[1]),T(quaternion[2]),T(quaternion[3])};
        const T normt = T(1) / sqrt(translation[0] * translation[0] + translation[1] * translation[1] + translation[2] * translation[2]);
        
        ceres::QuaternionRotatePoint(quat, point, p);
        
        p[0] += translation[0] * normt;//*point[3];
        p[1] += translation[1] * normt;//*point[3];
        p[2] += translation[2] * normt;//*point[3];
        
        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = p[0] / p[2]; // - p[0] / p[2];
        T yp = p[1] / p[2]; // - p[1] / p[2];
        
        // Apply radial distortion.
        const T k1 = T(distortion[0]);
        const T k2 = T(distortion[1]);
        const T p1 = T(distortion[2]);
        const T p2 = T(distortion[3]);
        const T k3 = T(distortion[4]);
        
        T r2 = xp*xp + yp*yp;
        T xpd = xp*( T(1.0) + r2  * (k1 + k2*r2 + k3*r2*r2) ) + T(2.0)*p1*xp*yp + p2*(r2 + T(2.0)*xp*xp);
        T ypd = yp*( T(1.0) + r2  * (k1 + k2*r2 + k3*r2*r2) ) + T(2.0)*p2*xp*yp + p1*(r2 + T(2.0)*yp*yp);
        
        // Compute final projected point position.
        T predicted_x = fx * xpd + cx;
        T predicted_y = fy * ypd + cy;
        
        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        
        return true;
    }
    
    double observed_x;
    double observed_y;
    double *intrinsics;
    double *distortion;
    double *quaternion;
};*/

struct RE_KQTS
{   
    RE_KQTS(double observed_x, double observed_y)
    : observed_x(observed_x), observed_y(observed_y) {}
    
    template <typename T>  // Allow optimization of parameters
    bool operator()(const T* const intrinsics, const T* const quaternion, const T* const translation, const T* const point, T* residuals) const 
    {
        const T fx = intrinsics[0];
        const T fy = intrinsics[1];
        const T cx = intrinsics[2];
        const T cy = intrinsics[3];
        // Use a quaternion rotation that doesn't assume the quaternion is
        // normalized, since one of the ways to run the bundler is to let Ceres
        // optimize all 4 quaternion parameters unconstrained.
        T p[3];
        
        ceres::QuaternionRotatePoint(quaternion, point, p);
        
        p[0] += translation[0];//*point[3];
        p[1] += translation[1];//*point[3];
        p[2] += translation[2];//*point[3];
        
        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = p[0] / p[2]; // - p[0] / p[2];
        T yp = p[1] / p[2]; // - p[1] / p[2];
        
        // Apply second and fourth order radial distortion.
        //     T r2 = xp*xp + yp*yp;
        //     T distortion = T(1.0) + r2  * (l1 + l2  * r2);
        
        // Compute final projected point position.
        T predicted_x = fx * xp + cx;
        T predicted_y = fy * yp + cy;
        
        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        
        return true;
    }
    
    double observed_x;
    double observed_y;
};

struct RE_KDQTS
{   
    RE_KDQTS(double observed_x, double observed_y)
    : observed_x(observed_x), observed_y(observed_y) {}
    
    template <typename T>  // Allow optimization of parameters
    bool operator()(const T* const intrinsics, const T* const distortion,
		const T* const quaternion, const T* const translation, const T* const point, T* residuals) const
    {
        const T fx = intrinsics[0];
        const T fy = intrinsics[1];
        const T cx = intrinsics[2];
        const T cy = intrinsics[3];
        // Use a quaternion rotation that doesn't assume the quaternion is
        // normalized, since one of the ways to run the bundler is to let Ceres
        // optimize all 4 quaternion parameters unconstrained.
        T p[3];
        ceres::QuaternionRotatePoint(quaternion, point, p);
        
        p[0] += translation[0];//*point[3];
        p[1] += translation[1];//*point[3];
        p[2] += translation[2];//*point[3];
        
        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = p[0] / p[2]; // - p[0] / p[2];
        T yp = p[1] / p[2]; // - p[1] / p[2];
        
        // Apply radial distortion.
        const T k1 = distortion[0];
        const T k2 = distortion[1];
        const T p1 = distortion[2];
        const T p2 = distortion[3];
        const T k3 = distortion[4];
        
        T r2 = xp*xp + yp*yp;
        T xpd = xp*( T(1.0) + r2  * (k1 + k2*r2 + k3*r2*r2) ) + T(2.0)*p1*xp*yp + p2*(r2 + T(2.0)*xp*xp);
        T ypd = yp*( T(1.0) + r2  * (k1 + k2*r2 + k3*r2*r2) ) + T(2.0)*p2*xp*yp + p1*(r2 + T(2.0)*yp*yp);
        
        // Compute final projected point position.
        T predicted_x = fx * xpd + cx;
        T predicted_y = fy * ypd + cy;
        
        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        
        return true;
    }
    
    double observed_x;
    double observed_y;
};

// Translation_Constraint
// {
//     Translation_Constraint() {}
//     template <typename T>
//     bool operator()(const T* const translation,
//                   T* residuals) const 
//     {
//         residuals[0] = T(1000.0)*(translation[0] - T(1.0));
//     }
// };

// Doesn't work, aparently the derivative calculation creates an error
// struct Scale_Constraint 
// {
//     Scale_Constraint() {}
//     template <typename T>
//     bool operator()(const T* const scale_translation,
//                   T* residuals) const 
//     {
//         residuals[0] = T(1000.0)*(scale_translation[0] - T(1.0));
//     }
// };


struct RE_Scale
{   
    RE_Scale(double observed_x, double observed_y, double *intrinsics, double *quaternion, double *translation)
    : observed_x(observed_x), observed_y(observed_y), intrinsics(intrinsics), quaternion(quaternion), translation(translation) {}
    
    template <typename T>  // Allow optimization of parameters, structure and scale
    bool operator()(const T* const scale_translation, const T* const point, T* residuals) const
    {
        
        const T fx = T(intrinsics[0]);
        const T fy = T(intrinsics[1]);
        const T cx = T(intrinsics[2]);
        const T cy = T(intrinsics[3]);
        // Use a quaternion rotation that doesn't assume the quaternion is
        // normalized, since one of the ways to run the bundler is to let Ceres
        // optimize all 4 quaternion parameters unconstrained.
        T p[3];
        const T quat[4] = {T(quaternion[0]), T(quaternion[1]), T(quaternion[2]), T(quaternion[3]) };
//         const T *ptr_quat = &quat[0];
        
        ceres::QuaternionRotatePoint(quat, point, p);
        
        p[0] += T(translation[0])*scale_translation[0];//point[3];
        p[1] += T(translation[1])*scale_translation[0];//point[3];
        p[2] += T(translation[2])*scale_translation[0];//point[3];
        
        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = p[0] / p[2]; // - p[0] / p[2];
        T yp = p[1] / p[2]; // - p[1] / p[2];
        
        // Apply second and fourth order radial distortion.
        //     T r2 = xp*xp + yp*yp;
        //     T distortion = T(1.0) + r2  * (l1 + l2  * r2);
        
        // Compute final projected point position.
        T predicted_x = fx * xp + cx;
        T predicted_y = fy * yp + cy;
        
        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        
        return true;
    }
    
    template <typename T>  // Constrained optimization for scale in translation, allowed parameter structure
    bool operator()(const T* const point, T* residuals) const {

    const T fx = T(intrinsics[0]);
    const T fy = T(intrinsics[1]);
    const T cx = T(intrinsics[2]);
    const T cy = T(intrinsics[3]);
    // Use a quaternion rotation that doesn't assume the quaternion is
    // normalized, since one of the ways to run the bundler is to let Ceres
    // optimize all 4 quaternion parameters unconstrained.
    T p[3];
    const T quat[4] = {T(quaternion[0]), T(quaternion[1]), T(quaternion[2]), T(quaternion[3]) };
//     const T *ptr_quat = &quat[0];
    
    ceres::QuaternionRotatePoint(quat, point, p);

    p[0] += T(translation[0]);//point[3];
    p[1] += T(translation[1]);//point[3];
    p[2] += T(translation[2]);//point[3];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = p[0] / p[2]; // - p[0] / p[2];
    T yp = p[1] / p[2]; // - p[1] / p[2];

    // Apply second and fourth order radial distortion.
//     T r2 = xp*xp + yp*yp;
//     T distortion = T(1.0) + r2  * (l1 + l2  * r2);

    // Compute final projected point position.
    T predicted_x = fx * xp + cx;
    T predicted_y = fy * yp + cy;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);

    return true;
  }
    
    double observed_x;
    double observed_y;
    double *intrinsics;
    double *quaternion;
    double *translation;
};

struct RE_Scalev2
{   
    RE_Scalev2(double observed_x, double observed_y, double *intrinsics, double *quaternion, double *translation, double *structure)
    : observed_x(observed_x), observed_y(observed_y), intrinsics(intrinsics), quaternion(quaternion), translation(translation), structure(structure) {}
    
    template <typename T>  // Allow optimization of parameter scale
    bool operator()(const T* const scale_translation, T* residuals) const
    {   
        const T fx = T(intrinsics[0]);
        const T fy = T(intrinsics[1]);
        const T cx = T(intrinsics[2]);
        const T cy = T(intrinsics[3]);
        // Use a quaternion rotation that doesn't assume the quaternion is
        // normalized, since one of the ways to run the bundler is to let Ceres
        // optimize all 4 quaternion parameters unconstrained.
        T p[3];
        const T quat[4] = { T(quaternion[0]), T(quaternion[1]), T(quaternion[2]), T(quaternion[3]) };
        const T *ptr_quat = &quat[0];
        
        const T point[3] = { T(structure[0]), T(structure[1]), T(structure[2]) };
        const T *ptr_point = &point[0];
        
        ceres::QuaternionRotatePoint(ptr_quat, ptr_point, p);
        
        p[0] += T(translation[0])*scale_translation[0];//point[3];
        p[1] += T(translation[1])*scale_translation[0];//point[3];
        p[2] += T(translation[2])*scale_translation[0];//point[3];
        
        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = p[0] / p[2]; // - p[0] / p[2];
        T yp = p[1] / p[2]; // - p[1] / p[2];
        
        // Apply second and fourth order radial distortion.
        //     T r2 = xp*xp + yp*yp;
        //     T distortion = T(1.0) + r2  * (l1 + l2  * r2);
        
        // Compute final projected point position.
        T predicted_x = fx * xp + cx;
        T predicted_y = fy * yp + cy;
        
        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        
        return true;
    }
    
    template <typename T>  // Constrained optimization for scale in translation, just add residual
    bool operator()(const T* const ph1, const T* const ph2, T* residuals) const {

    const T fx = T(intrinsics[0]);
    const T fy = T(intrinsics[1]);
    const T cx = T(intrinsics[2]);
    const T cy = T(intrinsics[3]);
    // Use a quaternion rotation that doesn't assume the quaternion is
    // normalized, since one of the ways to run the bundler is to let Ceres
    // optimize all 4 quaternion parameters unconstrained.
    T p[3];
    const T quat[4] = {T(quaternion[0]), T(quaternion[1]), T(quaternion[2]), T(quaternion[3]) };
    const T *ptr_quat = &quat[0];
    
    const T point[3] = { T(structure[0]), T(structure[1]), T(structure[2]) };
    const T *ptr_point = &point[0];
    
    ceres::QuaternionRotatePoint(ptr_quat, ptr_point, p);

    p[0] += T(translation[0]);//point[3];
    p[1] += T(translation[1]);//point[3];
    p[2] += T(translation[2]);//point[3];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = p[0] / p[2]; // - p[0] / p[2];
    T yp = p[1] / p[2]; // - p[1] / p[2];

    // Apply second and fourth order radial distortion.
//     T r2 = xp*xp + yp*yp;
//     T distortion = T(1.0) + r2  * (l1 + l2  * r2);

    // Compute final projected point position.
    T predicted_x = fx * xp + cx;
    T predicted_y = fy * yp + cy;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);

    return true;
  }
    
    double observed_x;
    double observed_y;
    double *intrinsics;
    double *quaternion;
    double *translation;
    double *structure;
};

// Reprojection error model with Quaternion for 3D projections from Kinect
// Use quaternion as rotation component
// Total parameters adjusted 7: Motion:7
// 4 quaternion (rotation) variables, 3 translation variables , 
struct RE3D
{   
    RE3D(double *pt1, double *pt2) //pt2 = q*pt1*~q + t ; ~q*(p2 - t1)*q = pt1
    : pt1(pt1), pt2(pt2)  {}

    template <typename T>
    bool operator()(const T* const quaternion, const T* const translation, T* residuals) const 
    {
        const T observed1_x = T(pt1[0]);
        const T observed1_y = T(pt1[1]);
        const T observed1_z = T(pt1[2]);
        const T observed2_x = T(pt2[0]);
        const T observed2_y = T(pt2[1]);
        const T observed2_z = T(pt2[2]);
        
        T point1[3] = { T(pt1[0]), T(pt1[1]), T(pt1[2])};
        T p1r[3];
        ceres::QuaternionRotatePoint(quaternion, point1, p1r);
        
        p1r[0] += translation[0];
        p1r[1] += translation[1];
        p1r[2] += translation[2];
        // final projected point position.
        T predicted2_x = p1r[0];
        T predicted2_y = p1r[1];
        T predicted2_z = p1r[2];
        
        T point2[3] = { T(pt2[0]), T(pt2[1]), T(pt2[2])};
        T p2r[3];
        T qconj[4] = {T(quaternion[0]), T(-quaternion[1]), T(-quaternion[2]), T(-quaternion[3]) };
        point2[0] -= translation[0];
        point2[1] -= translation[1];
        point2[2] -= translation[2];
        ceres::QuaternionRotatePoint(qconj, point2, p2r);
        
        // final projected point position.
        T predicted1_x = p2r[0];
        T predicted1_y = p2r[1];
        T predicted1_z = p2r[2];
        
        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted2_x - observed2_x;
        residuals[1] = predicted2_y - observed2_y;
        residuals[2] = predicted2_z - observed2_z;
        residuals[3] = predicted1_x - observed1_x;
        residuals[4] = predicted1_y - observed1_y;
        residuals[5] = predicted1_z - observed1_z;
        
        return true;
  }

  double *pt1;
  double *pt2;

};

struct RE3D_Covariance
{   
    RE3D_Covariance(double *pt1, double *pt2, double *variance1, double *variance2) //pt2 = q*pt1*~q + t ; ~q*(p2 - t1)*q = pt1
    : pt1(pt1), pt2(pt2), variance1(variance1), variance2(variance2) {}

    template <typename T>
    bool operator()(const T* const quaternion, const T* const translation, T* residuals) const 
    {
        const T observed1_x = T(pt1[0]);
        const T observed1_y = T(pt1[1]);
        const T observed1_z = T(pt1[2]);
        const T observed2_x = T(pt2[0]);
        const T observed2_y = T(pt2[1]);
        const T observed2_z = T(pt2[2]);
        
        T point1[3] = { T(pt1[0]), T(pt1[1]), T(pt1[2])};
        T p1r[3];
        ceres::QuaternionRotatePoint(quaternion, point1, p1r);
        
        p1r[0] += translation[0];
        p1r[1] += translation[1];
        p1r[2] += translation[2];
        // final projected point position.
        T predicted2_x = p1r[0];
        T predicted2_y = p1r[1];
        T predicted2_z = p1r[2];
        
        T point2[3] = { T(pt2[0]), T(pt2[1]), T(pt2[2])};
        T p2r[3];
        T qconj[4] = {T(quaternion[0]), T(-quaternion[1]), T(-quaternion[2]), T(-quaternion[3]) };
        point2[0] -= translation[0];
        point2[1] -= translation[1];
        point2[2] -= translation[2];
        ceres::QuaternionRotatePoint(qconj, point2, p2r);
        
        // final projected point position.
        T predicted1_x = p2r[0];
        T predicted1_y = p2r[1];
        T predicted1_z = p2r[2];
        
        // The error is the difference between the predicted and observed position.
        residuals[0] = (predicted2_x - observed2_x)/sqrt(T(variance2[0]));
        residuals[1] = (predicted2_y - observed2_y)/sqrt(T(variance2[1]));
        residuals[2] = (predicted2_z - observed2_z)/sqrt(T(variance2[2]));
        residuals[3] = (predicted1_x - observed1_x)/sqrt(T(variance1[0]));
        residuals[4] = (predicted1_y - observed1_y)/sqrt(T(variance1[1]));
        residuals[5] = (predicted1_z - observed1_z)/sqrt(T(variance1[2]));
        return true;
  }

  double *pt1;
  double *pt2;
  double *variance1;
  double *variance2;

};

struct RE3D_constS_QT_Cov
{
    RE3D_constS_QT_Cov(double *pt_global, double *pt_local, double *variance)
    : pt_global(pt_global), pt_local(pt_local), variance(variance) {}

    template <typename T>
    bool operator()(const T* const quaternion, const T* const translation, T* residuals) const 
    {
        const T observed_x = T(pt_global[0]);
        const T observed_y = T(pt_global[1]);
        const T observed_z = T(pt_global[2]);
        
        T point[3] = { T(pt_local[0]), T(pt_local[1]), T(pt_local[2])};
        T p2r[3];
        T qconj[4] = {T(quaternion[0]), T(-quaternion[1]), T(-quaternion[2]), T(-quaternion[3]) };
        point[0] -= translation[0];
        point[1] -= translation[1];
        point[2] -= translation[2];
        ceres::QuaternionRotatePoint(qconj, point, p2r);
        
        // final projected point position.
        T predicted_x = p2r[0];
        T predicted_y = p2r[1];
        T predicted_z = p2r[2];
        
        // The error is the difference between the predicted and observed position.
        residuals[0] = (predicted_x - observed_x)/sqrt(T(variance[0]));
        residuals[1] = (predicted_y - observed_y)/sqrt(T(variance[1]));
        residuals[2] = (predicted_z - observed_z)/sqrt(T(variance[2]));
        return true;
    }
    
    double *pt_global;
    double *pt_local;
    double *variance;
};

struct RE3D_QTS_Cov
{
    RE3D_QTS_Cov(double *pt_local, double *variance)
    : pt_local(pt_local), variance(variance) {}

    template <typename T>
    bool operator()(const T* const quaternion, const T* const translation, const T* const structure, T* residuals) const 
    {
        const T observed_x = T(structure[0]);
        const T observed_y = T(structure[1]);
        const T observed_z = T(structure[2]);
        
        T point[3] = { T(pt_local[0]), T(pt_local[1]), T(pt_local[2])};
        T p2r[3];
        T qconj[4] = {T(quaternion[0]), T(-quaternion[1]), T(-quaternion[2]), T(-quaternion[3]) };
        point[0] -= translation[0];
        point[1] -= translation[1];
        point[2] -= translation[2];
        ceres::QuaternionRotatePoint(qconj, point, p2r);
        
        // final projected point position.
        T predicted_x = p2r[0];
        T predicted_y = p2r[1];
        T predicted_z = p2r[2];
        
        // The error is the difference between the predicted and observed position.
        residuals[0] = (predicted_x - observed_x)/sqrt(T(variance[0]));
        residuals[1] = (predicted_y - observed_y)/sqrt(T(variance[1]));
        residuals[2] = (predicted_z - observed_z)/sqrt(T(variance[2]));
        return true;
    }
    
    double *pt_local;
    double *variance;
};


// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct SnavelyReprojectionError 
{
  SnavelyReprojectionError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);

    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    const T& focal = camera[6];
    T xp = - p[0] / p[2];
    T yp = - p[1] / p[2];

    // Apply second and fourth order radial distortion.
    const T& l1 = camera[7];
    const T& l2 = camera[8];
    T r2 = xp*xp + yp*yp;
    T distortion = T(1.0) + r2  * (l1 + l2  * r2);

    // Compute final projected point position.
    T predicted_x = focal * distortion * xp;
    T predicted_y = focal * distortion * yp;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);

    return true;
  }

  double observed_x;
  double observed_y;
};

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 10 parameters. 4 for rotation, 3 for
// translation, 1 for focal length and 2 for radial distortion. The
// principal point is not modeled (i.e. it is assumed be located at
// the image center).
struct Snavely_RE_KDQTS 
{
  // (u, v): the position of the observation with respect to the image
  // center point.
  Snavely_RE_KDQTS(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const camera_rotation,
                  const T* const translation_and_intrinsics,
                  const T* const point,
                  T* residuals) const {
    const T& focal = translation_and_intrinsics[3];
    const T& l1 = translation_and_intrinsics[4];
    const T& l2 = translation_and_intrinsics[5];

    // Use a quaternion rotation that doesn't assume the quaternion is
    // normalized, since one of the ways to run the bundler is to let Ceres
    // optimize all 4 quaternion parameters unconstrained.
    T p[3];
    ceres::QuaternionRotatePoint(camera_rotation, point, p);

    p[0] += translation_and_intrinsics[0];
    p[1] += translation_and_intrinsics[1];
    p[2] += translation_and_intrinsics[2];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = - p[0] / p[2];
    T yp = - p[1] / p[2];

    // Apply second and fourth order radial distortion.
    T r2 = xp*xp + yp*yp;
    T distortion = T(1.0) + r2  * (l1 + l2  * r2);

    // Compute final projected point position.
    T predicted_x = focal * distortion * xp;
    T predicted_y = focal * distortion * yp;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);

    return true;
  }

  double observed_x;
  double observed_y;
};

#endif
