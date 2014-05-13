Motion Optimization Package

Author: José David Tascón Vidarte

This package contains a set of functions and classes for Range Registration with RGB-D sensor and Structure from Motion.


Dependencies:
_____________________________________________________________
Library				Recomended version
_____________________________________________________________
siftgpu					0.5.4
eigen					3.2
sqlite					3.8
opencv					2.4
boost					1.55
vtk					5.10
pcl					1.7
ceres-solver 				1.8

_____________________________________________________________

Details:

./core/			Main files .cpp and .hpp of mop library
./bundle_adjustment/	Files for SfM and Bundle Adjustment
./range3d/		Files for range registration

Range Registration, Example:
1- Check dependencies of CMakeLists.txt
2- Compile the file CMakeLists.txt with cmake.
3- Run an example: 
    $ ./range_model -i images.xml -d depth.xml -k calibration_kinect.txt -df 40