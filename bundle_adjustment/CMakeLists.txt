cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project( BA_Project )

## ***** Find Main Packages *****
# OpenCV
find_package( OpenCV REQUIRED )

# OPENGL
find_package( OpenGL REQUIRED )
set(OPENGL_USED_LIBRARIES "GL" "GLU" "glut")

# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# VTK
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

# Ceres
find_package( Ceres REQUIRED )
include_directories(${CERES_INCLUDES})

# Threads (pthread)
set(THREAD_LIBRARIES "pthread")

# Boost
# find_package(Boost REQUIRED)
set(Boost_LIBRARIES "boost_system" "boost_chrono")

## ***** Other LIBS *****
# SiftGPU
set(SiftGPU_LIBRARIES "siftgpu")

# SQLITE3
set(SQLITE3_LIBRARIES "sqlite3")

## ***** Local includes *****
# Local include
set(LOCAL_DIR "./core/")
include_directories(${LOCAL_DIR})

file(GLOB_RECURSE INCS "*.hpp")
file(GLOB_RECURSE SRCS "core/*.cpp")

# Comprising all targets to link
set(LINK_TARGETS ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} ${Eigen3_LIBS} ${CERES_LIBRARIES} ${OPENGL_USED_LIBRARIES} 
	        ${Boost_LIBRARIES} ${SiftGPU_LIBRARIES} ${THREAD_LIBRARIES} ${SQLITE3_LIBRARIES})

# Compilation flags
ADD_DEFINITIONS(-std=c++11)

# General Library Compilation
add_library ( mop ${INCS} ${SRCS} )
target_link_libraries (mop ${LINK_TARGETS})

# Executables

# Pose tests
# add_executable( test_pose test_pose.cpp)
# target_link_libraries( test_pose "mop" ${LINK_TARGETS} )

# Triangulation test
# add_executable( test_triangulation test_triangulation.cpp Common.hpp Common.cpp Triangulation.hpp Triangulation.cpp ) #timer_wall.hpp timer_wall.cpp)
# target_link_libraries( test_triangulation ${OpenCV_LIBS} ${Eigen3_LIBS} ) ${Boost_LIBRARIES})

# test algorithm
# add_executable( test_algorithm test_algorithm.cpp Common.hpp Common.cpp CameraPose.hpp CameraPose.cpp Interface.hpp Interface.cpp )
# target_link_libraries( test_algorithm ${OpenCV_LIBS} ${Eigen3_LIBS} ${Boost_LIBRARIES} ${SQLITE3_LIBRARIES} ${PCL_LIBRARIES})

# Initialization of BA test
add_executable( test_initBA test_initBA.cpp )
target_link_libraries( test_initBA "mop" ${LINK_TARGETS} )

# Bundle Adjustment in the Large
add_executable( bal bal.cpp )
target_link_libraries( bal "mop" ${LINK_TARGETS} )

# Synthetic Bundle Adjustment
add_executable( synthetic synthetic.cpp )
target_link_libraries( synthetic "mop" ${LINK_TARGETS} )

# test synthetic
# add_executable( test_synthetic test_synthetic.cpp )
# target_link_libraries( test_synthetic "mop" ${LINK_TARGETS} )