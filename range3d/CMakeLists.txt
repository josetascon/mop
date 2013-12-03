cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Range3D)

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
# set(Boost_LIBRARIES "boost_chrono")

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
set(SRCS 
    "core/Common.cpp"
    "core/CameraPose.cpp"
    "core/Triangulation.cpp"
    "core/DepthProjection.cpp"
    "core/Interface.cpp"
    "core/InterfacePCL.cpp"
    "core/FeaturesEDM.cpp"
    "core/MultipleCamera.cpp"
    "core/Optimizer.cpp"
    "core/Plot.cpp"
)

# Compilation flags
ADD_DEFINITIONS(-std=c++11)

# add_executable (capture_pcd capture_pcd.cpp)
# target_link_libraries (capture_pcd ${PCL_LIBRARIES})
# 
# add_executable (sparse2dense sparse2dense.cpp)
# target_link_libraries (sparse2dense ${PCL_LIBRARIES})

# add_executable( test_cloud test_cloud.cpp ${INCS} ${SRCS})
# target_link_libraries( test_cloud ${OpenCV_LIBS} ${Eigen3_LIBS} ${CERES_LIBRARIES} ${OPENGL_USED_LIBRARIES} ${Boost_LIBRARIES}
# 			      ${SiftGPU_LIBRARIES} ${THREAD_LIBRARIES} ${SQLITE3_LIBRARIES} ${PCL_LIBRARIES})

add_executable (range_model range_model.cpp ${INCS} ${SRCS})
target_link_libraries (range_model ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} ${Eigen3_LIBS} ${CERES_LIBRARIES} ${OPENGL_USED_LIBRARIES} 
			      ${Boost_LIBRARIES} ${SiftGPU_LIBRARIES} ${THREAD_LIBRARIES} ${SQLITE3_LIBRARIES} )

add_executable (synthetic_range synthetic_range.cpp ${INCS} ${SRCS})
target_link_libraries (synthetic_range ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} ${Eigen3_LIBS} ${CERES_LIBRARIES} ${OPENGL_USED_LIBRARIES} 
			      ${Boost_LIBRARIES} ${SiftGPU_LIBRARIES} ${THREAD_LIBRARIES} ${SQLITE3_LIBRARIES})