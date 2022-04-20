cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_C_COMPILER "gcc")

set(CMAKE_BUILD_TYPE Release)
set(ROS_BUILD_TYPE Debug)
IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -std=c++14  -O3  ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++14  -O3")
#set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
ADD_COMPILE_OPTIONS(-std=c++14 )

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# author Bo guan
# file(GLOB librealsense2 /opt/ros/noetic/lib/x86_64-linux-gnu/librealsense*)
# set(Realsense2_INCLUDE_DIR /opt/ros/noetic/include/librealsense2)

# author Yuelin Zou
file(GLOB librealsense2 /usr/local/lib/librealsense*)
set(Realsense2_INCLUDE_DIR /usr/local/include/librealsense2)

find_package(OpenCV 4.5.3 EXACT REQUIRED)

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.3.7 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(realsense2)

find_package( G2O REQUIRED )
find_package(VTK 8.1.2 REQUIRED) #8.1.2
# Sophus
find_package( Sophus REQUIRED)

# PCL
find_package(PCL 1.11.0 REQUIRED)
add_definitions( ${PCL_DEFINITIONS} )
file(GLOB PCL_LIBRARIES /usr/local/lib/libpcl_*1.11)
set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.11)

#file(GLOB PCL_LIBRARIES /usr/lib/x86_64-linux-gnu/libpcl_*) #
#set(PCL_INCLUDE_DIRS /usr/include/pcl-1.10/) #

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/src
${PROJECT_SOURCE_DIR}/src/ORB-SLAM3/include
${PROJECT_SOURCE_DIR}/src/ORB-SLAM3/
${PROJECT_SOURCE_DIR}/src/ORB-SLAM3/include/CameraModels
${Realsense2_INCLUDE_DIR}
${G2O_INCLUDE_DIR}
${VTK_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS}
${Sophus_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

set(LIBS 
${OpenCV_LIBS}
${librealsense2}
${EIGEN3_LIBS}
${PCL_LIBRARIES}
${VTK_LIBRARIES}
${Pangolin_LIBRARIES}
${Sophus_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sim3 g2o_types_sba
-lboost_system
-lcrypto
)

# Node for RGB-D camera
rosbuild_add_executable(MedicalAR
src/MedicalAR/medicalSlam.cc
src/MedicalAR/vtkVisual.cpp
src/MedicalAR/vtkVisual.h
src/MedicalAR/pclVisual.h
src/MedicalAR/pclVisual.cpp
src/MedicalAR/SelfInteractorStyle.h
src/MedicalAR/SelfInteractorStyle.cpp
src/MedicalAR/cameraRender.h
src/MedicalAR/cameraRender.cpp

src/ORB-SLAM3/src/System.cc
src/ORB-SLAM3/src/Tracking.cc
src/ORB-SLAM3/src/LocalMapping.cc
src/ORB-SLAM3/src/LoopClosing.cc
src/ORB-SLAM3/src/ORBextractor.cc
src/ORB-SLAM3/src/ORBmatcher.cc
src/ORB-SLAM3/src/FrameDrawer.cc
src/ORB-SLAM3/src/Converter.cc
src/ORB-SLAM3/src/MapPoint.cc
src/ORB-SLAM3/src/KeyFrame.cc
src/ORB-SLAM3/src/Atlas.cc
src/ORB-SLAM3/src/Map.cc
src/ORB-SLAM3/src/MapDrawer.cc
src/ORB-SLAM3/src/Optimizer.cc
src/ORB-SLAM3/src/Frame.cc
src/ORB-SLAM3/src/KeyFrameDatabase.cc
src/ORB-SLAM3/src/Sim3Solver.cc
src/ORB-SLAM3/src/Initializer.cc
src/ORB-SLAM3/src/Viewer.cc
src/ORB-SLAM3/src/ImuTypes.cc
src/ORB-SLAM3/src/G2oTypes.cc
src/ORB-SLAM3/src/CameraModels/Pinhole.cpp
src/ORB-SLAM3/src/CameraModels/KannalaBrandt8.cpp
src/ORB-SLAM3/src/OptimizableTypes.cpp
src/ORB-SLAM3/src/MLPnPsolver.cpp
src/ORB-SLAM3/src/TwoViewReconstruction.cc
src/ORB-SLAM3/src/pointcloudmapping.cc
src/ORB-SLAM3/include/System.h
src/ORB-SLAM3/include/Tracking.h
src/ORB-SLAM3/include/LocalMapping.h
src/ORB-SLAM3/include/LoopClosing.h
src/ORB-SLAM3/include/ORBextractor.h
src/ORB-SLAM3/include/ORBmatcher.h
src/ORB-SLAM3/include/FrameDrawer.h
src/ORB-SLAM3/include/Converter.h
src/ORB-SLAM3/include/MapPoint.h
src/ORB-SLAM3/include/KeyFrame.h
src/ORB-SLAM3/include/Atlas.h
src/ORB-SLAM3/include/Map.h
src/ORB-SLAM3/include/MapDrawer.h
src/ORB-SLAM3/include/Optimizer.h
src/ORB-SLAM3/include/Frame.h
src/ORB-SLAM3/include/KeyFrameDatabase.h
src/ORB-SLAM3/include/Sim3Solver.h
src/ORB-SLAM3/include/Initializer.h
src/ORB-SLAM3/include/Viewer.h
src/ORB-SLAM3/include/ImuTypes.h
src/ORB-SLAM3/include/G2oTypes.h
src/ORB-SLAM3/include/CameraModels/GeometricCamera.h
src/ORB-SLAM3/include/CameraModels/Pinhole.h
src/ORB-SLAM3/include/CameraModels/KannalaBrandt8.h
src/ORB-SLAM3/include/OptimizableTypes.h
src/ORB-SLAM3/include/MLPnPsolver.h
src/ORB-SLAM3/include/TwoViewReconstruction.h
src/ORB-SLAM3/include/Config.h
src/ORB-SLAM3/include/pointcloudmapping.h
)

target_link_libraries(MedicalAR
${LIBS}
)

message(STATUS "******the opencv version is :${OpenCV_VERSION}")
