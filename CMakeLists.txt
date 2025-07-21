cmake_minimum_required(VERSION 3.5)
project(ORB_SLAM2)

# 1) Build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message("Build type: ${CMAKE_BUILD_TYPE}")

# 2) Compiler flags
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

# 3) Enforce C++11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  add_definitions(-DCOMPILEDWITHC11)
  message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCOMPILEDWITHC0X)
  message(STATUS "Using flag -std=c++0x.")
else()
  message(FATAL_ERROR "Your compiler has no C++11 support.")
endif()

# 4) Modules path (for Pangolin, etc)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

# 5) Find dependencies
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)      # header-only imported target
find_package(Pangolin REQUIRED)
find_package(realsense2 REQUIRED)

# 6) Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # Eigen3::Eigen automatically provides its include path, so no manual EIGEN3_INCLUDE_DIR needed
)

# 7) Output directories
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples)

# 8) Core SLAM library
add_library(ORB_SLAM2 SHARED
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/FrameDrawer.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Map.cc
  src/MapDrawer.cc
  src/Optimizer.cc
  src/PnPsolver.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/Initializer.cc
  src/Viewer.cc
)

target_link_libraries(ORB_SLAM2
  Eigen3::Eigen                          # header-only
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

# 9) Examples

## RGB-D
set_target_properties(ORB_SLAM2 PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D
)
add_executable(rgbd_tum        Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ORB_SLAM2)
target_link_libraries(rgbd_tum realsense2::realsense2)

## RGB-D with RealSense
add_executable(rgbd_live Examples/RGB-D/rgbd_live.cc)
target_link_libraries(rgbd_live ORB_SLAM2)
target_link_libraries(rgbd_live realsense2::realsense2)

## RGB-D multi-cam
add_executable(multi_cam_rgbd Examples/RGB-D/multi_cam_rgbd.cc)
target_link_libraries(multi_cam_rgbd ORB_SLAM2)
target_link_libraries(multi_cam_rgbd realsense2::realsense2)

## Stereo
set_target_properties(ORB_SLAM2 PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo
)
add_executable(stereo_kitti    Examples/Stereo/stereo_kitti.cc)
add_executable(stereo_euroc    Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_kitti ORB_SLAM2)
target_link_libraries(stereo_euroc ORB_SLAM2)

## Monocular
set_target_properties(ORB_SLAM2 PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular
)
add_executable(mono_tum        Examples/Monocular/mono_tum.cc)
add_executable(mono_kitti      Examples/Monocular/mono_kitti.cc)
add_executable(mono_euroc      Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_tum   ORB_SLAM2)
target_link_libraries(mono_kitti ORB_SLAM2)
target_link_libraries(mono_euroc ORB_SLAM2)
