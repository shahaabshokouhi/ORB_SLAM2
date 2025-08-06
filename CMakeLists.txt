cmake_minimum_required(VERSION 3.5)
project(ORB_SLAM2)

# 1) Build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

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
find_package(OpenCV    REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)      # header-only
find_package(Pangolin  REQUIRED)

# --- manually locate Intel librealsense2 SDK ---
find_path( REALSENSE2_INCLUDE_DIR
  NAMES rs.hpp
  PATHS /usr/include/librealsense2 /usr/local/include/librealsense2
)
find_library( REALSENSE2_LIBRARY
  NAMES realsense2
  PATHS /usr/lib/aarch64-linux-gnu /usr/lib /usr/local/lib
)
if(NOT REALSENSE2_INCLUDE_DIR OR NOT REALSENSE2_LIBRARY)
  message(FATAL_ERROR "Could not find the Intel librealsense2 SDK. "
                      "Install librealsense2-dev and rerun CMake.")
endif()

# 6) Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${REALSENSE2_INCLUDE_DIR}
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
  Eigen3::Eigen               # header-only
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

# 9) Examples

## RGB-D (TUM)
set_target_properties(ORB_SLAM2 PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D
)
add_executable(rgbd_tum        Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum
  ORB_SLAM2
  ${REALSENSE2_LIBRARY}
)

## RGB-D Live (RealSense)
add_executable(rgbd_live       Examples/RGB-D/rgbd_live.cc)
target_link_libraries(rgbd_live
  ORB_SLAM2
  ${REALSENSE2_LIBRARY}
)

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

# 10) system-wide install
install(
  TARGETS ORB_SLAM2
  EXPORT ORB_SLAM2Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/include/
  DESTINATION include/ORB_SLAM2
  FILES_MATCHING PATTERN "*.h"
)
