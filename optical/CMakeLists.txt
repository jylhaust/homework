cmake_minimum_required( VERSION 2.8 )
project(optical)
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Sophus
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

# Pangolin
find_package(Pangolin REQUIRED)
include_directories( ${Pangolin_INCLUDE_DIRS} )
        
add_executable( optical_flow optical_flow.cpp )
target_link_libraries( optical_flow 
${OpenCV_LIBS}
 )

add_executable( direct_method direct_method.cpp )
target_link_libraries( direct_method
 ${Sophus_LIBRARIES}
 ${Pangolin_LIBRARIES}
 ${OpenCV_LIBS}
  )