cmake_minimum_required( VERSION 2.8 )
project( disparity )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# pangolin
find_package(Pangolin REQUIRED)
include_directories( ${Pangolin_INCLUDE_DIRS} )

# eigen 
include_directories( "/usr/include/eigen3/" )

add_executable( disparity disparity.cpp )

target_link_libraries( disparity 
${OpenCV_LIBS}
${Pangolin_LIBRARIES}
 )
