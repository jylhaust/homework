cmake_minimum_required( VERSION 2.8 )
project(orb)
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( orbextrack computeORB.cpp )

target_link_libraries( orbextrack 
${OpenCV_LIBS}
 )