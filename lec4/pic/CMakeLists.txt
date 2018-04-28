cmake_minimum_required( VERSION 2.8 )
project( undistortion )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-O3" )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


add_executable( undistortion undistort_image.cpp )
target_link_libraries( undistortion ${OpenCV_LIBS} )
