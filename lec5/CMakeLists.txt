cmake_minimum_required( VERSION 2.8 )
project(orb)
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Sophus
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable( orbextrack computeORB.cpp )
target_link_libraries( orbextrack ${OpenCV_LIBS} )

 
add_executable( E2Rt E2Rt.cpp )
target_link_libraries( E2Rt ${Sophus_LIBRARIES} )

add_executable( BuddleJ GN-BA.cpp )
target_link_libraries( BuddleJ ${Sophus_LIBRARIES} )

add_executable( ORB ORB.cpp )
target_link_libraries( ORB ${OpenCV_LIBS} )
