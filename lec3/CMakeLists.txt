cmake_minimum_required(VERSION 2.8)
project(drawTraJectory)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package(Pangolin REQUIRED)
find_package( Sophus REQUIRED )

include_directories(
     ${Sophus_INCLUDE_DIRS}
     ${Pangolin_INCLUDE_DIRS}
      )

add_executable(drawTraJectory draw_trajectory.cpp)
target_link_libraries( drawTraJectory
${Sophus_LIBRARIES} 
${Pangolin_LIBRARIES}
)
add_executable(mse mse.cpp)
target_link_libraries( mse
${Sophus_LIBRARIES} 
${Pangolin_LIBRARIES}
)