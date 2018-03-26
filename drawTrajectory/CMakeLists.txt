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

#add_executable(drawTraJectory draw_trajectory.cpp)
add_executable(drawTraJectory mse.cpp)

target_link_libraries( ${PROJECT_NAME} 
${Sophus_LIBRARIES} 
${Pangolin_LIBRARIES}
)

#target_link_libraries(rdrawTraJectory ${PROJECT_NAME})
