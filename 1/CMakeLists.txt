cmake_minimum_required( VERSION 2.8 )
project(sayHello)
set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )

add_subdirectory(src)

install(FILES include/hello.h DESTINATION local/include)

include_directories(/usr/local/inlude/hello)

#add_executable(sayhello useHello.cpp)

#target_link_libraries(sayhello libhello.so)
