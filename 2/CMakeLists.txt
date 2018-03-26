cmake_minimum_required( VERSION 2.8 )
project(hello)
set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )

include_directories(
${PROJECT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME} SHARED
src/hello.cpp
)

add_executable(sayhello useHello.cpp)
target_link_libraries(sayhello ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
	LIBRARY DESTINATION /usr/local/lib)
install(FILES 
	${PROJECT_SOURCE_DIR}/include/hello.h 
	DESTINATION /usr/local/include)	

# ${PROJECT_NAME}