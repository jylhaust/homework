add_library(hello SHARED hello.cpp)
include_directories( ${PROJECT_SOURCE_DIR}/include )

install(TARGETS hello
	LIBRARY DESTINATION local/lib)

