# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/yuanlin/homework/drawTrajectory

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yuanlin/homework/drawTrajectory/build

# Include any dependencies generated for this target.
include CMakeFiles/drawTraJectory.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/drawTraJectory.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/drawTraJectory.dir/flags.make

CMakeFiles/drawTraJectory.dir/mse.cpp.o: CMakeFiles/drawTraJectory.dir/flags.make
CMakeFiles/drawTraJectory.dir/mse.cpp.o: ../mse.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yuanlin/homework/drawTrajectory/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/drawTraJectory.dir/mse.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drawTraJectory.dir/mse.cpp.o -c /home/yuanlin/homework/drawTrajectory/mse.cpp

CMakeFiles/drawTraJectory.dir/mse.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drawTraJectory.dir/mse.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yuanlin/homework/drawTrajectory/mse.cpp > CMakeFiles/drawTraJectory.dir/mse.cpp.i

CMakeFiles/drawTraJectory.dir/mse.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drawTraJectory.dir/mse.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yuanlin/homework/drawTrajectory/mse.cpp -o CMakeFiles/drawTraJectory.dir/mse.cpp.s

CMakeFiles/drawTraJectory.dir/mse.cpp.o.requires:

.PHONY : CMakeFiles/drawTraJectory.dir/mse.cpp.o.requires

CMakeFiles/drawTraJectory.dir/mse.cpp.o.provides: CMakeFiles/drawTraJectory.dir/mse.cpp.o.requires
	$(MAKE) -f CMakeFiles/drawTraJectory.dir/build.make CMakeFiles/drawTraJectory.dir/mse.cpp.o.provides.build
.PHONY : CMakeFiles/drawTraJectory.dir/mse.cpp.o.provides

CMakeFiles/drawTraJectory.dir/mse.cpp.o.provides.build: CMakeFiles/drawTraJectory.dir/mse.cpp.o


# Object files for target drawTraJectory
drawTraJectory_OBJECTS = \
"CMakeFiles/drawTraJectory.dir/mse.cpp.o"

# External object files for target drawTraJectory
drawTraJectory_EXTERNAL_OBJECTS =

drawTraJectory: CMakeFiles/drawTraJectory.dir/mse.cpp.o
drawTraJectory: CMakeFiles/drawTraJectory.dir/build.make
drawTraJectory: /home/yuanlin/3rdlib/Sophus/build/libSophus.so
drawTraJectory: /usr/local/lib/libpangolin.so
drawTraJectory: /usr/lib/x86_64-linux-gnu/libGLU.so
drawTraJectory: /usr/lib/x86_64-linux-gnu/libGL.so
drawTraJectory: /usr/lib/x86_64-linux-gnu/libGLEW.so
drawTraJectory: /usr/lib/x86_64-linux-gnu/libSM.so
drawTraJectory: /usr/lib/x86_64-linux-gnu/libICE.so
drawTraJectory: /usr/lib/x86_64-linux-gnu/libX11.so
drawTraJectory: /usr/lib/x86_64-linux-gnu/libXext.so
drawTraJectory: /usr/lib/x86_64-linux-gnu/libpython2.7.so
drawTraJectory: /usr/lib/x86_64-linux-gnu/libavcodec.so
drawTraJectory: /usr/lib/x86_64-linux-gnu/libavformat.so
drawTraJectory: /usr/lib/x86_64-linux-gnu/libavutil.so
drawTraJectory: /usr/lib/x86_64-linux-gnu/libswscale.so
drawTraJectory: /usr/lib/libOpenNI.so
drawTraJectory: /usr/lib/x86_64-linux-gnu/libpng.so
drawTraJectory: /usr/lib/x86_64-linux-gnu/libz.so
drawTraJectory: /usr/lib/x86_64-linux-gnu/libjpeg.so
drawTraJectory: /usr/lib/x86_64-linux-gnu/libtiff.so
drawTraJectory: CMakeFiles/drawTraJectory.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yuanlin/homework/drawTrajectory/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable drawTraJectory"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drawTraJectory.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/drawTraJectory.dir/build: drawTraJectory

.PHONY : CMakeFiles/drawTraJectory.dir/build

CMakeFiles/drawTraJectory.dir/requires: CMakeFiles/drawTraJectory.dir/mse.cpp.o.requires

.PHONY : CMakeFiles/drawTraJectory.dir/requires

CMakeFiles/drawTraJectory.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/drawTraJectory.dir/cmake_clean.cmake
.PHONY : CMakeFiles/drawTraJectory.dir/clean

CMakeFiles/drawTraJectory.dir/depend:
	cd /home/yuanlin/homework/drawTrajectory/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yuanlin/homework/drawTrajectory /home/yuanlin/homework/drawTrajectory /home/yuanlin/homework/drawTrajectory/build /home/yuanlin/homework/drawTrajectory/build /home/yuanlin/homework/drawTrajectory/build/CMakeFiles/drawTraJectory.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/drawTraJectory.dir/depend

