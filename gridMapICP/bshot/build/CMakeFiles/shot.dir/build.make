# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.9

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/marsa/Project/gridMapICP/bshot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/marsa/Project/gridMapICP/bshot/build

# Include any dependencies generated for this target.
include CMakeFiles/shot.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/shot.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/shot.dir/flags.make

CMakeFiles/shot.dir/src/shot.cpp.o: CMakeFiles/shot.dir/flags.make
CMakeFiles/shot.dir/src/shot.cpp.o: ../src/shot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/marsa/Project/gridMapICP/bshot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/shot.dir/src/shot.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/shot.dir/src/shot.cpp.o -c /home/marsa/Project/gridMapICP/bshot/src/shot.cpp

CMakeFiles/shot.dir/src/shot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/shot.dir/src/shot.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/marsa/Project/gridMapICP/bshot/src/shot.cpp > CMakeFiles/shot.dir/src/shot.cpp.i

CMakeFiles/shot.dir/src/shot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/shot.dir/src/shot.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/marsa/Project/gridMapICP/bshot/src/shot.cpp -o CMakeFiles/shot.dir/src/shot.cpp.s

CMakeFiles/shot.dir/src/shot.cpp.o.requires:

.PHONY : CMakeFiles/shot.dir/src/shot.cpp.o.requires

CMakeFiles/shot.dir/src/shot.cpp.o.provides: CMakeFiles/shot.dir/src/shot.cpp.o.requires
	$(MAKE) -f CMakeFiles/shot.dir/build.make CMakeFiles/shot.dir/src/shot.cpp.o.provides.build
.PHONY : CMakeFiles/shot.dir/src/shot.cpp.o.provides

CMakeFiles/shot.dir/src/shot.cpp.o.provides.build: CMakeFiles/shot.dir/src/shot.cpp.o


# Object files for target shot
shot_OBJECTS = \
"CMakeFiles/shot.dir/src/shot.cpp.o"

# External object files for target shot
shot_EXTERNAL_OBJECTS =

shot: CMakeFiles/shot.dir/src/shot.cpp.o
shot: CMakeFiles/shot.dir/build.make
shot: /usr/lib/x86_64-linux-gnu/libboost_system.so
shot: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
shot: /usr/lib/x86_64-linux-gnu/libboost_thread.so
shot: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
shot: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
shot: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
shot: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
shot: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
shot: /usr/lib/x86_64-linux-gnu/libboost_regex.so
shot: /usr/local/lib/libpcl_common.so
shot: /usr/local/lib/libpcl_octree.so
shot: /usr/lib/libOpenNI.so
shot: /usr/lib/libOpenNI2.so
shot: /usr/local/lib/libpcl_io.so
shot: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
shot: /usr/local/lib/libpcl_kdtree.so
shot: /usr/local/lib/libpcl_search.so
shot: /usr/local/lib/libpcl_sample_consensus.so
shot: /usr/local/lib/libpcl_filters.so
shot: /usr/local/lib/libpcl_features.so
shot: /usr/local/lib/libpcl_ml.so
shot: /usr/local/lib/libpcl_segmentation.so
shot: /usr/local/lib/libpcl_visualization.so
shot: /usr/lib/x86_64-linux-gnu/libqhull.so
shot: /usr/local/lib/libpcl_surface.so
shot: /usr/local/lib/libpcl_registration.so
shot: /usr/local/lib/libpcl_keypoints.so
shot: /usr/local/lib/libpcl_tracking.so
shot: /usr/local/lib/libpcl_recognition.so
shot: /usr/local/lib/libpcl_stereo.so
shot: /usr/local/lib/libpcl_outofcore.so
shot: /usr/local/lib/libpcl_people.so
shot: /usr/lib/x86_64-linux-gnu/libboost_system.so
shot: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
shot: /usr/lib/x86_64-linux-gnu/libboost_thread.so
shot: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
shot: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
shot: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
shot: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
shot: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
shot: /usr/lib/x86_64-linux-gnu/libboost_regex.so
shot: /usr/lib/x86_64-linux-gnu/libqhull.so
shot: /usr/lib/libOpenNI.so
shot: /usr/lib/libOpenNI2.so
shot: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
shot: /usr/lib/libvtkGenericFiltering.so.5.8.0
shot: /usr/lib/libvtkGeovis.so.5.8.0
shot: /usr/lib/libvtkCharts.so.5.8.0
shot: /usr/local/lib/libpcl_common.so
shot: /usr/local/lib/libpcl_octree.so
shot: /usr/local/lib/libpcl_io.so
shot: /usr/local/lib/libpcl_kdtree.so
shot: /usr/local/lib/libpcl_search.so
shot: /usr/local/lib/libpcl_sample_consensus.so
shot: /usr/local/lib/libpcl_filters.so
shot: /usr/local/lib/libpcl_features.so
shot: /usr/local/lib/libpcl_ml.so
shot: /usr/local/lib/libpcl_segmentation.so
shot: /usr/local/lib/libpcl_visualization.so
shot: /usr/local/lib/libpcl_surface.so
shot: /usr/local/lib/libpcl_registration.so
shot: /usr/local/lib/libpcl_keypoints.so
shot: /usr/local/lib/libpcl_tracking.so
shot: /usr/local/lib/libpcl_recognition.so
shot: /usr/local/lib/libpcl_stereo.so
shot: /usr/local/lib/libpcl_outofcore.so
shot: /usr/local/lib/libpcl_people.so
shot: /usr/lib/libvtkViews.so.5.8.0
shot: /usr/lib/libvtkInfovis.so.5.8.0
shot: /usr/lib/libvtkWidgets.so.5.8.0
shot: /usr/lib/libvtkVolumeRendering.so.5.8.0
shot: /usr/lib/libvtkHybrid.so.5.8.0
shot: /usr/lib/libvtkParallel.so.5.8.0
shot: /usr/lib/libvtkRendering.so.5.8.0
shot: /usr/lib/libvtkImaging.so.5.8.0
shot: /usr/lib/libvtkGraphics.so.5.8.0
shot: /usr/lib/libvtkIO.so.5.8.0
shot: /usr/lib/libvtkFiltering.so.5.8.0
shot: /usr/lib/libvtkCommon.so.5.8.0
shot: /usr/lib/libvtksys.so.5.8.0
shot: CMakeFiles/shot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/marsa/Project/gridMapICP/bshot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable shot"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/shot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/shot.dir/build: shot

.PHONY : CMakeFiles/shot.dir/build

CMakeFiles/shot.dir/requires: CMakeFiles/shot.dir/src/shot.cpp.o.requires

.PHONY : CMakeFiles/shot.dir/requires

CMakeFiles/shot.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/shot.dir/cmake_clean.cmake
.PHONY : CMakeFiles/shot.dir/clean

CMakeFiles/shot.dir/depend:
	cd /home/marsa/Project/gridMapICP/bshot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/marsa/Project/gridMapICP/bshot /home/marsa/Project/gridMapICP/bshot /home/marsa/Project/gridMapICP/bshot/build /home/marsa/Project/gridMapICP/bshot/build /home/marsa/Project/gridMapICP/bshot/build/CMakeFiles/shot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/shot.dir/depend
