# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_COMMAND = /home/leo/Downloads/clion-2017.1.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/leo/Downloads/clion-2017.1.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/leo/Desktop/learn_rgbdsalm/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leo/Desktop/learn_rgbdsalm/src/cmake-build-debug

# Include any dependencies generated for this target.
include rgbd_salm/CMakeFiles/rgbd_salm_node.dir/depend.make

# Include the progress variables for this target.
include rgbd_salm/CMakeFiles/rgbd_salm_node.dir/progress.make

# Include the compile flags for this target's objects.
include rgbd_salm/CMakeFiles/rgbd_salm_node.dir/flags.make

rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.o: rgbd_salm/CMakeFiles/rgbd_salm_node.dir/flags.make
rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.o: ../rgbd_salm/src/slam_base.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/Desktop/learn_rgbdsalm/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.o"
	cd /home/leo/Desktop/learn_rgbdsalm/src/cmake-build-debug/rgbd_salm && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.o -c /home/leo/Desktop/learn_rgbdsalm/src/rgbd_salm/src/slam_base.cpp

rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.i"
	cd /home/leo/Desktop/learn_rgbdsalm/src/cmake-build-debug/rgbd_salm && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/Desktop/learn_rgbdsalm/src/rgbd_salm/src/slam_base.cpp > CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.i

rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.s"
	cd /home/leo/Desktop/learn_rgbdsalm/src/cmake-build-debug/rgbd_salm && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/Desktop/learn_rgbdsalm/src/rgbd_salm/src/slam_base.cpp -o CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.s

rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.o.requires:

.PHONY : rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.o.requires

rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.o.provides: rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.o.requires
	$(MAKE) -f rgbd_salm/CMakeFiles/rgbd_salm_node.dir/build.make rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.o.provides.build
.PHONY : rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.o.provides

rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.o.provides.build: rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.o


rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.o: rgbd_salm/CMakeFiles/rgbd_salm_node.dir/flags.make
rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.o: ../rgbd_salm/src/slam.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leo/Desktop/learn_rgbdsalm/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.o"
	cd /home/leo/Desktop/learn_rgbdsalm/src/cmake-build-debug/rgbd_salm && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.o -c /home/leo/Desktop/learn_rgbdsalm/src/rgbd_salm/src/slam.cpp

rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.i"
	cd /home/leo/Desktop/learn_rgbdsalm/src/cmake-build-debug/rgbd_salm && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leo/Desktop/learn_rgbdsalm/src/rgbd_salm/src/slam.cpp > CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.i

rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.s"
	cd /home/leo/Desktop/learn_rgbdsalm/src/cmake-build-debug/rgbd_salm && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leo/Desktop/learn_rgbdsalm/src/rgbd_salm/src/slam.cpp -o CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.s

rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.o.requires:

.PHONY : rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.o.requires

rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.o.provides: rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.o.requires
	$(MAKE) -f rgbd_salm/CMakeFiles/rgbd_salm_node.dir/build.make rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.o.provides.build
.PHONY : rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.o.provides

rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.o.provides.build: rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.o


# Object files for target rgbd_salm_node
rgbd_salm_node_OBJECTS = \
"CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.o" \
"CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.o"

# External object files for target rgbd_salm_node
rgbd_salm_node_EXTERNAL_OBJECTS =

rgbd_salm/rgbd_salm_node: rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.o
rgbd_salm/rgbd_salm_node: rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.o
rgbd_salm/rgbd_salm_node: rgbd_salm/CMakeFiles/rgbd_salm_node.dir/build.make
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libcv_bridge.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libpcl_ros_filters.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libpcl_ros_io.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libpcl_ros_tf.so
rgbd_salm/rgbd_salm_node: /usr/lib/libpcl_common.so
rgbd_salm/rgbd_salm_node: /usr/lib/libpcl_octree.so
rgbd_salm/rgbd_salm_node: /usr/lib/libpcl_io.so
rgbd_salm/rgbd_salm_node: /usr/lib/libpcl_kdtree.so
rgbd_salm/rgbd_salm_node: /usr/lib/libpcl_search.so
rgbd_salm/rgbd_salm_node: /usr/lib/libpcl_sample_consensus.so
rgbd_salm/rgbd_salm_node: /usr/lib/libpcl_filters.so
rgbd_salm/rgbd_salm_node: /usr/lib/libpcl_features.so
rgbd_salm/rgbd_salm_node: /usr/lib/libpcl_keypoints.so
rgbd_salm/rgbd_salm_node: /usr/lib/libpcl_segmentation.so
rgbd_salm/rgbd_salm_node: /usr/lib/libpcl_visualization.so
rgbd_salm/rgbd_salm_node: /usr/lib/libpcl_outofcore.so
rgbd_salm/rgbd_salm_node: /usr/lib/libpcl_registration.so
rgbd_salm/rgbd_salm_node: /usr/lib/libpcl_recognition.so
rgbd_salm/rgbd_salm_node: /usr/lib/libpcl_surface.so
rgbd_salm/rgbd_salm_node: /usr/lib/libpcl_people.so
rgbd_salm/rgbd_salm_node: /usr/lib/libpcl_tracking.so
rgbd_salm/rgbd_salm_node: /usr/lib/libpcl_apps.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libqhull.so
rgbd_salm/rgbd_salm_node: /usr/lib/libOpenNI.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
rgbd_salm/rgbd_salm_node: /usr/lib/libvtkCommon.so.5.8.0
rgbd_salm/rgbd_salm_node: /usr/lib/libvtkRendering.so.5.8.0
rgbd_salm/rgbd_salm_node: /usr/lib/libvtkHybrid.so.5.8.0
rgbd_salm/rgbd_salm_node: /usr/lib/libvtkCharts.so.5.8.0
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libdynamic_reconfigure_config_init_mutex.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libnodeletlib.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libbondcpp.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libuuid.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libclass_loader.so
rgbd_salm/rgbd_salm_node: /usr/lib/libPocoFoundation.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libdl.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libroslib.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/librospack.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libpython2.7.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libtinyxml.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/librosbag.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/librosbag_storage.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libroslz4.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/liblz4.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libtopic_tools.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libtf.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libtf2_ros.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libactionlib.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libmessage_filters.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libtf2.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libroscpp.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libboost_signals.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/librosconsole.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/librosconsole_log4cxx.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/librosconsole_backend_interface.so
rgbd_salm/rgbd_salm_node: /usr/lib/liblog4cxx.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libxmlrpcpp.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libroscpp_serialization.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/librostime.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
rgbd_salm/rgbd_salm_node: /opt/ros/indigo/lib/libcpp_common.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libboost_system.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libpthread.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
rgbd_salm/rgbd_salm_node: /usr/lib/x86_64-linux-gnu/libcxsparse.so
rgbd_salm/rgbd_salm_node: rgbd_salm/CMakeFiles/rgbd_salm_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leo/Desktop/learn_rgbdsalm/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable rgbd_salm_node"
	cd /home/leo/Desktop/learn_rgbdsalm/src/cmake-build-debug/rgbd_salm && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rgbd_salm_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
rgbd_salm/CMakeFiles/rgbd_salm_node.dir/build: rgbd_salm/rgbd_salm_node

.PHONY : rgbd_salm/CMakeFiles/rgbd_salm_node.dir/build

rgbd_salm/CMakeFiles/rgbd_salm_node.dir/requires: rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam_base.cpp.o.requires
rgbd_salm/CMakeFiles/rgbd_salm_node.dir/requires: rgbd_salm/CMakeFiles/rgbd_salm_node.dir/src/slam.cpp.o.requires

.PHONY : rgbd_salm/CMakeFiles/rgbd_salm_node.dir/requires

rgbd_salm/CMakeFiles/rgbd_salm_node.dir/clean:
	cd /home/leo/Desktop/learn_rgbdsalm/src/cmake-build-debug/rgbd_salm && $(CMAKE_COMMAND) -P CMakeFiles/rgbd_salm_node.dir/cmake_clean.cmake
.PHONY : rgbd_salm/CMakeFiles/rgbd_salm_node.dir/clean

rgbd_salm/CMakeFiles/rgbd_salm_node.dir/depend:
	cd /home/leo/Desktop/learn_rgbdsalm/src/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leo/Desktop/learn_rgbdsalm/src /home/leo/Desktop/learn_rgbdsalm/src/rgbd_salm /home/leo/Desktop/learn_rgbdsalm/src/cmake-build-debug /home/leo/Desktop/learn_rgbdsalm/src/cmake-build-debug/rgbd_salm /home/leo/Desktop/learn_rgbdsalm/src/cmake-build-debug/rgbd_salm/CMakeFiles/rgbd_salm_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : rgbd_salm/CMakeFiles/rgbd_salm_node.dir/depend

