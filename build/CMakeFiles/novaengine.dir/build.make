# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ryuzaki/Desktop/NoveEngine

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ryuzaki/Desktop/NoveEngine/build

# Include any dependencies generated for this target.
include CMakeFiles/novaengine.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/novaengine.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/novaengine.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/novaengine.dir/flags.make

CMakeFiles/novaengine.dir/src/decode_and_display.cpp.o: CMakeFiles/novaengine.dir/flags.make
CMakeFiles/novaengine.dir/src/decode_and_display.cpp.o: ../src/decode_and_display.cpp
CMakeFiles/novaengine.dir/src/decode_and_display.cpp.o: CMakeFiles/novaengine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ryuzaki/Desktop/NoveEngine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/novaengine.dir/src/decode_and_display.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/novaengine.dir/src/decode_and_display.cpp.o -MF CMakeFiles/novaengine.dir/src/decode_and_display.cpp.o.d -o CMakeFiles/novaengine.dir/src/decode_and_display.cpp.o -c /home/ryuzaki/Desktop/NoveEngine/src/decode_and_display.cpp

CMakeFiles/novaengine.dir/src/decode_and_display.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/novaengine.dir/src/decode_and_display.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ryuzaki/Desktop/NoveEngine/src/decode_and_display.cpp > CMakeFiles/novaengine.dir/src/decode_and_display.cpp.i

CMakeFiles/novaengine.dir/src/decode_and_display.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/novaengine.dir/src/decode_and_display.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ryuzaki/Desktop/NoveEngine/src/decode_and_display.cpp -o CMakeFiles/novaengine.dir/src/decode_and_display.cpp.s

CMakeFiles/novaengine.dir/src/ffmpeg_encoder.cpp.o: CMakeFiles/novaengine.dir/flags.make
CMakeFiles/novaengine.dir/src/ffmpeg_encoder.cpp.o: ../src/ffmpeg_encoder.cpp
CMakeFiles/novaengine.dir/src/ffmpeg_encoder.cpp.o: CMakeFiles/novaengine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ryuzaki/Desktop/NoveEngine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/novaengine.dir/src/ffmpeg_encoder.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/novaengine.dir/src/ffmpeg_encoder.cpp.o -MF CMakeFiles/novaengine.dir/src/ffmpeg_encoder.cpp.o.d -o CMakeFiles/novaengine.dir/src/ffmpeg_encoder.cpp.o -c /home/ryuzaki/Desktop/NoveEngine/src/ffmpeg_encoder.cpp

CMakeFiles/novaengine.dir/src/ffmpeg_encoder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/novaengine.dir/src/ffmpeg_encoder.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ryuzaki/Desktop/NoveEngine/src/ffmpeg_encoder.cpp > CMakeFiles/novaengine.dir/src/ffmpeg_encoder.cpp.i

CMakeFiles/novaengine.dir/src/ffmpeg_encoder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/novaengine.dir/src/ffmpeg_encoder.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ryuzaki/Desktop/NoveEngine/src/ffmpeg_encoder.cpp -o CMakeFiles/novaengine.dir/src/ffmpeg_encoder.cpp.s

CMakeFiles/novaengine.dir/src/main.cpp.o: CMakeFiles/novaengine.dir/flags.make
CMakeFiles/novaengine.dir/src/main.cpp.o: ../src/main.cpp
CMakeFiles/novaengine.dir/src/main.cpp.o: CMakeFiles/novaengine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ryuzaki/Desktop/NoveEngine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/novaengine.dir/src/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/novaengine.dir/src/main.cpp.o -MF CMakeFiles/novaengine.dir/src/main.cpp.o.d -o CMakeFiles/novaengine.dir/src/main.cpp.o -c /home/ryuzaki/Desktop/NoveEngine/src/main.cpp

CMakeFiles/novaengine.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/novaengine.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ryuzaki/Desktop/NoveEngine/src/main.cpp > CMakeFiles/novaengine.dir/src/main.cpp.i

CMakeFiles/novaengine.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/novaengine.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ryuzaki/Desktop/NoveEngine/src/main.cpp -o CMakeFiles/novaengine.dir/src/main.cpp.s

CMakeFiles/novaengine.dir/src/sender_receiver.cpp.o: CMakeFiles/novaengine.dir/flags.make
CMakeFiles/novaengine.dir/src/sender_receiver.cpp.o: ../src/sender_receiver.cpp
CMakeFiles/novaengine.dir/src/sender_receiver.cpp.o: CMakeFiles/novaengine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ryuzaki/Desktop/NoveEngine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/novaengine.dir/src/sender_receiver.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/novaengine.dir/src/sender_receiver.cpp.o -MF CMakeFiles/novaengine.dir/src/sender_receiver.cpp.o.d -o CMakeFiles/novaengine.dir/src/sender_receiver.cpp.o -c /home/ryuzaki/Desktop/NoveEngine/src/sender_receiver.cpp

CMakeFiles/novaengine.dir/src/sender_receiver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/novaengine.dir/src/sender_receiver.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ryuzaki/Desktop/NoveEngine/src/sender_receiver.cpp > CMakeFiles/novaengine.dir/src/sender_receiver.cpp.i

CMakeFiles/novaengine.dir/src/sender_receiver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/novaengine.dir/src/sender_receiver.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ryuzaki/Desktop/NoveEngine/src/sender_receiver.cpp -o CMakeFiles/novaengine.dir/src/sender_receiver.cpp.s

CMakeFiles/novaengine.dir/src/slicer.cpp.o: CMakeFiles/novaengine.dir/flags.make
CMakeFiles/novaengine.dir/src/slicer.cpp.o: ../src/slicer.cpp
CMakeFiles/novaengine.dir/src/slicer.cpp.o: CMakeFiles/novaengine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ryuzaki/Desktop/NoveEngine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/novaengine.dir/src/slicer.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/novaengine.dir/src/slicer.cpp.o -MF CMakeFiles/novaengine.dir/src/slicer.cpp.o.d -o CMakeFiles/novaengine.dir/src/slicer.cpp.o -c /home/ryuzaki/Desktop/NoveEngine/src/slicer.cpp

CMakeFiles/novaengine.dir/src/slicer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/novaengine.dir/src/slicer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ryuzaki/Desktop/NoveEngine/src/slicer.cpp > CMakeFiles/novaengine.dir/src/slicer.cpp.i

CMakeFiles/novaengine.dir/src/slicer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/novaengine.dir/src/slicer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ryuzaki/Desktop/NoveEngine/src/slicer.cpp -o CMakeFiles/novaengine.dir/src/slicer.cpp.s

CMakeFiles/novaengine.dir/src/smart_collector.cpp.o: CMakeFiles/novaengine.dir/flags.make
CMakeFiles/novaengine.dir/src/smart_collector.cpp.o: ../src/smart_collector.cpp
CMakeFiles/novaengine.dir/src/smart_collector.cpp.o: CMakeFiles/novaengine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ryuzaki/Desktop/NoveEngine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/novaengine.dir/src/smart_collector.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/novaengine.dir/src/smart_collector.cpp.o -MF CMakeFiles/novaengine.dir/src/smart_collector.cpp.o.d -o CMakeFiles/novaengine.dir/src/smart_collector.cpp.o -c /home/ryuzaki/Desktop/NoveEngine/src/smart_collector.cpp

CMakeFiles/novaengine.dir/src/smart_collector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/novaengine.dir/src/smart_collector.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ryuzaki/Desktop/NoveEngine/src/smart_collector.cpp > CMakeFiles/novaengine.dir/src/smart_collector.cpp.i

CMakeFiles/novaengine.dir/src/smart_collector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/novaengine.dir/src/smart_collector.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ryuzaki/Desktop/NoveEngine/src/smart_collector.cpp -o CMakeFiles/novaengine.dir/src/smart_collector.cpp.s

CMakeFiles/novaengine.dir/src/udp_sender.cpp.o: CMakeFiles/novaengine.dir/flags.make
CMakeFiles/novaengine.dir/src/udp_sender.cpp.o: ../src/udp_sender.cpp
CMakeFiles/novaengine.dir/src/udp_sender.cpp.o: CMakeFiles/novaengine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ryuzaki/Desktop/NoveEngine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/novaengine.dir/src/udp_sender.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/novaengine.dir/src/udp_sender.cpp.o -MF CMakeFiles/novaengine.dir/src/udp_sender.cpp.o.d -o CMakeFiles/novaengine.dir/src/udp_sender.cpp.o -c /home/ryuzaki/Desktop/NoveEngine/src/udp_sender.cpp

CMakeFiles/novaengine.dir/src/udp_sender.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/novaengine.dir/src/udp_sender.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ryuzaki/Desktop/NoveEngine/src/udp_sender.cpp > CMakeFiles/novaengine.dir/src/udp_sender.cpp.i

CMakeFiles/novaengine.dir/src/udp_sender.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/novaengine.dir/src/udp_sender.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ryuzaki/Desktop/NoveEngine/src/udp_sender.cpp -o CMakeFiles/novaengine.dir/src/udp_sender.cpp.s

# Object files for target novaengine
novaengine_OBJECTS = \
"CMakeFiles/novaengine.dir/src/decode_and_display.cpp.o" \
"CMakeFiles/novaengine.dir/src/ffmpeg_encoder.cpp.o" \
"CMakeFiles/novaengine.dir/src/main.cpp.o" \
"CMakeFiles/novaengine.dir/src/sender_receiver.cpp.o" \
"CMakeFiles/novaengine.dir/src/slicer.cpp.o" \
"CMakeFiles/novaengine.dir/src/smart_collector.cpp.o" \
"CMakeFiles/novaengine.dir/src/udp_sender.cpp.o"

# External object files for target novaengine
novaengine_EXTERNAL_OBJECTS =

../bin/novaengine: CMakeFiles/novaengine.dir/src/decode_and_display.cpp.o
../bin/novaengine: CMakeFiles/novaengine.dir/src/ffmpeg_encoder.cpp.o
../bin/novaengine: CMakeFiles/novaengine.dir/src/main.cpp.o
../bin/novaengine: CMakeFiles/novaengine.dir/src/sender_receiver.cpp.o
../bin/novaengine: CMakeFiles/novaengine.dir/src/slicer.cpp.o
../bin/novaengine: CMakeFiles/novaengine.dir/src/smart_collector.cpp.o
../bin/novaengine: CMakeFiles/novaengine.dir/src/udp_sender.cpp.o
../bin/novaengine: CMakeFiles/novaengine.dir/build.make
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_alphamat.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_barcode.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_intensity_transform.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_mcc.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_rapid.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_wechat_qrcode.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libavcodec.so
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libavutil.so
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libswscale.so
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.5.4d
../bin/novaengine: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.5.4d
../bin/novaengine: CMakeFiles/novaengine.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ryuzaki/Desktop/NoveEngine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX executable ../bin/novaengine"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/novaengine.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/novaengine.dir/build: ../bin/novaengine
.PHONY : CMakeFiles/novaengine.dir/build

CMakeFiles/novaengine.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/novaengine.dir/cmake_clean.cmake
.PHONY : CMakeFiles/novaengine.dir/clean

CMakeFiles/novaengine.dir/depend:
	cd /home/ryuzaki/Desktop/NoveEngine/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ryuzaki/Desktop/NoveEngine /home/ryuzaki/Desktop/NoveEngine /home/ryuzaki/Desktop/NoveEngine/build /home/ryuzaki/Desktop/NoveEngine/build /home/ryuzaki/Desktop/NoveEngine/build/CMakeFiles/novaengine.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/novaengine.dir/depend

