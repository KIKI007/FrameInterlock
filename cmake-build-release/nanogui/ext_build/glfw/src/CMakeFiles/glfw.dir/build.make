# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/ziqwang/Documents/GitHub/FrameInterlock

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release

# Include any dependencies generated for this target.
include nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/depend.make

# Include the progress variables for this target.
include nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/progress.make

# Include the compile flags for this target's objects.
include nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/flags.make

# Object files for target glfw
glfw_OBJECTS =

# External object files for target glfw
glfw_EXTERNAL_OBJECTS = \
"/Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/context.c.o" \
"/Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/init.c.o" \
"/Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/input.c.o" \
"/Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/monitor.c.o" \
"/Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/vulkan.c.o" \
"/Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/window.c.o" \
"/Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_init.m.o" \
"/Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_joystick.m.o" \
"/Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_monitor.m.o" \
"/Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_window.m.o" \
"/Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_time.c.o" \
"/Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_tls.c.o" \
"/Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/nsgl_context.m.o"

nanogui/ext_build/glfw/src/libglfw3.a: nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/context.c.o
nanogui/ext_build/glfw/src/libglfw3.a: nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/init.c.o
nanogui/ext_build/glfw/src/libglfw3.a: nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/input.c.o
nanogui/ext_build/glfw/src/libglfw3.a: nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/monitor.c.o
nanogui/ext_build/glfw/src/libglfw3.a: nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/vulkan.c.o
nanogui/ext_build/glfw/src/libglfw3.a: nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/window.c.o
nanogui/ext_build/glfw/src/libglfw3.a: nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_init.m.o
nanogui/ext_build/glfw/src/libglfw3.a: nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_joystick.m.o
nanogui/ext_build/glfw/src/libglfw3.a: nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_monitor.m.o
nanogui/ext_build/glfw/src/libglfw3.a: nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_window.m.o
nanogui/ext_build/glfw/src/libglfw3.a: nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_time.c.o
nanogui/ext_build/glfw/src/libglfw3.a: nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_tls.c.o
nanogui/ext_build/glfw/src/libglfw3.a: nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/nsgl_context.m.o
nanogui/ext_build/glfw/src/libglfw3.a: nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/build.make
nanogui/ext_build/glfw/src/libglfw3.a: nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Linking C static library libglfw3.a"
	cd /Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release/nanogui/ext_build/glfw/src && $(CMAKE_COMMAND) -P CMakeFiles/glfw.dir/cmake_clean_target.cmake
	cd /Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release/nanogui/ext_build/glfw/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/glfw.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/build: nanogui/ext_build/glfw/src/libglfw3.a

.PHONY : nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/build

nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/requires:

.PHONY : nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/requires

nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/clean:
	cd /Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release/nanogui/ext_build/glfw/src && $(CMAKE_COMMAND) -P CMakeFiles/glfw.dir/cmake_clean.cmake
.PHONY : nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/clean

nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/depend:
	cd /Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/ziqwang/Documents/GitHub/FrameInterlock /Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glfw/src /Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release /Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release/nanogui/ext_build/glfw/src /Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-release/nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : nanogui/ext_build/glfw/src/CMakeFiles/glfw.dir/depend

