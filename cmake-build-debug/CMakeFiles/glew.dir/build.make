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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.10.1/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.10.1/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/ziqwang/Documents/GitHub/FrameInterlock

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/glew.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/glew.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/glew.dir/flags.make

CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.o: CMakeFiles/glew.dir/flags.make
CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.o: /Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.o   -c /Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c

CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c > CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.i

CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c -o CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.s

CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.o.requires:

.PHONY : CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.o.requires

CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.o.provides: CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.o.requires
	$(MAKE) -f CMakeFiles/glew.dir/build.make CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.o.provides.build
.PHONY : CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.o.provides

CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.o.provides.build: CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.o


# Object files for target glew
glew_OBJECTS = \
"CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.o"

# External object files for target glew
glew_EXTERNAL_OBJECTS =

libglew.a: CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.o
libglew.a: CMakeFiles/glew.dir/build.make
libglew.a: CMakeFiles/glew.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libglew.a"
	$(CMAKE_COMMAND) -P CMakeFiles/glew.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/glew.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/glew.dir/build: libglew.a

.PHONY : CMakeFiles/glew.dir/build

CMakeFiles/glew.dir/requires: CMakeFiles/glew.dir/Users/ziqwang/Documents/GitHub/libigl/external/nanogui/ext/glew/src/glew.c.o.requires

.PHONY : CMakeFiles/glew.dir/requires

CMakeFiles/glew.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/glew.dir/cmake_clean.cmake
.PHONY : CMakeFiles/glew.dir/clean

CMakeFiles/glew.dir/depend:
	cd /Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/ziqwang/Documents/GitHub/FrameInterlock /Users/ziqwang/Documents/GitHub/FrameInterlock /Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-debug /Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-debug /Users/ziqwang/Documents/GitHub/FrameInterlock/cmake-build-debug/CMakeFiles/glew.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/glew.dir/depend

