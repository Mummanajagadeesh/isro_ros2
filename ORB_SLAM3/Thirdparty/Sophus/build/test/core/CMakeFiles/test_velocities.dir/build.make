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
CMAKE_SOURCE_DIR = /home/niraamay/SLAM/ORB_SLAM3/Thirdparty/Sophus

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/niraamay/SLAM/ORB_SLAM3/Thirdparty/Sophus/build

# Include any dependencies generated for this target.
include test/core/CMakeFiles/test_velocities.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include test/core/CMakeFiles/test_velocities.dir/compiler_depend.make

# Include the progress variables for this target.
include test/core/CMakeFiles/test_velocities.dir/progress.make

# Include the compile flags for this target's objects.
include test/core/CMakeFiles/test_velocities.dir/flags.make

test/core/CMakeFiles/test_velocities.dir/test_velocities.cpp.o: test/core/CMakeFiles/test_velocities.dir/flags.make
test/core/CMakeFiles/test_velocities.dir/test_velocities.cpp.o: ../test/core/test_velocities.cpp
test/core/CMakeFiles/test_velocities.dir/test_velocities.cpp.o: test/core/CMakeFiles/test_velocities.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/niraamay/SLAM/ORB_SLAM3/Thirdparty/Sophus/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/core/CMakeFiles/test_velocities.dir/test_velocities.cpp.o"
	cd /home/niraamay/SLAM/ORB_SLAM3/Thirdparty/Sophus/build/test/core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT test/core/CMakeFiles/test_velocities.dir/test_velocities.cpp.o -MF CMakeFiles/test_velocities.dir/test_velocities.cpp.o.d -o CMakeFiles/test_velocities.dir/test_velocities.cpp.o -c /home/niraamay/SLAM/ORB_SLAM3/Thirdparty/Sophus/test/core/test_velocities.cpp

test/core/CMakeFiles/test_velocities.dir/test_velocities.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_velocities.dir/test_velocities.cpp.i"
	cd /home/niraamay/SLAM/ORB_SLAM3/Thirdparty/Sophus/build/test/core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/niraamay/SLAM/ORB_SLAM3/Thirdparty/Sophus/test/core/test_velocities.cpp > CMakeFiles/test_velocities.dir/test_velocities.cpp.i

test/core/CMakeFiles/test_velocities.dir/test_velocities.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_velocities.dir/test_velocities.cpp.s"
	cd /home/niraamay/SLAM/ORB_SLAM3/Thirdparty/Sophus/build/test/core && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/niraamay/SLAM/ORB_SLAM3/Thirdparty/Sophus/test/core/test_velocities.cpp -o CMakeFiles/test_velocities.dir/test_velocities.cpp.s

# Object files for target test_velocities
test_velocities_OBJECTS = \
"CMakeFiles/test_velocities.dir/test_velocities.cpp.o"

# External object files for target test_velocities
test_velocities_EXTERNAL_OBJECTS =

test/core/test_velocities: test/core/CMakeFiles/test_velocities.dir/test_velocities.cpp.o
test/core/test_velocities: test/core/CMakeFiles/test_velocities.dir/build.make
test/core/test_velocities: test/core/CMakeFiles/test_velocities.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/niraamay/SLAM/ORB_SLAM3/Thirdparty/Sophus/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_velocities"
	cd /home/niraamay/SLAM/ORB_SLAM3/Thirdparty/Sophus/build/test/core && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_velocities.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/core/CMakeFiles/test_velocities.dir/build: test/core/test_velocities
.PHONY : test/core/CMakeFiles/test_velocities.dir/build

test/core/CMakeFiles/test_velocities.dir/clean:
	cd /home/niraamay/SLAM/ORB_SLAM3/Thirdparty/Sophus/build/test/core && $(CMAKE_COMMAND) -P CMakeFiles/test_velocities.dir/cmake_clean.cmake
.PHONY : test/core/CMakeFiles/test_velocities.dir/clean

test/core/CMakeFiles/test_velocities.dir/depend:
	cd /home/niraamay/SLAM/ORB_SLAM3/Thirdparty/Sophus/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/niraamay/SLAM/ORB_SLAM3/Thirdparty/Sophus /home/niraamay/SLAM/ORB_SLAM3/Thirdparty/Sophus/test/core /home/niraamay/SLAM/ORB_SLAM3/Thirdparty/Sophus/build /home/niraamay/SLAM/ORB_SLAM3/Thirdparty/Sophus/build/test/core /home/niraamay/SLAM/ORB_SLAM3/Thirdparty/Sophus/build/test/core/CMakeFiles/test_velocities.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/core/CMakeFiles/test_velocities.dir/depend

