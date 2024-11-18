# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/ubuntu/work/armWorkCS/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/work/armWorkCS/build

# Include any dependencies generated for this target.
include arm_planning/aubo_arm_planning/CMakeFiles/arm_qpik_solver.dir/depend.make

# Include the progress variables for this target.
include arm_planning/aubo_arm_planning/CMakeFiles/arm_qpik_solver.dir/progress.make

# Include the compile flags for this target's objects.
include arm_planning/aubo_arm_planning/CMakeFiles/arm_qpik_solver.dir/flags.make

arm_planning/aubo_arm_planning/CMakeFiles/arm_qpik_solver.dir/src/arm_qpik_solver.cpp.o: arm_planning/aubo_arm_planning/CMakeFiles/arm_qpik_solver.dir/flags.make
arm_planning/aubo_arm_planning/CMakeFiles/arm_qpik_solver.dir/src/arm_qpik_solver.cpp.o: /home/ubuntu/work/armWorkCS/src/arm_planning/aubo_arm_planning/src/arm_qpik_solver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/work/armWorkCS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object arm_planning/aubo_arm_planning/CMakeFiles/arm_qpik_solver.dir/src/arm_qpik_solver.cpp.o"
	cd /home/ubuntu/work/armWorkCS/build/arm_planning/aubo_arm_planning && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/arm_qpik_solver.dir/src/arm_qpik_solver.cpp.o -c /home/ubuntu/work/armWorkCS/src/arm_planning/aubo_arm_planning/src/arm_qpik_solver.cpp

arm_planning/aubo_arm_planning/CMakeFiles/arm_qpik_solver.dir/src/arm_qpik_solver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/arm_qpik_solver.dir/src/arm_qpik_solver.cpp.i"
	cd /home/ubuntu/work/armWorkCS/build/arm_planning/aubo_arm_planning && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/work/armWorkCS/src/arm_planning/aubo_arm_planning/src/arm_qpik_solver.cpp > CMakeFiles/arm_qpik_solver.dir/src/arm_qpik_solver.cpp.i

arm_planning/aubo_arm_planning/CMakeFiles/arm_qpik_solver.dir/src/arm_qpik_solver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/arm_qpik_solver.dir/src/arm_qpik_solver.cpp.s"
	cd /home/ubuntu/work/armWorkCS/build/arm_planning/aubo_arm_planning && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/work/armWorkCS/src/arm_planning/aubo_arm_planning/src/arm_qpik_solver.cpp -o CMakeFiles/arm_qpik_solver.dir/src/arm_qpik_solver.cpp.s

# Object files for target arm_qpik_solver
arm_qpik_solver_OBJECTS = \
"CMakeFiles/arm_qpik_solver.dir/src/arm_qpik_solver.cpp.o"

# External object files for target arm_qpik_solver
arm_qpik_solver_EXTERNAL_OBJECTS =

/home/ubuntu/work/armWorkCS/devel/lib/libarm_qpik_solver.so: arm_planning/aubo_arm_planning/CMakeFiles/arm_qpik_solver.dir/src/arm_qpik_solver.cpp.o
/home/ubuntu/work/armWorkCS/devel/lib/libarm_qpik_solver.so: arm_planning/aubo_arm_planning/CMakeFiles/arm_qpik_solver.dir/build.make
/home/ubuntu/work/armWorkCS/devel/lib/libarm_qpik_solver.so: arm_planning/aubo_arm_planning/CMakeFiles/arm_qpik_solver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/work/armWorkCS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/ubuntu/work/armWorkCS/devel/lib/libarm_qpik_solver.so"
	cd /home/ubuntu/work/armWorkCS/build/arm_planning/aubo_arm_planning && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/arm_qpik_solver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
arm_planning/aubo_arm_planning/CMakeFiles/arm_qpik_solver.dir/build: /home/ubuntu/work/armWorkCS/devel/lib/libarm_qpik_solver.so

.PHONY : arm_planning/aubo_arm_planning/CMakeFiles/arm_qpik_solver.dir/build

arm_planning/aubo_arm_planning/CMakeFiles/arm_qpik_solver.dir/clean:
	cd /home/ubuntu/work/armWorkCS/build/arm_planning/aubo_arm_planning && $(CMAKE_COMMAND) -P CMakeFiles/arm_qpik_solver.dir/cmake_clean.cmake
.PHONY : arm_planning/aubo_arm_planning/CMakeFiles/arm_qpik_solver.dir/clean

arm_planning/aubo_arm_planning/CMakeFiles/arm_qpik_solver.dir/depend:
	cd /home/ubuntu/work/armWorkCS/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/work/armWorkCS/src /home/ubuntu/work/armWorkCS/src/arm_planning/aubo_arm_planning /home/ubuntu/work/armWorkCS/build /home/ubuntu/work/armWorkCS/build/arm_planning/aubo_arm_planning /home/ubuntu/work/armWorkCS/build/arm_planning/aubo_arm_planning/CMakeFiles/arm_qpik_solver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : arm_planning/aubo_arm_planning/CMakeFiles/arm_qpik_solver.dir/depend
