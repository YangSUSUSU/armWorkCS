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
include trajectory_planner/CMakeFiles/trajectory_planner.dir/depend.make

# Include the progress variables for this target.
include trajectory_planner/CMakeFiles/trajectory_planner.dir/progress.make

# Include the compile flags for this target's objects.
include trajectory_planner/CMakeFiles/trajectory_planner.dir/flags.make

trajectory_planner/CMakeFiles/trajectory_planner.dir/src/traj_planner_base.cpp.o: trajectory_planner/CMakeFiles/trajectory_planner.dir/flags.make
trajectory_planner/CMakeFiles/trajectory_planner.dir/src/traj_planner_base.cpp.o: /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/traj_planner_base.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/work/armWorkCS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object trajectory_planner/CMakeFiles/trajectory_planner.dir/src/traj_planner_base.cpp.o"
	cd /home/ubuntu/work/armWorkCS/build/trajectory_planner && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trajectory_planner.dir/src/traj_planner_base.cpp.o -c /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/traj_planner_base.cpp

trajectory_planner/CMakeFiles/trajectory_planner.dir/src/traj_planner_base.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trajectory_planner.dir/src/traj_planner_base.cpp.i"
	cd /home/ubuntu/work/armWorkCS/build/trajectory_planner && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/traj_planner_base.cpp > CMakeFiles/trajectory_planner.dir/src/traj_planner_base.cpp.i

trajectory_planner/CMakeFiles/trajectory_planner.dir/src/traj_planner_base.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trajectory_planner.dir/src/traj_planner_base.cpp.s"
	cd /home/ubuntu/work/armWorkCS/build/trajectory_planner && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/traj_planner_base.cpp -o CMakeFiles/trajectory_planner.dir/src/traj_planner_base.cpp.s

trajectory_planner/CMakeFiles/trajectory_planner.dir/src/traj_cp_planner.cpp.o: trajectory_planner/CMakeFiles/trajectory_planner.dir/flags.make
trajectory_planner/CMakeFiles/trajectory_planner.dir/src/traj_cp_planner.cpp.o: /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/traj_cp_planner.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/work/armWorkCS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object trajectory_planner/CMakeFiles/trajectory_planner.dir/src/traj_cp_planner.cpp.o"
	cd /home/ubuntu/work/armWorkCS/build/trajectory_planner && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trajectory_planner.dir/src/traj_cp_planner.cpp.o -c /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/traj_cp_planner.cpp

trajectory_planner/CMakeFiles/trajectory_planner.dir/src/traj_cp_planner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trajectory_planner.dir/src/traj_cp_planner.cpp.i"
	cd /home/ubuntu/work/armWorkCS/build/trajectory_planner && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/traj_cp_planner.cpp > CMakeFiles/trajectory_planner.dir/src/traj_cp_planner.cpp.i

trajectory_planner/CMakeFiles/trajectory_planner.dir/src/traj_cp_planner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trajectory_planner.dir/src/traj_cp_planner.cpp.s"
	cd /home/ubuntu/work/armWorkCS/build/trajectory_planner && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/traj_cp_planner.cpp -o CMakeFiles/trajectory_planner.dir/src/traj_cp_planner.cpp.s

trajectory_planner/CMakeFiles/trajectory_planner.dir/src/traj_s_planner.cpp.o: trajectory_planner/CMakeFiles/trajectory_planner.dir/flags.make
trajectory_planner/CMakeFiles/trajectory_planner.dir/src/traj_s_planner.cpp.o: /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/traj_s_planner.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/work/armWorkCS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object trajectory_planner/CMakeFiles/trajectory_planner.dir/src/traj_s_planner.cpp.o"
	cd /home/ubuntu/work/armWorkCS/build/trajectory_planner && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trajectory_planner.dir/src/traj_s_planner.cpp.o -c /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/traj_s_planner.cpp

trajectory_planner/CMakeFiles/trajectory_planner.dir/src/traj_s_planner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trajectory_planner.dir/src/traj_s_planner.cpp.i"
	cd /home/ubuntu/work/armWorkCS/build/trajectory_planner && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/traj_s_planner.cpp > CMakeFiles/trajectory_planner.dir/src/traj_s_planner.cpp.i

trajectory_planner/CMakeFiles/trajectory_planner.dir/src/traj_s_planner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trajectory_planner.dir/src/traj_s_planner.cpp.s"
	cd /home/ubuntu/work/armWorkCS/build/trajectory_planner && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/traj_s_planner.cpp -o CMakeFiles/trajectory_planner.dir/src/traj_s_planner.cpp.s

trajectory_planner/CMakeFiles/trajectory_planner.dir/src/sVelocityPlanner.cpp.o: trajectory_planner/CMakeFiles/trajectory_planner.dir/flags.make
trajectory_planner/CMakeFiles/trajectory_planner.dir/src/sVelocityPlanner.cpp.o: /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/sVelocityPlanner.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/work/armWorkCS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object trajectory_planner/CMakeFiles/trajectory_planner.dir/src/sVelocityPlanner.cpp.o"
	cd /home/ubuntu/work/armWorkCS/build/trajectory_planner && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trajectory_planner.dir/src/sVelocityPlanner.cpp.o -c /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/sVelocityPlanner.cpp

trajectory_planner/CMakeFiles/trajectory_planner.dir/src/sVelocityPlanner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trajectory_planner.dir/src/sVelocityPlanner.cpp.i"
	cd /home/ubuntu/work/armWorkCS/build/trajectory_planner && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/sVelocityPlanner.cpp > CMakeFiles/trajectory_planner.dir/src/sVelocityPlanner.cpp.i

trajectory_planner/CMakeFiles/trajectory_planner.dir/src/sVelocityPlanner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trajectory_planner.dir/src/sVelocityPlanner.cpp.s"
	cd /home/ubuntu/work/armWorkCS/build/trajectory_planner && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/sVelocityPlanner.cpp -o CMakeFiles/trajectory_planner.dir/src/sVelocityPlanner.cpp.s

trajectory_planner/CMakeFiles/trajectory_planner.dir/src/interpolation.cpp.o: trajectory_planner/CMakeFiles/trajectory_planner.dir/flags.make
trajectory_planner/CMakeFiles/trajectory_planner.dir/src/interpolation.cpp.o: /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/interpolation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/work/armWorkCS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object trajectory_planner/CMakeFiles/trajectory_planner.dir/src/interpolation.cpp.o"
	cd /home/ubuntu/work/armWorkCS/build/trajectory_planner && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trajectory_planner.dir/src/interpolation.cpp.o -c /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/interpolation.cpp

trajectory_planner/CMakeFiles/trajectory_planner.dir/src/interpolation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trajectory_planner.dir/src/interpolation.cpp.i"
	cd /home/ubuntu/work/armWorkCS/build/trajectory_planner && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/interpolation.cpp > CMakeFiles/trajectory_planner.dir/src/interpolation.cpp.i

trajectory_planner/CMakeFiles/trajectory_planner.dir/src/interpolation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trajectory_planner.dir/src/interpolation.cpp.s"
	cd /home/ubuntu/work/armWorkCS/build/trajectory_planner && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/work/armWorkCS/src/trajectory_planner/src/interpolation.cpp -o CMakeFiles/trajectory_planner.dir/src/interpolation.cpp.s

# Object files for target trajectory_planner
trajectory_planner_OBJECTS = \
"CMakeFiles/trajectory_planner.dir/src/traj_planner_base.cpp.o" \
"CMakeFiles/trajectory_planner.dir/src/traj_cp_planner.cpp.o" \
"CMakeFiles/trajectory_planner.dir/src/traj_s_planner.cpp.o" \
"CMakeFiles/trajectory_planner.dir/src/sVelocityPlanner.cpp.o" \
"CMakeFiles/trajectory_planner.dir/src/interpolation.cpp.o"

# External object files for target trajectory_planner
trajectory_planner_EXTERNAL_OBJECTS =

/home/ubuntu/work/armWorkCS/devel/lib/libtrajectory_planner.so: trajectory_planner/CMakeFiles/trajectory_planner.dir/src/traj_planner_base.cpp.o
/home/ubuntu/work/armWorkCS/devel/lib/libtrajectory_planner.so: trajectory_planner/CMakeFiles/trajectory_planner.dir/src/traj_cp_planner.cpp.o
/home/ubuntu/work/armWorkCS/devel/lib/libtrajectory_planner.so: trajectory_planner/CMakeFiles/trajectory_planner.dir/src/traj_s_planner.cpp.o
/home/ubuntu/work/armWorkCS/devel/lib/libtrajectory_planner.so: trajectory_planner/CMakeFiles/trajectory_planner.dir/src/sVelocityPlanner.cpp.o
/home/ubuntu/work/armWorkCS/devel/lib/libtrajectory_planner.so: trajectory_planner/CMakeFiles/trajectory_planner.dir/src/interpolation.cpp.o
/home/ubuntu/work/armWorkCS/devel/lib/libtrajectory_planner.so: trajectory_planner/CMakeFiles/trajectory_planner.dir/build.make
/home/ubuntu/work/armWorkCS/devel/lib/libtrajectory_planner.so: trajectory_planner/CMakeFiles/trajectory_planner.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/work/armWorkCS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX shared library /home/ubuntu/work/armWorkCS/devel/lib/libtrajectory_planner.so"
	cd /home/ubuntu/work/armWorkCS/build/trajectory_planner && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/trajectory_planner.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
trajectory_planner/CMakeFiles/trajectory_planner.dir/build: /home/ubuntu/work/armWorkCS/devel/lib/libtrajectory_planner.so

.PHONY : trajectory_planner/CMakeFiles/trajectory_planner.dir/build

trajectory_planner/CMakeFiles/trajectory_planner.dir/clean:
	cd /home/ubuntu/work/armWorkCS/build/trajectory_planner && $(CMAKE_COMMAND) -P CMakeFiles/trajectory_planner.dir/cmake_clean.cmake
.PHONY : trajectory_planner/CMakeFiles/trajectory_planner.dir/clean

trajectory_planner/CMakeFiles/trajectory_planner.dir/depend:
	cd /home/ubuntu/work/armWorkCS/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/work/armWorkCS/src /home/ubuntu/work/armWorkCS/src/trajectory_planner /home/ubuntu/work/armWorkCS/build /home/ubuntu/work/armWorkCS/build/trajectory_planner /home/ubuntu/work/armWorkCS/build/trajectory_planner/CMakeFiles/trajectory_planner.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : trajectory_planner/CMakeFiles/trajectory_planner.dir/depend

