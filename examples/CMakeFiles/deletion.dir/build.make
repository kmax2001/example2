# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.27

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
CMAKE_COMMAND = /home/kimyoungho/.local/lib/python3.10/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/kimyoungho/.local/lib/python3.10/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kimyoungho/raisimLib

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kimyoungho/raisimLib/example2

# Include any dependencies generated for this target.
include examples/CMakeFiles/deletion.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include examples/CMakeFiles/deletion.dir/compiler_depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/deletion.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/deletion.dir/flags.make

examples/CMakeFiles/deletion.dir/src/server/deletion.cpp.o: examples/CMakeFiles/deletion.dir/flags.make
examples/CMakeFiles/deletion.dir/src/server/deletion.cpp.o: /home/kimyoungho/raisimLib/examples/src/server/deletion.cpp
examples/CMakeFiles/deletion.dir/src/server/deletion.cpp.o: examples/CMakeFiles/deletion.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/kimyoungho/raisimLib/example2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/deletion.dir/src/server/deletion.cpp.o"
	cd /home/kimyoungho/raisimLib/example2/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/deletion.dir/src/server/deletion.cpp.o -MF CMakeFiles/deletion.dir/src/server/deletion.cpp.o.d -o CMakeFiles/deletion.dir/src/server/deletion.cpp.o -c /home/kimyoungho/raisimLib/examples/src/server/deletion.cpp

examples/CMakeFiles/deletion.dir/src/server/deletion.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/deletion.dir/src/server/deletion.cpp.i"
	cd /home/kimyoungho/raisimLib/example2/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kimyoungho/raisimLib/examples/src/server/deletion.cpp > CMakeFiles/deletion.dir/src/server/deletion.cpp.i

examples/CMakeFiles/deletion.dir/src/server/deletion.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/deletion.dir/src/server/deletion.cpp.s"
	cd /home/kimyoungho/raisimLib/example2/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kimyoungho/raisimLib/examples/src/server/deletion.cpp -o CMakeFiles/deletion.dir/src/server/deletion.cpp.s

# Object files for target deletion
deletion_OBJECTS = \
"CMakeFiles/deletion.dir/src/server/deletion.cpp.o"

# External object files for target deletion
deletion_EXTERNAL_OBJECTS =

examples/deletion: examples/CMakeFiles/deletion.dir/src/server/deletion.cpp.o
examples/deletion: examples/CMakeFiles/deletion.dir/build.make
examples/deletion: /home/kimyoungho/raisimLib/raisim/linux/lib/libraisim.so.1.1.7
examples/deletion: /home/kimyoungho/raisimLib/raisim/linux/lib/libraisimPng.so
examples/deletion: /home/kimyoungho/raisimLib/raisim/linux/lib/libraisimZ.so
examples/deletion: /home/kimyoungho/raisimLib/raisim/linux/lib/libraisimODE.so.1.1.7
examples/deletion: /home/kimyoungho/raisimLib/raisim/linux/lib/libraisimMine.so
examples/deletion: examples/CMakeFiles/deletion.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/kimyoungho/raisimLib/example2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable deletion"
	cd /home/kimyoungho/raisimLib/example2/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/deletion.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/deletion.dir/build: examples/deletion
.PHONY : examples/CMakeFiles/deletion.dir/build

examples/CMakeFiles/deletion.dir/clean:
	cd /home/kimyoungho/raisimLib/example2/examples && $(CMAKE_COMMAND) -P CMakeFiles/deletion.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/deletion.dir/clean

examples/CMakeFiles/deletion.dir/depend:
	cd /home/kimyoungho/raisimLib/example2 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kimyoungho/raisimLib /home/kimyoungho/raisimLib/examples /home/kimyoungho/raisimLib/example2 /home/kimyoungho/raisimLib/example2/examples /home/kimyoungho/raisimLib/example2/examples/CMakeFiles/deletion.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : examples/CMakeFiles/deletion.dir/depend
