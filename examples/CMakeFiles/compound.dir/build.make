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
include examples/CMakeFiles/compound.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include examples/CMakeFiles/compound.dir/compiler_depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/compound.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/compound.dir/flags.make

examples/CMakeFiles/compound.dir/src/server/compound.cpp.o: examples/CMakeFiles/compound.dir/flags.make
examples/CMakeFiles/compound.dir/src/server/compound.cpp.o: /home/kimyoungho/raisimLib/examples/src/server/compound.cpp
examples/CMakeFiles/compound.dir/src/server/compound.cpp.o: examples/CMakeFiles/compound.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/kimyoungho/raisimLib/example2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/compound.dir/src/server/compound.cpp.o"
	cd /home/kimyoungho/raisimLib/example2/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/compound.dir/src/server/compound.cpp.o -MF CMakeFiles/compound.dir/src/server/compound.cpp.o.d -o CMakeFiles/compound.dir/src/server/compound.cpp.o -c /home/kimyoungho/raisimLib/examples/src/server/compound.cpp

examples/CMakeFiles/compound.dir/src/server/compound.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/compound.dir/src/server/compound.cpp.i"
	cd /home/kimyoungho/raisimLib/example2/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kimyoungho/raisimLib/examples/src/server/compound.cpp > CMakeFiles/compound.dir/src/server/compound.cpp.i

examples/CMakeFiles/compound.dir/src/server/compound.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/compound.dir/src/server/compound.cpp.s"
	cd /home/kimyoungho/raisimLib/example2/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kimyoungho/raisimLib/examples/src/server/compound.cpp -o CMakeFiles/compound.dir/src/server/compound.cpp.s

# Object files for target compound
compound_OBJECTS = \
"CMakeFiles/compound.dir/src/server/compound.cpp.o"

# External object files for target compound
compound_EXTERNAL_OBJECTS =

examples/compound: examples/CMakeFiles/compound.dir/src/server/compound.cpp.o
examples/compound: examples/CMakeFiles/compound.dir/build.make
examples/compound: /home/kimyoungho/raisimLib/raisim/linux/lib/libraisim.so.1.1.7
examples/compound: /home/kimyoungho/raisimLib/raisim/linux/lib/libraisimPng.so
examples/compound: /home/kimyoungho/raisimLib/raisim/linux/lib/libraisimZ.so
examples/compound: /home/kimyoungho/raisimLib/raisim/linux/lib/libraisimODE.so.1.1.7
examples/compound: /home/kimyoungho/raisimLib/raisim/linux/lib/libraisimMine.so
examples/compound: examples/CMakeFiles/compound.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/kimyoungho/raisimLib/example2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable compound"
	cd /home/kimyoungho/raisimLib/example2/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/compound.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/compound.dir/build: examples/compound
.PHONY : examples/CMakeFiles/compound.dir/build

examples/CMakeFiles/compound.dir/clean:
	cd /home/kimyoungho/raisimLib/example2/examples && $(CMAKE_COMMAND) -P CMakeFiles/compound.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/compound.dir/clean

examples/CMakeFiles/compound.dir/depend:
	cd /home/kimyoungho/raisimLib/example2 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kimyoungho/raisimLib /home/kimyoungho/raisimLib/examples /home/kimyoungho/raisimLib/example2 /home/kimyoungho/raisimLib/example2/examples /home/kimyoungho/raisimLib/example2/examples/CMakeFiles/compound.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : examples/CMakeFiles/compound.dir/depend

