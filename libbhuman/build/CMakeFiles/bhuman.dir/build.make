# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nao/testbhuman/libbhuman

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nao/testbhuman/libbhuman/build

# Include any dependencies generated for this target.
include CMakeFiles/bhuman.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/bhuman.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/bhuman.dir/flags.make

CMakeFiles/bhuman.dir/bhuman.o: CMakeFiles/bhuman.dir/flags.make
CMakeFiles/bhuman.dir/bhuman.o: ../bhuman.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nao/testbhuman/libbhuman/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/bhuman.dir/bhuman.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/bhuman.dir/bhuman.o -c /home/nao/testbhuman/libbhuman/bhuman.cpp

CMakeFiles/bhuman.dir/bhuman.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bhuman.dir/bhuman.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/nao/testbhuman/libbhuman/bhuman.cpp > CMakeFiles/bhuman.dir/bhuman.i

CMakeFiles/bhuman.dir/bhuman.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bhuman.dir/bhuman.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/nao/testbhuman/libbhuman/bhuman.cpp -o CMakeFiles/bhuman.dir/bhuman.s

CMakeFiles/bhuman.dir/bhuman.o.requires:
.PHONY : CMakeFiles/bhuman.dir/bhuman.o.requires

CMakeFiles/bhuman.dir/bhuman.o.provides: CMakeFiles/bhuman.dir/bhuman.o.requires
	$(MAKE) -f CMakeFiles/bhuman.dir/build.make CMakeFiles/bhuman.dir/bhuman.o.provides.build
.PHONY : CMakeFiles/bhuman.dir/bhuman.o.provides

CMakeFiles/bhuman.dir/bhuman.o.provides.build: CMakeFiles/bhuman.dir/bhuman.o

# Object files for target bhuman
bhuman_OBJECTS = \
"CMakeFiles/bhuman.dir/bhuman.o"

# External object files for target bhuman
bhuman_EXTERNAL_OBJECTS =

../libbhuman.so: CMakeFiles/bhuman.dir/bhuman.o
../libbhuman.so: CMakeFiles/bhuman.dir/build.make
../libbhuman.so: CMakeFiles/bhuman.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library ../libbhuman.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bhuman.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/bhuman.dir/build: ../libbhuman.so
.PHONY : CMakeFiles/bhuman.dir/build

CMakeFiles/bhuman.dir/requires: CMakeFiles/bhuman.dir/bhuman.o.requires
.PHONY : CMakeFiles/bhuman.dir/requires

CMakeFiles/bhuman.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/bhuman.dir/cmake_clean.cmake
.PHONY : CMakeFiles/bhuman.dir/clean

CMakeFiles/bhuman.dir/depend:
	cd /home/nao/testbhuman/libbhuman/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nao/testbhuman/libbhuman /home/nao/testbhuman/libbhuman /home/nao/testbhuman/libbhuman/build /home/nao/testbhuman/libbhuman/build /home/nao/testbhuman/libbhuman/build/CMakeFiles/bhuman.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/bhuman.dir/depend

