# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/nao/gu_test2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nao/gu_test2/build

# Include any dependencies generated for this target.
include CMakeFiles/test_getup.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_getup.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_getup.dir/flags.make

CMakeFiles/test_getup.dir/test.cpp.o: CMakeFiles/test_getup.dir/flags.make
CMakeFiles/test_getup.dir/test.cpp.o: ../test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nao/gu_test2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_getup.dir/test.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_getup.dir/test.cpp.o -c /home/nao/gu_test2/test.cpp

CMakeFiles/test_getup.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_getup.dir/test.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nao/gu_test2/test.cpp > CMakeFiles/test_getup.dir/test.cpp.i

CMakeFiles/test_getup.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_getup.dir/test.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nao/gu_test2/test.cpp -o CMakeFiles/test_getup.dir/test.cpp.s

CMakeFiles/test_getup.dir/test.cpp.o.requires:

.PHONY : CMakeFiles/test_getup.dir/test.cpp.o.requires

CMakeFiles/test_getup.dir/test.cpp.o.provides: CMakeFiles/test_getup.dir/test.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_getup.dir/build.make CMakeFiles/test_getup.dir/test.cpp.o.provides.build
.PHONY : CMakeFiles/test_getup.dir/test.cpp.o.provides

CMakeFiles/test_getup.dir/test.cpp.o.provides.build: CMakeFiles/test_getup.dir/test.cpp.o


# Object files for target test_getup
test_getup_OBJECTS = \
"CMakeFiles/test_getup.dir/test.cpp.o"

# External object files for target test_getup
test_getup_EXTERNAL_OBJECTS =

test_getup: CMakeFiles/test_getup.dir/test.cpp.o
test_getup: CMakeFiles/test_getup.dir/build.make
test_getup: CMakeFiles/test_getup.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nao/gu_test2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_getup"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_getup.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_getup.dir/build: test_getup

.PHONY : CMakeFiles/test_getup.dir/build

CMakeFiles/test_getup.dir/requires: CMakeFiles/test_getup.dir/test.cpp.o.requires

.PHONY : CMakeFiles/test_getup.dir/requires

CMakeFiles/test_getup.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_getup.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_getup.dir/clean

CMakeFiles/test_getup.dir/depend:
	cd /home/nao/gu_test2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nao/gu_test2 /home/nao/gu_test2 /home/nao/gu_test2/build /home/nao/gu_test2/build /home/nao/gu_test2/build/CMakeFiles/test_getup.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_getup.dir/depend

