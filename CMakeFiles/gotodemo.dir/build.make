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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/alfmunny/TAS_wrapper

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alfmunny/TAS_wrapper

# Include any dependencies generated for this target.
include demo/matt/CMakeFiles/gotodemo.dir/depend.make

# Include the progress variables for this target.
include demo/matt/CMakeFiles/gotodemo.dir/progress.make

# Include the compile flags for this target's objects.
include demo/matt/CMakeFiles/gotodemo.dir/flags.make

demo/matt/CMakeFiles/gotodemo.dir/matt.o: demo/matt/CMakeFiles/gotodemo.dir/flags.make
demo/matt/CMakeFiles/gotodemo.dir/matt.o: demo/matt/matt.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/alfmunny/TAS_wrapper/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object demo/matt/CMakeFiles/gotodemo.dir/matt.o"
	cd /home/alfmunny/TAS_wrapper/demo/matt && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS)  -I/usr/local/include/player-3.0 -o CMakeFiles/gotodemo.dir/matt.o -c /home/alfmunny/TAS_wrapper/demo/matt/matt.cpp

demo/matt/CMakeFiles/gotodemo.dir/matt.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gotodemo.dir/matt.i"
	cd /home/alfmunny/TAS_wrapper/demo/matt && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS)  -I/usr/local/include/player-3.0 -E /home/alfmunny/TAS_wrapper/demo/matt/matt.cpp > CMakeFiles/gotodemo.dir/matt.i

demo/matt/CMakeFiles/gotodemo.dir/matt.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gotodemo.dir/matt.s"
	cd /home/alfmunny/TAS_wrapper/demo/matt && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS)  -I/usr/local/include/player-3.0 -S /home/alfmunny/TAS_wrapper/demo/matt/matt.cpp -o CMakeFiles/gotodemo.dir/matt.s

demo/matt/CMakeFiles/gotodemo.dir/matt.o.requires:
.PHONY : demo/matt/CMakeFiles/gotodemo.dir/matt.o.requires

demo/matt/CMakeFiles/gotodemo.dir/matt.o.provides: demo/matt/CMakeFiles/gotodemo.dir/matt.o.requires
	$(MAKE) -f demo/matt/CMakeFiles/gotodemo.dir/build.make demo/matt/CMakeFiles/gotodemo.dir/matt.o.provides.build
.PHONY : demo/matt/CMakeFiles/gotodemo.dir/matt.o.provides

demo/matt/CMakeFiles/gotodemo.dir/matt.o.provides.build: demo/matt/CMakeFiles/gotodemo.dir/matt.o

# Object files for target gotodemo
gotodemo_OBJECTS = \
"CMakeFiles/gotodemo.dir/matt.o"

# External object files for target gotodemo
gotodemo_EXTERNAL_OBJECTS =

demo/matt/matt: demo/matt/CMakeFiles/gotodemo.dir/matt.o
demo/matt/matt: demo/matt/CMakeFiles/gotodemo.dir/build.make
demo/matt/matt: code/libstagewrapper.so.3.0.3
demo/matt/matt: demo/matt/CMakeFiles/gotodemo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable matt"
	cd /home/alfmunny/TAS_wrapper/demo/matt && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gotodemo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
demo/matt/CMakeFiles/gotodemo.dir/build: demo/matt/matt
.PHONY : demo/matt/CMakeFiles/gotodemo.dir/build

demo/matt/CMakeFiles/gotodemo.dir/requires: demo/matt/CMakeFiles/gotodemo.dir/matt.o.requires
.PHONY : demo/matt/CMakeFiles/gotodemo.dir/requires

demo/matt/CMakeFiles/gotodemo.dir/clean:
	cd /home/alfmunny/TAS_wrapper/demo/matt && $(CMAKE_COMMAND) -P CMakeFiles/gotodemo.dir/cmake_clean.cmake
.PHONY : demo/matt/CMakeFiles/gotodemo.dir/clean

demo/matt/CMakeFiles/gotodemo.dir/depend:
	cd /home/alfmunny/TAS_wrapper && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alfmunny/TAS_wrapper /home/alfmunny/TAS_wrapper/demo/matt /home/alfmunny/TAS_wrapper /home/alfmunny/TAS_wrapper/demo/matt /home/alfmunny/TAS_wrapper/demo/matt/CMakeFiles/gotodemo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : demo/matt/CMakeFiles/gotodemo.dir/depend

