# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

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

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running interactive CMake command-line interface..."
	/usr/bin/cmake -i .
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	cd /home/alfmunny/TAS_wrapper && $(CMAKE_COMMAND) -E cmake_progress_start /home/alfmunny/TAS_wrapper/CMakeFiles /home/alfmunny/TAS_wrapper/demo/matt/CMakeFiles/progress.marks
	cd /home/alfmunny/TAS_wrapper && $(MAKE) -f CMakeFiles/Makefile2 demo/matt/all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/alfmunny/TAS_wrapper/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	cd /home/alfmunny/TAS_wrapper && $(MAKE) -f CMakeFiles/Makefile2 demo/matt/clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	cd /home/alfmunny/TAS_wrapper && $(MAKE) -f CMakeFiles/Makefile2 demo/matt/preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	cd /home/alfmunny/TAS_wrapper && $(MAKE) -f CMakeFiles/Makefile2 demo/matt/preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	cd /home/alfmunny/TAS_wrapper && $(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

# Convenience name for target.
demo/matt/CMakeFiles/gotodemo.dir/rule:
	cd /home/alfmunny/TAS_wrapper && $(MAKE) -f CMakeFiles/Makefile2 demo/matt/CMakeFiles/gotodemo.dir/rule
.PHONY : demo/matt/CMakeFiles/gotodemo.dir/rule

# Convenience name for target.
gotodemo: demo/matt/CMakeFiles/gotodemo.dir/rule
.PHONY : gotodemo

# fast build rule for target.
gotodemo/fast:
	cd /home/alfmunny/TAS_wrapper && $(MAKE) -f demo/matt/CMakeFiles/gotodemo.dir/build.make demo/matt/CMakeFiles/gotodemo.dir/build
.PHONY : gotodemo/fast

# target to build an object file
matt.o:
	cd /home/alfmunny/TAS_wrapper && $(MAKE) -f demo/matt/CMakeFiles/gotodemo.dir/build.make demo/matt/CMakeFiles/gotodemo.dir/matt.o
.PHONY : matt.o

# target to preprocess a source file
matt.i:
	cd /home/alfmunny/TAS_wrapper && $(MAKE) -f demo/matt/CMakeFiles/gotodemo.dir/build.make demo/matt/CMakeFiles/gotodemo.dir/matt.i
.PHONY : matt.i

# target to generate assembly for a file
matt.s:
	cd /home/alfmunny/TAS_wrapper && $(MAKE) -f demo/matt/CMakeFiles/gotodemo.dir/build.make demo/matt/CMakeFiles/gotodemo.dir/matt.s
.PHONY : matt.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... gotodemo"
	@echo "... rebuild_cache"
	@echo "... matt.o"
	@echo "... matt.i"
	@echo "... matt.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	cd /home/alfmunny/TAS_wrapper && $(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

