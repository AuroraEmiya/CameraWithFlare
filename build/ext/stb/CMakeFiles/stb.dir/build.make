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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sakura/DoubleLens

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sakura/DoubleLens/build

# Include any dependencies generated for this target.
include ext/stb/CMakeFiles/stb.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include ext/stb/CMakeFiles/stb.dir/compiler_depend.make

# Include the progress variables for this target.
include ext/stb/CMakeFiles/stb.dir/progress.make

# Include the compile flags for this target's objects.
include ext/stb/CMakeFiles/stb.dir/flags.make

ext/stb/CMakeFiles/stb.dir/stb.cpp.o: ext/stb/CMakeFiles/stb.dir/flags.make
ext/stb/CMakeFiles/stb.dir/stb.cpp.o: /home/sakura/DoubleLens/ext/stb/stb.cpp
ext/stb/CMakeFiles/stb.dir/stb.cpp.o: ext/stb/CMakeFiles/stb.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/sakura/DoubleLens/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ext/stb/CMakeFiles/stb.dir/stb.cpp.o"
	cd /home/sakura/DoubleLens/build/ext/stb && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT ext/stb/CMakeFiles/stb.dir/stb.cpp.o -MF CMakeFiles/stb.dir/stb.cpp.o.d -o CMakeFiles/stb.dir/stb.cpp.o -c /home/sakura/DoubleLens/ext/stb/stb.cpp

ext/stb/CMakeFiles/stb.dir/stb.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/stb.dir/stb.cpp.i"
	cd /home/sakura/DoubleLens/build/ext/stb && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sakura/DoubleLens/ext/stb/stb.cpp > CMakeFiles/stb.dir/stb.cpp.i

ext/stb/CMakeFiles/stb.dir/stb.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/stb.dir/stb.cpp.s"
	cd /home/sakura/DoubleLens/build/ext/stb && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sakura/DoubleLens/ext/stb/stb.cpp -o CMakeFiles/stb.dir/stb.cpp.s

# Object files for target stb
stb_OBJECTS = \
"CMakeFiles/stb.dir/stb.cpp.o"

# External object files for target stb
stb_EXTERNAL_OBJECTS =

/home/sakura/DoubleLens/target/lib/libstb.so: ext/stb/CMakeFiles/stb.dir/stb.cpp.o
/home/sakura/DoubleLens/target/lib/libstb.so: ext/stb/CMakeFiles/stb.dir/build.make
/home/sakura/DoubleLens/target/lib/libstb.so: ext/stb/CMakeFiles/stb.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/sakura/DoubleLens/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/sakura/DoubleLens/target/lib/libstb.so"
	cd /home/sakura/DoubleLens/build/ext/stb && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stb.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ext/stb/CMakeFiles/stb.dir/build: /home/sakura/DoubleLens/target/lib/libstb.so
.PHONY : ext/stb/CMakeFiles/stb.dir/build

ext/stb/CMakeFiles/stb.dir/clean:
	cd /home/sakura/DoubleLens/build/ext/stb && $(CMAKE_COMMAND) -P CMakeFiles/stb.dir/cmake_clean.cmake
.PHONY : ext/stb/CMakeFiles/stb.dir/clean

ext/stb/CMakeFiles/stb.dir/depend:
	cd /home/sakura/DoubleLens/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sakura/DoubleLens /home/sakura/DoubleLens/ext/stb /home/sakura/DoubleLens/build /home/sakura/DoubleLens/build/ext/stb /home/sakura/DoubleLens/build/ext/stb/CMakeFiles/stb.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : ext/stb/CMakeFiles/stb.dir/depend

