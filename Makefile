# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:

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
CMAKE_SOURCE_DIR = /home/brash/Light-Guide-MC

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/brash/Light-Guide-MC

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake --regenerate-during-build -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# Special rule for the target list_install_components
list_install_components:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Available install components are: \"Unspecified\""
.PHONY : list_install_components

# Special rule for the target list_install_components
list_install_components/fast: list_install_components
.PHONY : list_install_components/fast

# Special rule for the target install
install: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Install the project..."
	/usr/bin/cmake -P cmake_install.cmake
.PHONY : install

# Special rule for the target install
install/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Install the project..."
	/usr/bin/cmake -P cmake_install.cmake
.PHONY : install/fast

# Special rule for the target install/local
install/local: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing only the local directory..."
	/usr/bin/cmake -DCMAKE_INSTALL_LOCAL_ONLY=1 -P cmake_install.cmake
.PHONY : install/local

# Special rule for the target install/local
install/local/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing only the local directory..."
	/usr/bin/cmake -DCMAKE_INSTALL_LOCAL_ONLY=1 -P cmake_install.cmake
.PHONY : install/local/fast

# Special rule for the target install/strip
install/strip: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing the project stripped..."
	/usr/bin/cmake -DCMAKE_INSTALL_DO_STRIP=1 -P cmake_install.cmake
.PHONY : install/strip

# Special rule for the target install/strip
install/strip/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing the project stripped..."
	/usr/bin/cmake -DCMAKE_INSTALL_DO_STRIP=1 -P cmake_install.cmake
.PHONY : install/strip/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/brash/Light-Guide-MC/CMakeFiles /home/brash/Light-Guide-MC//CMakeFiles/progress.marks
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/brash/Light-Guide-MC/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named OpNovice2

# Build rule for target.
OpNovice2: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 OpNovice2
.PHONY : OpNovice2

# fast build rule for target.
OpNovice2/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/build
.PHONY : OpNovice2/fast

OpNovice2.o: OpNovice2.cc.o
.PHONY : OpNovice2.o

# target to build an object file
OpNovice2.cc.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/OpNovice2.cc.o
.PHONY : OpNovice2.cc.o

OpNovice2.i: OpNovice2.cc.i
.PHONY : OpNovice2.i

# target to preprocess a source file
OpNovice2.cc.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/OpNovice2.cc.i
.PHONY : OpNovice2.cc.i

OpNovice2.s: OpNovice2.cc.s
.PHONY : OpNovice2.s

# target to generate assembly for a file
OpNovice2.cc.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/OpNovice2.cc.s
.PHONY : OpNovice2.cc.s

src/ActionInitialization.o: src/ActionInitialization.cc.o
.PHONY : src/ActionInitialization.o

# target to build an object file
src/ActionInitialization.cc.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/ActionInitialization.cc.o
.PHONY : src/ActionInitialization.cc.o

src/ActionInitialization.i: src/ActionInitialization.cc.i
.PHONY : src/ActionInitialization.i

# target to preprocess a source file
src/ActionInitialization.cc.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/ActionInitialization.cc.i
.PHONY : src/ActionInitialization.cc.i

src/ActionInitialization.s: src/ActionInitialization.cc.s
.PHONY : src/ActionInitialization.s

# target to generate assembly for a file
src/ActionInitialization.cc.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/ActionInitialization.cc.s
.PHONY : src/ActionInitialization.cc.s

src/DetectorConstruction.o: src/DetectorConstruction.cc.o
.PHONY : src/DetectorConstruction.o

# target to build an object file
src/DetectorConstruction.cc.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/DetectorConstruction.cc.o
.PHONY : src/DetectorConstruction.cc.o

src/DetectorConstruction.i: src/DetectorConstruction.cc.i
.PHONY : src/DetectorConstruction.i

# target to preprocess a source file
src/DetectorConstruction.cc.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/DetectorConstruction.cc.i
.PHONY : src/DetectorConstruction.cc.i

src/DetectorConstruction.s: src/DetectorConstruction.cc.s
.PHONY : src/DetectorConstruction.s

# target to generate assembly for a file
src/DetectorConstruction.cc.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/DetectorConstruction.cc.s
.PHONY : src/DetectorConstruction.cc.s

src/DetectorMessenger.o: src/DetectorMessenger.cc.o
.PHONY : src/DetectorMessenger.o

# target to build an object file
src/DetectorMessenger.cc.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/DetectorMessenger.cc.o
.PHONY : src/DetectorMessenger.cc.o

src/DetectorMessenger.i: src/DetectorMessenger.cc.i
.PHONY : src/DetectorMessenger.i

# target to preprocess a source file
src/DetectorMessenger.cc.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/DetectorMessenger.cc.i
.PHONY : src/DetectorMessenger.cc.i

src/DetectorMessenger.s: src/DetectorMessenger.cc.s
.PHONY : src/DetectorMessenger.s

# target to generate assembly for a file
src/DetectorMessenger.cc.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/DetectorMessenger.cc.s
.PHONY : src/DetectorMessenger.cc.s

src/EventAction.o: src/EventAction.cc.o
.PHONY : src/EventAction.o

# target to build an object file
src/EventAction.cc.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/EventAction.cc.o
.PHONY : src/EventAction.cc.o

src/EventAction.i: src/EventAction.cc.i
.PHONY : src/EventAction.i

# target to preprocess a source file
src/EventAction.cc.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/EventAction.cc.i
.PHONY : src/EventAction.cc.i

src/EventAction.s: src/EventAction.cc.s
.PHONY : src/EventAction.s

# target to generate assembly for a file
src/EventAction.cc.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/EventAction.cc.s
.PHONY : src/EventAction.cc.s

src/HistoManager.o: src/HistoManager.cc.o
.PHONY : src/HistoManager.o

# target to build an object file
src/HistoManager.cc.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/HistoManager.cc.o
.PHONY : src/HistoManager.cc.o

src/HistoManager.i: src/HistoManager.cc.i
.PHONY : src/HistoManager.i

# target to preprocess a source file
src/HistoManager.cc.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/HistoManager.cc.i
.PHONY : src/HistoManager.cc.i

src/HistoManager.s: src/HistoManager.cc.s
.PHONY : src/HistoManager.s

# target to generate assembly for a file
src/HistoManager.cc.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/HistoManager.cc.s
.PHONY : src/HistoManager.cc.s

src/PrimaryGeneratorAction.o: src/PrimaryGeneratorAction.cc.o
.PHONY : src/PrimaryGeneratorAction.o

# target to build an object file
src/PrimaryGeneratorAction.cc.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/PrimaryGeneratorAction.cc.o
.PHONY : src/PrimaryGeneratorAction.cc.o

src/PrimaryGeneratorAction.i: src/PrimaryGeneratorAction.cc.i
.PHONY : src/PrimaryGeneratorAction.i

# target to preprocess a source file
src/PrimaryGeneratorAction.cc.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/PrimaryGeneratorAction.cc.i
.PHONY : src/PrimaryGeneratorAction.cc.i

src/PrimaryGeneratorAction.s: src/PrimaryGeneratorAction.cc.s
.PHONY : src/PrimaryGeneratorAction.s

# target to generate assembly for a file
src/PrimaryGeneratorAction.cc.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/PrimaryGeneratorAction.cc.s
.PHONY : src/PrimaryGeneratorAction.cc.s

src/PrimaryGeneratorMessenger.o: src/PrimaryGeneratorMessenger.cc.o
.PHONY : src/PrimaryGeneratorMessenger.o

# target to build an object file
src/PrimaryGeneratorMessenger.cc.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/PrimaryGeneratorMessenger.cc.o
.PHONY : src/PrimaryGeneratorMessenger.cc.o

src/PrimaryGeneratorMessenger.i: src/PrimaryGeneratorMessenger.cc.i
.PHONY : src/PrimaryGeneratorMessenger.i

# target to preprocess a source file
src/PrimaryGeneratorMessenger.cc.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/PrimaryGeneratorMessenger.cc.i
.PHONY : src/PrimaryGeneratorMessenger.cc.i

src/PrimaryGeneratorMessenger.s: src/PrimaryGeneratorMessenger.cc.s
.PHONY : src/PrimaryGeneratorMessenger.s

# target to generate assembly for a file
src/PrimaryGeneratorMessenger.cc.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/PrimaryGeneratorMessenger.cc.s
.PHONY : src/PrimaryGeneratorMessenger.cc.s

src/Run.o: src/Run.cc.o
.PHONY : src/Run.o

# target to build an object file
src/Run.cc.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/Run.cc.o
.PHONY : src/Run.cc.o

src/Run.i: src/Run.cc.i
.PHONY : src/Run.i

# target to preprocess a source file
src/Run.cc.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/Run.cc.i
.PHONY : src/Run.cc.i

src/Run.s: src/Run.cc.s
.PHONY : src/Run.s

# target to generate assembly for a file
src/Run.cc.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/Run.cc.s
.PHONY : src/Run.cc.s

src/RunAction.o: src/RunAction.cc.o
.PHONY : src/RunAction.o

# target to build an object file
src/RunAction.cc.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/RunAction.cc.o
.PHONY : src/RunAction.cc.o

src/RunAction.i: src/RunAction.cc.i
.PHONY : src/RunAction.i

# target to preprocess a source file
src/RunAction.cc.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/RunAction.cc.i
.PHONY : src/RunAction.cc.i

src/RunAction.s: src/RunAction.cc.s
.PHONY : src/RunAction.s

# target to generate assembly for a file
src/RunAction.cc.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/RunAction.cc.s
.PHONY : src/RunAction.cc.s

src/SteppingAction.o: src/SteppingAction.cc.o
.PHONY : src/SteppingAction.o

# target to build an object file
src/SteppingAction.cc.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/SteppingAction.cc.o
.PHONY : src/SteppingAction.cc.o

src/SteppingAction.i: src/SteppingAction.cc.i
.PHONY : src/SteppingAction.i

# target to preprocess a source file
src/SteppingAction.cc.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/SteppingAction.cc.i
.PHONY : src/SteppingAction.cc.i

src/SteppingAction.s: src/SteppingAction.cc.s
.PHONY : src/SteppingAction.s

# target to generate assembly for a file
src/SteppingAction.cc.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/SteppingAction.cc.s
.PHONY : src/SteppingAction.cc.s

src/TrackInformation.o: src/TrackInformation.cc.o
.PHONY : src/TrackInformation.o

# target to build an object file
src/TrackInformation.cc.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/TrackInformation.cc.o
.PHONY : src/TrackInformation.cc.o

src/TrackInformation.i: src/TrackInformation.cc.i
.PHONY : src/TrackInformation.i

# target to preprocess a source file
src/TrackInformation.cc.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/TrackInformation.cc.i
.PHONY : src/TrackInformation.cc.i

src/TrackInformation.s: src/TrackInformation.cc.s
.PHONY : src/TrackInformation.s

# target to generate assembly for a file
src/TrackInformation.cc.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/TrackInformation.cc.s
.PHONY : src/TrackInformation.cc.s

src/TrackingAction.o: src/TrackingAction.cc.o
.PHONY : src/TrackingAction.o

# target to build an object file
src/TrackingAction.cc.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/TrackingAction.cc.o
.PHONY : src/TrackingAction.cc.o

src/TrackingAction.i: src/TrackingAction.cc.i
.PHONY : src/TrackingAction.i

# target to preprocess a source file
src/TrackingAction.cc.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/TrackingAction.cc.i
.PHONY : src/TrackingAction.cc.i

src/TrackingAction.s: src/TrackingAction.cc.s
.PHONY : src/TrackingAction.s

# target to generate assembly for a file
src/TrackingAction.cc.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/OpNovice2.dir/build.make CMakeFiles/OpNovice2.dir/src/TrackingAction.cc.s
.PHONY : src/TrackingAction.cc.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... install"
	@echo "... install/local"
	@echo "... install/strip"
	@echo "... list_install_components"
	@echo "... rebuild_cache"
	@echo "... OpNovice2"
	@echo "... OpNovice2.o"
	@echo "... OpNovice2.i"
	@echo "... OpNovice2.s"
	@echo "... src/ActionInitialization.o"
	@echo "... src/ActionInitialization.i"
	@echo "... src/ActionInitialization.s"
	@echo "... src/DetectorConstruction.o"
	@echo "... src/DetectorConstruction.i"
	@echo "... src/DetectorConstruction.s"
	@echo "... src/DetectorMessenger.o"
	@echo "... src/DetectorMessenger.i"
	@echo "... src/DetectorMessenger.s"
	@echo "... src/EventAction.o"
	@echo "... src/EventAction.i"
	@echo "... src/EventAction.s"
	@echo "... src/HistoManager.o"
	@echo "... src/HistoManager.i"
	@echo "... src/HistoManager.s"
	@echo "... src/PrimaryGeneratorAction.o"
	@echo "... src/PrimaryGeneratorAction.i"
	@echo "... src/PrimaryGeneratorAction.s"
	@echo "... src/PrimaryGeneratorMessenger.o"
	@echo "... src/PrimaryGeneratorMessenger.i"
	@echo "... src/PrimaryGeneratorMessenger.s"
	@echo "... src/Run.o"
	@echo "... src/Run.i"
	@echo "... src/Run.s"
	@echo "... src/RunAction.o"
	@echo "... src/RunAction.i"
	@echo "... src/RunAction.s"
	@echo "... src/SteppingAction.o"
	@echo "... src/SteppingAction.i"
	@echo "... src/SteppingAction.s"
	@echo "... src/TrackInformation.o"
	@echo "... src/TrackInformation.i"
	@echo "... src/TrackInformation.s"
	@echo "... src/TrackingAction.o"
	@echo "... src/TrackingAction.i"
	@echo "... src/TrackingAction.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

