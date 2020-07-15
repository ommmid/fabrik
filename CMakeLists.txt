cmake_minimum_required(VERSION 3.5)
project(fabrik
        VERSION 0.1.0
)

# --- set the default build type
set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Type of build" FORCE)
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# --- GNUInstallDirs set the variable for paths (relative or absolute)
# the base point(absolute path starting point), ${CMAKE_INSTALL_PREFIX}, is set to /usr/local
include(GNUInstallDirs)
message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")
message(STATUS "CMAKE_INSTALL_LIBDIR: ${CMAKE_INSTALL_LIBDIR}")
message(STATUS "CMAKE_INSTALL_INCLUDEDIR: ${CMAKE_INSTALL_INCLUDEDIR}")
message(STATUS "CMAKE_INSTALL_DATAROOTDIR: ${CMAKE_INSTALL_DATAROOTDIR}")


# --- find depend upon packages
find_package(Eigen3 REQUIRED)
include_directories(
    ${EIGEN3_INCLUDE_DIRS}
)

# create config file using cmake helper
include(CMakePackageConfigHelpers)
# 1 arg: input, 2 arg: ouput
# INSTALL_DESTINATION is where the ouput will be installed
# NO_CHECK_REQUIRED_COMPONENTS_MACRO prevents @package_init@ from defining some helper function
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})
set(LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
configure_package_config_file(fabrikConfig.cmake.in ${CMAKE_BINARY_DIR}/fabrikConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/fabrik/cmake
    PATH_VARS INCLUDE_INSTALL_DIR LIB_INSTALL_DIR
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

enable_testing()
# include(CTest)

# --- This is a list of directories to be used as header search paths, 
# all of which must be absolute paths
set(FABRIK_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/src;${CMAKE_BINARY_DIR}/src")
include_directories(${FABRIK_INCLUDE_DIRS})
# the paths after FABRIK_INCLUDE_DIRS should be used to reach to the specific header file. example:
# to use world.h anywhere in *.cpp files of this project, I have to include "fabrik/world/world.h"
# full path: [CMAKE_CURRENT_SOURCE_DIR/src/][fabrik/world/world.h]

# put shared and static library in lib folder and executables in bin folder.
# these must be set before add_library and add_executable
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(src)
add_subdirectory(tests)

# fabrikConfig.cmake was created by configure_package_config_file() using fabrikConfig.cmake.in that I have created
install(FILES ${CMAKE_BINARY_DIR}/fabrikConfig.cmake
    # ${CMAKE_CURRENT_BINARY_DIR}/fabrikConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/fabrik/cmake
    #COMPONENT ompl what is component ??????????????????????????
)
