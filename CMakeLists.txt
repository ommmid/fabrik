cmake_minimum_required(VERSION 3.0.0)
project(fabrik VERSION 0.1.0)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# message([WARNING] ${CMAKE_INSTALL_LIBDIR})
# set(LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})

# do the include_dirs here

# ${CMAKE_CURRENT_SOURCE_DIR} is the address of where this CMakeLists.txt
set(FABRIK_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src;${CMAKE_CURRENT_BINARY_DIR}/src")
include_directories("${FABRIK_INCLUDE_DIRS}")

enable_testing()
# include(CTest)

add_subdirectory(src)
add_subdirectory(tests)











