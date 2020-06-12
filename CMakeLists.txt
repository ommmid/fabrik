cmake_minimum_required(VERSION 3.0.0)
project(fabrik VERSION 0.1.0)

# do the include_dirs here

# ${CMAKE_CURRENT_SOURCE_DIR} is the address of where this CMakeLists.txt
set(FABRIK_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src;${CMAKE_CURRENT_BINARY_DIR}/src")
include_directories("${FABRIK_INCLUDE_DIRS}")

enable_testing()
# include(CTest)

add_subdirectory(src)
add_subdirectory(tests)



# how to cout message in boost unit test








