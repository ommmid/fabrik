
#  add_libraries are here, also install
# no include_dirs here

# Generate a list of files and store them in variables FABRIK_SOURCE_CODE
# and FABRIK_HEADERS for source and headers
file(GLOB_RECURSE FABRIK_SOURCE_CODE  world/src/*.cpp model/src/*.cpp)
file(GLOB_RECURSE FABRIK_HEADERS      world/*.h   model/*.h)

add_library(exception INTERFACE)
target_include_directories(exception INTERFACE util/)

include_directories(world)
add_library(world world/src/world.cpp)

find_package(Eigen3 REQUIRED)
include_directories(
    SYSTEM ${EIGEN3_INCLUDE_DIRS}
)

include_directories(model)
add_library(fabrik ${FABRIK_SOURCE_CODE})
target_link_libraries(fabrik Eigen3::Eigen exception)

add_executable(hello_world world/tests/test_world.cpp)
target_link_libraries(hello_world world)

add_executable(test_robot_model model/tests/test_robot_model.cpp)
target_link_libraries(test_robot_model world fabrik Eigen3::Eigen)

