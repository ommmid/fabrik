
#  add_libraries are here, also install
# no include_dirs here

# Generate a list of files and store them in variables FABRIK_SOURCE_CODE
# and FABRIK_HEADERS for source and headers
file(GLOB_RECURSE FABRIK_SOURCE_CODE world/*.cpp robot_model/*.cpp robot_state/*.cpp util/*.cpp base/*.cpp)
file(GLOB_RECURSE FABRIK_HEADERS world/*.h robot_model/*.h robot_state/*.h util/*.h base/*.h)

# add_library(exception INTERFACE)
# target_include_directories(exception INTERFACE util/)

find_package(Eigen3 REQUIRED)
include_directories(
    SYSTEM ${EIGEN3_INCLUDE_DIRS}
)


# make fabrik library here
include_directories(robot_model world robot_state util)
add_library(fabrik SHARED ${FABRIK_SOURCE_CODE})
target_link_libraries(fabrik Eigen3::Eigen)


