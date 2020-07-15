# Generate a list of files and store them in variables FABRIK_SOURCE_CODE
# and FABRIK_HEADERS for source and headers
file(GLOB_RECURSE FABRIK_SOURCE_CODE robot_model/*.cpp robot_state/*.cpp util/*.cpp base/*.cpp)
file(GLOB_RECURSE FABRIK_HEADERS robot_model/*.h robot_state/*.h util/*.h base/*.h)

# make fabrik library here
add_library(fabrik SHARED ${FABRIK_SOURCE_CODE})
# target_link_libraries(fabrik Eigen3::Eigen)

# install the library
install(TARGETS fabrik 
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)


