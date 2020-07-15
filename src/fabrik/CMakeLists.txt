
# Generate a list of files and store them in variables FABRIK_SOURCE_CODE
# and FABRIK_HEADERS for source and headers
file(GLOB_RECURSE FABRIK_SOURCE_CODE world/*.cpp robot_model/*.cpp robot_state/*.cpp util/*.cpp base/*.cpp)
file(GLOB_RECURSE FABRIK_HEADERS world/*.h robot_model/*.h robot_state/*.h util/*.h base/*.h)


# make fabrik library here
add_library(fabrik SHARED ${FABRIK_SOURCE_CODE})
target_link_libraries(fabrik Eigen3::Eigen)


# install the library
install(TARGETS fabrik
  		DESTINATION ${CMAKE_INSTALL_LIBDIR}
  	  	COMPONENT fabrik)


# ----- install headers
# installs the content of DIRECTORY to a given DESTINATION
# The directory structure is copied verbatim(word by word = exactly) to the destination
install(
    DIRECTORY fabrik/ 
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/fabrik"
    # COMPONENT fabrik
    FILES_MATCHING 
    PATTERN "*.h" 
    PATTERN ".DS_Store" EXCLUDE
    PATTERN "sample_contrib" EXCLUDE
    REGEX "/src$" EXCLUDE
    REGEX "/doc$" EXCLUDE
    REGEX "/tests$" EXCLUDE
)



