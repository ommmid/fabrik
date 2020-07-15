add_subdirectory(fabrik)

# ----- install headers
# installs the DIRECTORY and its contents to a given DESTINATION
# The directory structure is copied verbatim(word by word = exactly) to the destination
install(
    DIRECTORY fabrik/
    DESTINATION  ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT fabrik
    FILES_MATCHING 
    PATTERN "*.h" 
    PATTERN ".DS_Store" EXCLUDE
    PATTERN "sample_contrib" EXCLUDE
    REGEX "/src$" EXCLUDE
    REGEX "/doc$" EXCLUDE
    REGEX "/tests$" EXCLUDE
)

