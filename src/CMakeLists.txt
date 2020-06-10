add_subdirectory(fabrik)
install(
    DIRECTORY fabrik/ 
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/fabrik"
    COMPONENT fabrik
    FILES_MATCHING 
    PATTERN "*.h"
    PATTERN ".DS_Store" EXCLUDE
    PATTERN "sample_contrib" EXCLUDE
    REGEX "/src$" EXCLUDE
    REGEX "/doc$" EXCLUDE
    REGEX "/tests$" EXCLUDE)
