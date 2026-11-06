
# --- Find Webots ---
# Try to find the Supervisor.hpp file to determine the include directory
find_path(WEBOTS_INCLUDE_DIR
    NAMES webots/Supervisor.hpp
    HINTS /Applications/Webots-R2025b.app/Contents/include/controller/cpp /usr/local/include /usr/local/webots/include/controller/cpp # Add common installation paths as hints
    PATH_SUFFIXES webots
    DOC "Directory containing Webots headers"
)

if(NOT WEBOTS_INCLUDE_DIR)
    message(FATAL_ERROR "Webots include directory not found. Please ensure Webots is installed and its include path is accessible.")
endif()

message(STATUS "Webots include directory found: ${WEBOTS_INCLUDE_DIR}")

set(sim_SRC
    # Source
    cbbeacon.cpp
    cbgraph.cpp
    cblab.cpp
    cblabhandler.cpp
    cbpoint.cpp
    cbtarget.cpp
    cbwall.cpp
)

add_library(cbwebots SHARED ${sim_SRC})

target_include_directories(cbwebots PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${WEBOTS_INCLUDE_DIR})
target_link_libraries(cbwebots
    Qt5::Widgets Qt5::Network Qt5::Xml
    /Applications/Webots-R2025b.app/Contents/lib/controller/libCppController.dylib
)

set_target_properties(cbwebots PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
