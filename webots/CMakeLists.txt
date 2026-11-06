add_executable(C2_supervisor controllers/C2_supervisor/C2_supervisor.cpp)

target_include_directories(C2_supervisor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${WEBOTS_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../simulator
)

target_link_libraries(C2_supervisor
    Qt5::Widgets
    Qt5::Network
    Qt5::Xml
    /Applications/Webots-R2025b.app/Contents/lib/controller/libCppController.dylib
    cbwebots
)

set_target_properties(C2_supervisor PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/controllers/C2_supervisor"
    BUILD_RPATH "/Applications"
)

add_executable(C3_supervisor controllers/C3_supervisor/C3_supervisor.cpp)

target_include_directories(C3_supervisor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${WEBOTS_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../simulator
)

target_link_libraries(C3_supervisor
    Qt5::Widgets
    Qt5::Network
    Qt5::Xml
    /Applications/Webots-R2025b.app/Contents/lib/controller/libCppController.dylib
    cbwebots
)

set_target_properties(C3_supervisor PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/controllers/C3_supervisor"
    BUILD_RPATH "/Applications"
)
