cmake_minimum_required(VERSION 3.0)
project(DetectionEnhancer)

find_package( OpenCV REQUIRED )
find_package( Threads REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
add_library(kalman src/kalman.cpp)
add_library(objecthistory src/objecthistory.cpp)
add_library(detection src/detection.cpp)
add_library(genboxes src/genboxes.cpp)

add_executable(app src/main.cpp)
add_executable(test_udu test/test.cpp src/utils.hpp)
# add_executable(genboxes src/genboxes.cpp)

set_target_properties(app kalman genboxes objecthistory test_udu PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
	RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

target_link_libraries(app ${OpenCV_LIBS} kalman objecthistory detection genboxes)
# target_link_libraries(genboxes ${OpenCV_LIBS})
target_link_libraries(test_udu ${OpenCV_LIBS})