cmake_minimum_required(VERSION 3.0)
project(DetectionEnhancer)

find_package( OpenCV REQUIRED )
find_package( Threads REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_library(kalman src/kalman.cpp src/kalman.hpp)
add_library(objecthistory src/objecthistory.cpp src/objecthistory.hpp)
add_library(detection src/detection.cpp src/detection.hpp)
add_library(genboxes src/genboxes.cpp)

add_executable(app src/main.cpp)
# add_executable(genboxes src/genboxes.cpp)

set_target_properties(app kalman genboxes objecthistory PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
	RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

target_link_libraries(app ${OpenCV_LIBS} kalman objecthistory detection genboxes)
target_link_libraries(genboxes ${OpenCV_LIBS})