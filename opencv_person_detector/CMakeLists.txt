# Required version of CMake
cmake_minimum_required(VERSION 3.10)

# Name of this package
project(opencv_person_detector)

# Find the catkin build system and the openCV package 
find_package(
    catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp 
    sensor_msgs 
    vision_msgs 
)
find_package(
    OpenCV REQUIRED
)

# Declare our catkin package
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs vision_msgs 
)

# Specify locations of header files of both catkin and OpenCV
include_directories(
    include 
    ${catkin_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS}
) 

# Declare the executable along with its source files
add_executable(
    ${PROJECT_NAME}_node 
    src/opencv_person_detector_node.cpp 
    src/person_detector.cpp
)

# Tell CMake the compilation targets have dependencies
add_dependencies(
    ${PROJECT_NAME}_node 
    ${catkin_EXPORTED_TARGETS}
)

# Specify libraries against which to link
target_link_libraries(
    ${PROJECT_NAME}_node 
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS}
) 

# Install executables
install(
    TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install cpp header files
install(
    DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)
