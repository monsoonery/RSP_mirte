# Required version of CMake
cmake_minimum_required(VERSION 3.10)

# Name of this package
project(control_barrel_world)

# Find the catkin build system
find_package(
    catkin REQUIRED COMPONENTS 
    geometry_msgs 
    roscpp
    vision_msgs
)

# Declare our catkin package
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS geometry_msgs roscpp vision_msgs
)

# Specify locations of header files
include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
) 

# Declare the executable along with its source files
add_executable(
    ${PROJECT_NAME}_node 
    src/control_barrel_world_node.cpp 
    src/mirte_controller.cpp
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

# Install launch file
install(
    FILES launch/solution.launch 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
