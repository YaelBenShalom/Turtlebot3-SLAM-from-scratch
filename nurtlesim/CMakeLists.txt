cmake_minimum_required(VERSION 3.9)
project(nurtlesim)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
find_package(
  catkin
  REQUIRED
  COMPONENTS
    geometry_msgs
    message_generation
    nav_msgs
    rigid2d
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
)

## System dependencies are found with CMake's conventions
find_package(
  Doxygen
)

set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md) # Use the readme in your doxygen docs
doxygen_add_docs(${PROJECT_NAME}_doxygen src/ README.md ALL)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS 
    geometry_msgs
    message_runtime
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(tube_world src/tube_world.cpp)

## Rename C++ executable without prefix
set_target_properties(tube_world PROPERTIES OUTPUT_NAME tube_world PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(tube_world ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(
  tube_world
  ${rigid2d_LIBRARIES}
  ${nuturtlebot_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(
  TARGETS tube_world
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/config
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/launch
)
