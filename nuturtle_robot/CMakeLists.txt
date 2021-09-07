cmake_minimum_required(VERSION 3.9)
project(nuturtle_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-Wall -Wextra -pipe -Wno-psabi)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
find_package(
  catkin 
  REQUIRED
  COMPONENTS
    catch_ros
    geometry_msgs
    message_generation
    message_runtime
    nav_msgs
    nuturtlebot
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

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Control.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

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
add_executable(turtle_interface_node src/turtle_interface_node.cpp)
add_executable(follow_circle_node src/follow_circle_node.cpp)

## Rename C++ executable without prefix
set_target_properties(turtle_interface_node PROPERTIES OUTPUT_NAME turtle_interface_node PREFIX "")
set_target_properties(follow_circle_node PROPERTIES OUTPUT_NAME follow_circle_node PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(turtle_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(follow_circle_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(
  turtle_interface_node
  ${rigid2d_LIBRARIES}
  ${nuturtlebot_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(
  follow_circle_node
  ${rigid2d_LIBRARIES}
  ${nuturtlebot_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS follow_circle_node turtle_interface_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/launch)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catch_add_rostest_node(turtle_interface_test tests/turtle_interface_test.cpp)
  target_link_libraries(turtle_interface_test ${catkin_LIBRARIES} ${rigid2d_LIBRARIES} ${nuturtlebot_LIBRARIES})
  add_rostest(tests/turtle_interface_test.test)
endif()