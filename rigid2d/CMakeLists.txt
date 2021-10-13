cmake_minimum_required(VERSION 3.9)
project(rigid2d)

## Find catkin macros and libraries
find_package(
  catkin
  REQUIRED
  COMPONENTS
    catch_ros
    geometry_msgs
    message_generation
    nav_msgs
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
  SetPose.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
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
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/diff_drive.cpp
  src/rigid2d.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include/)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17) # enable C++ 17
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(${PROJECT_NAME}_main src/main.cpp)
add_executable(odometer src/odometer.cpp)
add_executable(fake_turtle src/fake_turtle.cpp)

## Rename C++ executable without prefix
set_target_properties(fake_turtle PROPERTIES OUTPUT_NAME fake_turtle PREFIX "")
set_target_properties(odometer PROPERTIES OUTPUT_NAME odometer PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(odometer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(fake_turtle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_main ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(odometer ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(fake_turtle ${catkin_LIBRARIES} ${PROJECT_NAME})


#############
## Install ##
#############

## Mark executables for installation
install(TARGETS fake_turtle odometer ${PROJECT_NAME}_main 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/config
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  catch_add_test(${PROJECT_NAME}_test tests/${PROJECT_NAME}_test.cpp tests/diff_drive_test.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()