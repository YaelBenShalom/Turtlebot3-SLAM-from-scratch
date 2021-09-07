cmake_minimum_required(VERSION 3.9)
project(nuslam)

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

find_package(
  Armadillo
  REQUIRED
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
## Your package locations should be listed before other locations
include_directories(
  include
  ${ARMADILLO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC include/)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17) # enable C++ 17
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(visualization src/visualization.cpp)
add_executable(slam src/slam.cpp)
add_executable(landmarks src/landmarks.cpp)

## Rename C++ executable without prefix
set_target_properties(visualization PROPERTIES OUTPUT_NAME visualization PREFIX "")
set_target_properties(slam PROPERTIES OUTPUT_NAME slam PREFIX "")
set_target_properties(landmarks PROPERTIES OUTPUT_NAME landmarks PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(visualization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(slam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(landmarks ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(
  visualization
  ${rigid2d_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

target_link_libraries(
  slam
  ${rigid2d_LIBRARIES}
  ${catkin_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
  ${PROJECT_NAME}
)

target_link_libraries(
  landmarks
  ${rigid2d_LIBRARIES}
  ${catkin_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
  ${PROJECT_NAME}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${rigid2d_LIBRARIES}
  ${catkin_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS
  landmarks
  slam
  visualization 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/config)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
