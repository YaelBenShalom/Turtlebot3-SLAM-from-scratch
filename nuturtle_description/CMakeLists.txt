cmake_minimum_required(VERSION 3.9)
project(nuturtle_description)

## Find catkin macros and libraries
find_package(
  catkin
  REQUIRED
  COMPONENTS
    roscpp
    urdf
    xacro
)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package( 
  CATKIN_DEPENDS
    roscpp
    urdf
    xacro
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY
    meshes
    urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
