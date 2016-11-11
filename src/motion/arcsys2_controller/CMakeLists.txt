cmake_minimum_required(VERSION 2.8.3)
project(arcsys2_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  arm_msgs
  controller_manager
  hardware_interface
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arcsys2_controller
  CATKIN_DEPENDS roscpp arm_msgs controller_manager hardware_interface
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(arcsys2_controller_node src/arcsys2_controller_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(arcsys2_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(arcsys2_controller_node
  ${catkin_LIBRARIES}
)
