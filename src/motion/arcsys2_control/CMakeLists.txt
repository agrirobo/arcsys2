cmake_minimum_required(VERSION 2.8.3)
project(arcsys2_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
  hardware_interface
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp controller_manager hardware_interface
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(arcsys2_control_node src/arcsys2_control_node.cpp)
add_dependencies(arcsys2_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(arcsys2_control_node ${catkin_LIBRARIES})
