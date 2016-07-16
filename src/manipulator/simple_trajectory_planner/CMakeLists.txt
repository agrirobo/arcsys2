cmake_minimum_required(VERSION 2.8.3)
project(simple_trajectory_planner)

find_package(catkin REQUIRED roscpp tf2 tf2_ros geometry_msgs servo_msgs)

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
#  LIBRARIES simple_trajectory_planner
  CATKIN_DEPENDS roscpp tf2 tf2_ros geometry_msgs servo_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Declare a C++ executable
add_executable(simple_trajectory_planner_node src/trajectory_planner.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(simple_trajectory_planner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(simple_trajectory_planner_node
  ${catkin_LIBRARIES}
  math
)
