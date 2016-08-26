cmake_minimum_required(VERSION 2.8.3)
project(main_aciton_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  servo_msgs
  tf2
  tf2_ros
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
#  LIBRARIES main_aciton_planner
  CATKIN_DEPENDS roscpp servo_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(main_aciton_planner_node src/main_aciton_planner_node.cpp)

add_dependencies(main_aciton_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(main_aciton_planner_node
  ${catkin_LIBRARIES}
)

