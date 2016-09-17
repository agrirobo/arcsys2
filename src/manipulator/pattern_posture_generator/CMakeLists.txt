cmake_minimum_required(VERSION 2.8.3)
project(pattern_posture_generator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(
  FILES
  PatternKeyPosture.srv
  PatternPosture.srv
  RequestNum.srv
)

generate_messages(
#  DEPENDENCIES
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
  INCLUDE_DIRS include
#  LIBRARIES pattern_posture_generator
  CATKIN_DEPENDS roscpp message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pattern_posture_generator_node src/pattern_posture_generator_node.cpp)

add_dependencies(pattern_posture_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

target_link_libraries(pattern_posture_generator_node
  ${catkin_LIBRARIES}
)

