cmake_minimum_required(VERSION 2.8.3)
project(krs_servo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  servo_msgs
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
#  LIBRARIES KrsServo
  CATKIN_DEPENDS roscpp servo_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(krs_servo_node src/krs_servo_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(krs_servo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(Kkrs_servo_node
  krsservo
  ${catkin_LIBRARIES}
)

