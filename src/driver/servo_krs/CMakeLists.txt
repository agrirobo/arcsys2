cmake_minimum_required(VERSION 2.8.3)
project(servo_krs)

ADD_DEFINITIONS(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  servo_msgs
)

#find_package(ics3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PACKAGE_NAME}
  CATKIN_DEPENDS roscpp servo_msgs
  DEPENDS ics3
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ics3
)
