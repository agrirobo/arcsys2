cmake_minimum_required(VERSION 2.8.3)
project(arcsys2_move_group_interface CXX)

add_definitions("-std=c++11")

set(${PROJECT_NAME}_DEPENDS
  roscpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  tf2
  tf2_ros
)

file(GLOB ${PROJECT_NAME}_SOURCES
  # ${PROJECT_SOURCE_DIR}/src/*.cpp
  ${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME}.cpp
)

find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_DEPENDS}
)

catkin_package(
  CATKIN_DEPENDS ${${PROJECT_NAME}_DEPENDS}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_SOURCES}
)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
