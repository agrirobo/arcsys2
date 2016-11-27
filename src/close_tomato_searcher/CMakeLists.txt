cmake_minimum_required(VERSION 2.8.3)
project(close_tomato_searcher)

set(CATKIN_DEPS
  roscpp
  cv_bridge
  geometry_msgs
  image_transport
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS})

catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPS}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(close_tomato_searcher_node
  src/close_tomato_searcher_node.cpp
)
add_executable(close_tomato_broadcaster
  src/close_tomato_broadcaster.cpp
)

add_dependencies(close_tomato_searcher_node
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(close_tomato_broadcaster
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(close_tomato_searcher_node
  ${catkin_LIBRARIES}
)
target_link_libraries(close_tomato_broadcaster
  ${catkin_LIBRARIES}
)
