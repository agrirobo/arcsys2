cmake_minimum_required(VERSION 2.8.3)
project(kinect_tomato_searcher CXX)

add_definitions("-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

generate_dynamic_reconfigure_options(
  cfg/Search.cfg
)

catkin_package(CATKIN_DEPENDS
  roscpp
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(kinect_tomato_searcher_node src/kinect_tomato_searcher_node.cpp)
add_executable(kinect_tomato_broadcaster_node src/kinect_tomato_broadcaster_node.cpp)
add_executable(kinect_tomato_selector_node src/kinect_tomato_selector_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(kinect_tomato_searcher_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(kinect_tomato_broadcaster_node ${catkin_EXPORTED_TARGETS})
add_dependencies(kinect_tomato_selector_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(kinect_tomato_searcher_node ${catkin_LIBRARIES})
target_link_libraries(kinect_tomato_broadcaster_node ${catkin_LIBRARIES})
target_link_libraries(kinect_tomato_selector_node ${catkin_LIBRARIES})
