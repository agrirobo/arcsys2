cmake_minimum_required(VERSION 2.8.3)
project(kinect_tomato_searcher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  geometry_msgs
  image_transport
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS roscpp cv_bridge geometry_msgs image_transport sensor_msgs std_msgs tf2 tf2_ros
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(kinect_tomato_searcher_node src/kinect_tomato_searcher_node.cpp)
add_executable(kinect_tomato_group_search_node src/kinect_tomato_group_search_node.cpp)
add_executable(tomato_broadcaster src/tomato_broadcaster.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(kinect_tomato_searcher_node ${catkin_EXPORTED_TARGETS})
add_dependencies(kinect_tomato_group_search_node ${catkin_EXPORTED_TARGETS})
add_dependencies(tomato_broadcaster ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(kinect_tomato_searcher_node ${catkin_LIBRARIES})
target_link_libraries(kinect_tomato_group_search_node ${catkin_LIBRARIES})
target_link_libraries(tomato_broadcaster ${catkin_LIBRARIES})