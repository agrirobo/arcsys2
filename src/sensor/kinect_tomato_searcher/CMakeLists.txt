cmake_minimum_required(VERSION 2.8.3)
project(kinect_tomato_search)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES kinect_tomato_search
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs geometry_msgs
  #  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(kinect_tomato_search_node src/kinect_tomato_search_node.cpp)
add_executable(kinect_tomato_group_search_node src/kinect_tomato_group_search_node.cpp)
add_executable(tomato_broadcaster src/tomato_broadcaster.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(kinect_tomato_search_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} kinect_tomato_search_generate_messages_cpp)
add_dependencies(kinect_tomato_group_search_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} kinect_tomato_search_generate_messages_cpp)
add_dependencies(tomato_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} kinect_tomato_search_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(kinect_tomato_search_node ${catkin_LIBRARIES})
target_link_libraries(kinect_tomato_group_search_node ${catkin_LIBRARIES})
target_link_libraries(tomato_broadcaster ${catkin_LIBRARIES})
