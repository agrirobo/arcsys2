cmake_minimum_required(VERSION 2.8.3)
project(close_tomato_searcher)

set(CATKIN_DEPS cv_bridge image_transport roscpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS})

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
#  LIBRARIES search_close_tomato
CATKIN_DEPENDS ${CATKIN_DEPS}
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(search_close_tomato_node src/search_close_tomato_node.cpp)
add_executable(close_tomato_broadcaster src/close_tomato_broadcaster.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(search_close_tomato_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(close_tomato_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(search_close_tomato_node
  ${catkin_LIBRARIES}
)
target_link_libraries(close_tomato_broadcaster
  ${catkin_LIBRARIES}
)
