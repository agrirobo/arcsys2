cmake_minimum_required(VERSION 2.8.3)
project(main_action_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  state_msgs
  std_srvs
  tf2
  tf2_ros
)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp state_msgs std_srvs tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(main_action_planner_node src/main_action_planner_node.cpp src/request_posture.cpp src/send_posture.cpp src/state.cpp)
add_dependencies(main_action_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_action_planner_node
  ${catkin_LIBRARIES}
)

