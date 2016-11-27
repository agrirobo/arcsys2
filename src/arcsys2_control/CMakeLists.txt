cmake_minimum_required(VERSION 2.8.3)
project(arcsys2_control)

set(${PROJECT_NAME}_DEPENDS
  actionlib
  actionlib_msgs
  control_msgs
  controller_manager
  hardware_interface
  roscpp
  trajectory_msgs
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_DEPENDS}
)

catkin_package(
  # INCLUDE_DIRS   include
  CATKIN_DEPENDS ${${PROJECT_NAME}_DEPENDS}
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(arcsys2_action_server_node
  src/arcsys2_action_server_node.cpp
)

add_executable(arcsys2_action_client_node
  src/arcsys2_action_client_node.cpp
)

add_dependencies(arcsys2_action_server_node
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(arcsys2_action_client_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(arcsys2_action_server_node
  ${catkin_LIBRARIES}
)

target_link_libraries(arcsys2_action_client_node
  ${catkin_LIBRARIES}
  ics3
)

add_executable(arcsys2_control_node src/arcsys2_control_node.cpp)
add_dependencies(arcsys2_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(arcsys2_control_node ${catkin_LIBRARIES})
