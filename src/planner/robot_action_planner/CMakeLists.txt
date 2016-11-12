cmake_minimum_required(VERSION 2.8.3)
project(robot_action_planner CXX)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")

set(${PROJECT_NAME}_DEPENDS
  roscpp
)

find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_DEPENDS}
)

catkin_package(
  INCLUDE_DIR    ${PROJECT_SOURCE_DIR}/include
  CATKIN_DEPENDS ${${PROJECT_NAME}_DEPENDS}
  DEPENDS        system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_SOURCES}
)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
