cmake_minimum_required(VERSION 2.8.3)
project(arcsys2_control)

set(${PROJECT_NAME}_DEPENDS
  roscpp
  controller_manager
  hardware_interface
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
  CATKIN_DEPENDS ${${PROJECT_NAME}_DEPENDS}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(arcsys2_control_node
  src/arcsys2_control_node.cpp
)

add_dependencies(arcsys2_control_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(arcsys2_control_node
  ${catkin_LIBRARIES}
)

add_executable(arcsys2_dammy_hw_node
  src/arcsys2_dammy_hw_node.cpp
)

add_dependencies(arcsys2_dammy_hw_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(arcsys2_dammy_hw_node
  ${catkin_LIBRARIES}
)
