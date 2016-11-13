cmake_minimum_required(VERSION 2.8.3)
project(arcsys2_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
  hardware_interface
)

find_package(ics3)

INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  MESSAGE(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp controller_manager hardware_interface
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ics3_INCLUDE_DIRS}
)

add_executable(arcsys2_control_node src/arcsys2_control_node.cpp)
add_dependencies(arcsys2_control_node ${catkin_EXPORTED_TARGETS} ${ics3_EXPORTED_TARGETS})
target_link_libraries(arcsys2_control_node ${catkin_LIBRARIES} ${ics3_LIBRARIES})
