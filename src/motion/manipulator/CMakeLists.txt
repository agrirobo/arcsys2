cmake_minimum_required(VERSION 2.8.3)
project(manipulator CXX)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")

set(${PROJECT_NAME}_DEPENDS
  roscpp
)

set(${PROJECT_NAME}_BUILD_DEPENDS
)

set(${PROJECT_NAME}_RUN_DEPENDS
)

file(GLOB ${PROJECT_NAME}_SOURCES
  src/*.cpp
)

find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_DEPENDS}
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_RUN_DEPENDS}
)

catkin_package(
  #   INCLUDE_DIR    ${PROJECT_SOURCE_DIR}/include
  #   CATKIN_DEPENDS ${${PROJECT_NAME}_DEPENDS}
  #                  ${${PROJECT_NAME}_BUILD_DEPENDS}
  #                  ${${PROJECT_NAME}_RUN_DEPENDS}
  #   DEPENDS        system_lib
)

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
# )
# 
# add_executable(${PROJECT_NAME}_node
#   ${${PROJECT_NAME}_SOURCES}
# )
# 
# add_dependencies(${PROJECT_NAME}_node
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )
# 
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
