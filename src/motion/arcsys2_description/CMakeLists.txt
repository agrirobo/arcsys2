cmake_minimum_required(VERSION 2.8.3)
project(arcsys2_description CXX)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(${PROJECT_NAME}_DEPENDS
)

set(${PROJECT_NAME}_BUILD_DEPENDS
)

set(${PROJECT_NAME}_RUN_DEPENDS
)

find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_DEPENDS}
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_RUN_DEPENDS}
)

catkin_package()
