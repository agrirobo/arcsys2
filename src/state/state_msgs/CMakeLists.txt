cmake_minimum_required(VERSION 2.8.3)
project(state_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
  FILES
  State.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)
