cmake_minimum_required(VERSION 3.0.2)
project(my_interfaces)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  message_generation
)

## Declare the message files
add_message_files(
  FILES
  ChernobylSensors.msg   # add all your .msg filenames here
)

## Generate added messages
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

## Export this package for others
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)
