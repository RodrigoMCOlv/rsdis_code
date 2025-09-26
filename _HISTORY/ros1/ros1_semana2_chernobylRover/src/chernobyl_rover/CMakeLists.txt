cmake_minimum_required(VERSION 3.0.2)
project(chernobyl_rover)

# Warnings & C++ standard (Noetic is fine with C++17)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  my_interfaces        # assumes a ROS1 catkin pkg that exports the msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs my_interfaces
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rover_node src/rover.cpp)
target_link_libraries(rover_node ${catkin_LIBRARIES})
add_dependencies(rover_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS rover_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
