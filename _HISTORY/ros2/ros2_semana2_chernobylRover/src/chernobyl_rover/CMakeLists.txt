cmake_minimum_required(VERSION 3.8)
project(chernobyl_rover)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Optional but common
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(my_interfaces REQUIRED)

add_executable(rover_node src/rover.cpp)
ament_target_dependencies(rover_node rclcpp std_msgs sensor_msgs my_interfaces)

install(TARGETS
  rover_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
