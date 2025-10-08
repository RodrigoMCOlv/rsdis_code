cmake_minimum_required(VERSION 3.8)
project(semana3_parte1_ros2)


if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


add_executable(control_node src/turtle_control.cpp)
add_executable(cylinder_detect src/cylinder_detect.cpp)


ament_target_dependencies(control_node rclcpp geometry_msgs sensor_msgs nav_msgs)
ament_target_dependencies(cylinder_detect rclcpp sensor_msgs visualization_msgs)


target_include_directories(cylinder_detect PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cylinder_detect ${OpenCV_LIBS})

install(TARGETS

  control_node
  cylinder_detect
  
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
