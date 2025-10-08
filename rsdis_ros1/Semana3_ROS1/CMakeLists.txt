cmake_minimum_required(VERSION 3.0.2)
project(Semana3_ROS1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(circle_detector_node src/cylinder_detect.cpp)
target_link_libraries(circle_detector_node
  ${catkin_LIBRARIES}
)
add_dependencies(circle_detector_node ${catkin_EXPORTED_TARGETS})

add_executable(turtle_control_node src/turtle_control.cpp)
target_link_libraries(turtle_control_node
  ${catkin_LIBRARIES}
)
add_dependencies(turtle_control_node ${catkin_EXPORTED_TARGETS})

if(TARGET Eigen3::Eigen)
  target_link_libraries(circle_detector_node Eigen3::Eigen)
  target_link_libraries(turtle_control_node Eigen3::Eigen)
endif()
