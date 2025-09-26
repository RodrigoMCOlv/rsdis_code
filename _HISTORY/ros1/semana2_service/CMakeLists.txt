cmake_minimum_required(VERSION 3.0.2)
project(semana2_service)

## Find catkin and message generation
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## Declare services
add_service_files(
  FILES
  D2R.srv
  R2D.srv
  G2ECEF.srv
  ECEF2G.srv
)

## Generate service headers
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Export to downstream pkgs
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

## Build your node
include_directories(${catkin_INCLUDE_DIRS})

add_executable(converter_service src/converter_service.cpp)
add_dependencies(converter_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(converter_service ${catkin_LIBRARIES})

install(TARGETS converter_service
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
