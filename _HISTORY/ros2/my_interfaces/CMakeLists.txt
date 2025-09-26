cmake_minimum_required(VERSION 3.8)
project(my_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

# Generate code for your services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/D2R.srv"
  "srv/R2D.srv"
  "srv/G2ECEF.srv"
  "srv/ECEF2G.srv"
)

add_executable(converter_service src/converter_service.cpp)
ament_target_dependencies(converter_service rclcpp)

# Link against the C++ typesupport produced above
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(converter_service ${cpp_typesupport_target})

# Install only real targets
install(TARGETS converter_service
  DESTINATION lib/${PROJECT_NAME}
)

# Export runtime deps so others can find your interfaces
ament_export_dependencies(rosidl_default_runtime)
ament_package()
