cmake_minimum_required(VERSION 3.8)
project(maxon_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Export include directory
include_directories(
  include
)
include_directories(/home/pavetech/crawler_ws/src/maxon_hardware/include/maxon_hardware/)

# Link EPOS library directories
link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/maxon_hardware/lib
)

# --------------------------
# Test program
# --------------------------
add_executable(test_motors src/test_motors.cpp)
ament_target_dependencies(test_motors rclcpp)
target_link_libraries(test_motors EposCmd)

# --------------------------
# Main Hardware Interface Library
# --------------------------
add_library(${PROJECT_NAME} SHARED
  src/maxon_system.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

target_link_libraries(${PROJECT_NAME}
  EposCmd
)

pluginlib_export_plugin_description_file(
  hardware_interface
  my_robot_hardware_interface.xml
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install library
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# Install plugin xml
install(FILES my_robot_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS test_motors
  DESTINATION lib/${PROJECT_NAME}/
)

# Export include folders
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

ament_package()
