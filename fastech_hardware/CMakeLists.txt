cmake_minimum_required(VERSION 3.8)
project(fastech_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-pedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/Include
)


link_directories(${CMAKE_CURRENT_SOURCE_DIR}/vendor/Lib)

add_compile_options(-Wno-pedantic)

add_library(fastech_system SHARED src/fastech_system.cpp)
ament_target_dependencies(fastech_system rclcpp hardware_interface pluginlib)
target_link_libraries(fastech_system EziMOTIONPlusR)

pluginlib_export_plugin_description_file(hardware_interface fastech_system_plugin.xml)


install(TARGETS fastech_system
  DESTINATION lib)


ament_package()
