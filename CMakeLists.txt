cmake_minimum_required(VERSION 3.5)
project(dynamixel_hardware_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(dynamixel_wrapper REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

add_library(dynamixel_hardware_interface SHARED src/dynamixel_hardware_interface.cpp)
target_include_directories(
  dynamixel_hardware_interface
  PRIVATE
  include
)
ament_target_dependencies(dynamixel_hardware_interface
  hardware_interface
  controller_manager
  dynamixel_wrapper
  rclcpp
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

install(
  TARGETS dynamixel_hardware_interface
  DESTINATION lib
)

ament_export_libraries(
  dynamixel_hardware_interface
)

ament_export_dependencies(
  hardware_interface
  controller_manager
  dynamixel_wrapper
  rclcpp
  pluginlib
)

ament_package()


