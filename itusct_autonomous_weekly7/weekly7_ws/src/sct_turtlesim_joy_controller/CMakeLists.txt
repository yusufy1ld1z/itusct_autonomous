cmake_minimum_required(VERSION 3.8)
project(sct_turtlesim_joy_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(joy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(joy_controller src/joy_controller.cpp)
ament_target_dependencies(joy_controller rclcpp sensor_msgs geometry_msgs turtlesim joy std_msgs std_srvs)

install(TARGETS
  joy_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
