cmake_minimum_required(VERSION 3.8)
project(sct_img_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(img_publisher src/img_publisher.cpp)
add_executable(img_filterer src/img_filterer.cpp)
ament_target_dependencies(img_publisher rclcpp sensor_msgs cv_bridge image_transport OpenCV)
ament_target_dependencies(img_filterer rclcpp sensor_msgs cv_bridge image_transport OpenCV)

install(TARGETS
  img_publisher
  img_filterer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
