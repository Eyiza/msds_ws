cmake_minimum_required(VERSION 3.8)
project(msds_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/twist_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
