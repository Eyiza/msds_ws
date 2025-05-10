cmake_minimum_required(VERSION 3.8)
project(msds_firmware)

# Enables compiler warnings to help catch common mistakes 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# These make sure your system has all necessary packages available for compiling 
# and linking the C++ hardware plugin and Python scripts.

# Required build systems
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ROS2 core dependencies
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# For hardware interfaces and lifecycle nodes
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# To detect and link with libserial (used for Arduino serial comm)
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

# Install Python modules - any Python files in your package directory (e.g. your MPU driver and serial test scripts).
ament_python_install_package(${PROJECT_NAME})

# Add your hardware interface implementation as a shared library
# SHARED means this compiles to a .so shared object that ROS2 can load as a plugin at runtime.
add_library(${PROJECT_NAME} SHARED 
  src/msds_interface.cpp
)

# Specify where to look for header files (your own and libserial’s)
target_include_directories(${PROJECT_NAME} PRIVATE 
  include
  "${SERIAL_INCLUDE_DIRS}"
)

# Link against libserial
target_link_libraries(${PROJECT_NAME}
 "${SERIAL_LDFLAGS}")

# Declare Dependencies for the Target
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export Plugin Info
pluginlib_export_plugin_description_file(hardware_interface msds_interface.xml)


add_executable(simple_serial_receiver src/simple_serial_receiver.cpp)
ament_target_dependencies(simple_serial_receiver rclcpp std_msgs)
target_include_directories(simple_serial_receiver PUBLIC
 "${SERIAL_INCLUDE_DIRS}")
target_link_libraries(simple_serial_receiver
 "${SERIAL_LDFLAGS}")

add_executable(simple_serial_transmitter src/simple_serial_transmitter.cpp)
ament_target_dependencies(simple_serial_transmitter rclcpp std_msgs)
target_include_directories(simple_serial_transmitter PUBLIC
 "${SERIAL_INCLUDE_DIRS}")
target_link_libraries(simple_serial_transmitter
 "${SERIAL_LDFLAGS}")


# Install the shared library (.so file)
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# Install header files (for external use)
install(
  DIRECTORY include
  DESTINATION include
)

# Install launch files (if any)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts (as executables)
install(PROGRAMS
  ${PROJECT_NAME}/simple_serial_transmitter.py
  ${PROJECT_NAME}/simple_serial_receiver.py
  ${PROJECT_NAME}/mpu6050_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  simple_serial_receiver
  simple_serial_transmitter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

# Export include dirs (for other packages that depend on this one)
ament_export_include_directories(
  include
)

# Export the shared library (so other packages can link against it)
ament_export_libraries(
  ${PROJECT_NAME}
)

# Export the package dependencies (so other packages can find this one)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export the package name (so other packages can find this one)
ament_package()
