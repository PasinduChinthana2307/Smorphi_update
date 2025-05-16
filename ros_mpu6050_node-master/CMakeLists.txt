cmake_minimum_required(VERSION 3.8)
project(ros_mpu6050_node)

# Default to C++14 for ROS2
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Define path to i2cdevlib
set(PATH_I2CDEVLIB "/usr/share/arduino/libraries/i2cdevlib/")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${PATH_I2CDEVLIB}RaspberryPi_bcm2835/I2Cdev
  ${PATH_I2CDEVLIB}Arduino/MPU6050/
)

# Add executable
add_executable(mpu6050_node
  src/mpu6050_node.cpp
  ${PATH_I2CDEVLIB}RaspberryPi_bcm2835/I2Cdev/I2Cdev.cpp
  ${PATH_I2CDEVLIB}Arduino/MPU6050/MPU6050.cpp
)

# Link libraries
target_link_libraries(mpu6050_node
  bcm2835
  m
)

# Specify dependencies
ament_target_dependencies(mpu6050_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Install executable
install(TARGETS
  mpu6050_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if you have any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.py"
)

# Install config files if you have any
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
