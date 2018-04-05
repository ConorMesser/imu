cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Needed only for Arduino without native USB
add_definitions(-DUSB_CON)

generate_arduino_firmware(hello
  SRCS MPU9250_ros.ino ${ROS_LIB_DIR}/time.cpp
  BOARD mega
  PORT /dev/ttyACM0
)
