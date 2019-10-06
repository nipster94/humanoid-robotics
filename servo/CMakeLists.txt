cmake_minimum_required(VERSION 2.8.3)
project(servo)

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  std_msgs 
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware servo ALL)
rosserial_add_client_target(firmware servo-upload)

