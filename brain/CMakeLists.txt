cmake_minimum_required(VERSION 3.5.1)
project(brain)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs	
  message_generation
)

add_service_files(DIRECTORY srv
  FILES
  RequestTerminal.srv
)

add_message_files(DIRECTORY msg
   FILES
   Access.msg
   Feedback.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_generation roscpp sensor_msgs std_msgs geometry_msgs message_runtime
)

include_directories(include/brain
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(hubert_brain src/brain_node.cpp src/brain.cpp)
target_link_libraries(hubert_brain ${catkin_LIBRARIES})
add_dependencies(hubert_brain brain_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
