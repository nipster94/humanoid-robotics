cmake_minimum_required(VERSION 2.8.3)
project(face_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs	
  dynamic_reconfigure
  message_generation
)

#find_package(dlib REQUIRED)
#include_directories(${dlib_INCLUDE_DIRS})
#link_directories(${DLIB_LIBRARY_DIRS})

#message(STATUS "include dir: ${catkin_LIBRARIES}" )
#message(STATUS "include dir: ${OpenCV_LIBS}"  )
#message(STATUS "include dir: ${DLIB_LIBRARY_DIRS}" )
#message(STATUS "include dir: ${dlib_INCLUDE_DIRS}" )


## ADDING C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES face_tracking
  CATKIN_DEPENDS cv_bridge image_transport message_generation roscpp sensor_msgs std_msgs geometry_msgs message_runtime
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include/face_detection
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
)

#include_directories(include)
#include_directories(/usr/include)

add_library(dlib SHARED IMPORTED) # Imported(!) dlib target
set_target_properties(dlib PROPERTIES IMPORTED_LOCATION "/home/nipun/dlib/build/libdlib.so")

#include(/home/nipun/Backup_set_1/our_project/face_detection/src/dlib/shared_build/CMakeFiles/Export/lib/cmake)
#dlib glog

#add_subdirectory(/home/nipun/dlib shared_build)

#add_library(face_tracking src/face_tracking.cpp)
#target_link_libraries(face_tracking ${catkin_LIBRARIES} ${OpenCV_LIBS} dlib glog -llapack)
#add_dependencies(face_tracking ${catkin_EXPORTED_TARGETS})

#add_executable(ros_face_detection src/face_detection_node.cpp src/face_detection.cpp)
#target_link_libraries(ros_face_detection face_tracking ${catkin_LIBRARIES})
#add_dependencies(ros_face_detection ${catkin_EXPORTED_TARGETS})

add_executable(test_class src/test.cpp)
target_link_libraries(test_class ${catkin_LIBRARIES} ${OpenCV_LIBS} dlib glog -llapack -lblas -lpthread -lX11)
add_dependencies(test_class ${catkin_EXPORTED_TARGETS})

#add_executable(talker src/talker.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})
#add_dependencies(talker ${catkin_EXPORTED_TARGETS})

#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener ${catkin_EXPORTED_TARGETS})


#install( TARGETS
#  ros_face_detection face_tracking
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY include/face_detection/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)
