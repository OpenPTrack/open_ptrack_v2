cmake_minimum_required(VERSION 2.8.3)
project(mr_room)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  opt_msgs
  detection
  opt_utils
  message_filters
  bayes
  opt_msgs
  )

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS} include ${catkin_INCLUDE_DIRS})




## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )

catkin_package(
  LIBRARIES mr_room
  CATKIN_DEPENDS geometry_msg roscpp rospy std_msgs detection opt_utils message_filters bayes opt_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(plane_detector src/plane_detector.cpp)
target_link_libraries(plane_detector ${catkin_LIBRARIES})
add_dependencies(plane_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
