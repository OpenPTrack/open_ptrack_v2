cmake_minimum_required(VERSION 2.8.3)
project(pose_recognition)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gallery_poses
  roscpp
  roslib
  rospy
  standard_pose
  template_recognizer
  opt_msgs
  message_filters
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pose_recognition
#  CATKIN_DEPENDS gallery_poses roscpp roslib rospy standard_pose template_recognizer
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/pose_recognition.cpp
  include/body_pose_recognition/pose_recognition.h
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  )

