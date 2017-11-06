cmake_minimum_required(VERSION 2.8.3)
project(standard_pose)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  body_pose_estimation
  opt_msgs
  roscpp
  roslib
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES standard_pose
  CATKIN_DEPENDS body_pose_estimation opt_msgs roscpp roslib
  #  DEPENDS system_lib
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

SET(HEADERS
  include/body_pose_recognition/standardpose.h
  )
SET(SOURCES
  src/standardpose.cpp
  )

add_library(${PROJECT_NAME}
  ${HEADERS}
  ${SOURCES}
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  )

