cmake_minimum_required(VERSION 2.8.3)
project(body_pose_estimation)

#SET(CMAKE_BUILD_TYPE RelWithDebInfo)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  cmake_modules
  opt_msgs
)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  include/body_pose_estimation/skeleton_base.h
  src/skeleton_base.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  )
