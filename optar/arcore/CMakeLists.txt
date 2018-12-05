cmake_minimum_required(VERSION 2.8.3)
project(arcore)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

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

#add_message_files(
#  FILES
#  ARCoreSkeletonTrackArray.msg
#)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS
 LIBRARIES arcore
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


## Add folders to be run by python nosetests


add_executable(mobile_receiver src/mobile_receiver.cpp)
target_link_libraries(mobile_receiver ${catkin_LIBRARIES})
add_dependencies(mobile_receiver mobile_receivercpp)

add_executable(keep_tag_living src/keep_tag_living.cpp)
target_link_libraries(keep_tag_living ${catkin_LIBRARIES})
add_dependencies(keep_tag_living keep_tag_livingcpp)

add_executable(tag_file src/tag_file.cpp)
target_link_libraries(tag_file ${catkin_LIBRARIES})
add_dependencies(tag_file tag_filecpp)

add_executable(origin_sender src/origin_sender.cpp)
target_link_libraries(origin_sender ${catkin_LIBRARIES})
add_dependencies(origin_sender origin_sendercpp)

add_executable(centroid_modifier src/centroid_modifier.cpp)
target_link_libraries(centroid_modifier ${catkin_LIBRARIES})
add_dependencies(centroid_modifier centroid_modifiercpp)

add_executable(skeleton_modifier src/skeleton_modifier.cpp)
target_link_libraries(skeleton_modifier ${catkin_LIBRARIES})
add_dependencies(skeleton_modifier skeleton_modifier_cpp)

add_executable(motion_control src/motion_control.cpp)
target_link_libraries(motion_control ${catkin_LIBRARIES})
add_dependencies(motion_control motion_control_cpp)

add_executable(arcore_camera_image_republisher src/arcore_camera_image_republisher.cpp)
target_link_libraries(arcore_camera_image_republisher ${catkin_LIBRARIES})
add_dependencies(arcore_camera_image_republisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})