cmake_minimum_required(VERSION 2.8.3)
project(rtpose_wrapper)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS roscpp roslib cv_bridge
                                        std_msgs message_filters
                                        message_generation
                                        dynamic_reconfigure)

find_package(CUDA REQUIRED)

find_package(OpenCV REQUIRED)

add_definitions("-std=c++11")

generate_dynamic_reconfigure_options(
  cfg/Joints.cfg
)

add_message_files(
  FILES
  Joint2DMsg.msg
  Joint3DMsg.msg
  SkeletonArrayMsg.msg
  SkeletonMsg.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)


include_directories(include
    build/src
    ${catkin_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${gflags_INCLUDE_DIRS}
    )
link_directories( build/lib)

add_executable( rtpose apps/rtpose.cpp)
target_link_libraries( rtpose
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${gflags_LIBRARIES}
    caffe
    glog
    gflags
    )
add_dependencies( rtpose
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
    ${PROJECT_NAME}_gencfg
    )
