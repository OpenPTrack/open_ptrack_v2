cmake_minimum_required(VERSION 2.8.3)
project(optar)

#set(USE_CUDA true)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wall)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  tf
  roscpp
  rospy
  message_filters
  dynamic_reconfigure
  cv_bridge
  opt_msgs
  image_transport
  eigen_conversions
  )



unset(OpenCV_DIR)
#unset(CMAKE_PREFIX_PATH)
unset(OpenCV_VERSION)
unset(OpenCV_INSTALL_PATH)
unset(OpenCV_CONFIG_PATH)
unset(OpenCV_INCLUDE_DIRS)
unset(OpenCV_LIBS)

message(STATUS "OpenCV_FOUND =${OpenCV_FOUND}")
message(STATUS "OpenCV_DIR =${OpenCV_DIR}")
message(STATUS "prefix ${CMAKE_PREFIX_PATH}")
message(STATUS "version ${OpenCV_VERSION}")
message(STATUS "install path ${OpenCV_INSTALL_PATH}")
message(STATUS "config path ${OpenCV_CONFIG_PATH}") # look at the output of this message
message(STATUS "include dirs ${OpenCV_INCLUDE_DIRS}")
message(STATUS "libs ${OpenCV_LIBS}")

if(USE_CUDA)
  #then use the custom built opencv
  message(STATUS "using cuda for optar")
  find_package(OpenCV REQUIRED
    NO_MODULE
    PATHS /usr/local
    HINTS /usr/local
    EXACT 3.1.0
    NO_DEFAULT_PATH)
  include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
  link_directories(${OpenCV_LIB_DIR})
  add_definitions(-DUSE_CUDA_OPENCV)
else()
  message(STATUS "not using cuda for optar")
  find_package(OpenCV REQUIRED)
  include_directories(${OpenCV_INCLUDE_DIRS})
  link_directories(${OpenCV_LIB_DIR})
endif()

message(STATUS "OpenCV_FOUND =${OpenCV_FOUND}")
message(STATUS "OpenCV_DIR =${OpenCV_DIR}")
message(STATUS "prefix ${CMAKE_PREFIX_PATH}")
message(STATUS "version ${OpenCV_VERSION}")
message(STATUS "install path ${OpenCV_INSTALL_PATH}")
message(STATUS "config path ${OpenCV_CONFIG_PATH}") # look at the output of this message
message(STATUS "include dirs ${OpenCV_INCLUDE_DIRS}")
message(STATUS "libs ${OpenCV_LIBS}")


find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

include_directories( cfg/cpp )

#add_message_files(
#  FILES
#  ARCoreSkeletonTrackArray.msg
#)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  opt_msgs
  )


#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/OptarSingleCameraParameters.cfg
  cfg/OptarAggregatorParameters.cfg
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
 LIBRARIES optar
 CATKIN_DEPENDS geometry_msg roscpp rospy std_msgs message_filters dynamic_reconfigure opt_msgs
 DEPENDS system_lib Eigen
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


# Build the utils file as a library
add_library(optarUtils src/utils.cpp)
target_link_libraries(optarUtils ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(optarUtils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)



add_executable(mobile_receiver src/withMarker/mobile_receiver.cpp)
target_link_libraries(mobile_receiver ${catkin_LIBRARIES})
add_dependencies(mobile_receiver mobile_receivercpp ${PROJECT_NAME}_generate_messages_cpp)

add_executable(keep_tag_living src/withMarker/keep_tag_living.cpp)
target_link_libraries(keep_tag_living ${catkin_LIBRARIES})
add_dependencies(keep_tag_living keep_tag_livingcpp ${PROJECT_NAME}_generate_messages_cpp)

add_executable(tag_file src/withMarker/tag_file.cpp)
target_link_libraries(tag_file ${catkin_LIBRARIES})
add_dependencies(tag_file tag_filecpp ${PROJECT_NAME}_generate_messages_cpp)

add_executable(origin_sender src/withMarker/origin_sender.cpp)
target_link_libraries(origin_sender ${catkin_LIBRARIES})
add_dependencies(origin_sender origin_sendercpp ${PROJECT_NAME}_generate_messages_cpp)

add_executable(centroid_modifier src/withMarker/centroid_modifier.cpp)
target_link_libraries(centroid_modifier optarUtils ${catkin_LIBRARIES} )
add_dependencies(centroid_modifier centroid_modifiercpp optarUtils ${PROJECT_NAME}_generate_messages_cpp)

add_executable(skeleton_modifier src/withMarker/skeleton_modifier.cpp)
target_link_libraries(skeleton_modifier ${catkin_LIBRARIES})
add_dependencies(skeleton_modifier skeleton_modifier_cpp ${PROJECT_NAME}_generate_messages_cpp)

add_executable(motion_control src/withMarker/motion_control.cpp)
target_link_libraries(motion_control ${catkin_LIBRARIES})
add_dependencies(motion_control motion_control_cpp ${PROJECT_NAME}_generate_messages_cpp)



add_executable(ardevices_pose_estimator_single_camera_raw
                    src/noMarker/ardevices_pose_estimator_single_camera_raw/ardevices_pose_estimator_single_camera_raw.cpp
                    src/noMarker/ardevices_pose_estimator_single_camera_raw/CameraPoseEstimator.cpp
                    src/noMarker/ardevices_pose_estimator_single_camera_raw/ARDevicePoseEstimatorSingleCamera.cpp
                    src/noMarker/ardevices_pose_estimator_single_camera_raw/FeaturesMemory.cpp)
target_link_libraries(ardevices_pose_estimator_single_camera_raw optarUtils ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(ardevices_pose_estimator_single_camera_raw optarUtils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(dummy_track_publisher src/noMarker/dummyTrackPublisher.cpp)
target_link_libraries(dummy_track_publisher ${catkin_LIBRARIES})
add_dependencies(dummy_track_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(ntp_server src/ntp_server.cpp)
target_link_libraries(ntp_server optarUtils ${catkin_LIBRARIES})
add_dependencies(ntp_server optarUtils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(skeleton_centroid_publisher src/skeletonCentroidPublisher.cpp )
target_link_libraries(skeleton_centroid_publisher optarUtils ${catkin_LIBRARIES})
add_dependencies(skeleton_centroid_publisher optarUtils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)


add_executable(ardevices_poses_republisher src/noMarker/ardevices_poses_republisher.cpp )
target_link_libraries(ardevices_poses_republisher optarUtils ${catkin_LIBRARIES})
add_dependencies(ardevices_poses_republisher optarUtils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)


add_executable(ardevices_registration_aggregator
                    src/noMarker/ardevices_registration_aggregator/ardevices_registration_aggregator.cpp
                    src/noMarker/ardevices_registration_aggregator/TransformFilterKalman.cpp)
target_link_libraries(ardevices_registration_aggregator optarUtils ${catkin_LIBRARIES})
add_dependencies(ardevices_registration_aggregator optarUtils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)


add_executable(ardevices_registration_estimator
                    src/noMarker/ardevices_registration_estimator/ardevices_registration_estimator.cpp
                    src/noMarker/ARDevicesManager.cpp
                    src/noMarker/ardevices_registration_estimator/ARDeviceRegistrationEstimator.cpp
                    src/noMarker/ardevices_registration_estimator/PoseFilterEuler.cpp
                    src/noMarker/ardevices_registration_estimator/Position3DKalmanFilter.cpp)
target_link_libraries(ardevices_registration_estimator optarUtils ${catkin_LIBRARIES})
add_dependencies(ardevices_registration_estimator optarUtils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)



# make sure configure headers are built before any node using them
add_dependencies(ardevices_pose_estimator_single_camera_raw ${PROJECT_NAME}_gencfg)


add_executable(integrate_calibration_refinement src/noMarker/integrate_calibration_refinement.cpp )
target_link_libraries(integrate_calibration_refinement optarUtils ${catkin_LIBRARIES})
add_dependencies(integrate_calibration_refinement ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)



#add_executable(poses_writer src/poses_writer.cpp )
#target_link_libraries(poses_writer optarUtils ${catkin_LIBRARIES})
#add_dependencies(poses_writer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
