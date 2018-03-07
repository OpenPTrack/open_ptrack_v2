cmake_minimum_required(VERSION 2.8.3)
project(tracking)

add_definitions("-std=c++11")

#set(CMAKE_BUILD_TYPE RelWithDebInfo)
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rosconsole
  image_transport
  tf
  tf_conversions
  bayes
  pcl_ros
  detection
  opt_msgs
  opt_utils
  dynamic_reconfigure
  body_pose_estimation
  standard_pose
  message_filters
  )

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS} include ${catkin_INCLUDE_DIRS})

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(
  cfg/SkeletonTracker.cfg
  cfg/Tracker.cfg
  #cfg/TrackerSmoother.cfg
  cfg/MovingAverageSmoother.cfg
)

include_directories(cfg/cpp)

catkin_package(
   INCLUDE_DIRS 
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp bayes opencv2 pcl_ros detection tf tf_conversions opt_msgs opt_utils message_filters
)

add_library(${PROJECT_NAME}
  src/munkres.cpp
  src/kalman_filter.cpp
  src/kalman_filter3d.cpp
  src/track.cpp
  src/track3d.cpp
  src/tracker.cpp
  src/tracker3d.cpp
  src/skeleton_tracker.cpp
  src/skeleton_track.cpp
  src/track_object.cpp
  src/tracker_object.cpp
  )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)


add_executable(tracker apps/tracker_node.cpp)
target_link_libraries(tracker ${PROJECT_NAME} ${catkin_LIBRARIES})
add_executable(tracker_object apps/tracker_object_node.cpp)
target_link_libraries(tracker_object ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(tracker_object ${PROJECT_NAME}_gencfg)
add_executable(tracker3d apps/tracker_node_3d.cpp)
target_link_libraries(tracker3d ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_skeleton_tracker_node
  apps/skeleton_tracker_node.cpp
  )
target_link_libraries(${PROJECT_NAME}_skeleton_tracker_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
 )

add_executable(moving_average_filter apps/moving_average_filter_node.cpp)
add_dependencies(moving_average_filter ${PROJECT_NAME}_gencfg)
target_link_libraries(moving_average_filter ${PROJECT_NAME} ${catkin_LIBRARIES})
