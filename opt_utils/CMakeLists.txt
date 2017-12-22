cmake_minimum_required(VERSION 2.8.3)
project(opt_utils)
#SET(CMAKE_BUILD_TYPE RelWithDebInfo)
add_definitions(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  cmake_modules roscpp rosconsole image_transport cv_bridge opt_msgs
  body_pose_estimation tf_conversions dynamic_reconfigure)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

find_package(PCL REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS} include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
  cfg/ManualCalibration.cfg
)

add_library(json src/json.cpp)
target_link_libraries(json ${catkin_LIBRARIES})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME} json
   CATKIN_DEPENDS roscpp
)

add_library(${PROJECT_NAME} src/conversions.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(roi_viewer apps/roi_viewer.cpp)
target_link_libraries(roi_viewer boost_system boost_filesystem boost_signals ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(tracking_viewer apps/tracking_viewer.cpp)
#target_link_libraries(tracking_viewer boost_system boost_filesystem boost_signals ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(ros2udp_converter apps/ros2udp_converter.cpp src/udp_messaging.cpp)# src/json.cpp)
target_link_libraries(ros2udp_converter ${catkin_LIBRARIES} json)

add_executable(ros2udp_converter_pose apps/ros2udp_converter_pose.cpp src/udp_messaging.cpp)# src/json.cpp)
target_link_libraries(ros2udp_converter_pose ${catkin_LIBRARIES} json)

add_executable(ros2udp_converter_objects apps/ros2udp_converter_objects.cpp src/udp_messaging.cpp)
target_link_libraries(ros2udp_converter_objects ${catkin_LIBRARIES} json)

add_executable(udp_listener apps/udp_listener.cpp src/udp_messaging.cpp)
target_link_libraries(udp_listener ${catkin_LIBRARIES})

add_executable(cloud_refinement apps/cloud_refinement.cpp)
target_link_libraries(cloud_refinement ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(cloud_manual_refinement
  apps/cloud_manual_refinement.cpp
  )
target_link_libraries(cloud_manual_refinement
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )
add_dependencies(cloud_manual_refinement
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )
