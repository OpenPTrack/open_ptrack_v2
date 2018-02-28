cmake_minimum_required(VERSION 2.8.3)
project(detection)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_definitions("-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    cmake_modules
    roscpp
    rospy
    calibration_common
    message_filters 
    sensor_msgs
    image_transport
    pcl_conversions 
    pcl_ros
    nodelet
    rosconsole
    cv_bridge
    opt_msgs
    opt_utils
    dynamic_reconfigure
    std_msgs 
    rostime
    compressed_image_transport 
    compressed_depth_image_transport 
    kinect2_bridge
)

include_directories(${catkin_INCLUDE_DIRS})

find_package(OpenMP REQUIRED)

find_package(Boost REQUIRED)
include_directories( ${BOOST_INCLUDE_DIRS} )
find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN_INCLUDE_DIRS})
find_package(PCL REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )


include_directories( include )   
include_directories( cfg/cpp )
# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/HaarDispAdaDetector.cfg cfg/GroundBasedPeopleDetector.cfg cfg/multiple_objects_detection.cfg)

catkin_package(
    CATKIN_DEPENDS message_filters sensor_msgs image_transport roscpp pcl_ros pcl_conversions opt_utils opt_msgs 
    DEPENDS OpenCV
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
)


add_library(${PROJECT_NAME}
  src/detection_source.cpp
  src/detection.cpp
  src/multiple_objects_detection/object_detector.cpp
  src/multiple_objects_detection/roi_zz.cpp
  src/skeleton_detection.cpp
  )
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(multiple_objects_detection_node apps/multiple_objects_detection_node.cpp)
target_link_libraries(multiple_objects_detection_node ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(ground_based_people_detector apps/ground_based_people_detector_node.cpp)
SET_TARGET_PROPERTIES(ground_based_people_detector PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})
target_link_libraries(ground_based_people_detector ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ground_based_people_detector_sr apps/ground_based_people_detector_node_sr.cpp)
SET_TARGET_PROPERTIES(ground_based_people_detector_sr PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})
target_link_libraries(ground_based_people_detector_sr ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_library(haar_disp_ada src/haardispada.cpp apps/haardispada_nodelet.cpp)
target_link_libraries(haar_disp_ada ${catkin_LIBRARIES})

add_executable(HaarDispAda174 apps/haardispada_node.cpp)
target_link_libraries(HaarDispAda174 haar_disp_ada ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} boost_system boost_signals)

add_executable(HaarDispAda174_sr apps/haardispada_node_sr.cpp)
target_link_libraries(HaarDispAda174_sr haar_disp_ada ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} boost_system boost_signals)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

add_executable(ground_based_people_detector_zed apps/ground_based_people_detector_node_zed.cpp)
SET_TARGET_PROPERTIES(ground_based_people_detector_zed PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})
target_link_libraries(ground_based_people_detector_zed ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
