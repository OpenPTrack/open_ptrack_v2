cmake_minimum_required(VERSION 2.8.3)
project(recognition)

add_definitions(-std=c++11)

# set(CMAKE_BUILD_TYPE RelWithDebInfo)
find_package(catkin REQUIRED COMPONENTS 
	rospy 
	roscpp
	tf
	tf_conversions
	std_msgs
	sensor_msgs
    pcl_ros
    pcl_conversions
	opt_msgs
    cv_bridge
	dynamic_reconfigure
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# find_package(Eigen REQUIRED)
# include_directories(${Eigen_INCLUDE_DIRS} include ${catkin_INCLUDE_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})

add_service_files(FILES OPTSetPredefinedFaces.srv OPTSaveRegisteredFaces.srv OPTLoadRegisteredFaces.srv)
generate_messages(DEPENDENCIES std_msgs sensor_msgs opt_msgs)

generate_dynamic_reconfigure_options(
	cfg/FaceDetection.cfg
	cfg/FaceFeatureExtraction.cfg
    cfg/FaceRecognition.cfg
	cfg/RecognitionVisualization.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES face_reconstruction
  CATKIN_DEPENDS roscpp std_msgs opt_msgs
#  DEPENDS system_lib
)

add_executable(face_recognition_node apps/face_recognition_node.cpp)
target_link_libraries(face_recognition_node ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(face_recognition_node ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)

add_executable(map_generation_node apps/map_generation_node.cpp)
target_link_libraries(map_generation_node ${OpenCV_LIBS} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(map_generation_node ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
