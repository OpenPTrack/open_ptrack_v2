cmake_minimum_required(VERSION 2.8.3)
project(yolo_detector)

#SET(CMAKE_BUILD_TYPE RelWithDebInfo)


find_package(catkin REQUIRED
    COMPONENTS dynamic_reconfigure 
    		   message_filters 
    		   sensor_msgs 
    		   cv_bridge 
    		   image_transport
    		   opt_msgs
    		   opt_utils)

generate_dynamic_reconfigure_options(
  cfg/open_ptrack_yolo.cfg
)

find_package(OpenCV 3.1 REQUIRED)

catkin_package( CATKIN_DEPENDS message_filters sensor_msgs image_transport)
  
find_package(CUDA QUIET REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN_INCLUDE_DIRS})
SET(DARKNET_SRC_DIR "darknet_opt")

file(GLOB SRC_LIST "${DARKNET_SRC_DIR}/src/*.c")
file(GLOB CUDA_SRC_LIST "${DARKNET_SRC_DIR}/src/*.cu")
file(GLOB H_LIST "${DARKNET_SRC_DIR}/src/*.h")
list (REMOVE_ITEM SRC_LIST ${DARKNET_SRC_DIR}/src/darknet.c)

include_directories(
  include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${CUDA_INCLUDE_DIRS}
	${DARKNET_SRC_DIR}/include
	${DARKNET_SRC_DIR}/src
	/usr/local/cuda/include #for CuDNN, CuBLAS, CuRAND
)

link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${DARKNET_SRC_DIR}/src)
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(/usr/local/cuda/lib64) #for CuDNN, CuBLAS, CuRAND

set(
    CUDA_NVCC_FLAGS
      "-gencode arch=compute_60,code=sm_61 -gencode arch=compute_20,code=[sm_20,sm_21] -gencode arch=compute_30,code=sm_30 -gencode arch=compute_35,code=sm_35 -gencode arch=compute_50,code=[sm_50,compute_50] -gencode arch=compute_52,code=[sm_52,compute_52] -DGPU -I/usr/local/cuda/include/ -DCUDNN  --compiler-options \"-Wall -Wfatal-errors  -Ofast -DOPENCV -DGPU -DCUDNN\""
)




cuda_add_library(yolo_cuda_lib ${CUDA_SRC_LIST})

add_library(yolo_lib ${H_LIST} ${SRC_LIST} include/run_yolo_obj.h include/run_yolo_obj.c)

set_target_properties(yolo_lib PROPERTIES COMPILE_FLAGS "-DOPENCV -DGPU -I/usr/local/cuda/include/ -DCUDNN  -Ofast")
target_link_libraries(yolo_lib ${CUDA_LIBRARIES} cudnn cublas curand)

add_executable(open_ptrack_yolo_object_detector_node src/yolo_based_object_detector_node.cpp )
target_link_libraries(open_ptrack_yolo_object_detector_node yolo_lib yolo_cuda_lib ${catkin_LIBRARIES} ${CUDA_LIBRARIES} ${OpenCV_LIBS} )
add_dependencies(open_ptrack_yolo_object_detector_node ${PROJECT_NAME}_gencfg)
