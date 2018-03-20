cmake_minimum_required(VERSION 2.8.3)
project(opt_msgs)
find_package(catkin REQUIRED COMPONENTS roscpp message_generation
  geometry_msgs sensor_msgs)

add_message_files(FILES
  Rois.msg
  RoiRect.msg
  HumanEntry.msg
  HumanEntries.msg
  BoundingBox2D.msg
  BoundingBox3D.msg
  Detection.msg
  DetectionArray.msg
  Track.msg
  Track3D.msg
  TrackArray.msg
  Track3DArray.msg
  SkeletonTrack.msg
  SkeletonTrackArray.msg
  StandardSkeletonTrack.msg
  StandardSkeletonTrackArray.msg
  CalibrationStatus.msg
  IDArray.msg
  PosePredictionResult.msg
  PoseRecognition.msg
  PoseRecognitionArray.msg
  Image2D_roi.msg
  Image2D_roi_array.msg
  World3D_roi.msg
  World3D_roi_array.msg
  ObjectName.msg
  ObjectNameArray.msg
  Image2D_roi_file.msg
  FeatureVectorArray.msg 
  Association.msg 
  NameArray.msg
  )

add_service_files(FILES OPTSensor.srv OPTTransform.srv)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)


catkin_package(
   INCLUDE_DIRS 
   LIBRARIES 
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime
)

include_directories(include include/open_ptrack/${PROJECT_NAME}/ ${catkin_INCLUDE_DIRS})

add_library(roi_msgs src/overlap.cpp )
add_dependencies(roi_msgs ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(roi_msgs ${catkin_LIBRARIES})
