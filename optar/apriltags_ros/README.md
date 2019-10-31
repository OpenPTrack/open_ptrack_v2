apriltags_ros  [![Build Status](https://api.travis-ci.org/RIVeR-Lab/apriltags_ros.png)](https://travis-ci.org/RIVeR-Lab/apriltags_ros)
=============
General Info about April Tags [https://april.eecs.umich.edu/wiki/AprilTags](https://april.eecs.umich.edu/wiki/AprilTags)

AprilTags for ROS

## How to Use
- Clone this repository into the src directory of your catkin_workspace
- Run catkin_make from the root of your workspace
- Change remap parameters in the launch files to match your input topics

## Compiling from Source Performance
Catkin builds by default without compiler optimizations. If you build from source and want better performance, compile a release build. See [here](https://answers.ros.org/question/71965/catkin-compiled-code-runs-3x-slower/) for more info.

`catkin_make -DCMAKE_BUILD_TYPE=Release`

## Topics
##### /apriltag_detector/compressed/parameter_descriptions
I don't know what this does please add details and submit a pull request
##### /apriltag_detector/compressed/parameter_updates
I don't know what this does please add details and submit a pull request
##### /rosout
I don't know what this does please add details and submit a pull request
##### /rosout_agg
I don't know what this does please add details and submit a pull request
##### /tag_detections
This is a custome message type that is an array of poses including the tag id for each pose
##### /tag_detections_image
I don't know what this does please add details and submit a pull request
##### /tag_detections_image/compressed
I don't know what this does please add details and submit a pull request
##### /tag_detections_image/compressed/parameter_descriptions
I don't know what this does please add details and submit a pull request
##### /tag_detections_image/compressed/parameter_updates
I don't know what this does please add details and submit a pull request
##### /tag_detections_image/compressedDepth
I don't know what this does please add details and submit a pull request
##### /tag_detections_image/compressedDepth/parameter_descriptions
I don't know what this does please add details and submit a pull request
##### /tag_detections_image/compressedDepth/parameter_updates
I don't know what this does please add details and submit a pull request
##### /tag_detections_image/theora
I don't know what this does please add details and submit a pull request
##### /tag_detections_image/theora/parameter_descriptions
I don't know what this does please add details and submit a pull request
##### /tag_detections_image/theora/parameter_updates
I don't know what this does please add details and submit a pull request
##### /tag_detections_pose
This is an output array of April tag poses
##### /tf
I don't know what this does please add details and submit a pull request
