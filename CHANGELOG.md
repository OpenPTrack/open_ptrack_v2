# Changelog for OpenPTrack

## Unreleased
- Sync tracking/launch files with [open_ptrack_docker_config](https://github.com/OpenPTrack/open_ptrack_docker_config/tree/1804) repo


## [2.2] - 2019-12-18
- Upgrade to Ubuntu Linux 18.04 LTS (from Ubuntu Linux 16.04 LTS)
- Upgrade to CUDA 10.0 (from CUDA 8.0)
- Upgrade to ROS Melodic (from ROS Kinetic Kame)
- Support for Stereolabs Zed (Zed SDK v2.8)
- Support for Intel RealSense (Intel RealSense v2.0)
- Preliminary support for Kinect Azure for single camera person tracking (Azure Kinect SDK v1.3.0)
- YOLO based single camera person tracking for Zed
- Deprecated support for Swissranger, Point Grey Blackfly imagers
- Deprecation of old launch files
- rt_pose wrapper moved to github repo from private bitbucket
- Fix broken heights for object tracking
- Add status plots during multi-camera calibration
- Fix support for multiple sensors of same type from single machine
- Fix frame_id bug for face detection
