recognition module
===========

Description
----------
This module performs face recognition-based person re-identification to keep track of people identities even if they move out from the camera's field of view once.


Installation
----------
Make sure that OpenPTrack has been installed to your PC.

This module requires: 
- [OpenFace](https://cmusatyalab.github.io/openface/) >= 0.2.1
- [dlib](http://dlib.net/) >= 19.4
- [torch7](http://torch.ch/)

The following script installs OpenFace and dlib on your python environment, installs torch to ~/torch, and downloads face model files from OpenFace and dlib projects.
```bash
cd open_ptrack/recognition/install_scripts
./install_all.sh
```

Usage
----------
For every sensor, launch the face detection and feature extraction nodes:
```bash
roslaunch recognition face_detection_and_feature_extraction.launch sensor_name:="kinect2_head"
```

Then, start the face recognition node on the master PC.
```bash
rosrun recognition face_recognition_node
```

For visualization, run recognition_visualization_node:
```bash
rosrun recognition recognition_visualization_node.py
```

There is a drag-and-drop people registration tool for predefined people recognition:
```bash
rosrun recognition drag_and_drop.py
```


Topics
----------
```bash
/face_recognition/people_tracks
/face_recognition/people_names
```

people_tracks contains tracked people. The data are same as /tracker/people_tracks except that people IDs are replaced according to face recognition results. If a person ID is larger than 10000, it means that his/her face has not been recognized by the system yet.

people_names is the result of predefined people recognition. It contains associations between people IDs and predefined people names.



