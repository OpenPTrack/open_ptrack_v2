#!/bin/bash
# this script installs torch to ~/torch, 
# and adds openface.so and dlib.so to /usr/local/lib/python2.7/dist-packages, 
# and then downloads openface and dlib model files to ../data

./install_torch.sh
./install_openface.sh
./install_dlib.sh
./download_face_models.sh
