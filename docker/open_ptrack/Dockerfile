FROM openptrack/open_ptrack-dep
LABEL maintainer "Samir Tabriz"

ARG branch=kinect

RUN  cd     \ 
    && cd workspace     \ 
    && git clone https://github.com/itabr/libfreenect2.git     \ 
    && cd libfreenect2     \ 
    && git checkout v0.2.0     \ 
    && cd depends/     \ 
    && apt-get install -y git cmake cmake-curses-gui libxmu-dev libxi-dev libgl1-mesa-dev xorg-dev libglu1-mesa-dev libtool automake libudev-dev libgtk2.0-dev pkg-config libjpeg-turbo8-dev libturbojpeg libglewmx-dev     \ 
    && ln -f -s /usr/lib/x86_64-linux-gnu/libturbojpeg.so.0.1.0 /usr/lib/x86_64-linux-gnu/libturbojpeg.so     \ 
    && cd .. \
    && mkdir build \ 
    && cd build     \ 
    && /bin/bash -c "source /opt/ros/kinetic/setup.bash    \ 
    && export NVCUDASAMPLES_ROOT='/root/workspace/ros/src/NVIDIA_CUDA-8.0_Samples' \
    && cmake .. -DCMAKE_INSTALL_PREFIX=/root/workspace/ros/src/freenect2   \ 
    && make     \
    && make install"     \
    && echo '# ATTR{product}=="Kinect2" SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c4", MODE="0666" SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d8", MODE="0666" SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d9", MODE="0666"' > ~/90-kinect2.rules     \ 
    && cp /root/workspace/libfreenect2/platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/    

RUN  cd ~/workspace/ros/src     \ 
    && git clone https://github.com/openptrack/iai_kinect2     \ 
    && cd iai_kinect2     \ 
    && git checkout 1604       

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash \
    && git clone https://github.com/openptrack/open_ptrack_v2 ~/workspace/ros/src/open_ptrack \
    && cd ~/workspace/ros/src/open_ptrack \
    && git checkout $branch \
    && cd ../..  \
    && export LD_LIBRARY_PATH=/root/workspace/ros/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/usr/local/lib/x86_64-linux-gnu:/usr/local/lib/i386-linux-gnu:/usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu:/usr/local/cuda/lib64:/usr/local/opencv3/lib \
    && catkin_make"

RUN /bin/bash -c "source /root/workspace/ros/devel/setup.bash \
    && roscd yolo_detector/darknet_opt \
    && wget -O coco.weights https://pjreddie.com/media/files/yolo.weights"

RUN cd /root/workspace/ros/src/open_ptrack/recognition/install_scripts \
    && wget http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2 \
    && bunzip2 shape_predictor_68_face_landmarks.dat.bz2 \ 
    && mv shape_predictor_68_face_landmarks.dat ../data/ \
    && wget https://storage.cmusatyalab.org/openface-models/nn4.small2.v1.t7 \
    && mv nn4.small2.v1.t7 ../data/

RUN ln -s /root/workspace/ros/src/open_ptrack /root/open_ptrack

WORKDIR /root
