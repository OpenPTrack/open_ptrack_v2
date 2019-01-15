FROM openptrack/open_ptrack-dep
LABEL maintainer "Samir Tabriz"

ARG branch=zed

RUN echo ttf-mscorefonts-installer msttcorefonts/accepted-mscorefonts-eula select true | debconf-set-selections && \
    apt-get update && apt-get install -y apt-transport-https apt-utils less lsb-release udev wget && \
    mkdir -p /etc/udev/rules.d/ && \
    cd /tmp/ && \
    wget https://www.stereolabs.com/developers/downloads/ZED_SDK_Linux_Ubuntu16_v2.6.0.run && \
    chmod +x ./ZED_SDK_Linux_Ubuntu16_*.run && \
    ./ZED_SDK_Linux_Ubuntu16_*.run --noexec --keep --target temp_zed && \
    rm -f ZED_SDK_Linux_Ubuntu16_*.run && \
    cd temp_zed/ && \
    ./linux_install_release.sh silent && \
    rm -rf /var/lib/apt/lists/* && \
    cd / && \
    rm -rf /tmp/temp_zed

RUN git clone --branch v2.5.x https://github.com/stereolabs/zed-ros-wrapper.git ~/workspace/ros/src/zed-ros-wrapper

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash \
    && git clone https://github.com/openptrack/open_ptrack_v2 ~/workspace/ros/src/open_ptrack \
    && cd ~/workspace/ros/src/open_ptrack \
    && git checkout $branch \
    && rm -rf yolo_detector \
    && cd ../..  \
    && export LD_LIBRARY_PATH=/root/workspace/ros/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/usr/local/lib/x86_64-linux-gnu:/usr/local/lib/i386-linux-gnu:/usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu:/usr/local/cuda/lib64:/usr/local/opencv3/lib \
    && catkin_make"

#RUN /bin/bash -c "source /root/workspace/ros/devel/setup.bash \
#    && roscd yolo_detector/darknet_opt \
#    && wget -O coco.weights https://pjreddie.com/media/files/yolo.weights"

RUN cd /root/workspace/ros/src/open_ptrack/recognition/install_scripts \
    && wget http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2 \
    && bunzip2 shape_predictor_68_face_landmarks.dat.bz2 \ 
    && mv shape_predictor_68_face_landmarks.dat ../data/ \
    && wget https://storage.cmusatyalab.org/openface-models/nn4.small2.v1.t7 \
    && mv nn4.small2.v1.t7 ../data/

RUN ln -s /root/workspace/ros/src/open_ptrack /root/open_ptrack

WORKDIR /root
