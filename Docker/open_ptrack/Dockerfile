FROM openptrack/open_ptrack-dep
LABEL maintainer "Samir Tabriz"

ARG branch=master

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash \
    && cd ~/workspace/ros/src \
    && git clone https://github.com/openptrack/open_ptrack_v2 open_ptrack \
    && cd open_ptrack \
    && git checkout $branch \
    && apt-get update && apt-get install -y ros-kinetic-qt-build qml-module-qtquick-controls \
    && rm -rf /var/lib/apt/lists/* \
    && source /root/.bashrc \
    && cd ../..  \
    && LD_LIBRARY_PATH=/root/workspace/ros/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/usr/local/lib/x86_64-linux-gnu:/usr/local/lib/i386-linux-gnu:/usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu:/usr/local/cuda/lib64:/usr/local/opencv3/lib \
    && export LD_LIBRARY_PATH \
    && catkin_make"

RUN /bin/bash -c "source /root/workspace/ros/devel/setup.bash \
    && roscd yolo_detector/darknet_opt \
    && wget https://pjreddie.com/media/files/yolo.weights \
    && mv yolo.weights coco.weights"

RUN ln -s /root/workspace/ros/src/open_ptrack /root/open_ptrack 