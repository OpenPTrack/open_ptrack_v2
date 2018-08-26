FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04

LABEL maintainer "Samir Tabriz"

ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES},display,compute,utility

RUN apt-get update \ 
    && apt-get install -y --no-install-recommends mesa-utils \ 
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \ 
    && apt-get install -y --no-install-recommends         ocl-icd-libopencl1         clinfo \ 
    &&     rm -rf /var/lib/apt/lists/*
    
RUN mkdir -p /etc/OpenCL/vendors \ 
    && echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd

ENV NVIDIA_VISIBLE_DEVICES=all

RUN  echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections     \ 
    && echo keyboard-configuration keyboard-configuration/layout select 'English (US)' | debconf-set-selections     \ 
    && echo keyboard-configuration keyboard-configuration/layoutcode select 'us' | debconf-set-selections     \ 
    && apt-get update     \ 
    && apt-get install -y --no-install-recommends software-properties-common wget udev apt-utils sudo 

RUN  add-apt-repository -y ppa:danielrichter2007/grub-customizer     \ 
    && wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | apt-key add -     \ 
    && apt-get install -y apt-transport-https     \ 
    && echo "deb https://download.sublimetext.com/ apt/stable/" | tee /etc/apt/sources.list.d/sublime-text.list     \ 
    && apt-add-repository -y ppa:obsproject/obs-studio     \ 
    && apt-add-repository -y ppa:graphics-drivers/ppa     \ 
    && add-apt-repository -y ppa:yannubuntu/boot-repair     \ 
    && apt-get update     \ 
    && apt-get install -y sublime-text grub-customizer exfat-fuse exfat-utils vim terminator gitg cmake-curses-gui gparted meld cowsay fortune gimp synaptic libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev ncdu filezilla p7zip-full openssh-client openssh-server boot-repair

RUN  sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'     \ 
    && apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116     \ 
    && apt-get update     \ 
    && apt-get install ros-kinetic-desktop-full python-rosinstall -y     \ 
    && rosdep init     \ 
    && rosdep update         

RUN  /bin/bash -c "echo 'source /opt/ros/kinetic/setup.bash' >> /root/.bashrc \ 
    && source /root/.bashrc"      

RUN add-apt-repository -y ppa:levi-armstrong/ppa     \ 
    && apt-get update

RUN  /bin/bash -c "source /opt/ros/kinetic/setup.bash     \ 
    && cd     \ 
    && mkdir -p workspace/ros/src     \ 
    && cd workspace/ros     \ 
    && catkin_make"     

RUN  /bin/bash -c "echo 'source /root/workspace/ros/devel/setup.bash' >> /root/.bashrc"          

RUN  cd ~/workspace/ros     \ 
    && apt-get install -y ocl-icd-opencl-dev     \ 
    && wget https://developer.nvidia.com/compute/cuda/8.0/Prod2/local_installers/cuda_8.0.61_375.26_linux-run -O cuda.run     \ 
    && chmod +x cuda.run     \ 
    && ./cuda.run -silent --toolkit     \ 
    && wget "https://www.dropbox.com/s/cx95583vbf1ifzm/cudnn-8.0-linux-x64-v5.1.tgz?dl%3D0&sa=D&ust=1507541105896000&usg=AFQjCNEU511fT00n547PxPt_P6cfSLPVWw"     \ 
    && tar -zxvf cudnn*     \ 
    && cp cuda/include/* /usr/local/cuda/include     \ 
    && cp cuda/lib64/* /usr/local/cuda/lib64/     \ 
    && echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64' >> ~/.bashrc     \ 
    && echo 'export PATH=$PATH:/usr/local/cuda/bin' >> ~/.bashrc          

RUN  cd     \ 
    && cd workspace     \ 
    && git clone https://github.com/openptrack/libfreenect2     \ 
    && cd libfreenect2     \ 
    && git checkout 1604     \ 
    && cd depends/     \ 
    && apt-get install -y git cmake cmake-curses-gui libxmu-dev libxi-dev libgl1-mesa-dev dos2unix xorg-dev libglu1-mesa-dev libtool automake libudev-dev libgtk2.0-dev pkg-config libjpeg-turbo8-dev libturbojpeg libglewmx-dev     \ 
    && ./install_ubuntu.sh     \ 
    && ln -f -s /usr/lib/x86_64-linux-gnu/libturbojpeg.so.0.1.0 /usr/lib/x86_64-linux-gnu/libturbojpeg.so     \ 
    && cd ../examples/protonect/     \ 
    && mkdir build \ 
    && cd build     \ 
    && /bin/bash -c "source /opt/ros/kinetic/setup.bash    \ 
    && cmake ..     \ 
    && make     \ 
    && make install"     \ 
    && echo '# ATTR{product}=="Kinect2" SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c4", MODE="0666" SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d8", MODE="0666" SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d9", MODE="0666"' > ~/90-kinect2.rules     \ 
    && mv ~/90-kinect2.rules /etc/udev/rules.d/90-kinect2.rules     

RUN  cd ~/workspace/ros/src     \ 
    && git clone https://github.com/openptrack/iai_kinect2     \ 
    && cd iai_kinect2     \ 
    && git checkout 1604       

RUN  apt-get install cmake libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev -y --force-yes     \ 
    && mkdir /tmp/ceres_install     \ 
    && cd /tmp/ceres_install     \ 
    && git clone https://ceres-solver.googlesource.com/ceres-solver     \ 
    && cd ceres-solver     \ 
    && git fetch --tags     \ 
    && git checkout tags/1.9.0     \ 
    && cd ..     \ 
    && mkdir ceres-bin     \ 
    && cd ceres-bin     \ 
    && cmake ../ceres-solver     \ 
    && make -j8     \ 
    && make test     \ 
    && make install     \ 
    && rm -R /tmp/ceres_install     \ 
    && cd ~/workspace/ros/src     \ 
    && git clone https://github.com/iaslab-unipd/calibration_toolkit     \ 
    && cd calibration_toolkit     \ 
    && git fetch origin --tags     \ 
    && git checkout tags/v0.2         

RUN cd     \ 
    && cd workspace     \ 
    && apt-get update     \ 
    && git clone https://github.com/marketto89/opencv     \ 
    && cd opencv     \ 
    && git checkout 3.1.0-with-cuda8     \ 
    && mkdir release     \ 
    && cd release     \ 
    && cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local/opencv3 -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D ENABLE_FAST_MATH=1 -D CUDA_FAST_MATH=1 -D WITH_CUBLAS=1 -D WITH_IPP=ON -D CMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs ..     \ 
    && make -j8 -l8     \ 
    && make install     \ 
    && printf '#OpenCV\nPKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/opencv3/lib/pkgconfig\nexport PKG_CONFIG_PATH\n' >> /root/.bashrc     \ 
    && printf 'PATH=$PATH:/usr/local/opencv3/bin\nexport PATH\n' >> /root/.bashrc      \ 
    && printf 'LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/opencv3/lib\nexport LD_LIBRARY_PATH\n' >> /root/.bashrc       \ 
    && /bin/bash -c "source /root/.bashrc"     \ 
    && ln -s /usr/local/opencv3/share/OpenCV/3rdparty/lib/libippicv.a /usr/local/lib/libippicv.a  

RUN  /bin/bash -c "source /root/.bashrc    \ 
    && cd ~/workspace/ros/src      \ 
    && git clone https://bitbucket.org/mcarraro/rtpose_wrapper     \ 
    && cd rtpose_wrapper     \ 
    && git checkout integration-1604     \ 
    && apt-get --assume-yes install libprotobuf-dev libleveldb-dev libsnappy-dev libhdf5-serial-dev protobuf-compiler libboost-all-dev libgflags-dev libgoogle-glog-dev liblmdb-dev     \ 
    && make all -j8"

RUN  /bin/bash -c "git clone https://github.com/torch/distro.git ~/torch --recursive \ 
    && cd ~/torch \ 
    && bash install-deps \ 
    && ./install.sh -b \ 
    && source ~/.bashrc \ 
    && cd install/bin \ 
    && ./luarocks install dpnn \ 
    && ./luarocks install csvigo"

RUN /bin/bash -c "cd /tmp \
    && git clone https://github.com/cmusatyalab/openface.git \
    && cd openface \
    && sudo python setup.py install"

RUN /bin/bash -c "cd /tmp \
    && rm -rf dlib \
    && git clone https://github.com/davisking/dlib.git \
    && cd dlib \
    && git checkout v19.4 \
    && mkdir python_examples/build \
    && cd python_examples/build \
    && cmake ../../tools/python -DUSE_AVX_INSTRUCTIONS=ON \
    && cmake --build . --config Release \
    && sudo cp dlib.so /usr/local/lib/python2.7/dist-packages"

RUN apt-get update \
    && apt-get install -y python-qt4 python-pip \
    && pip install requests

RUN apt-get install ntp -y  \ 
    && rm -rf /var/lib/apt/lists/*
