FROM openptrack/open_ptrack
LABEL maintainer "Samir Tabriz"
WORKDIR /root

ENV ROS_MASTER_URI = http://192.168.100.101:11311/
ENV ROS_IP = 192.168.100.101
ENV ROS_PC_NAME = PC1

ARG MACHINE_TYPE=Server

COPY ./open_ptrack_config/$MACHINE_TYPE/etc/ntp.conf /etc/ntp.conf