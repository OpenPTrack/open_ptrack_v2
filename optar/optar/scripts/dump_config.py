#!/usr/bin/env python

import rospy
from dynamic_reconfigure import Client
import time

def getNodeConf(nodeName):
    print("getting config for ",nodeName,"...")
    dynConfClient = dynamic_reconfigure.Client(nodeName,3)
    if dynConfClient == None:
        rospy.logerr("Failed to connect to dynamic_reconfigure for node ",nodeName)
        return
    conf == dynConfClient.getConfiguration(3)
    if conf == None:
        rospy.logerr("Failed to get configuration for node ",nodeName)
        return
    return conf


time.sleep(5)
filestamp = rospy.get_param("testing_start_time")

k01Conf = getNodeConf("ardevices_pose_estimator_single_camera_raw_kinect01")
k02Conf = getNodeConf("ardevices_pose_estimator_single_camera_raw_kinect02")
estimatorConf = getNodeConf("ardevices_registration_estimator")

file = open(filestamp+"conf","w")
file.write("\n")
file.write("kinect01 configuration:\n")
file.write(k01Conf)
file.write("\n")
file.write("kinect02 configuration:\n")
file.write(k02Conf)
file.write("\n")
file.write("estimator configuration:\n")
file.write(estimator)
file.close()
