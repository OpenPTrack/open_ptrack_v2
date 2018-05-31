# -*- coding: utf-8 -*-
#!/usr/bin/python

import numpy
import rospy
import rospkg
import cv_bridge
import message_filters
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from opt_msgs.msg import *

from open_face_recognition import *
import recognition_utils as recutils


class FaceRecognitionNode:
	def __init__(self):
		self.load_parameters()

		subscribers = [
			message_filters.Subscriber('/detector/detections', DetectionArray),
			message_filters.Subscriber('/face_feature_extractor/features', FeatureVectorArray)
		]

		# TypeSynchronizer doesn't work, the image time and the detection time are slightly different?
		# self.ts = message_filters.TimeSynchronizer(subscribers, 5)
		self.ts = message_filters.ApproximateTimeSynchronizer(subscribers, 5, 0.0001)
		self.ts.registerCallback(self.callback)

	# load parameters from somewhere
	def load_parameters(self):
		package_path = rospkg.RosPack().get_path('recognition')

	# callback
	def callback(self, people_detection_msg, face_features_msg):
		print 'receive'


def main():
	print '--- face_recognition_node ---'
	rospy.init_node('face_recognition_node')
	node = FaceRecognitionNode()
	rospy.spin()


if __name__ == '__main__':
	main()
