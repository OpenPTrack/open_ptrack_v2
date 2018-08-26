#!/usr/bin/python
import os
import cv2
import numpy
import shutil

import tf
import rospy
import cv_bridge
import message_filters
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from opt_msgs.msg import *
from tf2_msgs.msg import *

import recognition_utils as recutils


class FaceSaverNode:
	def __init__(self, sensor_names):
		self.syncs = []
		self.id_table = {}
		self.bridge = cv_bridge.CvBridge()

		for sensor_name in sensor_names:
			try:
				rospy.client.wait_for_message('/%s/rgb/image' % sensor_name, Image, 1.0)
				image_sub = message_filters.Subscriber('/%s/rgb/image' % sensor_name, Image)
			except:
				image_sub = message_filters.Subscriber('/%s/rgb/image/compressed' % sensor_name, CompressedImage)

			self.subscribers = [
				image_sub,
				message_filters.Subscriber('/%s/face_detector/detections' % sensor_name, DetectionArray),
				message_filters.Subscriber('/face_feature_extractor/features', FeatureVectorArray),
				message_filters.Subscriber('/tracker/association_result', Association)
			]
			self.syncs.append(recutils.TimeSynchronizer(self.subscribers, 8192, 1000))
			self.syncs[-1].registerCallback(self.callback)

		self.subscribers2 = [
			message_filters.Subscriber('/tracker/tracks_smoothed', TrackArray),
			message_filters.Subscriber('/face_recognition/people_tracks', TrackArray)
		]
		self.sync2 = message_filters.TimeSynchronizer(self.subscribers2, 1024)
		self.sync2.registerCallback(self.track_callback)

	def track_callback(self, track_msg, face_track_msg):
		if len(track_msg.tracks) != len(face_track_msg.tracks):
			return

		for i in range(len(track_msg.tracks)):
			track = track_msg.tracks[i]
			ftrack = face_track_msg.tracks[i]
			if ftrack.id < 10000:
				self.id_table[track.id] = ftrack.id

	def callback(self, img_msg, detection_msg, feature_msg, association_msg):
		results = []
		for detection, feature, tracker_id in zip(detection_msg.detections, feature_msg.vectors, association_msg.track_ids):
			if len(feature.data) < 5:
				continue

			box = detection.box_2D
			if box.width <= 0:
				continue

			if tracker_id not in self.id_table:
				continue

			results.append([box, tracker_id, self.id_table[tracker_id]])

		if len(results) == 0:
			return

		if type(img_msg) is Image:
			image = self.bridge.imgmsg_to_cv2(img_msg)
		else:
			image = recutils.decompress(img_msg)

		for box, tracker_id, face_id in results:
			dirname = 'face_saver/%02d' % face_id
			if not os.path.isdir(dirname):
				os.makedirs(dirname)

			face = image[box.y:box.y+box.height, box.x:box.x+box.width]
			filename = '%s/%04d_%03d.jpg' % (dirname, association_msg.header.seq, tracker_id)
			cv2.imwrite(filename, face)


def main():
	if os.path.isdir('face_saver'):
		shutil.rmtree('face_saver')
	print '--- face_saver ---'
	rospy.init_node('face_saver_node')
	# sensor_names = ['kinect2_back_l', 'kinect2_back_r', 'kinect2_front_l', 'kinect2_fr_r', 'kinect2_cent_r']
	sensor_names = ['kinect2_head', 'kinect2_far', 'kinect2_lenovo']
	node = FaceSaverNode(sensor_names)
	rospy.spin()

if __name__ == '__main__':
	main()
