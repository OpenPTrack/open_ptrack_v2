#!/usr/bin/python
import csv
import rospy
import rospkg
import cv_bridge
import message_filters
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from opt_msgs.msg import *

from dynamic_reconfigure.server import Server
from recognition.cfg import FaceFeatureExtractionConfig
from face_comparing.srv import *


class EvalReidNode:
	def __init__(self):
		self.appeared_time = {}
		self.recognized_ids = {}
		self.face_name = {}
		self.read_data()

		self.recognition_time = []
		self.success = 0
		self.failure = 0

		subs = [
			message_filters.Subscriber('/face_recognition/people_tracks', TrackArray),
			message_filters.Subscriber('/tracker/tracks_smoothed', TrackArray)
		]
		self.sync = message_filters.TimeSynchronizer(subs, 100)
		self.sync.registerCallback(self.callback)

	def read_data(self):
		names = {}
		n = 0
		for line in csv.reader(open('data2', 'r'), delimiter=' '):
			for id in line[1:]:
				names[int(id)] = line[0]
				n += 1
			n -= 1
		print names, n
		self.names = names

		self.face_name = {0: 'Kenji', 1: 'Francisca', 2: 'Enrico', 3: 'Andrea', 4: 'Yongheng', 5: 'Stefano'}

	def callback(self, face_msg, track_msg):
		for face, track in zip(face_msg.tracks, track_msg.tracks):
			track_id = track.id
			face_id = face.id

			if track_id not in self.names:
				continue

			if track_id not in self.appeared_time:
				self.appeared_time[track_id] = rospy.Time.now()

			if face_id < 10000 and track_id not in self.recognized_ids:
				self.recognized_ids[track_id] = (rospy.Time.now() - self.appeared_time[track_id]).to_sec()
				self.recognition_time.append(self.recognized_ids[track_id])

				print track_id, face_id, self.names[track_id]

				if face_id in self.face_name:
					if self.face_name[face_id] != self.names[track_id]:
						self.failure += 1
					else:
						self.success += 1
				self.face_name[face_id] = self.names[track_id]
				print self.failure, self.success, sum(self.recognition_time) / len(self.recognition_time)
				print self.face_name



def main():
	rospy.init_node('eval_reid')
	node = EvalReidNode()
	rospy.spin()


if __name__ == '__main__':
	main()
