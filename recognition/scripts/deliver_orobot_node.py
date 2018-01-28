#!/usr/bin/python
import re
import cv2
import math
import numpy

import tf
import rospy
import actionlib
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from opt_msgs.msg import *
from tf2_msgs.msg import *
from move_base_msgs.msg import *



class DeliverOrobotNode:
	def __init__(self):
		self.names = {}

		self.target_name = 'Kenji Koide'
		self.names_sub = rospy.Subscriber('/face_recognition/people_names', NameArray, self.names_callback, queue_size=1, buff_size=2**10)
		self.tracks_sub = rospy.Subscriber('/face_recognition/people_tracks', TrackArray, self.tracks_callback, queue_size=1, buff_size=2**10)

		self.done = False
		self.move_base_action = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		while not self.move_base_action.wait_for_server(rospy.Duration(5)):
			print 'waiting for the action server'

	def tracks_callback(self, tracks_msg):
		if self.done:
			return

		for track in tracks_msg.tracks:
			face_id = track.id

			if face_id in self.names and self.names[face_id] == self.target_name:
				pos = (track.x, track.y, 0.0)
				print 'target found!!', pos
				self.move_to(pos)
				self.done = True

	def names_callback(self, names_msg):
		names = {}
		for i in range(len(names_msg.ids)):
			names[names_msg.ids[i]] = names_msg.names[i]
		self.names = names

	def move_to(self, pos):
		goal = MoveBaseGoal()
		goal_pose = goal.target_pose
		goal_pose.header.frame_id = '/world'
		goal_pose.header.stamp = rospy.Time.now()
		goal_pose.pose.position.x = pos[0]
		goal_pose.pose.position.y = pos[1]
		goal_pose.pose.position.z = 0.0

		goal_pose.pose.orientation.x = 0.0
		goal_pose.pose.orientation.y = 0.0
		goal_pose.pose.orientation.z = 0.0
		goal_pose.pose.orientation.w = 1.0

		self.move_base_action.send_goal_and_wait(goal)


def main():
	print '--- deliver_orobot_node ---'
	rospy.init_node('deliver_orobot_node')
	node = DeliverOrobotNode()
	rospy.spin()

if __name__ == '__main__':
	main()
