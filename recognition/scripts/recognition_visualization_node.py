#!/usr/bin/python
import re
import cv2
import math
import numpy
import threading

import tf
import rospy
import cv_bridge
import message_filters
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from opt_msgs.msg import *
from tf2_msgs.msg import *

from dynamic_reconfigure.server import Server
from recognition.cfg import RecognitionVisualizationConfig

import recognition_utils as recutils


class Camera:
	def __init__(self, camera_name, tf_listener, image_width, refresh_span):
		self.rgb_image = None
		self.image_pos = None
		self.grabbed = False
		self.image_width = image_width
		self.camera_name = camera_name
		self.cv_bridge = cv_bridge.CvBridge()


		self.refresh_span = refresh_span
		self.prev_time = rospy.Time.now()

		print 'waiting for transform'
		#self.camera2world = recutils.lookupTransform(tf_listener, '/%s_rgb_optical_frame' % camera_name, 'world', 10.0, rospy.Time(0))
		#try:
		#	rospy.client.wait_for_message('/%s/rgb/image' % camera_name, Image, 1.0)
		#	self.sub = rospy.Subscriber('/%s/rgb/image' % camera_name, Image, self.image_callback, queue_size=1, buff_size=2**24)
		#except:
		#	self.sub = rospy.Subscriber('/%s/rgb/image/compressed' % camera_name, CompressedImage, self.image_callback, queue_size=1)

	def image_callback(self, image_msg):
		if abs((rospy.Time.now() - self.prev_time).to_sec()) < self.refresh_span:
			return
		self.prev_time = rospy.Time.now()

		if type(image_msg) is CompressedImage:
			rgb_image = recutils.decompress(image_msg)
		else:
			rgb_image = self.cv_bridge.imgmsg_to_cv2(image_msg)
		height = int(rgb_image.shape[0] * self.image_width / float(rgb_image.shape[1]))
		rgb_image = cv2.resize(rgb_image, (self.image_width, height))
		cv2.putText(rgb_image, self.camera_name, (5, 15), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 255), 2)
		cv2.putText(rgb_image, self.camera_name, (5, 15), cv2.FONT_HERSHEY_PLAIN, 0.8, (64, 64, 64), 1)
		self.rgb_image = rgb_image

	def draw(self, canvas, world2map, image_width):
		self.image_width = image_width
		camera2map = numpy.dot(world2map, self.camera2world)
		org = int(camera2map[0, 3]), int(camera2map[1, 3])
		if self.rgb_image is None:
			return

		if self.image_pos is None:
			self.reset_image_pos(world2map, camera2map, (self.rgb_image.shape[1], self.rgb_image.shape[0]))
		mat = numpy.float32([[1, 0, self.image_pos[0]], [0, 1, self.image_pos[1]]])
		color = (128, 128, 128) if not self.grabbed else (128, 128, 255)

		cv2.line(canvas, org, (self.image_pos[0] + self.rgb_image.shape[1]/2, self.image_pos[1] + self.rgb_image.shape[0]/2), color)

		cv2.warpAffine(self.rgb_image, mat, (canvas.shape[1], canvas.shape[0]), canvas, cv2.INTER_LINEAR, cv2.BORDER_TRANSPARENT)
		pt1 = self.image_pos
		pt2 = self.image_pos + (self.rgb_image.shape[1], self.rgb_image.shape[0])
		cv2.rectangle(canvas, (pt1[0], pt1[1]), (pt2[0], pt2[1]), color, 2)

		lf = numpy.dot(camera2map, [1, 0, 1, 1]).astype(numpy.int32)
		rf = numpy.dot(camera2map, [-1, 0, 1, 1]).astype(numpy.int32)
		cv2.circle(canvas, org, 3, (0, 255, 0), -1)
		cv2.line(canvas, org, (lf[0], lf[1]), (0, 255, 0))
		cv2.line(canvas, org, (rf[0], rf[1]), (0, 255, 0))
		self.canvas = canvas

	def reset_image_pos(self, world2map, camera2map, image_size):
		org = int(camera2map[0, 3]), int(camera2map[1, 3])
		outer = org - numpy.array((world2map[0, 3], world2map[1, 3]))
		outer = outer / numpy.linalg.norm(outer)

		# self.image_pos = org
		image_center = (outer * (image_size[0]/2, image_size[1]/2) * 1.5 + org).astype(numpy.int32)
		self.image_pos = image_center - (image_size[0] / 2, image_size[1] / 2)

	def hit_test(self, x, y):
		if self.image_pos is None:
			return False
		return (
			x > self.image_pos[0] and
			y > self.image_pos[1] and
			x < self.image_pos[0] + self.rgb_image.shape[1] and
			y < self.image_pos[1] + self.rgb_image.shape[0]
		)

	def mouse_pushed(self, x, y):
		self.grabbed = True
		self.prev_pos = numpy.array((x, y))

	def mouse_moved(self, x, y):
		current_pos = numpy.array((x, y))
		self.image_pos += current_pos - self.prev_pos
		self.prev_pos = current_pos

	def mouse_released(self):
		self.grabbed = False
		self.prev_pos = None


class RecognitionVisualizationNode:
	def __init__(self):
		#self.cameras = []
		self.cfg_server = Server(RecognitionVisualizationConfig, self.cfg_callback)
		self.cv_bridge = cv_bridge.CvBridge()
		self.tf_listener = tf.TransformListener()
		#self.find_cameras()

		self.names = {}
		self.face_visible_trackers = {}
		self.face_visible_trackers_lock = threading.Lock()
		self.prev_time = rospy.Time.now()

		self.names_sub = rospy.Subscriber('/face_recognition/people_names', NameArray, self.names_callback, queue_size=1, buff_size=2**10)

		self.track_subscribers = [
			message_filters.Subscriber('/tracker/tracks_smoothed', TrackArray),
# 			message_filters.Subscriber('/tracker/people_tracks', TrackArray),
			message_filters.Subscriber('/face_recognition/people_tracks', TrackArray)
		]
		self.tracks_sub = recutils.TimeSynchronizer(self.track_subscribers, 1000, 1)
		self.tracks_sub.registerCallback(self.track_callback)

		self.subscribers = [
			message_filters.Subscriber('/face_feature_extractor/features', FeatureVectorArray),
			message_filters.Subscriber('/tracker/association_result', Association)
		]
		self.features_sub = recutils.TimeSynchronizer(self.subscribers, 1000, 1000)
		self.features_sub.registerCallback(self.features_callback)

	def show(self):
		if not hasattr(self, 'canvas'):
			return

		cv2.imshow('map', self.canvas)
		cv2.setMouseCallback('map', self.mouse_callback)
		cv2.waitKey(10)

	def cfg_callback(self, config, level):
		self.map_size_m = config.map_size_m
		self.map_size_pix = config.map_size_pix
		self.image_width = config.image_width
		self.image_refresh_span = config.image_refresh_span

		self.world2map = numpy.zeros((2, 4), dtype=numpy.float32)
		self.world2map[0, 0] = -self.map_size_pix / self.map_size_m #00
		self.world2map[1, 1] = self.map_size_pix / self.map_size_m #11
		self.world2map[0, 3] = self.map_size_pix / 2 #03
		self.world2map[1, 3] = self.map_size_pix / 2 #13

		#for camera in self.cameras:
		#	camera.refresh_span = config.image_refresh_span

		return config

	def find_cameras(self):
		self.cameras = []

		print 'waiting for topics'
		rospy.client.wait_for_message('/tf', TFMessage, 10.0)
		for topic_name, msg_type in rospy.get_published_topics():
			match = re.match(r'/(kinect.*)/rgb/image', topic_name)
			if not match:
				continue
			self.cameras.append(Camera(match.group(1), self.tf_listener, self.image_width, self.image_refresh_span))

	def track_callback(self, tracker_track_msg, face_track_msg):
		duration = abs(rospy.Time.now() - self.prev_time)
		if duration < rospy.Duration(0.1) or (duration < rospy.Duration(0.5) and not len(tracker_track_msg.tracks)):
			return
		self.prev_time = rospy.Time.now()

		canvas = numpy.ones((self.map_size_pix, self.map_size_pix, 3), dtype=numpy.uint8) * 255
		#for camera in self.cameras:
		#	camera.draw(canvas, self.world2map, self.image_width)

		self.face_visible_trackers_lock.acquire()
		for track, face_tracker in zip(tracker_track_msg.tracks, face_track_msg.tracks):
			p = numpy.dot(self.world2map, (track.x, track.y, track.height, 1)).astype(numpy.int32)
			p = (p[0], p[1])

			tracker_id = track.id
			face_id = face_tracker.stable_id
			if tracker_id in self.face_visible_trackers:
				cv2.circle(canvas, p, 8, (0, 128, 255), -1)

			if face_id >= 10000:
				color = (255, 0, 0)
			elif face_id in self.names:
				color = (0, 255, 0)
			else:
				color = (0, 0, 255)
			cv2.circle(canvas, p, 5, color, -1)
			cv2.putText(canvas, 'id:%d' % face_id, (p[0] + 5, p[1] - 5), cv2.FONT_HERSHEY_PLAIN, 0.8, (64, 64, 64))

			if face_id in self.names:
				name = self.names[face_id]
				cv2.putText(canvas, '%s' % name, (p[0] + 5, p[1] + 10), cv2.FONT_HERSHEY_PLAIN, 0.8, (64, 64, 64))

		self.face_visible_trackers = {k: v for k, v in self.face_visible_trackers.iteritems() if (self.prev_time - v).to_sec() < 0.25}
		self.face_visible_trackers_lock.release()
		self.canvas = canvas

	def names_callback(self, names_msg):
		names = {}
		for i in range(len(names_msg.ids)):
			names[names_msg.ids[i]] = names_msg.names[i]
		self.names = names

	def features_callback(self, features_msg, association_msg):
		now = rospy.Time.now()

		self.face_visible_trackers_lock.acquire()
		for feature, track in zip(features_msg.vectors, association_msg.track_ids):
			if len(feature.data) < 5:
				continue
			self.face_visible_trackers[track] = now
		self.face_visible_trackers_lock.release()

	def mouse_callback(self, event, x, y, flags, userdata):
		# callbacks for left button events
		if event == cv2.EVENT_LBUTTONDOWN:
			self.grabbed = []
			#for camera in self.cameras:
			#	if camera.hit_test(x, y):
			#		self.grabbed.append(camera)
			if not len(self.grabbed):
				self.grabbed.append(self)
				# self.grabbed.extend(self.cameras)

			for grabbed in self.grabbed:
				grabbed.mouse_pushed(x, y)
		elif event == cv2.EVENT_LBUTTONUP:
			for grabbed in self.grabbed:
				grabbed.mouse_released()
			self.grabbed = []
		elif hasattr(self, 'grabbed'):
			for grabbed in self.grabbed:
				grabbed.mouse_moved(x, y)

		# callbacks for right button events
		if event == cv2.EVENT_RBUTTONDOWN:
			self.rgrabbed = [self]
			for grabbed in self.rgrabbed:
				grabbed.mouse_rpushed(x, y)
		elif event == cv2.EVENT_RBUTTONUP:
			for grabbed in self.rgrabbed:
				grabbed.mouse_rreleased()
			self.rgrabbed = []
		elif hasattr(self, 'rgrabbed'):
			for grabbed in self.rgrabbed:
				grabbed.mouse_rmoved(x, y)

	# mouse left button callbacks #
	# left button pushed
	def mouse_pushed(self, x, y):
		self.prev_pos = numpy.array((x, y))

	# mouse moved while pushing the left button
	def mouse_moved(self, x, y):
		current_pos = numpy.array((x, y))
		self.world2map[:, 3] += (current_pos - self.prev_pos)
		self.prev_pos = current_pos

	# left button relased
	def mouse_released(self):
		pass

	# mouse right button callbacks #
	# right button pushed
	def mouse_rpushed(self, x, y):
		self.prev_rpos = numpy.array((x, y))

	# mouse moved while pushing the right button
	def mouse_rmoved(self, x, y):
		current_pos = numpy.array((x, y))
		scroll = 10 ** (-(current_pos[1] - self.prev_rpos[1]) / 100.0)
		self.prev_rpos = current_pos

		self.world2map[0, 0] *= scroll
		self.world2map[1, 1] *= scroll

	# right button released
	def mouse_rreleased(self):
		pass


def main():
	print '--- recognition_visualization_node ---'
	rospy.init_node('recognition_visualization_node')
	node = RecognitionVisualizationNode()
	while not rospy.is_shutdown():
		node.show()

if __name__ == '__main__':
	main()
