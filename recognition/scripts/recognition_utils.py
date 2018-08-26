#!/usr/bin/python
import cv2
import numpy
import threading

import tf
import rospy
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from message_filters import *
from opt_msgs.msg import *


def decompress(msg, flags=1):
	data = numpy.fromstring(msg.data, dtype=numpy.uint8)
	if msg.format.count('compressedDepth'):
		# skip depth format(enum) and params(float x 2)
		data = data[12:]
	img = cv2.imdecode(data, flags)
	return img


def lookupTransform(tf_listener, source, target, timeout, now):
	tf_listener.waitForTransform(source, target, now, rospy.Duration(timeout))
	return tf_listener.asMatrix(target, rospy.Header(frame_id=source))


class TimeSynchronizer(SimpleFilter):
	def __init__(self, fs, queue_size, nsecs_round):
		SimpleFilter.__init__(self)
		self.nsecs_round = nsecs_round
		self.connectInput(fs)
		self.queue_size = queue_size
		self.lock = threading.Lock()
		self.last_time = rospy.Time(0, 0)

	def connectInput(self, fs):
		self.queues = [{} for f in fs]
		self.input_connections = [f.registerCallback(self.add, q) for (f, q) in zip(fs, self.queues)]

	def add(self, msg, my_queue):
		self.lock.acquire()
		time = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
		duration = (self.last_time - time).to_sec()
		self.last_time = time
		if duration > 15.0:
			print 'timestamp skiped to the past!!\nclear the synchronization queue!!'
			print 'time difference', -duration, ' [sec]'
			while len(my_queue):
				my_queue.clear()
			for queue in self.queues:
				queue.clear()
			self.lock.release()
			return

		msg.header.stamp.nsecs = (msg.header.stamp.nsecs / self.nsecs_round) * self.nsecs_round
		my_queue[msg.header.stamp] = msg
		while len(my_queue) > self.queue_size:
			del my_queue[min(my_queue)]
		# common is the set of timestamps that occur in all queues
		common = reduce(set.intersection, [set(q) for q in self.queues])
		for t in sorted(common):
			# msgs is list of msgs (one from each queue) with stamp t
			msgs = [q[t] for q in self.queues]
			self.signalMessage(*msgs)
			for q in self.queues:
				del q[t]
		self.lock.release()
