#!/usr/bin/python
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

from open_face_recognition import *
import recognition_utils as recutils


class FaceFeatureExtractionNode:
	def __init__(self, sensor_name):
		self.sensor_name = sensor_name

		self.cfg_server = Server(FaceFeatureExtractionConfig, self.cfg_callback)
		self.cv_bridge = cv_bridge.CvBridge()
		self.embedding = OpenFaceEmbedding(self.image_dim_for_openface, self.shape_predictor_path, self.network_path)

		self.pub = rospy.Publisher('/face_feature_extractor/features', FeatureVectorArray, queue_size=10)

		try:
			print 'trying to listen raw rgb image topic...'
			rospy.client.wait_for_message(self.sensor_name + '/rgb/image', Image, 1.0)
			img_subscriber = message_filters.Subscriber(self.sensor_name + '/rgb/image', Image, queue_size=30)
		except rospy.ROSException:
			print 'failed, listen compressed rgb image topic'
			img_subscriber = message_filters.Subscriber(self.sensor_name + '/rgb/image/compressed', CompressedImage, queue_size=30)

		self.subscribers = [
			img_subscriber,
			message_filters.Subscriber(self.sensor_name + '/face_detector/detections', DetectionArray, queue_size=30)
		]

		# TypeSynchronizer doesn't work, the image time and the detection time are slightly different?
		# self.ts = message_filters.TimeSynchronizer(subscribers, 5)
		# self.ts = message_filters.ApproximateTimeSynchronizer(self.subscribers, 5, 0.0001)
		self.ts = recutils.TimeSynchronizer(self.subscribers, 60, 1000)
		self.ts.registerCallback(self.callback)

		self.reset_time_sub = rospy.Subscriber('/reset_time', Empty, self.reset_time)

	# cfg_callback
	def cfg_callback(self, config, level):
		package_path = rospkg.RosPack().get_path('recognition')

		self.roi_upscale_for_dlib = config.roi_upscale_for_dlib
		self.image_dim_for_dlib = config.image_dim_for_dlib
		self.shape_predictor_path = package_path + config.shape_predictor_path

		self.image_dim_for_openface = config.image_dim_for_openface
		self.network_path = package_path + config.network_path

		print '--- cfg_callback ---'
		print 'image_dim_for_dlib', config.image_dim_for_dlib
		print 'image_dim_for_openface', config.image_dim_for_openface
		print 'roi_upscale_for_dlib', config.roi_upscale_for_dlib
		return config

	def reset_time(self, msg):
		print 'reset time'
		self.ts = message_filters.ApproximateTimeSynchronizer(self.subscribers, 5, 0.001)
		self.ts.registerCallback(self.callback)

	# callback
	def callback(self, rgb_image_msg, detection_msg):
		if self.sensor_name not in detection_msg.header.frame_id:
			print 'frame ids conflicted!!'
			return

		# if all the detected face regions are invalid, skip the feature extraction
		if all((x.box_2D.width <= 0 for x in detection_msg.detections)):
			return

		t1 = rospy.Time.now()
		# read rgb image
		if type(rgb_image_msg) is CompressedImage:
			rgb_image = recutils.decompress(rgb_image_msg)
		else:
			rgb_image = self.cv_bridge.imgmsg_to_cv2(rgb_image_msg)
		rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)

		features = map(lambda x: self.extract_features(rgb_image, x.box_2D), detection_msg.detections)

		features_msg = FeatureVectorArray()
		features_msg.header = detection_msg.header
		features_msg.images = map(lambda x: self.cv2_to_imgmsg(x), map(lambda x: x[0], features))
		features_msg.vectors = map(lambda x: self.vector2multiarray(x), map(lambda x: x[1], features))

		if any(x is not None for x in features):
			self.pub.publish(features_msg)

		t2 = rospy.Time.now()
		duration = (t2 - t1).to_sec()
		print duration, '[sec], ', 1.0 / max(duration, 0.001), '[fps]'

	def cv2_to_imgmsg(self, x):
		if x is None:
			return Image()
		return self.cv_bridge.cv2_to_imgmsg(x)

	def extract_features(self, rgb_image, box_2d):
		if box_2d.width <= 0:
			return None, None

		half_scale = (self.roi_upscale_for_dlib - 1) / 2
		left = max(0, int(box_2d.x - box_2d.width * half_scale))
		right = min(rgb_image.shape[1], int(box_2d.x + box_2d.width * (1 + half_scale)))
		top = max(0, int(box_2d.y - box_2d.height * half_scale))
		bottom = min(rgb_image.shape[0], int(box_2d.y + box_2d.height * (1 + half_scale)))

		face_roi = rgb_image[top:bottom, left:right]
		if face_roi.shape[0] < self.image_dim_for_dlib:
			face_roi = cv2.resize(face_roi, (self.image_dim_for_dlib, self.image_dim_for_dlib))
		return rgb_image[top:bottom, left:right], self.embedding.embed(face_roi)

	def vector2multiarray(self, vector):
		multiarray = Float32MultiArray()
		multiarray.layout.dim = [MultiArrayDimension()]
		multiarray.layout.dim[0].label = 'features'
		multiarray.layout.dim[0].size = len(vector) if vector is not None else 0
		multiarray.layout.dim[0].stride = 0
		multiarray.layout.data_offset = 0

		multiarray.data = vector if vector is not None else []
		return multiarray


def main():
	sensor_name = '/kinect2_head' if len(sys.argv) < 2 else '/' + sys.argv[1]
	print 'sensor_name', sensor_name

	rospy.init_node('face_feature_extraction_node_' + sensor_name[1:])
	node = FaceFeatureExtractionNode(sensor_name)
	rospy.spin()


if __name__ == '__main__':
	main()
