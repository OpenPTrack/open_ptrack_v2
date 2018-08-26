#!/usr/bin/env python
import cv2
import dlib
import openface


# wrapper for openface
class OpenFaceEmbedding:
	# constructor
	def __init__(self, image_dim, shape_predictor_path, network_path):
		print 'shape_predictor_path', shape_predictor_path
		print 'network_path', network_path
		print 'loading network...',
		self.image_dim = image_dim
		self.align = openface.AlignDlib(shape_predictor_path)
		self.net = openface.TorchNeuralNet(network_path, self.image_dim)
		print 'done'

	# this method calculates a real vector which represents the given face image
	def embed(self, bgr_image, is_rgb=False):
		rgb_image = bgr_image
		if not is_rgb:
			rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)

		bb = self.align.getLargestFaceBoundingBox(rgb_image)
		# bb = self.align_hueristic(rgb_image)
		if bb is None:
			if rgb_image.shape[0] > 64:
				scale = 0.5
				print 're-try with smaller size', int(rgb_image.shape[0] * scale)
				rgb_image = cv2.resize(rgb_image, (int(rgb_image.shape[1] * scale), int(rgb_image.shape[0] * scale)))
				return self.embed(rgb_image, True)
			print 'warning : failed to obtain face bounding box!!'
			return None
			print 'warning : use heuristic align!!'
			bb = self.align_hueristic(rgb_image)

		aligned = self.align.align(self.image_dim, rgb_image, bb, landmarkIndices=openface.AlignDlib.OUTER_EYES_AND_NOSE)
		if aligned is None:
			print 'warning : failed to align input face!!'
			return None

		return self.net.forward(aligned)

	def align_hueristic(self, rgb_image):
		top = int(rgb_image.shape[0] * 0.25)
		bottom = int(rgb_image.shape[0] * 0.9)
		left = int(rgb_image.shape[1] * 0.15)
		right = int(rgb_image.shape[1] * 0.85)
		return dlib.rectangle(top, left, bottom, right)
