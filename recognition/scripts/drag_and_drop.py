#!/usr/bin/python
import os
import csv
import sys
import cv2
import dlib
import numpy, math
import requests
from PyQt4 import QtGui

import rospy
import rospkg
from std_msgs.msg import *
from opt_msgs.msg import *
from recognition.srv import *
from cv_bridge import CvBridge

from dynamic_reconfigure.server import Server
from recognition.cfg import FaceFeatureExtractionConfig

from open_face_recognition import *


# a widget for displaying a face and a name in one row
class FaceAndNameWidget(QtGui.QWidget):
	# constructor
	# img : the face image (numpy.array)
	# name : the name (string)
	def __init__(self, img, name):
		super(FaceAndNameWidget, self).__init__()
		self.img = img
		self.name = name

		img = cv2.resize(img, (128, 128))
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		self.qimg = QtGui.QImage(img.data, img.shape[1], img.shape[0], img.shape[1] * img.shape[2], QtGui.QImage.Format_RGB888)

		self.img_label = QtGui.QLabel()
		self.img_label.setPixmap(QtGui.QPixmap.fromImage(self.qimg))

		self.name_label = QtGui.QLabel()
		self.name_label.setText(name)

		self.layout = QtGui.QHBoxLayout(self)
		self.layout.addWidget(self.img_label)
		self.layout.addWidget(self.name_label)


# a dialog for inputing a person's name
class AskNameDialog(QtGui.QDialog):
	# constructor
	# img : the face image (numpy.array)
	def __init__(self, img):
		super(AskNameDialog, self).__init__()

		self.setGeometry(400, 400, 0, 0)
		self.layout = QtGui.QVBoxLayout(self)

		self.label = QtGui.QLabel('Please input his/her name!')
		self.layout.addWidget(self.label)

		img = cv2.resize(img, (128, 128))
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		self.qimg = QtGui.QImage(img.data, img.shape[1], img.shape[0], img.shape[1] * img.shape[2], QtGui.QImage.Format_RGB888)

		self.img_label = QtGui.QLabel()
		self.img_label.setPixmap(QtGui.QPixmap.fromImage(self.qimg))
		self.layout.addWidget(self.img_label)

		self.edit = QtGui.QLineEdit()
		self.layout.addWidget(self.edit)

		self.button_layout = QtGui.QHBoxLayout()
		self.ok_button = QtGui.QPushButton('OK')
		self.ok_button.clicked.connect(self.accept)
		self.cancel_button = QtGui.QPushButton('Cancel')
		self.cancel_button.clicked.connect(self.reject)
		self.button_layout.addWidget(self.ok_button)
		self.button_layout.addWidget(self.cancel_button)

		self.buttons = QtGui.QWidget()
		self.buttons.setLayout(self.button_layout)
		self.layout.addWidget(self.buttons)


# main widget
class Widget(QtGui.QWidget):
	# constructor
	# app : QApplication
	def __init__(self, app):
		super(Widget, self).__init__()
		self.app = app
		self.initUI()
		self.setAcceptDrops(True)

		self.face_widgets = []
		self.detector = dlib.get_frontal_face_detector()

		self.embedding = None
		self.cfg_server = Server(FaceFeatureExtractionConfig, self.cfgCallback)

	# reconfigure callback
	def cfgCallback(self, config, level):
		package_path = rospkg.RosPack().get_path('recognition')
		self.image_dim_for_dlib = config.image_dim_for_dlib
		self.shape_predictor_path = package_path + config.shape_predictor_path

		self.image_dim_for_openface = config.image_dim_for_openface
		self.network_path = package_path + config.network_path

		print '--- cfg_callback ---'
		print 'image_dim_for_dlib', config.image_dim_for_dlib
		print 'image_dim_for_openface', config.image_dim_for_openface
		return config

	# initializes the UI
	def initUI(self):
		self.setWindowTitle('drop a face image!!')
		self.setGeometry(300, 300, 400, 300)

		self.layout = QtGui.QVBoxLayout(self)
		self.progressbar = QtGui.QProgressBar()
		self.progressbar.setTextVisible(True)
		self.layout.addWidget(self.progressbar)

		self.scroll = QtGui.QScrollArea()
		self.scroll.setWidgetResizable(True)
		self.scroll.setFixedWidth(400)
		self.scroll.setFixedHeight(300)
		self.layout.addWidget(self.scroll)

		self.scroll_contents = QtGui.QWidget(self.scroll)
		self.scroll.setWidget(self.scroll_contents)

		self.faces_layout = QtGui.QVBoxLayout()
		self.scroll_contents.setLayout(self.faces_layout)

		self.button_layout = QtGui.QHBoxLayout()
		self.save_button = QtGui.QPushButton('Save')
		self.load_button = QtGui.QPushButton('Load')
		self.send_button = QtGui.QPushButton('Send')
		self.save_button.clicked.connect(self.saveButtonCallback)
		self.load_button.clicked.connect(self.loadButtonCallback)
		self.send_button.clicked.connect(self.sendButtonCallback)
		self.button_layout.addWidget(self.save_button)
		self.button_layout.addWidget(self.load_button)
		self.button_layout.addWidget(self.send_button)

		self.layout.addLayout(self.button_layout)

	# callback for save button pushing event
	def saveButtonCallback(self, e):
		self.setProgress(50, 'choose save filename')
		dialog = QtGui.QFileDialog()
		if not dialog.exec_():
			self.setProgress(0, 'canceled')
			return

		save_filename = dialog.selectedFiles()
		if len(save_filename) == 0:
			self.setProgress(0, 'no files are selected')
			return

		self.setProgress(75, 'saving')
		save_filename = str(save_filename[0])
		save_dirname = save_filename + '_files'
		if not os.path.isdir(save_dirname):
			os.mkdir(save_dirname)

		with open(save_filename, 'w') as file:
			for i, face_widget in enumerate(self.face_widgets):
				img_filename = save_dirname + '/%03d.jpg' % i
				cv2.imwrite(img_filename, face_widget.img)
				print >> file, '"%s" "%03d.jpg"' % (face_widget.name, i)
		self.setProgress(100, 'saved')

	# callback for load button pushing event
	def loadButtonCallback(self, e):
		self.setProgress(25, 'choose load filename')
		load_filename = QtGui.QFileDialog.getOpenFileName()
		if len(load_filename) == 0:
			self.setProgress(0, 'canceled')
			return
		load_filename = str(load_filename)
		load_dirname = load_filename + '_files'

		if not os.path.isdir(load_dirname):
			self.setProgress(0, 'directory not found')
			print 'directory not found'
			return

		self.setProgress(50, 'clearing the face widgets')
		self.face_widgets = []
		for i in reversed(range(self.faces_layout.count())):
			wid = self.faces_layout.itemAt(i).widget()
			self.faces_layout.removeWidget(wid)
			wid.setParent(None)

		self.setProgress(75, 'loading')
		with open(load_filename, 'r') as file:
			for row in csv.reader(file, delimiter=' ', quotechar='"'):
				name = row[0]
				img_filename = load_dirname + '/' + row[1]
				img = cv2.imread(img_filename)
				if img is None:
					print 'failed to open the image'
					continue
				
				#img = self.improve(img)
				face_widget = FaceAndNameWidget(img, name)
				self.face_widgets.append(face_widget)
				self.faces_layout.addWidget(self.face_widgets[-1])
				print name, img_filename
		self.setProgress(100, 'done')

	# callback for send button pushing event
	# sends faces to a ROS node
	def sendButtonCallback(self, e):
		if len(self.face_widgets) == 0:
			self.setProgress(0, 'no faces are registered')
			return

		if self.embedding is None:
			self.setProgress(25, 'loading the network model', 10)
			self.embedding = OpenFaceEmbedding(self.image_dim_for_openface, self.shape_predictor_path, self.network_path)

		self.setProgress(50, 'sending the faces', 1000)
		bridge = CvBridge()
		set_predefined_faces = rospy.ServiceProxy('/face_recognition/set_predefined_faces', OPTSetPredefinedFaces)
		req = OPTSetPredefinedFacesRequest()
		for face_widget in self.face_widgets:
			req.names.append(face_widget.name)
			req.faces.append(bridge.cv2_to_imgmsg(face_widget.img))
			vec = self.embedding.embed(face_widget.img)
			req.features.append(self.vector2multiarray(vec))

		try:
			set_predefined_faces(req)
		except:
			self.setProgress(0, 'service unavailable')
			return

		self.setProgress(100, 'done')
		print 'sent images to the face reocognition node'

	# reads an image from the url
	# url : the location to the image (string, like 'http://www.*.jpg' or 'file:///home/*/.jpg'
	def readImage(self, url):
		if url.count('file:///'):
			# get the image from the local directory
			img = cv2.imread(url[url.find('file:///') + 7:])
		else:
			# get the image from the internet
			response = requests.get(url)
			data = numpy.fromstring(response.content, numpy.uint8)
			img = cv2.imdecode(data, 1)

		return img

	def improve(self, rgb_image):
		if numpy.amax(rgb_image) > 1:
			info = numpy.iinfo(rgb_image.dtype)
			rgb_image = rgb_image.astype(numpy.float) / info.max
		"""
		sz = rgb_image.shape
		B = rgb_image[:,:,0]
		G = rgb_image[:,:,1]
		R = rgb_image[:,:,2]
		B = numpy.pad(B, (10,), 'edge')
		G = numpy.pad(G, (10,), 'edge')
		R = numpy.pad(R, (10,), 'edge')
		rgb_image = numpy.dstack((B, G, R))
		"""

		w = 0.8

		Inv = 1 - rgb_image
		B = Inv[:,:,0]
		G = Inv[:,:,1]
		R = Inv[:,:,2]

		B1 = numpy.ravel(B)
		G1 = numpy.ravel(G)
		R1 = numpy.ravel(R)

		I = (B1 + G1 + R1) / 3

		n = I.size
		N = math.floor(n * 0.002)

		Be = cv2.erode(B, numpy.ones((7,7), numpy.uint8), 1)
		Ge = cv2.erode(G, numpy.ones((7,7), numpy.uint8), 1)
		Re = cv2.erode(R, numpy.ones((7,7), numpy.uint8), 1)

		dc = numpy.minimum(numpy.minimum(numpy.ravel(Be), numpy.ravel(Ge)), numpy.ravel(Re))
		i = numpy.argsort(-dc)
		tmp = I[i[0:int(N)]]
		j = numpy.argsort(-tmp)

		Ab = B1[i[j[0]]]
		Ag = G1[i[j[0]]]
		Ar = R1[i[j[0]]]

		t = numpy.maximum(1 - w * numpy.minimum(numpy.minimum(Re/Ar, Ge/Ag), Be/Ab), 10**(-7))
		lc = t < 0.5
		t[lc] = 2 * t[lc]**2

		Sb = (B - Ab) / t + Ab
		Sg = (G - Ag) / t + Ag
		Sr = (R - Ar) / t + Ar
		Sb = numpy.clip(Sb, 0, 1)
		Sg = numpy.clip(Sg, 0, 1)
		Sr = numpy.clip(Sr, 0, 1)

		comb = numpy.dstack((Sb, Sg, Sr))
		out = numpy.uint8((1 - comb)*255.999) #[11:sz[0]+10, 11:sz[1]+10, :]
		#cv2.imwrite('improved.png', out)

		return out

	# detects a face in the image
	# img : the input image
	def detectFace(self, img):
		# the img is upscaled so that it become larger than 512 pix
		upscale = 512 / max(img.shape[0], img.shape[1])
		print 'detecting', upscale, img.shape
		#img = self.improve(img)
		detected = self.detector(img, upscale)
		if len(detected) == 0:
			return None

		face = detected[0]
		img = img[face.top(): face.bottom(), face.left(): face.right()]
		return img

	# sets the text of the progress bar
	def setProgress(self, value, text, wait=10):
		self.progressbar.setFormat(text)
		self.progressbar.setValue(value)
		cv2.waitKey(10)

	# the callback for the drag enter event
	def dragEnterEvent(self, e):
		if e.mimeData().hasUrls():
			e.accept()

	# the callback for the drop event
	def dropEvent(self, e):
		for url in e.mimeData().urls():
			# gets the image from the url
			self.setProgress(25, 'getting the image...')
			img = self.readImage(str(url.toString()))
			if img is None:
				self.setProgress(0, 'failed to obtain the image...')
				continue

			# detects a face in the image
			self.setProgress(50, 'detecting a face...', 2000)
			face = self.detectFace(img)
			if face is None:
				self.setProgress(0, 'failed to detect a face...')
				continue

			# shows a dialog to ask his/her name
			self.setProgress(75, 'input his/her name')
			dialog = AskNameDialog(face)
			dialog.show()
			if not dialog.exec_():
				self.setProgress(0, 'canceled')
				continue

			# adds the face and the name to the widget
			self.setProgress(100, 'done')
			face_widget = FaceAndNameWidget(face, str(dialog.edit.text()))
			self.face_widgets.append(face_widget)
			self.faces_layout.addWidget(self.face_widgets[-1])

	def vector2multiarray(self, vector):
		multiarray = Float32MultiArray()
		multiarray.layout.dim = [MultiArrayDimension()]
		multiarray.layout.dim[0].label = 'features'
		multiarray.layout.dim[0].size = len(vector) if vector is not None else 0
		multiarray.layout.dim[0].stride = 0
		multiarray.layout.data_offset = 0

		multiarray.data = vector if vector is not None else []
		return multiarray


# entry point
def main():
	rospy.init_node('drag_and_drop')
	app = QtGui.QApplication(sys.argv)
	widget = Widget(app)
	widget.show()
	app.exec_()


if __name__ == '__main__':
	main()
