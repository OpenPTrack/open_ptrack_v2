#!/usr/bin/python
import os
import csv
import sys
import cv2
import dlib
import numpy
import requests
from PyQt4 import QtGui

import rospy
import rospkg
from std_msgs.msg import *
from opt_msgs.msg import *
from recognition.srv import *
from cv_bridge import CvBridge


# main widget
class Widget(QtGui.QWidget):
	# constructor
	# app : QApplication
	def __init__(self, app):
		super(Widget, self).__init__()
		self.app = app
		self.initUI()

	# initializes the UI
	def initUI(self):
		self.setWindowTitle('registered_features')
		self.setGeometry(300, 300, 200, 100)

		self.layout = QtGui.QVBoxLayout(self)
		self.save_button = QtGui.QPushButton('Save')
		self.load_button = QtGui.QPushButton('Load')
		self.save_button.clicked.connect(self.saveButtonCallback)
		self.load_button.clicked.connect(self.loadButtonCallback)
		self.layout.addWidget(self.save_button)
		self.layout.addWidget(self.load_button)

	# callback for save button pushing event
	def saveButtonCallback(self, e):
		dialog = QtGui.QFileDialog()
		if not dialog.exec_():
			return

		save_filename = dialog.selectedFiles()
		if len(save_filename) == 0:
			return

		save_filename = str(save_filename[0])
		print save_filename

		save_service = rospy.ServiceProxy('/face_recognition/save_registered_faces', OPTSaveRegisteredFaces)
		req = OPTSaveRegisteredFacesRequest()
		req.path = save_filename

		res = save_service(req)
		print res

	# callback for load button pushing event
	def loadButtonCallback(self, e):
		load_filename = QtGui.QFileDialog.getOpenFileName()
		if len(load_filename) == 0:
			return
		load_filename = str(load_filename)
		print load_filename

		load_service = rospy.ServiceProxy('/face_recognition/load_registered_faces', OPTLoadRegisteredFaces)
		req = OPTLoadRegisteredFacesRequest()
		req.path = load_filename

		res = load_service(req)
		print res


# entry point
def main():
	rospy.init_node('save_load_registered_features')
	app = QtGui.QApplication(sys.argv)
	widget = Widget(app)
	widget.show()
	app.exec_()


if __name__ == '__main__':
	main()
