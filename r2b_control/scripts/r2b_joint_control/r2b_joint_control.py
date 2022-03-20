#!/usr/bin/python3

from math import degrees
import sys
import signal

from PyQt5 import QtWidgets
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow

import rospy
from std_msgs.msg import Float64, Bool

#########################################
# copied from r2b_joint_control_ui.py
from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_main_window(object):
	def setupUi(self, main_window):
		main_window.setObjectName("main_window")
		main_window.resize(755, 242)
		self.central_widget = QtWidgets.QWidget(main_window)
		self.central_widget.setObjectName("central_widget")
		self.jnt_1_lb = QtWidgets.QLabel(self.central_widget)
		self.jnt_1_lb.setGeometry(QtCore.QRect(20, 20, 161, 21))
		self.jnt_1_lb.setAlignment(QtCore.Qt.AlignCenter)
		self.jnt_1_lb.setObjectName("jnt_1_lb")
		self.jnt_2_lb = QtWidgets.QLabel(self.central_widget)
		self.jnt_2_lb.setGeometry(QtCore.QRect(20, 130, 161, 21))
		self.jnt_2_lb.setAlignment(QtCore.Qt.AlignCenter)
		self.jnt_2_lb.setObjectName("jnt_2_lb")
		self.reset_btn = QtWidgets.QPushButton(self.central_widget)
		self.reset_btn.setGeometry(QtCore.QRect(600, 190, 111, 31))
		self.reset_btn.setObjectName("reset_btn")
		self.jnt_1_slider = QtWidgets.QSlider(self.central_widget)
		self.jnt_1_slider.setGeometry(QtCore.QRect(210, 40, 491, 31))
		self.jnt_1_slider.setMinimum(-157)
		self.jnt_1_slider.setMaximum(157)
		self.jnt_1_slider.setProperty("value", 0)
		self.jnt_1_slider.setSliderPosition(0)
		self.jnt_1_slider.setOrientation(QtCore.Qt.Horizontal)
		self.jnt_1_slider.setTickPosition(QtWidgets.QSlider.TicksAbove)
		self.jnt_1_slider.setTickInterval(10)
		self.jnt_1_slider.setObjectName("jnt_1_slider")
		self.jnt_2_slider = QtWidgets.QSlider(self.central_widget)
		self.jnt_2_slider.setGeometry(QtCore.QRect(210, 150, 491, 31))
		self.jnt_2_slider.setMinimum(-314)
		self.jnt_2_slider.setMaximum(314)
		self.jnt_2_slider.setProperty("value", 0)
		self.jnt_2_slider.setSliderPosition(0)
		self.jnt_2_slider.setOrientation(QtCore.Qt.Horizontal)
		self.jnt_2_slider.setTickPosition(QtWidgets.QSlider.TicksAbove)
		self.jnt_2_slider.setTickInterval(10)
		self.jnt_2_slider.setObjectName("jnt_2_slider")
		self.jnt_1_value = QtWidgets.QLabel(self.central_widget)
		self.jnt_1_value.setGeometry(QtCore.QRect(40, 50, 111, 31))
		font = QtGui.QFont()
		font.setPointSize(13)
		self.jnt_1_value.setFont(font)
		self.jnt_1_value.setFrameShape(QtWidgets.QFrame.Box)
		self.jnt_1_value.setFrameShadow(QtWidgets.QFrame.Plain)
		self.jnt_1_value.setAlignment(QtCore.Qt.AlignCenter)
		self.jnt_1_value.setObjectName("jnt_1_value")
		self.jnt_2_value = QtWidgets.QLabel(self.central_widget)
		self.jnt_2_value.setGeometry(QtCore.QRect(40, 160, 111, 31))
		font = QtGui.QFont()
		font.setPointSize(13)
		self.jnt_2_value.setFont(font)
		self.jnt_2_value.setFrameShape(QtWidgets.QFrame.Box)
		self.jnt_2_value.setFrameShadow(QtWidgets.QFrame.Plain)
		self.jnt_2_value.setAlignment(QtCore.Qt.AlignCenter)
		self.jnt_2_value.setObjectName("jnt_2_value")
		self.attach_btn = QtWidgets.QPushButton(self.central_widget)
		self.attach_btn.setGeometry(QtCore.QRect(400, 190, 181, 31))
		self.attach_btn.setObjectName("attach_btn")
		main_window.setCentralWidget(self.central_widget)

		self.retranslateUi(main_window)
		QtCore.QMetaObject.connectSlotsByName(main_window)

	def retranslateUi(self, main_window):
		_translate = QtCore.QCoreApplication.translate
		main_window.setWindowTitle(_translate("main_window", "R2B Robot Joint Control"))
		self.jnt_1_lb.setText(_translate("main_window", "Joint 1"))
		self.jnt_2_lb.setText(_translate("main_window", "Joint 2"))
		self.reset_btn.setText(_translate("main_window", "Reset"))
		self.jnt_1_value.setText(_translate("main_window", "0.0"))
		self.jnt_2_value.setText(_translate("main_window", "0.0"))
		self.attach_btn.setText(_translate("main_window", "Send attach command"))

#########################################

def sigint_handler(*args):
	rospy.loginfo("Exiting node: r2b_joint_control ...")
	QApplication.quit()


class R2bJointControl(QMainWindow, Ui_main_window):
	def __init__(self, parent=None):
		super().__init__(parent)

		self.attach_rqst_sent = False

		# creating publishers
		self.jnt_1_pub = rospy.Publisher("/r2b/jnt_1_controller/command", Float64, queue_size=1)
		self.jnt_2_pub = rospy.Publisher("/r2b/jnt_2_controller/command", Float64, queue_size=1)
		self.attach_rqst_pub = rospy.Publisher("/r2b/connect_box", Bool, queue_size=1)

		self.setupUi(self)
		self.connect_signals()
	
	def connect_signals(self):
		"""
			Connect signals to methods
		"""
		self.jnt_1_slider.valueChanged.connect(lambda : self.on_slider_value_changed(0))
		self.jnt_2_slider.valueChanged.connect(lambda : self.on_slider_value_changed(1))
		self.reset_btn.clicked.connect(self.on_reset_click)
		self.attach_btn.clicked.connect(self.on_attach_click)

	def on_slider_value_changed(self, slider_id):
		# joint_1
		if slider_id == 0:
			position = self.jnt_1_slider.value() / 100
			self.jnt_1_value.setText(str(round(90 - degrees(position),2)))
			self.jnt_1_pub.publish(position)
		
		# joint_2
		elif slider_id == 1:
			position = self.jnt_2_slider.value() / 100
			self.jnt_2_value.setText(str(round(degrees(position),2)))
			self.jnt_2_pub.publish(position)
	
	def on_reset_click(self):
		self.jnt_1_slider.setValue(0)
		self.jnt_2_slider.setValue(0)

	def on_attach_click(self):
		if(self.attach_rqst_sent):
			self.attach_btn.setText("Send attach command")
			self.attach_rqst_sent = False
		else:
			self.attach_btn.setText("Send detach command")
			self.attach_rqst_sent = True
		self.attach_rqst_pub.publish(self.attach_rqst_sent)

if __name__ == "__main__":
	
	signal.signal(signal.SIGINT, sigint_handler)

	rospy.init_node("r2b_joint_control")
	app = QtWidgets.QApplication(sys.argv)
	win = R2bJointControl()

	# keep on checking for CTRL+C every 100 ms
	timer = QTimer()
	timer.start(100)
	timer.timeout.connect(lambda : None)

	win.show()
	app.exec_()