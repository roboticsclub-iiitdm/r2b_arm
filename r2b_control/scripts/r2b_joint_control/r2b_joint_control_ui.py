# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'r2b_joint_control.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


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
