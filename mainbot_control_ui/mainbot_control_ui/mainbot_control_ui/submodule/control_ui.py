# Form implementation generated from reading ui file 'controlui.ui'
#
# Created by: PyQt6 UI code generator 6.5.1
#
# WARNING: Any manual changes made to this file will be lost when pyuic6 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt6 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(980, 833)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Form.sizePolicy().hasHeightForWidth())
        Form.setSizePolicy(sizePolicy)
        Form.setMinimumSize(QtCore.QSize(50, 0))
        self.gridLayout_3 = QtWidgets.QGridLayout(Form)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setSizeConstraint(QtWidgets.QLayout.SizeConstraint.SetMinimumSize)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.BL_angVel = QtWidgets.QLabel(parent=Form)
        self.BL_angVel.setObjectName("BL_angVel")
        self.gridLayout.addWidget(self.BL_angVel, 4, 0, 1, 1)
        self.BR_angVel = QtWidgets.QLabel(parent=Form)
        self.BR_angVel.setObjectName("BR_angVel")
        self.gridLayout.addWidget(self.BR_angVel, 4, 2, 1, 1)
        self.mecanum = QtWidgets.QGraphicsView(parent=Form)
        self.mecanum.setMinimumSize(QtCore.QSize(190, 217))
        self.mecanum.setMaximumSize(QtCore.QSize(190, 217))
        self.mecanum.setObjectName("mecanum")
        self.gridLayout.addWidget(self.mecanum, 0, 1, 5, 1)
        self.label_3 = QtWidgets.QLabel(parent=Form)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 3, 0, 1, 1)
        self.label_4 = QtWidgets.QLabel(parent=Form)
        self.label_4.setObjectName("label_4")
        self.gridLayout.addWidget(self.label_4, 3, 2, 1, 1)
        self.label = QtWidgets.QLabel(parent=Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        self.label.setMinimumSize(QtCore.QSize(77, 0))
        self.label.setMaximumSize(QtCore.QSize(77, 16777215))
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)
        self.FL_angVel = QtWidgets.QLabel(parent=Form)
        self.FL_angVel.setObjectName("FL_angVel")
        self.gridLayout.addWidget(self.FL_angVel, 1, 0, 1, 1)
        self.label_5 = QtWidgets.QLabel(parent=Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_5.sizePolicy().hasHeightForWidth())
        self.label_5.setSizePolicy(sizePolicy)
        self.label_5.setMinimumSize(QtCore.QSize(77, 0))
        self.label_5.setMaximumSize(QtCore.QSize(77, 16777215))
        self.label_5.setObjectName("label_5")
        self.gridLayout.addWidget(self.label_5, 0, 2, 1, 1)
        self.FR_angVel = QtWidgets.QLabel(parent=Form)
        self.FR_angVel.setObjectName("FR_angVel")
        self.gridLayout.addWidget(self.FR_angVel, 1, 2, 1, 1)
        spacerItem = QtWidgets.QSpacerItem(20, 125, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Fixed)
        self.gridLayout.addItem(spacerItem, 2, 2, 1, 1)
        spacerItem1 = QtWidgets.QSpacerItem(20, 125, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Fixed)
        self.gridLayout.addItem(spacerItem1, 2, 0, 1, 1)
        self.verticalLayout_2.addLayout(self.gridLayout)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        spacerItem2 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
        self.verticalLayout.addItem(spacerItem2)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label_2 = QtWidgets.QLabel(parent=Form)
        self.label_2.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.verticalLayout_3.addWidget(self.label_2)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem3)
        self.label_8 = QtWidgets.QLabel(parent=Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_8.sizePolicy().hasHeightForWidth())
        self.label_8.setSizePolicy(sizePolicy)
        self.label_8.setMinimumSize(QtCore.QSize(50, 0))
        self.label_8.setLayoutDirection(QtCore.Qt.LayoutDirection.RightToLeft)
        self.label_8.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight|QtCore.Qt.AlignmentFlag.AlignTrailing|QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.label_8.setObjectName("label_8")
        self.horizontalLayout_4.addWidget(self.label_8)
        self.vel_veiw = QtWidgets.QGraphicsView(parent=Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.vel_veiw.sizePolicy().hasHeightForWidth())
        self.vel_veiw.setSizePolicy(sizePolicy)
        self.vel_veiw.setMinimumSize(QtCore.QSize(154, 154))
        self.vel_veiw.setMaximumSize(QtCore.QSize(154, 154))
        self.vel_veiw.setBaseSize(QtCore.QSize(230, 230))
        self.vel_veiw.setContextMenuPolicy(QtCore.Qt.ContextMenuPolicy.PreventContextMenu)
        self.vel_veiw.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        self.vel_veiw.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.vel_veiw.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.vel_veiw.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.vel_veiw.setObjectName("vel_veiw")
        self.horizontalLayout_4.addWidget(self.vel_veiw)
        self.label_7 = QtWidgets.QLabel(parent=Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_7.sizePolicy().hasHeightForWidth())
        self.label_7.setSizePolicy(sizePolicy)
        self.label_7.setMinimumSize(QtCore.QSize(50, 0))
        self.label_7.setObjectName("label_7")
        self.horizontalLayout_4.addWidget(self.label_7)
        spacerItem4 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem4)
        self.verticalLayout_3.addLayout(self.horizontalLayout_4)
        self.label_6 = QtWidgets.QLabel(parent=Form)
        self.label_6.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_6.setObjectName("label_6")
        self.verticalLayout_3.addWidget(self.label_6)
        spacerItem5 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
        self.verticalLayout_3.addItem(spacerItem5)
        self.verticalLayout.addLayout(self.verticalLayout_3)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setSizeConstraint(QtWidgets.QLayout.SizeConstraint.SetFixedSize)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.Vx_fix_label = QtWidgets.QLabel(parent=Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Vx_fix_label.sizePolicy().hasHeightForWidth())
        self.Vx_fix_label.setSizePolicy(sizePolicy)
        self.Vx_fix_label.setObjectName("Vx_fix_label")
        self.horizontalLayout.addWidget(self.Vx_fix_label)
        self.Vx_horizontalSlider = QtWidgets.QSlider(parent=Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Vx_horizontalSlider.sizePolicy().hasHeightForWidth())
        self.Vx_horizontalSlider.setSizePolicy(sizePolicy)
        self.Vx_horizontalSlider.setMinimumSize(QtCore.QSize(235, 0))
        self.Vx_horizontalSlider.setStyleSheet("QSlider {\n"
"            \"background-color: #FF3B3B3B;\"\n"
"            \"color: white;\"\n"
"            \"border : 0.1em;\"\n"
"            \"height:  1.5em;\"\n"
"        }")
        self.Vx_horizontalSlider.setMinimum(-23)
        self.Vx_horizontalSlider.setMaximum(23)
        self.Vx_horizontalSlider.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.Vx_horizontalSlider.setTickPosition(QtWidgets.QSlider.TickPosition.NoTicks)
        self.Vx_horizontalSlider.setObjectName("Vx_horizontalSlider")
        self.horizontalLayout.addWidget(self.Vx_horizontalSlider)
        self.Vx_value = QtWidgets.QLabel(parent=Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Vx_value.sizePolicy().hasHeightForWidth())
        self.Vx_value.setSizePolicy(sizePolicy)
        self.Vx_value.setObjectName("Vx_value")
        self.horizontalLayout.addWidget(self.Vx_value)
        self.Vx_doubleSpinBox = QtWidgets.QDoubleSpinBox(parent=Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Vx_doubleSpinBox.sizePolicy().hasHeightForWidth())
        self.Vx_doubleSpinBox.setSizePolicy(sizePolicy)
        self.Vx_doubleSpinBox.setMinimum(-0.23)
        self.Vx_doubleSpinBox.setMaximum(0.23)
        self.Vx_doubleSpinBox.setSingleStep(0.01)
        self.Vx_doubleSpinBox.setObjectName("Vx_doubleSpinBox")
        self.horizontalLayout.addWidget(self.Vx_doubleSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setSizeConstraint(QtWidgets.QLayout.SizeConstraint.SetFixedSize)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.Vy_fix_label = QtWidgets.QLabel(parent=Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Vy_fix_label.sizePolicy().hasHeightForWidth())
        self.Vy_fix_label.setSizePolicy(sizePolicy)
        self.Vy_fix_label.setObjectName("Vy_fix_label")
        self.horizontalLayout_2.addWidget(self.Vy_fix_label)
        self.Vy_horizontalSlider = QtWidgets.QSlider(parent=Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Vy_horizontalSlider.sizePolicy().hasHeightForWidth())
        self.Vy_horizontalSlider.setSizePolicy(sizePolicy)
        self.Vy_horizontalSlider.setMinimumSize(QtCore.QSize(235, 0))
        self.Vy_horizontalSlider.setMinimum(-23)
        self.Vy_horizontalSlider.setMaximum(23)
        self.Vy_horizontalSlider.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.Vy_horizontalSlider.setObjectName("Vy_horizontalSlider")
        self.horizontalLayout_2.addWidget(self.Vy_horizontalSlider)
        self.Vy_value = QtWidgets.QLabel(parent=Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Vy_value.sizePolicy().hasHeightForWidth())
        self.Vy_value.setSizePolicy(sizePolicy)
        self.Vy_value.setObjectName("Vy_value")
        self.horizontalLayout_2.addWidget(self.Vy_value)
        self.Vy_doubleSpinBox = QtWidgets.QDoubleSpinBox(parent=Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Vy_doubleSpinBox.sizePolicy().hasHeightForWidth())
        self.Vy_doubleSpinBox.setSizePolicy(sizePolicy)
        self.Vy_doubleSpinBox.setMinimum(-0.23)
        self.Vy_doubleSpinBox.setMaximum(0.23)
        self.Vy_doubleSpinBox.setSingleStep(0.01)
        self.Vy_doubleSpinBox.setObjectName("Vy_doubleSpinBox")
        self.horizontalLayout_2.addWidget(self.Vy_doubleSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setSizeConstraint(QtWidgets.QLayout.SizeConstraint.SetFixedSize)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.W_fix_label = QtWidgets.QLabel(parent=Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.W_fix_label.sizePolicy().hasHeightForWidth())
        self.W_fix_label.setSizePolicy(sizePolicy)
        self.W_fix_label.setMinimumSize(QtCore.QSize(19, 0))
        self.W_fix_label.setObjectName("W_fix_label")
        self.horizontalLayout_3.addWidget(self.W_fix_label)
        self.W_horizontalSlider = QtWidgets.QSlider(parent=Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.W_horizontalSlider.sizePolicy().hasHeightForWidth())
        self.W_horizontalSlider.setSizePolicy(sizePolicy)
        self.W_horizontalSlider.setMinimumSize(QtCore.QSize(235, 0))
        self.W_horizontalSlider.setMinimum(-23)
        self.W_horizontalSlider.setMaximum(23)
        self.W_horizontalSlider.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.W_horizontalSlider.setObjectName("W_horizontalSlider")
        self.horizontalLayout_3.addWidget(self.W_horizontalSlider)
        self.W_value = QtWidgets.QLabel(parent=Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.W_value.sizePolicy().hasHeightForWidth())
        self.W_value.setSizePolicy(sizePolicy)
        self.W_value.setObjectName("W_value")
        self.horizontalLayout_3.addWidget(self.W_value)
        self.W_doubleSpinBox = QtWidgets.QDoubleSpinBox(parent=Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.W_doubleSpinBox.sizePolicy().hasHeightForWidth())
        self.W_doubleSpinBox.setSizePolicy(sizePolicy)
        self.W_doubleSpinBox.setMinimum(-0.35)
        self.W_doubleSpinBox.setMaximum(0.35)
        self.W_doubleSpinBox.setSingleStep(0.01)
        self.W_doubleSpinBox.setObjectName("W_doubleSpinBox")
        self.horizontalLayout_3.addWidget(self.W_doubleSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.vel_rst_btn = QtWidgets.QPushButton(parent=Form)
        self.vel_rst_btn.setObjectName("vel_rst_btn")
        self.verticalLayout.addWidget(self.vel_rst_btn)
        self.verticalLayout_2.addLayout(self.verticalLayout)
        self.horizontalLayout_5.addLayout(self.verticalLayout_2)
        self.gridLayout_3.addLayout(self.horizontalLayout_5, 1, 0, 1, 1)
        spacerItem6 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Minimum)
        self.gridLayout_3.addItem(spacerItem6, 1, 1, 1, 1)
        self.gridLayout_2 = QtWidgets.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.tabWidget = QtWidgets.QTabWidget(parent=Form)
        self.tabWidget.setObjectName("tabWidget")
        self.tab_1 = QtWidgets.QWidget()
        self.tab_1.setObjectName("tab_1")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.tab_1)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.mappingend_btn = QtWidgets.QPushButton(parent=self.tab_1)
        self.mappingend_btn.setObjectName("mappingend_btn")
        self.gridLayout_4.addWidget(self.mappingend_btn, 2, 0, 1, 1)
        self.map = QtWidgets.QGraphicsView(parent=self.tab_1)
        self.map.setMinimumSize(QtCore.QSize(0, 0))
        self.map.setObjectName("map")
        self.gridLayout_4.addWidget(self.map, 0, 0, 1, 2)
        self.mappingstart_btn = QtWidgets.QPushButton(parent=self.tab_1)
        self.mappingstart_btn.setObjectName("mappingstart_btn")
        self.gridLayout_4.addWidget(self.mappingstart_btn, 1, 0, 1, 1)
        self.laser_scan_view_btn = QtWidgets.QPushButton(parent=self.tab_1)
        self.laser_scan_view_btn.setObjectName("laser_scan_view_btn")
        self.gridLayout_4.addWidget(self.laser_scan_view_btn, 1, 1, 1, 1)
        self.tabWidget.addTab(self.tab_1, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.tabWidget.addTab(self.tab_2, "")
        self.tab_3 = QtWidgets.QWidget()
        self.tab_3.setObjectName("tab_3")
        self.tabWidget.addTab(self.tab_3, "")
        self.gridLayout_2.addWidget(self.tabWidget, 0, 0, 1, 1)
        self.gridLayout_3.addLayout(self.gridLayout_2, 1, 2, 1, 1)

        self.retranslateUi(Form)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.BL_angVel.setText(_translate("Form", "TextLabel"))
        self.BR_angVel.setText(_translate("Form", "TextLabel"))
        self.label_3.setText(_translate("Form", "Back Left"))
        self.label_4.setText(_translate("Form", "Back Rigth"))
        self.label.setText(_translate("Form", "Front Left"))
        self.FL_angVel.setText(_translate("Form", "TextLabel"))
        self.label_5.setText(_translate("Form", "Front Right"))
        self.FR_angVel.setText(_translate("Form", "TextLabel"))
        self.label_2.setText(_translate("Form", "Foward"))
        self.label_8.setText(_translate("Form", "Left"))
        self.label_7.setText(_translate("Form", "Right"))
        self.label_6.setText(_translate("Form", "Backward"))
        self.Vx_fix_label.setText(_translate("Form", "Vx"))
        self.Vx_value.setText(_translate("Form", "0.00"))
        self.Vy_fix_label.setText(_translate("Form", "Vy"))
        self.Vy_value.setText(_translate("Form", "0.00"))
        self.W_fix_label.setText(_translate("Form", "W"))
        self.W_value.setText(_translate("Form", "0.00"))
        self.vel_rst_btn.setText(_translate("Form", "velocity rest"))
        self.mappingend_btn.setText(_translate("Form", "mapping end"))
        self.mappingstart_btn.setText(_translate("Form", "mapping start"))
        self.laser_scan_view_btn.setText(_translate("Form", "laser scan view"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_1), _translate("Form", "map"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("Form", "Imu graph"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_3), _translate("Form", "tab"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec())
