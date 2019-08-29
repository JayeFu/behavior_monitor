#!/usr/bin/env python

import rospy
import cv2
import rospkg
import os

from PyQt5 import QtCore, QtGui, QtWidgets

from behavior_monitor.TempWindow import Ui_TempWindow
from behavior_monitor.subContainer import subContainer

class MonitorWindow(Ui_TempWindow):
    def __init__(self):
        super(MonitorWindow, self).__init__()
        self._robot_prefix = "robot"
        self.subCon = subContainer()
        rp = rospkg.RosPack()
        # img_path = os.path.join(rp.get_path('behavior_monitor'), "media", "simple_field.png")
        img_path = os.path.join(rp.get_path('behavior_monitor'), "media", "simple_field.png")
        self.raw_field = cv2.imread(img_path)
        self.marked_field = None
        self.FieldScene = None
        self.FieldItem = None

    def setupSlots(self, MainWindow):
        self.setupConnections()
        self.setupCameras()
        self._timer = QtCore.QTimer(MainWindow)
        self._timer.timeout.connect(self.refresh)
        self._timer.start(500)

    def setupConnections(self):
        self.Robot1_Con.clicked.connect(lambda: self.connect_robot(self._robot_prefix+"1"))
        self.Robot2_Con.clicked.connect(lambda: self.connect_robot(self._robot_prefix+"2"))
        self.Robot3_Con.clicked.connect(lambda: self.connect_robot(self._robot_prefix+"3"))
        self.Robot4_Con.clicked.connect(lambda: self.connect_robot(self._robot_prefix+"4"))

    def setupCameras(self):
        self.Robot1_Cam.clicked.connect(lambda: self.show_camera(self._robot_prefix+"1"))
        self.Robot2_Cam.clicked.connect(lambda: self.show_camera(self._robot_prefix+"2"))
        self.Robot3_Cam.clicked.connect(lambda: self.show_camera(self._robot_prefix+"3"))
        self.Robot4_Cam.clicked.connect(lambda: self.show_camera(self._robot_prefix+"4"))

    def refresh(self):
        # TODO: refresh camera and field
        for rname, rpos in self.subCon.pos.iteritems():
            rospy.loginfo("IN REFRESH: {}'s pos is x:{}, y:{}".format(rname, rpos['x'], rpos['y']))
        for rbname, rbpos in self.subCon.ballpos.iteritems():
            rospy.loginfo("IN REFRESH: {}'s BALL pos is x:{}, y:{}".format(rbname, rbpos['x'], rbpos['y']))
        self.field_refresh()

    def connect_robot(self, name):
        # [x] subscribe to some topics
        # default topic is /name/pos, type:String(JSON encoding data)
        print "connect to "+name
        self.subCon.start_sub(name)
        

    def show_camera(self, name):
        # TODO: subscribe to some topics
        print "show image of "+name

    def field_refresh(self):
        self.marked_field = self.raw_field.copy()
        robots_pos = self.subCon.get_pos()
        balls_pos = self.subCon.get_ballpos()
        self.draw_robots(self.marked_field, robots_pos)
        self.draw_balls(self.marked_field, balls_pos)
        self.draw_lines(self.marked_field, robots_pos, balls_pos)
        self.marked_field = cv2.cvtColor(self.marked_field, cv2.COLOR_BGR2RGB)
        x = self.marked_field.shape[1]
        y = self.marked_field.shape[0]
        frame = QtGui.QImage(self.marked_field, x, y, QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(frame)
        self.FieldItem = QtWidgets.QGraphicsPixmapItem(pix)
        self.FieldScene = QtWidgets.QGraphicsScene()
        self.FieldScene.addItem(self.FieldItem)
        self.FieldView.setScene(self.FieldScene)
        self.FieldView.fitInView(self.FieldItem)
    
    def draw_robots(self, field, pos):
        robot_radius = 20
        robot_color = (0, 0, 255)
        robot_width = 4
        for rname, rpos in pos.iteritems():
            cv2.circle(field, self.map_origin(rpos), robot_radius, robot_color, robot_width)

    def draw_balls(self, field, pos):
        ball_radius = 10
        ball_color = (255, 0, 0)
        ball_width = 2
        for bname, bpos in pos.iteritems():
            cv2.circle(field, self.map_origin(bpos), ball_radius, ball_color, ball_width)

    def draw_lines(self, field, robots_pos, balls_pos):
        line_color = (255, 255, 255)
        line_width = 2
        for rname, rpos in robots_pos.iteritems():
            bpos = balls_pos[rname]
            cv2.line(field, self.map_origin(rpos), self.map_origin(bpos), line_color, line_width)


    def map_origin(self, center):
        # this function maps the coordinate from center to upper left coner
        # center_x, center_y in meters
        center_x = center['x']
        center_y = center['y']
        center_x = center_x * 100
        center_y = center_y * 100
        corner_x = int(450 + center_y)
        corner_y = int(300 - center_x)
        return (corner_x, corner_y)
