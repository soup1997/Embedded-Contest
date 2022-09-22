#!/usr/bin/env python
# -*- coding:utf-8 -*-

from tkinter import N
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan

class HPSensor:

    def __init__(self):

        # video
        self.cam = None
        self.bridge = CvBridge()
        rospy.Subscriber("/usb_cam/image_raw", Image, self.callback_cam)

        # camera
        self.real_cam = None
        rospy.Subscriber("/embe/usb_cam/image_raw", Image, self.callback_real_cam)

        # lidar filtered
        self.lidar_filtered = None
        rospy.Subscriber("scan_filtered", LaserScan, self.callback_lidar_filtered)

    def callback_real_cam(self, msg):
        self.real_cam = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def callback_cam(self, msg):
        self.cam = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def callback_lidar_filtered(self, msg):
        self.lidar_filtered = msg

    def init(self, rate):
        while self.cam is None:
            rate.sleep()
        rospy.loginfo("video ready")

        while self.real_cam is None:
            rate.sleep()
        rospy.loginfo("usb_cam ready")

        while self.lidar_filtered is None:
            rate.sleep()
        rospy.loginfo("filtered lidar ready")