#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from time import time
from ackermann_msgs.msg import AckermannDriveStamped

from horse_power_sensor import HPSensor
from Detector.lane_detector import LaneDetector
from Detector.obstacle_detector import Clustering
from controller import Stanley

class HP:

    def __init__(self):
        self.rate = rospy.Rate(20)  # 20Hz로 토픽 발행
        self.motor_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=20)
        self.motor_msg = AckermannDriveStamped()  # 제어를 위한 속도, 조향각 정보를 담고 있는 ackermann_cmd 호출
        
        self.sensor = HPSensor()
        self.sensor.init(self.rate)
        
        self.lane_detector = LaneDetector()
        self.obstacle_detector = Clustering()
        self.stanley = Stanley()
        self.start_time = time()

    def calc_speed(self, angle):  # 최저속도(min): 10.0, 최고속도: 50.0(50.0)
        if angle > 0:
            slope = -0.8

        elif angle < 0:
            slope = 0.8

        else:
            slope = 0.0
        
        speed = (slope * angle) + 50.0

        return speed

    def control(self):
        try:
            steering_angle = self.obstacle_detector.process(self.sensor.lidar_filtered, self.sensor.real_cam)
            
            # 장애물이 정면에 있으면 정지
            if abs(steering_angle) < 3:
                steering_angle = 0
                speed = 0

            else:
                steering_angle *= 2.5
                speed = 10
        
        except ValueError:
            curvature_angle = self.lane_detector.process(self.sensor.cam)

            steering_angle = self.stanley.control(self.lane_detector.avg_middle, 320 , 1, curvature_angle)
            steering_angle *= 2.5 # 모터로 보내는 조향각
            speed = self.calc_speed(steering_angle)

        print("Current motor speed: {}, Current motor angle: {}".format(
            self.motor_msg.drive.speed, self.motor_msg.drive.steering_angle))
            
        self.motor_msg.drive.speed = int(speed)
        self.motor_msg.drive.steering_angle = int(steering_angle)

        self.motor_pub.publish(self.motor_msg)
        self.rate.sleep()