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
        # 주행 시작하자마자 조향틀어지는 문제 해결 (2.0초 동안은 speed=10, angle = 0으로 강제 명령)
        if time() - self.start_time <= 2.0:
            speed = 0
            steering_angle = 0
        speed = (slope * angle) + 50.0

        return speed

    def control(self):
        try:
            steering_angle = self.obstacle_detector.process(self.sensor.lidar_filtered, self.sensor.real_cam)
            left_lane_detected = False
            right_lane_detected = False
        
        except ValueError:
            curvature_angle, left_lane_detected, right_lane_detected = self.lane_detector.process(self.sensor.cam)

            steering_angle = self.stanley.control(self.lane_detector.avg_middle, 320 , 1, curvature_angle)

        steering_angle *= 2.5 # 모터로 보내는 조향각
        speed = self.calc_speed(steering_angle)
        
        # 양쪽 차선 하나라도 검출 안되면 속도 5로 고정
        if not(left_lane_detected) or not(right_lane_detected):
            self.motor_msg.drive.speed = 10

        print("Current motor speed: {}, Current motor angle: {}".format(
            self.motor_msg.drive.speed, self.motor_msg.drive.steering_angle))
            
        self.motor_msg.drive.speed = int(speed)
        self.motor_msg.drive.steering_angle = int(steering_angle)

        self.motor_pub.publish(self.motor_msg)
        self.rate.sleep()