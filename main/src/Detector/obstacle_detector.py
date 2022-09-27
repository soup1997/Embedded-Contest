#!/usr/bin/env python
#-*- coding:utf-8 -*-

import numpy as np
import math
from sklearn.cluster import DBSCAN
######################
import rospy
from time import time
from ackermann_msgs.msg import AckermannDriveStamped
from Detector.reprojection import RP
######################

class Clustering:
    def __init__(self):
        ###################
        self.rate = rospy.Rate(30)  # 30Hz로 토픽 발행
        self.motor_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=30)  # motor msg publisher
        self.motor_msg = AckermannDriveStamped()  # xycar 제어를 위한 속도, 조향각 정보를 담고 있는 xycar_motor 호출
        self.motor_msg.drive.speed = 5
        self.motor_msg.drive.steering_angle = 0
        ###################
        epsilon = 0.2   # 중심점 기준 반경 길이 (meter)
        min_sample = 10  # 클러스터로 묶기위한 최소 포인트 갯수, 이보다 작으면 노이즈로 간주

        self.range1_sin = np.sin(np.radians(np.linspace(90, 180, 126)))     # 좌 = 90 ~ 180
        self.range1_cos = np.cos(np.radians(np.linspace(90, 180, 126)))

        self.range2_sin = np.sin(np.radians(np.linspace(180, 270, 126)))    # 우 = 180 ~ 270
        self.range2_cos = np.cos(np.radians(np.linspace(180, 270, 126)))

        self.model = DBSCAN(eps=epsilon, min_samples=min_sample, algorithm='ball_tree', leaf_size=20)
        self.rp = RP()
        #################
        self.steering_angle = 0

        self.count = 0
        self.wait_flag = 0
        
        self.BOTH, self.LEFT, self.RIGHT = 0, 1, 2
        self.roi_setting = self.BOTH

        self.mission_finished = False
        #################

    def coordinate_transform(self, distance):

        # shape: (180, 1)   270도 = 378 (좌 = 90 ~ 180) # 126:252
        coordinate_x1 = np.array([distance[126:252] * self.range1_sin]).reshape(-1, 1)
        coordinate_y1 = np.array([distance[126:252] * self.range1_cos]).reshape(-1, 1)

        # shape: (180, 1)   90도 = 126  (우 = 180 ~ 270)    # 252:378
        coordinate_x2 = np.array([distance[252:378] * self.range2_sin]).reshape(-1, 1)
        coordinate_y2 = np.array([distance[252:378] * self.range2_cos]).reshape(-1, 1)

        # shape: (360, 1)
        coordinate_x = np.concatenate((coordinate_x1, coordinate_x2), axis=0)
        coordinate_y = np.concatenate((coordinate_y1, coordinate_y2), axis=0)

        # shape: (360, 2)
        coordinate = np.concatenate((coordinate_x, coordinate_y), axis=1)

        return coordinate

    def value_handling(self, data):
        # inf값을 NaN으로 대체
        data[np.isposinf(data)] = np.NaN
        data[np.isneginf(data)] = np.NaN
        
        # x좌표가 0.30m 이하인 점의 인덱스를 찾아 그 부분만 살림
        condition = np.where(np.abs(data[:, 1]) <= 0.30)
        data = data[condition, :].reshape(-1, 2)
        return data

    def clustering(self, data):
        centroid_list = []  # 중심점을 저장할 리스트 선언

        # 학습 시작
        # 반경(epsilon)내에 최소 20개의 포인트(min_sample)을 가지고 있을 경우 하나의 군집으로 판단이 된다.
        self.model.fit(data)

        for label in list(np.unique(self.model.labels_)):
            if label == -1: # label이 -1일 경우 Noise 성분
                continue

            label_index = np.where(self.model.labels_ == label)
            cluster = data[label_index]
            centroid = np.mean(cluster, axis=0)
            centroid_list.append(centroid)
        
        print(np.unique(self.model.labels_))
        return centroid_list

    def avoidance(self, centroid_list):

        # 중심점 list에서 euclidean distance를 구함
        distance_list = [np.linalg.norm(centroid) for centroid in centroid_list]
        
        # 가장 거리가 가까운 점이 회피 대상이라 가정, x, y좌표가 반대로 출력되므로 역 인덱싱 적용
        dist = min(distance_list)
        centroid_list = centroid_list[np.argmin(distance_list)][::-1]

        angle = (math.atan(centroid_list[1] / centroid_list[0]) * (180 / math.pi))      

        ### 장애물과의 각도와 반대 방향으로 동일한 크기의 각도 결정
        if angle < 0.0:
            if angle <= -20.0: # 각도 범위 제한
                angle = -20.0
        
        elif angle > 0.0:
            if angle >= 20.0: # 각도 범위 제한
                angle = 20.0
        
        else:
            pass

        return angle, dist

    def process(self, msg, img):
        coordinate = self.coordinate_transform(msg.ranges)
        processed_data = self.value_handling(coordinate)
        centroid_list = self.clustering(processed_data)

        self.rp.main(msg, img)
        self.steering_angle, dist = self.avoidance(centroid_list)

        return self.steering_angle, dist