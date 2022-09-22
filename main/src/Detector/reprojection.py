#!/usr/bin/env python

import cv2
from pyquaternion import Quaternion
import yaml
import numpy as np
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2

class RP:
    def __init__(self):
        
        calib_file = '/home/aeslab/embe_ws/src/camera_2d_lidar_calibration/data/calibration_result.txt'
        config_file = '/home/aeslab/embe_ws/src/camera_2d_lidar_calibration/config/config.yaml'
        self.lp = lg.LaserProjection()

        with open(calib_file, 'r') as f:
            data = f.read().split()
            qx = float(data[0])
            qy = float(data[1])
            qz = float(data[2])
            qw = float(data[3])
            tx = float(data[4])
            ty = float(data[5])
            tz = float(data[6])
        self.q = Quaternion(qw,qx,qy,qz).transformation_matrix
        self.q[0,3] = tx
        self.q[1,3] = ty
        self.q[2,3] = tz
        print("Extrinsic parameter - camera to laser")
        print(self.q)
        self.tvec = self.q[:3,3]
        rot_mat = self.q[:3,:3]
        self.rvec, _ = cv2.Rodrigues(rot_mat)

        with open(config_file, 'r') as f:
            f.readline()
            config = yaml.load(f)
            lens = config['lens']
            fx = float(config['fx'])
            fy = float(config['fy'])
            cx = float(config['cx'])
            cy = float(config['cy'])
            k1 = float(config['k1'])
            k2 = float(config['k2'])
            p1 = float(config['p1/k3'])
            p2 = float(config['p2/k4'])  
        self.K = np.matrix([[fx, 0.0, cx],
                    [0.0, fy, cy],
                    [0.0, 0.0, 1.0]])
        self.D = np.array([k1, k2, p1, p2])
        print("Camera parameters")
        print("Lens = %s" % lens)
        print("K =")
        print(self.K)
        print("D =")
        print(self.D)

    def get_z(self, T_cam_world, T_world_pc, K):
        R = T_cam_world[:3,:3]
        t = T_cam_world[:3,3]
        proj_mat = np.dot(K, np.hstack((R, t[:,np.newaxis])))
        xyz_hom = np.hstack((T_world_pc, np.ones((T_world_pc.shape[0], 1))))
        xy_hom = np.dot(proj_mat, xyz_hom.T).T
        z = xy_hom[:, -1]
        z = np.asarray(z).squeeze()
        return z

    def extract(self, point):
        return [point[0], point[1], point[2]]

    def main(self, scan, img):
        cloud = self.lp.projectLaser(scan)
        points = pc2.read_points(cloud)

        objPoints = np.array(map(self.extract, points))
        Z = self.get_z(self.q, objPoints, self.K)

        objPoints = objPoints[Z > 0]
        objPoints = np.reshape(objPoints, (1,objPoints.shape[0],objPoints.shape[1]))
        img_points, _ = cv2.fisheye.projectPoints(objPoints, self.rvec, self.tvec, self.K, self.D)
        img_points = np.squeeze(img_points)
        
        for i in range(len(img_points)):
            try:
                cv2.circle(img, (int(round(img_points[i][0])),int(round(img_points[i][1]))), 3, (0,255,0), 1)
            except OverflowError:
                continue

        cv2.imshow('Obstacle', img)