#!/usr/bin/env python
# -*- coding:utf-8 -*-
import numpy as np
import cv2

class Camera:

    def __init__(self):
        self.WIDTH, self.HEIGHT = 640, 480  # 카메라 가로, 세로 크기

        # ====================
        # ROI - array 순서 : [좌하, 좌상, 우상, 우하]

        vertices = np.array([(0, 330), (self.WIDTH // 2 - 160, 220),
                                    (self.WIDTH // 2 + 160, 220), (self.WIDTH, 330)],
                                   dtype=np.int32)


        # Bird's eye View 변환을 위한 src, dst point 설정 (src 좌표에서 dst 좌표로 투시 변환)
        self.points_src = np.float32(list(vertices))
        self.points_dst = np.float32(
            [(100, self.HEIGHT), (100, 0), (self.WIDTH - 100, 0), (self.WIDTH - 100, self.HEIGHT)])

        # 만든 src, dst point 를 이용하여 투시 변환 행렬 생성
        self.transform_matrix = cv2.getPerspectiveTransform(self.points_src, self.points_dst)
        # 원본 영상으로 되돌리기 위한 역변환 행렬
        self.inv_transform_matrix = cv2.getPerspectiveTransform(self.points_dst, self.points_src)

    # ========================================
    # 히스토그램 평활화
    # ========================================
    def histogram_equalization(self, img):
        return cv2.equalizeHist(img)
   
    # ========================================
    # 반사광 제거
    # ========================================
    def glare_removal(self, img, radius=45):
        img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l_channel = img_lab[:, :, 0] # 밝기를 나타내는 채널
        l_channel = self.histogram_equalization(l_channel)
        l_channel = cv2.medianBlur(l_channel, radius) # 메디안 필터링 (물체는 필요없고, 실제 조명과 가까워지게 블러링 강하게 적용)
        inverse_l_channel = cv2.bitwise_not(l_channel) # 빛 반사가 높을수록 어두워지고, 빛반사가 없을수록 밝아짐(not연산자)
        img_lab[:, :, 0] += inverse_l_channel # 원본 L 채널과 합성
        img_lab[:, :, 0] = img_lab[:, :, 0] // 2
        img = cv2.cvtColor(img_lab, cv2.COLOR_LAB2BGR) # BGR로 변경
        return img
    

    # ========================================
    # 윤곽선 검출
    # ========================================
    def canny_edge(self, img):
        img = cv2.Canny(img, 50, 120)
        return img
    
    # ========================================
    # 세로선 검출 (Sobel Edge Detection)
    # ========================================    
    def sobel_edge(self, img):
        img = cv2.Sobel(img, cv2.CV_8U, 1, 0, 3)
        return img


    # ========================================
    # 흑백 영상 변환
    # ========================================
    def gray_scale(self, img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # 그레이 스케일 이미지로 변경하여 이미지 반환


    # ========================================
    # 가우시안 블러링
    # ========================================
    def gaussian_blur(self, img):
        return cv2.GaussianBlur(img, (3, 3), 0)  # 노이즈 제거(솔트 & 페퍼 노이즈) 이미지 반환

    
    # ========================================
    # 모폴로지 닫힘 연산
    # ========================================
    def mophology_close(self, img):
        return cv2.morphologyEx(img, cv2.MORPH_CLOSE, None)


    # ========================================
    # Bird's eye View 변환
    # ========================================
    def perspective_transform(self, img):  # birds eye view
        result_img = cv2.warpPerspective(img, self.transform_matrix, (self.WIDTH, self.HEIGHT))
        return result_img  # 변환한 이미지 반환

    # ========================================
    # 영상 전처리
    # ========================================
    def pre_processing(self, img, mode=1):
        img = self.glare_removal(img)
        img = self.gray_scale(img)
        img = self.gaussian_blur(img)
        img = self.canny_edge(img)
        img = self.perspective_transform(img)

        return img
