#!/usr/bin/env python
# -*- coding:utf-8 -*-


import numpy as np
import cv2
from camera import Camera
import math


# =============================================
# 주행을 위한 알고리즘 클래스 정의
# =============================================
class LaneDetector:

    # ========================================
    # 변수 선언 및 초기화
    # ========================================
    def __init__(self):

        self.camera = Camera()
        # 슬라이딩 윈도우 출력 창 크기 좌우 확장 값으로, 좌우로 window_margin 만큼 커짐
        # 슬라이딩 윈도우 출력 창 가로 크기 : WIDTH + 2*window_margin
        self.window_margin = 24
        self.car_length = 2.5  # 0.35 meter
        self.leftx_mid, self.rightx_mid = self.camera.WIDTH // 6, self.camera.WIDTH * 5 // 6  # 슬라이딩 윈도우 기준점 초기 좌표
        self.leftx_base, self.rightx_base = self.leftx_mid, self.rightx_mid  # 슬라이딩 윈도우 이전값

        self.left_a, self.left_b, self.left_c = [0], [0], [self.leftx_mid]  # 왼쪽 차선으로부터 나온 2차 곡선 방정식의 계수를 저장하기 위한 변수
        self.right_a, self.right_b, self.right_c = [0], [0], [self.rightx_mid]
        # 오른쪽 차선으로부터 나온 2차 곡선 방정식의 계수를 저장하기 위한 변수
        # 처음 차선이 인식되지 않는 경우를 대비하여, 초기값은 슬라이딩 윈도우 기준점으로부터 직진으로 방정식을 그리도록 함
        self.leftx_current, self.rightx_current = [self.leftx_mid], [self.rightx_mid]
        self.lefty_current, self.righty_current = [480], [480]

        # 양쪽 차선 곡선 좌표 생성
        ### Linear y 값 생성 (0, 1, 2, ..., 479)
        self.ploty = np.linspace(0, self.camera.HEIGHT - 1, self.camera.HEIGHT)
        self.wins_y = np.linspace(464, 16, 15)

        # 양쪽 차선 인식 기준 x값 평균을 저장하는 변수, 이전 조향각을 저장하기위한 변수
        self.avg_middle, self.steering_memory = 0.0, 0.0


    # ========================================
    # 슬라이딩 윈도우
    # ====================
    # < input >
    # img : 입력 이미지
    # nwindows : 조사창 개수 (좌우 각각 nwindows개씩)
    # margin : 현재 기준 위치로부터 조사창의 좌우 길이 (-margin ~ +margin)
    #          조사창 가로 길이 : margin*2
    # minpix : 조사창 내부에서 차선이 검출된 것으로 판단할 최소 픽셀 개수
    # draw_windows : 결과창 출력 여부
    #
    # ====================
    # < return >
    # out_img : 출력 이미지
    # window_img : 슬라이딩 윈도우 출력을 위한 이미지 (너비 확장)
    # left_fitx : 왼쪽 차선 곡선 방정식 x 좌표
    # right_fitx : 오른쪽 차선 곡선 방정식 x 좌표
    # left_lane_detected : 왼쪽 차선 인식 여부
    # right_lane_detected : 오른쪽 차선 인식 여부
    #
    # ========================================
    def sliding_window(self, img, nwindows=5, margin=45, minpix=45, draw_windows=False):
        # 크기 3의 비어있는 배열 생성
        left_fit_ = np.empty(3)
        right_fit_ = np.empty(3)

        # 0과 1로 이진화된 영상을 3채널의 영상으로 만들기 위해 3개를 쌓은 후 *255
        out_img = np.dstack((img, img, img)) * 255
        
        # 너비의 중앙값
        midpoint = self.camera.WIDTH // 2

        # 조사창의 높이 설정
        # 전체 높이에서 설정한 조사창 개수만큼 나눈 값
        window_height = self.camera.HEIGHT // nwindows

        # 0이 아닌 픽셀의 x,y 좌표 반환 (흰색 영역(차선)으로 인식할 수 있는 좌표)
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # 이전 프레임으로부터 기준점의 좌표를 받아옴
        # 양쪽 차선 확인 위치 업데이트
        # 이 값을 기준으로 조사창을 생성하여 차선 확인
        leftx_current = self.leftx_base
        rightx_current = self.rightx_base

        # 차선 확인 시 검출이 되지 않는 경우를 대비해서 바로 이전(화면상 바로 아래) 조사창으로부터 정보를 얻기 위한 변수
        # 이를 이용해서 조사창 위치의 변화량을 파악
        # 초기값으로는 현재 기준 좌표를 넣어줌
        leftx_past = leftx_current
        rightx_past = rightx_current
        rightx_past2 = rightx_past

        # 양쪽 차선 픽셀 인덱스를 담기 위한 빈 배열 선언
        # 차선의 방정식을 구하거나 차선 인식 판단 여부에 사용
        left_lane_inds = []
        right_lane_inds = []

        # 슬라이딩 윈도우 좌표 값을 담기 위한 빈 배열
        left_wins_x = []
        right_wins_x = []

        # 설정한 조사창 개수만큼 슬라이딩 윈도우 생성
        for window in range(nwindows):
            # 조사창 크기 및 위치 설정
            win_y_low = self.camera.HEIGHT - ((window + 1) * window_height)
            # n번째 조사창 윗변 y 좌표 : (전체 높이) - (n * 조사창 높이)
            win_y_high = self.camera.HEIGHT - (window * window_height)
            # 양쪽 차선의 조사창의 너비를 현재 좌표로부터 margin만큼 양 옆으로 키움
            win_xleft_low = int(leftx_current - margin)
            win_xleft_high = int(leftx_current + margin)
            win_xright_low = int(rightx_current - margin)
            win_xright_high = int(rightx_current + margin)

            # 조사창 그리기
            if draw_windows == True:
                # 왼쪽 차선
                cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (100, 100, 255), 3)
                # 오른쪽 차선
                cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (100, 100, 255), 3)

            # 조사창 내부에서 0이 아닌 픽셀의 인덱스 저장
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high)
                              & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high)
                               & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
 
            # 양쪽 차선의 인덱스 저장
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            # 조사창 내부에서 0이 아닌 픽셀 개수가 기준치를 넘으면 해당 픽셀들의 인덱스 평균값(x좌표 평균)으로 다음 조사창의 위치(x좌표)를 결정
            if len(good_left_inds) > minpix:
                leftx = nonzerox[good_left_inds]
                leftx_current = int(np.mean(leftx))
            
            if len(good_right_inds) > minpix:
                rightx = nonzerox[good_right_inds]
                rightx_current = int(np.mean(rightx))
                

            x_diff = rightx_current - leftx_current

            # 양쪽 차선 중 하나만 인식된 경우 반대편 차선에서 나타난 인덱스 변화량과 동일하게 인덱스 설정
            # 인식된 차선의 방향과 동일하게 그려짐
            if len(good_left_inds) < minpix:
                if len(good_right_inds) < minpix:
                    leftx_current = leftx_current + (rightx_past - rightx_past2)
                else:
                    if x_diff < self.camera.WIDTH // 2:
                        leftx_current = rightx_current - (self.camera.WIDTH // 2)
                    else:
                        leftx_current = leftx_current + (rightx_current - rightx_past)
                        
            elif len(good_right_inds) < minpix:
                if x_diff < self.camera.WIDTH // 2:
                    rightx_current = leftx_current + (self.camera.WIDTH // 2)
                else:
                    rightx_current = rightx_current + (leftx_current - leftx_past)

            # 가장 하단에 있는 첫번째 조사창에서 결정된 두번째 조사창의 좌표를 다음 프레임의 기준점으로 결정
            # 기준점의 위치가 고정되어 변화되는 차선을 따라가지 못하는 것을 방지하고,
            # 차선이 끊기거나 여러 개의 선이 나타날 때 큰 변화 없이 현재 인식중인 차선의 방향대로 따라가며 효과적인 차선 인식이 가능
                # 왼쪽 차선의 기준점이 중앙 기준 우측으로 넘어가지 않도록 제한
            if leftx_current > midpoint:
                leftx_current = midpoint

            # 오른쪽 차선의 기준점이 중앙 기준 좌측으로 넘어가지 않도록 제한
            if rightx_current < midpoint:
                rightx_current = midpoint

            if window == 0:
                # 왼쪽 차선의 기준점이 왼쪽 화면 밖으로 나가지 않도록 제한
                if leftx_current < 10:
                    leftx_current = 10

                # 오른쪽 차선의 기준점이 오른쪽 화면 밖으로 나가지 않도록 제한
                if rightx_current > self.camera.WIDTH - 10:
                    rightx_current = self.camera.WIDTH - 10

            # 두번째 조사창의 현재 좌표를 다음 프레임의 기준점으로 설정
                self.leftx_base = leftx_current                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
                self.rightx_base = rightx_current
            

            # 슬라이딩 윈도우 중앙 좌표 값 저장
            left_wins_x.append(leftx_current)
            right_wins_x.append(rightx_current)

            # 현재 인덱스 값을 이전 값으로 저장
            # 한쪽 차선이 인식되지 않은 경우 인식된 차선을 따라가기 위해 사용되는 변수
            leftx_past = leftx_current
            rightx_past2 = rightx_past
            rightx_past = rightx_current

        # 배열 연결
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # 양쪽 차선 픽셀 추출
        ### 0이 아닌 픽셀 중에서 왼쪽 차선으로 인식된 좌표만 가져옴
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        ### 0이 아닌 픽셀 중에서 오른쪽 차선으로 인식된 좌표만 가져옴
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]


        # 차선으로 인식된 픽셀 수가 일정치 이상일 경우에만 차선이 인식된 것으로 판단
        ### 왼쪽 차선으로 인식된 좌표가 1000개 미만이라면 False, 이상이라면 True
        if (leftx.size < 800):
            left_lane_detected = False
        else:
            left_lane_detected = True
        ### 오른쪽 차선으로 인식된 좌표가 1000개 미만이라면 False, 이상이라면 True
        if (rightx.size < 800):
            right_lane_detected = False
        else:
            right_lane_detected = True

        # 차선이 인식된 것으로 판단되었다면 검출된 좌표로부터 차선의 2차 곡선을 구함
        # 왼쪽 차선이 인식된 경우
        if left_lane_detected:
            # 검출된 차선 좌표들을 통해 왼쪽 차선의 2차 방정식 계수를 구함
            left_fit = np.polyfit(lefty, leftx, 2)

            # 왼쪽 차선 계수
            self.left_a.append(left_fit[0])
            self.left_c.append(left_fit[2])

        # 오른쪽 차선이 인식된 경우
        if right_lane_detected:
            # 검출된 차선 좌표들을 통해 오른쪽 차선의 2차 방정식 계수를 구함
            right_fit = np.polyfit(righty, rightx, 2)

            # 오른쪽 차선 계수
            self.right_a.append(right_fit[0])
            self.right_b.append(right_fit[1])
            self.right_c.append(right_fit[2])

        if draw_windows:
            # 차선으로 검출된 픽셀 값 변경
            # 왼쪽 차선은 파란색, 오른쪽 차선은 빨간색으로 표시
            out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
            out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

        # 계수마다 각각 마지막 10개의 평균으로 최종 계수 결정
        # 왼쪽 차선의 계수 결정
        left_fit_[0] = np.mean(self.left_a[-10:])
        left_fit_[1] = np.mean(self.left_b[-10:])
        left_fit_[2] = np.mean(self.left_c[-10:])
        # 오른쪽 차선의 계수 결정
        right_fit_[0] = np.mean(self.right_a[-10:])
        right_fit_[1] = np.mean(self.right_b[-10:])
        right_fit_[2] = np.mean(self.right_c[-10:])

        # y 값에 해당하는 x 값 결정
        # 왼쪽 차선
        left_fitx = left_fit_[0] * self.ploty ** 2 + left_fit_[1] * self.ploty + left_fit_[2]

        # 오른쪽 차선
        right_fitx = right_fit_[0] * self.ploty ** 2 + right_fit_[1] * self.ploty + right_fit_[2]

        # 양쪽 모두 차선인식이 안됐다면 슬라이딩 윈도우 조사창 재설정
        if (left_lane_detected is False) and (right_lane_detected is False):
            self.leftx_base = self.leftx_mid
            self.rightx_base = self.rightx_mid

        # 출력 이미지, 양쪽 곡선 x 좌표, 차선 인식 여부 반환
        return out_img, left_fitx, right_fitx, left_lane_detected, right_lane_detected

    # sliding window 기준으로 가운데에 초록색 경로 그리기
    def draw_path(self, img, left_fitx, right_fitx, draw_windows=False):
        left_fitx = np.array([left_fitx])
        right_fitx = np.array([right_fitx])

        path_x = np.concatenate([left_fitx, right_fitx], axis=0)
        path_x = np.mean(path_x, axis=0).reshape(-1)

        path_y = self.ploty

        if draw_windows is True:
            for (x, y) in zip(path_x, path_y):
                cv2.circle(img, (int(x), int(y)), 3, (0, 255, 0), -1)

        return path_x, path_y

    # 곡률을 구하여 조향각 구하기
    def get_angle(self, path_x, path_y, left_lane_detected, right_lane_detected):

        # 차선 두 개 모두 인식 안될 경우
        if left_lane_detected is False and right_lane_detected is False:
            return self.steering_memory

        # 차선 하나라도 인식될 경우
        else:
            path = np.concatenate((path_x.reshape(-1, 1), path_y.reshape(-1, 1)), axis=1)

            self.avg_middle = np.mean(path_x, axis=0)

            point_a = path[0, :]  # Top Point
            point_b = path[-1, :]  # Bottom Point
            point_m = [(point_a[0] + point_b[0]) / 2, (point_a[1] + point_b[1]) / 2]  # point_a와 point_b의 중점

            W = math.sqrt(((point_a[0] - point_b[0]) ** 2) + ((point_a[1] - point_b[1]) ** 2))
            H = math.sqrt(np.min(np.sum((path - point_m) ** 2, axis=1)))

            # print("middle_distance: {}".format(middle_dist))

            # 640 pixel = 0.64m  ->  1 pixel = 0.001m
            radius = ((H / 2) + (W ** 2) / (8 * H)) * 0.0084
            steering_angle = math.atan(self.car_length / radius) * (180 / math.pi)

            # 2차 곡선 기울기 계수 구하기
            direction = np.polyfit(path[:, 1], path[:, 0], deg = 2)[0]

            if direction > 0.0:
                steering_angle *= 1.0

                if steering_angle >= 20.0:
                    steering_angle = 20.0

            elif direction < 0.0:
                steering_angle *= -1.0

                if steering_angle <= -20.0:
                    steering_angle = -20.0

            # 두 차선 모두 인식 안될 경우를 위해 현재 값 저장
            self.steering_memory = steering_angle

            return steering_angle


    # ========================================
    # 슬라이딩 윈도우를 원본 이미지에 투영하기 위한 역변환 행렬 구하기
    # ========================================
    def inv_perspective_transform(self, img):
        result_img = cv2.warpPerspective(img, self.camera.inv_transform_matrix, (self.camera.WIDTH, self.camera.HEIGHT))
        return result_img


    # ========================================
    # 원본 이미지와 최종 처리된 이미지를 합치기
    # ========================================
    def combine_img(self, origin_img, result_img):
        return cv2.addWeighted(origin_img, 0.5, result_img, 1.0, 0.8)


    # ========================================
    # Main 함수
    # ========================================
    def process(self, origin_img):
        origin_img = cv2.resize(origin_img, (640,480), cv2.INTER_LINEAR)
        img = self.camera.pre_processing(origin_img)
        sliding_img, left_fitx, right_fitx, left_lane_detected, right_lane_detected = self.sliding_window(img, draw_windows=True)  # 슬라이딩 윈도우로 곡선 차선 인식

        path_x, path_y = self.draw_path(sliding_img, left_fitx, right_fitx, draw_windows=True)
        curvature_angle = self.get_angle(path_x, path_y, left_lane_detected, right_lane_detected)
        
        # 주석 처리
        sliding_result_img = self.inv_perspective_transform(sliding_img)
        combined_img = self.combine_img(origin_img, sliding_result_img)
        # cv2.imshow('sliding_canny', sliding_img)
        cv2.putText(combined_img, 'Angle: {}'.format(int(curvature_angle)), (0, 50), 1, 5, (255, 255, 255), 3)
        cv2.imshow('Lane', combined_img)
        # 주석 처리

        return curvature_angle
