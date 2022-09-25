# Embedded-System-Contest
   
2022 한국공학대학교 전자공학부 임베디드시스템 경진대회   
![image](https://user-images.githubusercontent.com/86957779/191813158-23ea2df7-d0a8-465a-a691-7ab65fa194ad.png)

## Frame
![KakaoTalk_20220916_175417917_01](https://user-images.githubusercontent.com/86957779/190599690-af84a95d-373e-4e92-aee9-21f6c8fdbec0.jpg)
![KakaoTalk_20220916_175417917](https://user-images.githubusercontent.com/86957779/190599709-ffaa85e7-c243-48ac-99f2-f410f173140a.jpg)
---

### 명령어   

Sensor 실행 (Camera, LiDAR)   
```
roslaunch sensor sensor.launch
```
Sensor, Lane Detection, Obstacle Avoidance, LiDAR-Camera Calibration, Motor Control 실행
```
roslaunch main main.launch
```

---

### 진행 상황   


**(2022-09-06)**

* 모든 센서 usb 포트 지정 완료

* 모터, 서보모터 - 아두이노를 통한 제어 동작 확인

* 모터, 서보모터 동작 범위 확인   

  > 모터 : 150 ~ 250 (255까지 가능하지만 편의상 250으로 맞춤)   
  > 서보모터 : 140(Left) ~ 90(Middle) ~ 40(Right)   
  
   
**(2022-09-08)**

* Arduino - Ackermann_msgs 추가 및 코드 작성 완료   

* Yolo 설치는 했으나, OpenCV 버전으로 인해 동작되지 않음   
   아래 링크에 나와있는 순서대로 진행했으며, catkin_make 에서 오류 발생   
   
   https://github.com/Tossy0423/yolov4-for-darknet_ros   
   
   OpenCV 설치 완료 후 여기부터 진행하면 됨   
   
   https://github.com/Tossy0423/yolov4-for-darknet_ros#make-pkg   
   ```
   catkin_make
   ```
* OpenCV 3.4.0 버전 설치 미완료   
   아래 링크에 나와있는 순서대로 진행했으며, 컴파일만 하면 완료   
   
   https://keyog.tistory.com/7   
   ```
   cd ~/opencv/opencv-3.4.0/build/
   make -j
   ```
* 기존에 설치되어 있던 OpenCV를 제거 후 진행하여 sensor.launch 실행 후 rqt_image_view 확인이 되지는 않음


**(2022-09-13)**

* OpenCV 설치 완료 (version 3.4.0)   

* Servo 헌팅현상 방지 위해 330uF의 바이패스 역할의 커패시터를 전원부에 연결 

* Yolo 설치 후 catking_make 실행 시 image_transport, image_common 관련 Error 발생   
  ~~아래 링크에 있는 image_transport 를 직접 다운 받는 건 어떤가 하는 의견.~~   
  ~~https://github.com/ros-perception/image_common~~   
  
  문제가 발생한 image_transport 폴더 삭제 후 catkin_make 실행 후 정상 동작   
   
* Yolo 설치 완료 후 아래 명령어로 yolo 실행 시 에러 발생
  ```
  roslaunch darknet_ros yolo_v4.launch
  ```
  1) /camera/rgb/image_raw 를 읽어오지 못하는 문제   
     사용 중인 카메라에 맞게 usb_cam/image_raw 로 변경   
     
  2) 그 외에도 알 수 없는 에러가 뜸   
     아직 해결 못함 --> 금요일 해결 예정


**(2022-09-15)**   

* Jetson Nano 에서 YoloV4 사용 불가   
  -> yaml, launch 파일에서 yolov4를 모두 yolov3로 수정 후 동작 성공   
  -> But, 속도가 너무 느리며 yolo 자체 용량 차지가 심함   
  -> Yolo 사용 X   
* 하단 프레임(DC모터, 서보모터, MCU)과 상단 프레임 부착(Jetson Nano, LiDAR, Camera)을 위한 프로파일 부착 완료   
* LiDAR & Camera Calibration 방법 아래 링크 참고   
  https://github.com/ehong-tl/camera_2d_lidar_calibration   


**(2022-09-16)**   

* 프레임 제작 완료   
  > 카메라 높이 : 16.3 cm   
  > 라이다 높이 : 19.0 cm   

* 카메라 & 라이다 캘리브레이션 작업 완료   
  참고 링크   
  https://github.com/ehong-tl/camera_2d_lidar_calibration/blob/master/how%20to%20use.pdf   
  
  - 캘리브레이션 진행 화면
  ![Screenshot from 2022-09-16 17-13-55](https://user-images.githubusercontent.com/96249554/190599502-17fb2c1d-2463-49e5-85fc-4a32be8ffcd8.png)

     
   ```
   Camera parameters
   Lens = fisheye
   K =
   [[1.01633497e+03 0.00000000e+00 6.86163990e+02]
    [0.00000000e+00 1.01039193e+03 3.87096590e+02]
    [0.00000000e+00 0.00000000e+00 1.00000000e+00]]
   D =
   [ 0.067985 -0.106608  0.00214   0.00214 ]
   Transform from camera to laser
   T = 
   [[-0.05222866]
    [-0.07402507]
    [ 0.12833293]]
   R = 
   [[-0.36307376 -0.92952075  0.06456481]
    [-0.22650008  0.020832   -0.97378835]
    [ 0.90381146 -0.36818093 -0.21810008]]
   Quaternion = 
   -0.332 -0.457i +0.633j -0.530k
   RMSE in pixel = 28.498005
   Result output format: qx qy qz qw tx ty tz
   ```
   
   ```
   calibration_result.txt
   -0.456671 0.632852 -0.530128 -0.331534 -0.052229 -0.074025 0.128333
   ```
   
   - 캘리브레이션 결과
   ![Screenshot from 2022-09-16 17-54-52](https://user-images.githubusercontent.com/96249554/190599124-ed285672-170b-428b-bdf7-b168ffbcc2fa.png)
   
   - LiDAR Frame to Camera Frame   
   https://medium.com/mlearning-ai/a-single-camera-3d-functions-fdec7ffa9a83   
   https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html
   ![image](https://user-images.githubusercontent.com/86957779/190612827-b2205850-f6b7-4734-ac3a-86f3c85af870.png)
   ![화면 캡처 2022-09-16 184804](https://user-images.githubusercontent.com/86957779/190611424-0b1d273f-22ac-4f83-b6ed-f68f8bde11f6.png)


**(2022-09-20)**

* 기존 코드 수정 완료, 클러스터링 좌표 수정 완료   
* 서보모터 동작 확인, 무게로 인한 DC모터 구동 실패   
* bag 파일 녹화 후 시연 때 재생 예정   
* 작품 설명서 제작 예정, 라이다-카메라 센서 퓨전 시각화 적용 예정   
* 배터리 충전해 놓을것!!   

* 카메라 위치 재조정 후 데이터 재수집, 캘리브레이션 결과 -> 위에 덮어 씀   
* 작품 설명서 제작 완료   
* 트랙 bag 녹화 예정   
* 속도제어 코드   

**(2022-09-22)**

* 트랙 녹화 영상 사용하기에 부적합   
  -> project_video 영상 재생으로 대체   
* project_video.mp4 -> test.bag 변환   
* 장애물 클러스터링 및 회피 동작 완료   
* 장애물 미검출 시 test.bag 파일로 차선 인식 수행   
  장애물 검출 시 usb camera의 영상에 장애물 투영   

**(2022-09-23)**   
* 장애물 위치에 따른 각도 조정 및 정면에 위치 시 속도 0   
* 완료!   
