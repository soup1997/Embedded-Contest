# Embedded-System-Contest
   
2022 한국공학대학교 전자공학부 임베디드시스템 경진대회

---

### 명령어   

Sensor 실행 (Camera, LiDAR)
```
roslaunch sensor sensor.launch
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

