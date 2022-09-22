#!/usr/bin/env python
import cv2
from cv_bridge import CvBridge
from ros import rosbag
import rospy

topic = 'usb_cam/image_raw'
video_path = '/home/aeslab/embe_ws/src/main/src/project_video.mp4'

def main(video_path, bag_name):
    bag = rosbag.Bag(bag_name, 'w')
    cap = cv2.VideoCapture(video_path)
    bridge = CvBridge()
    frame_id = 0
    prop_fps = cap.get(cv2.CAP_PROP_FPS)
   
    while True:
        print('running', frame_id)
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.resize(frame, (640, 480), cv2.INTER_LINEAR)
        stamp = rospy.rostime.Time.from_sec(float(frame_id)/ prop_fps)
        image = bridge.cv2_to_imgmsg(frame, "bgr8")
        frame_id += 1
        image.header.frame_id = 'usb_cam'
        image.header.stamp = stamp
        bag.write(topic, image, stamp)
    cap.release()
    bag.close()


main(video_path, 'test.bag')