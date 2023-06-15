#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import sys
import time
from threading import Thread


bridge=CvBridge()

def cam1_callback(ros_image):
    #print("Image received")
    global bridge

    try:
        cv_image=bridge.imgmsg_to_cv2(ros_image,"bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Left",cv_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("Closing...")

def cam2_callback(ros_image):
    #print("Image received")
    global bridge

    try:
        cv_image=bridge.imgmsg_to_cv2(ros_image,"bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Right",cv_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("Closing...")


def cam1():
    rospy.Subscriber("/zed2/zed_node/left/image_rect_color",Image,cam1_callback)
    rospy.spin()


def cam2():
    rospy.Subscriber("/zed2/zed_node/right/image_rect_color",Image,cam2_callback)
    rospy.spin()


def main(args):
    rospy.init_node("camera",anonymous=True)

    t1 = Thread(target=cam1)
    t2 = Thread(target=cam2)

    t1.start()
    t2.start()

    t1.join()
    t2.join()
if __name__=='__main__':
    main(sys.argv)