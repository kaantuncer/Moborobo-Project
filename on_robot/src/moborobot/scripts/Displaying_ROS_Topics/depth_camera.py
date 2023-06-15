#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import sys
import time

font=cv2.FONT_HERSHEY_SIMPLEX
org=(10,30)
font_scale=0.6
color=(255,0,255)
thickness=2


bridge=CvBridge()

def depth_callback(ros_image):
    #print("Image received")
    global bridge

    try:
        cv_image=bridge.imgmsg_to_cv2(ros_image)
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Depth Registered",cv_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("Closing...")

def main(args):
    rospy.init_node("camera",anonymous=True)
    rospy.Subscriber("/zed2/zed_node/depth/depth_registered",Image,depth_callback)
    try:
        rospy.spin()

    except KeyboardInterrupt:
        print("Exiting...")
        cv2.destroyAllWindows()

if __name__=='__main__':
    main(sys.argv)