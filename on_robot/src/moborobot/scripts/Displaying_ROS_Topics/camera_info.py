#!/usr/bin/env python
# -*- coding: UTF-8 -*-


from sensor_msgs.msg import CameraInfo
import rospy
import sys
import time
import numpy as np
from threading import Thread
counter=0


def left_callback(data):
    global counter
    print("*******************************"+data.header.frame_id+"*******************************")
    rospy.loginfo("height: "+str(data.height)+" - width: "+str(data.width))
    p=np.array(data.P).reshape(3,4)
    print(p)


def right_callback(data):
    global counter
    print("*******************************"+data.header.frame_id+"*******************************")
    rospy.loginfo("height: "+str(data.height)+" - width: "+str(data.width))
    p=np.array(data.P).reshape(3,4)
    print(p)


def left_image():
    rospy.Subscriber("/zed2/zed_node/left_raw/camera_info", CameraInfo,left_callback)
    rospy.spin()


def right_image():
    rospy.Subscriber("/zed2/zed_node/right_raw/camera_info", CameraInfo,right_callback)
    rospy.spin()


# Projection/camera matrix
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]


def main(args):

    rospy.init_node("cam_info",anonymous=True)
    t1 = Thread(target=left_image)
    t2 = Thread(target=right_image)
    
    t1.start()
    t2.start()

    t1.join()
    t2.join()
