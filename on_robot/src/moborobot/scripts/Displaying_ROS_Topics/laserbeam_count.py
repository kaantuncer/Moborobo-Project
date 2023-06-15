#!/usr/bin/env python
# -*- coding: UTF-8 -*-


from sensor_msgs.msg import LaserScan
import rospy
import sys
import time
import numpy as np


def callback(data):
    print("**************************************************************")
    print(len(data.ranges))



rospy.init_node("subscriber",anonymous=True)



rospy.Subscriber("/scan", LaserScan,callback)
rospy.spin()
rospy.sleep(2)