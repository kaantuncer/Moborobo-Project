#!/usr/bin/env python
# -*- coding: UTF-8 -*-


from sensor_msgs.msg import Imu
from tf.msg import tfMessage 
import rospy
import sys
import time
import numpy as np


def callback(data):
    print("**************************************************************")
    print(data)



rospy.init_node("subscriber",anonymous=True)



rospy.Subscriber("/tf", tfMessage,callback)
rospy.spin()
rospy.sleep(2)