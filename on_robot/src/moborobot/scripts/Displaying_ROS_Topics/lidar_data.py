#!/usr/bin/env python
# -*- coding: UTF-8 -*-


import ctypes
import os
import struct
import sys
import time
from datetime import datetime
from threading import Thread

import numpy as np
import open3d as o3d
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

counter_ply = 0
timer_limit = 15


def lidar_callback(ros_point_cloud):
    global counter_ply
    xyz = np.array([[0, 0, 0]])
    rgb = np.array([[0, 0, 0]])
    # self.lock.acquire()
    gen = pc2.read_points(ros_point_cloud, skip_nans=True)
    int_data = list(gen)

    for x in int_data:
        test = x[3]
        # cast float32 to int so that bitwise operations are possible
        s = struct.pack('>f', test)
        i = struct.unpack('>l', s)[0]
        # you can get back the float value by the inverse operations
        pack = ctypes.c_uint32(i).value
        r = (pack & 0x00FF0000) >> 16
        g = (pack & 0x0000FF00) >> 8
        b = (pack & 0x000000FF)
        # prints r,g,b values in the 0-255 range
        # x,y,z can be retrieved from the x[0],x[1],x[2]
        xyz = np.append(xyz, [[x[0], x[1], x[2]]], axis=0)
        rgb = np.append(rgb, [[r, g, b]], axis=0)

    out_pcd = o3d.geometry.PointCloud()
    out_pcd.points = o3d.utility.Vector3dVector(xyz)
    out_pcd.colors = o3d.utility.Vector3dVector(rgb)
    #timestr = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    timestr = datetime.now().strftime("%Y_%m_%d-%H_%M_%S_%f")[:-3]
    file_path = "/home/moborobot/catkin_ws/src/staj/scripts/saved_data/ply/lidar-"+timestr+".ply"
    o3d.io.write_point_cloud(file_path, out_pcd)
    counter_ply += 1


def timer():
    global counter_ply
    global timer_limit
    counter = 0
    while counter < timer_limit:
        counter += 1
        print("Time: ", counter)
        time.sleep(1)
    print("Closing...")

    path2 = r"/home/moborobot/catkin_ws/src/staj/scripts/saved_data/ply/"

    print(
        f"Saved ply count: {counter_ply}\nFile count in ply folder: {len(os.listdir(path2))}")
    rospy.signal_shutdown("Closing")

    sys.exit(1)


def ros_lidar():
    rospy.Subscriber("/rslidar_points", PointCloud2, lidar_callback)
    rospy.spin()


def main(args):
    rospy.init_node("lidar_data", anonymous=True)

    ros_lidar()
    """
    t2 = Thread(target=ros_lidar)
    t3 = Thread(target=timer)

    t2.start()
    t3.start()

    t2.join()
    t3.join()
    """

if __name__ == '__main__':
    main(sys.argv)
