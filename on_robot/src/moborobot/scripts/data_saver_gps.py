#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

from socketserver import ThreadingUnixDatagramServer
import rospy
import cv2
from sensor_msgs.msg import Image, PointCloud2, PointField, Imu
from tf.msg import tfMessage
from cv_bridge import CvBridge, CvBridgeError
import sys
import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from threading import Thread
from datetime import datetime
import os
from nmea_msgs.msg import Sentence

# --Initializes
bridge = CvBridge()
SAVE = False
FINISH = False
MAIN_PATH = r"/home/moborobot/saved_data/"
LEFT_IMG_PATH = None
RIGHT_IMG_PATH = None
DEPTH_PATH = None
PCD_PATH = None
IMU_PATH = None
IMU_FILE = False
TF_PATH = None
TF_FILE = False
GPS_PATH = None
GPS_FILE = False

# --Time formats
#timestr = datetime.now().strftime("%Y_%m_%d-%H_%M_%S_%f")[:-3]
#timestr = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")


#### CALLBACK FUNCTIONS ####
def left_camera_callback(ros_image):
    global bridge
    global LEFT_IMG_PATH
    if not SAVE:
        return
    try:
        # Convert ros image to cv2 image
        cv_image = bridge.imgmsg_to_cv2(ros_image)
    except CvBridgeError as e:
        print(e)
    timestr = datetime.now().strftime(
        "%Y_%m_%d-%H_%M_%S_%f")[:-3]  # Take current time


    header_time = str(ros_image.header.stamp.to_time())
    file_name = "image-"+header_time+".png"  # Generate file name according to time

    if not LEFT_IMG_PATH == None:
        file_path = os.path.join(LEFT_IMG_PATH, file_name)
    else:
        return
    cv2.imwrite(file_path, cv_image)


def right_camera_callback(ros_image):
    global bridge
    global RIGHT_IMG_PATH
    if not SAVE:
        return
    try:
        # Convert ros image to cv2 image
        cv_image = bridge.imgmsg_to_cv2(ros_image)
    except CvBridgeError as e:
        print(e)
    
    header_time = str(ros_image.header.stamp.to_time())
    timestr = datetime.now().strftime(
        "%Y_%m_%d-%H_%M_%S_%f")[:-3]  # Take current time
    file_name = "image-"+header_time+".png"  # Generate file name according to time
    if not RIGHT_IMG_PATH == None:
        file_path = os.path.join(RIGHT_IMG_PATH, file_name)
    else:
        return
    cv2.imwrite(file_path, cv_image)


def depth_callback(depth_image):
    global bridge
    if not SAVE:
        return
    try:
        # Convert ros image to cv2 image
        cv_image = bridge.imgmsg_to_cv2(depth_image)
    except CvBridgeError as e:
        print(e)
    
    timestr = datetime.now().strftime(
        "%Y_%m_%d-%H_%M_%S_%f")[:-3]  # Take current time

    header_time = str(depth_image.header.stamp.to_time())

    # Generate file name according to time
    file_name = "depth_image-"+header_time+".png"
    if not DEPTH_PATH == None:
        file_path = os.path.join(DEPTH_PATH, file_name)
    else:
        return
    # Convert cv2 image to numpy array in np.float32 data type
    depth_array = np.array(cv_image, dtype=np.float32)
    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
        cv_image, alpha=255/depth_array.max()), cv2.COLORMAP_BONE)
    cv2.imwrite(file_path, depth_colormap)


def lidar_callback(ros_cloud):
    if not SAVE:
        return
    # Get cloud data from ros_cloud
    field_names = [field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(
        ros_cloud, skip_nans=True, field_names=field_names))

    # Check empty
    open3d_cloud = o3d.geometry.PointCloud()
    if len(cloud_data) == 0:
        print("Converting an empty cloud")
        return None

    xyz = [(x, y, z) for x, y, z, i in cloud_data]  # get xyzi
    i = [(i) for x, y, z, i in cloud_data]  # get xyzi
    open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
    i = np.array(i).reshape(-1, 1)
    i = np.concatenate((i, i, i), axis=1)

    open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(i))
    timestr = datetime.now().strftime(
        "%Y_%m_%d-%H_%M_%S_%f")[:-3]  # Take current time


    header_time = str(ros_cloud.header.stamp.to_time())
    file_name = "lidar-"+header_time+".pcd"     # Generate file name according to time
    if not PCD_PATH == None:
        file_path = os.path.join(PCD_PATH, file_name)
    else:
        return
    o3d.io.write_point_cloud(file_path, open3d_cloud)


def imu_callback(imu_data):
    global IMU_PATH
    global IMU_FILE
    if not SAVE:
        return
    if not IMU_PATH == None:
        file_name = "imu.txt"       # Generate file name
        file_path = os.path.join(IMU_PATH, file_name)
        timestr = datetime.now().strftime(
            "%Y_%m_%d-%H_%M_%S_%f")[:-3]  # Take current time

        header_time = str(imu_data.header.stamp.to_time())
        if not IMU_FILE:
            txt_file = open(file_path, 'w')
            IMU_FILE = True
        else:
            txt_file = open(file_path, 'a')
        data = "Time: " + header_time+"\n" + \
            str(imu_data)+"\n**********************************************************************\n"
        txt_file.write(data)
        txt_file.close()
    else:
        return


def tf_callback(tf_data):
    global TF_PATH
    global TF_FILE
    if not SAVE or TF_PATH == None:
        return
    timestr = datetime.now().strftime(
        "%Y_%m_%d-%H_%M_%S_%f")[:-3]  # Take current time
    
    # header_time = str(tf_data.header.stamp.to_time())
    file_name = "tf.txt"        # Generate file name according to time
    file_path = os.path.join(TF_PATH, file_name)
    if not TF_FILE:
        txt_file = open(file_path, 'w')
        TF_FILE = True
    else:
        txt_file = open(file_path, 'a')
    data = "Time: "+timestr+"\n" + \
        str(tf_data)+"\n**********************************************************************\n"
    txt_file.write(data)
    txt_file.close()




def gps_callback(gps_data):
    global GPS_PATH
    global GPS_FILE
    if not SAVE or GPS_PATH == None:
        return
    '''Callback function of subscribed topic. '''
    header_time = str(gps_data.header.stamp.to_time())
    sentence = gps_data.sentence
    #header_time = str(rospy.Time.now().to_time())

    timestr = datetime.now().strftime(
        "%Y_%m_%d-%H_%M_%S_%f")[:-3]  # Take current time
    file_name = "gps.txt"        # Generate file name according to time
    file_path = os.path.join(GPS_PATH, file_name)

    if not GPS_FILE:
        myfile = open(file_path, 'w')
        GPS_FILE = True
    else:
        myfile = open(file_path, 'a')

    myfile.write("New GPS Data:\n")
    myfile.write(header_time)
    myfile.write("\n")
    myfile.write(sentence)
    myfile.write("\n")




#### SUBSCRIBE FUNCTIONS ####
def ros_left_image():
    rospy.Subscriber("/zed2/zed_node/left/image_rect_color",
                     Image, left_camera_callback)
    rospy.spin()


def ros_right_image():
    rospy.Subscriber("/zed2/zed_node/right/image_rect_color",
                     Image, right_camera_callback)
    rospy.spin()


def ros_depth_image():
    rospy.Subscriber("/zed2/zed_node/depth/depth_registered",
                     Image, depth_callback)
    rospy.spin()


def ros_lidar():
    rospy.Subscriber("/rslidar_points", PointCloud2, lidar_callback)
    rospy.spin()


def ros_imu():
    rospy.Subscriber("/zed2/zed_node/imu/data", Imu, imu_callback)
    rospy.spin()


def ros_tf():
    rospy.Subscriber("/tf", tfMessage, tf_callback)
    rospy.spin()

def ros_gps():
    gps_sub = rospy.Subscriber("/gps", Sentence, gps_callback)
    rospy.spin()


"""
Function to wait for user input. 
    r -> start record
    p -> pause record
    f -> finish record

Every time user presses 'r' button, new folder will be created. If user pauses the record, new folder won't be created.

"""


def take_input():
    global SAVE
    global FINISH
    global MAIN_PATH
    global LEFT_IMG_PATH
    global RIGHT_IMG_PATH
    global DEPTH_PATH
    global PCD_PATH
    global IMU_PATH
    global IMU_FILE
    global TF_PATH
    global TF_FILE
    global GPS_PATH
    global GPS_FILE
    while not FINISH:
        my_input = input(
            "\t'r' to (re)start recording (or create a new folder)\n\t'p' to pause recording\n\t'f' to finish record\nInput -->> ")
        if my_input.lower() == 'r':
            # Folder count in main folder
            dir_count = len(os.listdir(MAIN_PATH))
            timestr = datetime.now().strftime("%Y_%m_%d")  # Take current time
            # Generate new folder's name
            folder_name = str(dir_count+1)+"-"+timestr
            new = os.path.join(MAIN_PATH, folder_name)
            os.mkdir(new)

            # Generating data folders' names
            LEFT_IMG_PATH = os.path.join(new, "left_images")
            RIGHT_IMG_PATH = os.path.join(new, "right_images")
            DEPTH_PATH = os.path.join(new, "depth")
            PCD_PATH = os.path.join(new, "pcd")
            IMU_PATH = os.path.join(new, "imu")
            TF_PATH = os.path.join(new, "tf")
            GPS_PATH = os.path.join(new, "gps")

            # Creating data folders
            os.mkdir(LEFT_IMG_PATH)
            os.mkdir(RIGHT_IMG_PATH)
            os.mkdir(DEPTH_PATH)
            os.mkdir(PCD_PATH)
            os.mkdir(IMU_PATH)
            os.mkdir(TF_PATH)
            os.mkdir(GPS_PATH)
            print("Recording is started.")
            SAVE = True  # Convert SAVE param to true to start recording
            IMU_FILE = False
            TF_FILE = False
            GPS_FILE = False
        elif my_input.lower() == 'p':
            print("Recording is stopped.")
            SAVE = False  # Convert SAVE param to false to pause recording
        elif my_input.lower() == 'f':
            SAVE = False
            FINISH = True
            print("Closing...")
            rospy.signal_shutdown("Closing")
            cv2.destroyAllWindows()
            sys.exit(1)
        else:
            print("Press a valid button.")
        print("**************************************************************")


def main(args):
    rospy.init_node("collector", anonymous=True)
    t1 = Thread(target=ros_left_image)
    t2 = Thread(target=ros_right_image)
    t3 = Thread(target=ros_depth_image)
    t4 = Thread(target=ros_lidar)
    t5 = Thread(target=ros_imu)
    t6 = Thread(target=ros_tf)
    t7 = Thread(target=take_input)
    t8 = Thread(target=ros_gps)

    t1.start()
    t2.start()
    t3.start()
    t4.start()
    t5.start()
    t6.start()
    t7.start()
    t8.start()

    t1.join()
    t2.join()
    t3.join()
    t4.join()
    t5.join()
    t6.join()
    t7.join()
    t8.join()


if __name__ == '__main__':
    main(sys.argv)
