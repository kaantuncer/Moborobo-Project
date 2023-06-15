

import open3d
import numpy as np
from ctypes import *  # convert float to uint32

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from datetime import datetime


# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8


def convert_rgbUint32_to_tuple(rgb_uint32): return (
    (rgb_uint32 & 0x00ff0000) >> 16, (rgb_uint32 &
                                      0x0000ff00) >> 8, (rgb_uint32 & 0x000000ff)
)


def convert_rgbFloat_to_tuple(rgb_float): return convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)


def lidar_callback(ros_cloud):

    # Get cloud data from ros_cloud
    field_names = [field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(
        ros_cloud, skip_nans=True, field_names=field_names))

    # Check empty
    open3d_cloud = open3d.geometry.PointCloud()
    if len(cloud_data) == 0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD = 3  # x, y, z, rgb

        # Get xyz
        # (why cannot put this line below rgb?)
        xyz = [(x, y, z) for x, y, z, rgb in cloud_data]

        # Get rgb
        # Check whether int or float
        # if float (from pcl::toROSMsg)
        if type(cloud_data[0][IDX_RGB_IN_FIELD]) == float:
            rgb = [convert_rgbFloat_to_tuple(rgb)
                   for x, y, z, rgb in cloud_data]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb)
                   for x, y, z, rgb in cloud_data]

        # combine
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = open3d.utility.Vector3dVector(
            np.array(rgb)/255.0)
    else:
        xyz = [(x, y, z) for x, y, z, i in cloud_data]  # get xyzi
        i = [(i) for x, y, z, i in cloud_data]  # get xyzi
        open3d_cloud.points = open3d.utility.Vector3dVector(
            np.array(xyz))
        i=np.array(i).reshape(-1,1)
        i=np.concatenate((i,i,i),axis=1)

        open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(i))
    timestr=datetime.now().strftime("%Y_%m_%d-%H_%M_%S_%f")[:-3]
    file_path="/home/semooww/catkin_ws/src/staj/scripts/saved_data/ply/lidar-"+timestr+".ply"
    open3d.io.write_point_cloud(file_path, open3d_cloud)


rospy.init_node("lidar_data", anonymous = True)
rospy.Subscriber("/rslidar_points", PointCloud2, lidar_callback)
rospy.spin()
