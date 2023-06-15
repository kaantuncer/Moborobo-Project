# ROS

## Setup your sources.list

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

## Set up your keys

```
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

## Installation

```
sudo apt update 
```

* Desktop-Full Install: (Recommended) : Everything in Desktop plus 2D/3D simulators and 2D/3D perception packages 
```
sudo apt install ros-noetic-desktop-full
```
* Desktop Install: Everything in ROS-Base plus tools like rqt and rviz
```
sudo apt install ros-noetic-desktop
```
* ROS-Base: (Bare Bones) ROS packaging, build, and communication libraries. No GUI tools. 
```
sudo apt install ros-noetic-ros-base
```

## Source

```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Dependencies
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

## Initialize rosdep

```
sudo apt install python3-rosdep
```
With the following, you can initialize rosdep. 
```
sudo rosdep init
rosdep update
```

## Create a ROS Workspace

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

# ZED CAMERA

* Firstly, appropriate SDK must be installed from [this link](https://www.stereolabs.com/developers/release/)

* [This](https://www.stereolabs.com/docs/installation/linux/) instructions can be followed for installation ZED SDK.

* Required ros packages are [here](https://www.stereolabs.com/docs/ros/).

# NECESSARY LINKES

```
All these packages need their own paramaters and configs. If a package is to be downloaded and used for the first time, attention should be paid to all its settings.
```

```
* RoboSense Lidar SDK
https://github.com/RoboSense-LiDAR/rslidar_sdk

*RoboSense Lidar driver 
https://github.com/RoboSense-LiDAR/rs_driver

*Package contain necessary msgs
https://github.com/ros/geometry2

*Hector Slam Package (Mapping-map saver in .twf and .tif extensions)
https://github.com/tu-darmstadt-ros-pkg/hector_slam

*Camera Lidar Calibration
https://github.com/heethesh/lidar_camera_calibration

*Necessary package for others
https://github.com/dimatura/pypcd

*Some packages need velodyne msg
https://github.com/ros-drivers/velodyne

*Navigation
http://wiki.ros.org/navigation
https://github.com/ros-planning/navigation

*Converting rslidar pointcloud to laser scan
https://github.com/ros-perception/pointcloud_to_laserscan

*Karto Slam -> This package is used for mapping
http://wiki.ros.org/slam_karto
https://github.com/ros-perception/slam_karto

*Generating odometry topic from /vel topic (there is no page for this package anymore. But in ftp server there may be.)
vel_to_odom


Default packages send from manufactory

**CONTROL
    *joy_to_vel -> Joystick messages to be translated to velocity commands.
    *kinematics -> Give motor commands and publish odometry according to robot movements

**Definitions -> Necessary msgs for robot

**Roboteq_driver -> need for robot movement

**Robot_launch -> contain start command for motors and roscore

**Serial 


```









