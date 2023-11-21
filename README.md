# Moborobot
You can find the website for the project here: https://tuanacetinkaya.github.io/Moborobot/

In today’s world, many tasks were automated by various types of robots. Mobile robots are a subfield of robotics that are automatic machines capable of locomotion. They are used in various fields such as entertainment, domestic appliances, military and medical fields. These applications require mobile robots to perform complicated tasks and navigate in different environments. This project focuses on a real mobile robot that is capable of autonomously navigate from point A to point B. Our robot features a 3D LidAR sensor, a ZED2 camera, odometry data through its motors and a gps sensor. For the purposes of this project we only used the LidAR sensor and the odometry data. We used these to map the environment and generate costmaps to avoid obstacles. 


## Table of Contents

- [1. Simulation Setup](#2-simulation-setup)
- [2. Orin Setup](#1-orin-setup)
- [3. Project on Robot Setup](#3-project-on-robot-setup)
- [4. Plugins](#4-plugins)
- [5. Packages](#5-packages)

## 1. Simulation Setup

Our first objective was to create a model of our robot for the Gazebo simulation. This was a crucial step because all of our future development was dependent on this objective being accurate. We measured the robot and made a model with a 1-1 scale. We also needed to find accurate versions of the robot's sensor to use in the simulation. We’ve found the exact versions of each. 

![Simulation Setup Screenshot](/path/to/simulation-setup-screenshot.png)

## 2. Orin Setup
### Configuring via nVidia SDK Manager
In order to use nVidia Jetson Orin properly with our robot, first we need to complete its setup on the operating system side as well as on the ROS side. Nvidia has an SDK manager for managing Orin’s operating system and its other components such as CUDA, Computer Vision packages and so on.
NVIDIA SDK Manager is a tool which provides an end-to-end development environment setup solution for NVIDIA’s DRIVE, Jetson, Holoscan, Rivermax, DOCA and Ethernet Switch SDKs for both host and target devices.
Right before starting the installation process, we need to put Orin in recovery mode. This can be done with starting Orin by pressing on the recovery button.
1. After putting Orin into recovery mode and connecting it to the host machine via its USB cable, it should appear in the SDK manager. We set the Orin up with JetPack 5.1.1 and DeepStream package as additional SDK.
2. As we continue to the next steps, the SDK manager will download the necessary files and start installing them into Orin by itself. It might take a while since the files that are being downloaded and transferred are large files.
3. After flashing Jetson OS, the SDK manager will ask about the setup type that we want to continue as well as username and password for our Orin.
4. We pick Manual Setup for Jetson Orin 32GB - Developer Kit and put username, password into the given boxes. We used “moborobot” as our username and “root” as our password. Rest of the settings can be left with their default values.
5. After this step and right before additional SDKs installation, SDK manager will ask again for our username and password. In this step it won’t create a new user but it will use the credentials for installation purposes, so we used the credentials that we gave in the steps before. Before confirming the SDK installation, we went into our Orin and completed the initial setup, so it can continue without any problems.
After completing these operations and waiting enough till the whole process is completed, we can finally start our Jetson and use it.

![Orion Setup Screenshot](/path/to/orin-setup-screenshot.png)

### ROS Setup 
Now, it is time to set up ROS and install our robot’s packages. Orin’s operating system is based on Ubuntu 20.04. So, as we did in the simulation, we will use ROS Noetic Ninjemys here as well[3]. We followed the recommended configurations[4] and installed Desktop-Full Install. In order to have it installed, we run:

`sudo apt install ros-noetic-desktop-full`
Then added ROS’ path to our bash with:

`$ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
 $source ~/.bashrc`

 Then installed ROS’ dependencies and then initialized it with:
 ```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update 
```
 After completing ROS installation as well, we can start working on our packages. We placed “src” folder under “/home/moborobot/robot” directory and as it is necessary.
 #### install the dependencies with:
  `rosdep install --from-paths src --ignore-src --rosdistro noetic -y`
 With this code, rosdep reads CMakeLists.txt and package.xml files and looks up for dependencies from these definitions. With a correctly written package list, this command should install all necessary dependencies. 
 After running these commands, we should install the Zed2 SDK package as well. This package cannot be installed via the rosdep command, so we have to install it manually.
 - The ZED SDK v4.x and its documentation can be reach from: [the official website](https://www.stereolabs.com/developers/release/) and [documentation](https://www.stereolabs.com/docs/get-started-with-zed/)
 The installation process of ZED SDK starts downloading the proper file: ZED SDK for JetPack 5.1.1 (L4T 35.3) 4.0, and making it executable with:
 `
 sudo apt install zstd
 chmod +x ZED…`
 After installing it, we can finally start installing ZED SDK with running: `./ZED…`

 After completing the ZED installation as well, we will run `catkin_make` command to compile our code.
 After getting the build done successfully, we should run this command:
 `source devel/setup.bash`
So, we can execute our codes on the robot with roslaunch. Otherwise, our codes will not be reachable via ROS.

## 3. Project on Robot Setup

Explanation of the setup process for deploying the project on a robot. Include step-by-step instructions and relevant screenshots.

![Project on Robot Setup Screenshot](/path/to/project-on-robot-setup-screenshot.png)
 ### Configuring move_base
1. Adding obstacle layer

2. Adding inflation layer

3. Deciding on a path planning algorithm



## 4. Plugins

List of plugins used in the project:

- Controller: libgazebo_ros_diff_drive.so
This plugin is a component of the Gazebo simulator that integrates with the Robot Operating System (ROS) and enables a control interface for differential drive robots.The plugin provides a manual controller for the robot and lets us control the robot by using the keyboard.
- Camera:  RGB camera sensor
The actual model normally has a ZED2 camera which is a 3D-RGB camera. However, since the Depth and 3D features of the ZED2 camera were not used in the simulation as well as on the real robot, instead of a ZED2 camera, we used a normal RGB camera sensor without any depth or 3D feature. This camera represents the ZED2 camera in our actual robot and it faces in front of the robot and provides a real time camera feature for our simulation.
- RS-LiDAR 16 (Robosense): Description of plugin 3.
The RS-Lidar 16 robosense's ROS package provides a ROS driver for the RS-LiDAR-16, a 16-line mechanical LiDAR sensor. The ROS driver allows you to interface with the RS-LiDAR-16 in ROS and receive point cloud data from the sensor. The LiDAR sensor gathers object data in the form of a point cloud, which consists of precise angle and distance information represented by a set of scattered points.

- Mapping (SLAM): slam_toolbox
The SLAM Toolbox is a library that provides a collection of tools and algorithms for Simultaneous Localization and Mapping (SLAM) in robotics and computer vision applications. The SLAM Toolbox offers various functionalities to perform SLAM tasks, such as data association, map management, state estimation, and sensor fusion. It includes implementations of popular SLAM algorithms like EKF-SLAM (Extended Kalman Filter SLAM), FastSLAM, and GraphSLAM.

## 5. Packages

List of packages used in the project:

- point_cloud_to_laser_scan
After we set up the model in the simulation we moved on to the mapping objective. The robot should be able to generate a 2D map of its environment using its lidar sensor. Since the lidar sensor produces its output in 3D we needed a separate converter library called point_cloud_to_laser_scan. This library converts the 3D output of the lidar sensor to a 2D output called scan.
- move_base
After the creation of the map, when a target is given to our model, our robot can move toward the given target using the move_base module. While moving towards the given target, in some specified period of time our robot scans its surroundings and updates global and local paths
