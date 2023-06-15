# ROSLAUNCH


To start a launch file 'roslaunch' command must be used. After roslaunch command user must enter the package name then enter the launch file name.
```
roslaunch [package_name] [launch_file_name]
```

# Launch files in moborobot package

```
roslaunch moborobot [package_name]
```
```
* amcl.launch      
    AMCL -> Adaptive Monte Carlo Localization
    This launch file is for localization. (map-odom transform)
    Must be used after the map loading
```

```
* default.launch
    This launch file is for default start.
        -Motors, ZED2, RSLidar, LaserScan, rslidar-base_link tf
```

<p align="center">
  <img src="/home/moborobot/Pictures/default_start.png" width="" title="Default Start">
</p>


```
* full.launch
    This launch file is for starting all packages for navigation.
        -Motors, ZED2, RSLidar, LaserScanü rslidar-base_link tf, Map Loader, AMCL, move_base
        Map file-> /home/moborobot/maps/tammap.yaml
        move_base param folder -> /home/moborobot/catkin_ws/src/moborobot/nav
```



```
* lidar_only.launch
    This launch file is for starting only rslidar and laser scan packages.
```



```
* mapping.launch
    This launch file is for mapping.
        -Motors, ZED2, RSLidar, LaserScan, rslidar-base_link tf, karto-slam
```

```
* motor_only.launch
    This launch file is for starting only motors to move robot.
```

```
* move_base.launch
    This launch file is for generating global and local cost maps according to current map. Also, it provides navigation for robot. A point can be selected from map(rviz) the robot can go there in an autonomous way.
```

# CALIBRATION

* For calibration, a rosbag file must be recorded firstly. 

* Then copy this rosbag file to <b> `/home/moborobot/catkin_ws/src/lidar_camera_calibration/bagfiles` </b> folder.

* Change default bagfile argument in <b> `play_rosbag.launch` </b> file.

* After the starting roscore, play this rosbag file with;

```
roslaunch lidar_camera_calibration play_rosbag.launch
```

* While rosbag file is playing, start the calibration code with;
```
rosrun lidar_camera_calibration calibrate_camera_lidar.py
```
* This python script must have the permission. Give permission with;
```
sudo chmod 777 /home/moborobot/catkin_ws/src/lidar_camera_calibration/scripts/calibrate_camera_lidar_py
```