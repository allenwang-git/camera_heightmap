# Camera_PCL
_Author: Wang Yinuo_\
_Date: 10/10/2020_

## Introduction
This project uses ROS & PCL to obtain and filt point cloud, and output 3D height map. The height map can be saved, loaded, and publish to other programs by LCM. For now, this project only support Realsense d435i and Orbbec Astra depth camera.

## Dependencies
* ROS (kinetic)
* PCL
* LCM (1.4.0 or newer)

## Build & Run 
```
$ mkdir catkin_ws 
$ cd catkin_ws
$ catkin_make
$ roslaunch astra_launch astra.launch     <!-- /opt/ros/kinetic/share/astra_launch/launch -->
```
Open a new terminal:
```
$ roslaunch camera_heightmap mypcl.launch    
```
To load saved height map & traversability map and published by LCM:
```
$ roslaunch camera_heightmap hsload.launch
```
You can load different map file in `/data` by modify the `/src/hsload.cpp`.

## Nodes 
```
$ rosnode list
    /camera/camera_nodelet_manager
    /downsample (my_pcl/downsample)
    /rviz (rviz/rviz)
    /xlimit (my_pcl/xlimit)
    /ylimit (my_pcl/ylimit)
    /zlimit (my_pcl/zlimit)
```

## Topics 
```
$ rostopic list

    /camera/depth/points
    /downsampled
    /xlimited
    /ylimited
    /zfinal
```
