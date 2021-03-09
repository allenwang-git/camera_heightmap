# Introduction
_Author: Wang Yinuo_\
_Date: 10/10/2020_\
通过ros获得pcl点云，并滤波输出


# Run 
```
$ cd catkin_ws
$ catkin_make
$ roslaunch astra_launch astra.launch     <!-- /opt/ros/kinetic/share/astra_launch/launch -->
```
Open a new terminal:
```
$ roslaunch my_pcl mypcl.launch    
```

# Node 
```
$ rosnode list
    /camera/camera_nodelet_manager
    /downsample (my_pcl/downsample)
    /rviz (rviz/rviz)
    /xlimit (my_pcl/xlimit)
    /ylimit (my_pcl/ylimit)
    /zlimit (my_pcl/zlimit)
```

# Topic 
```
$ rostopic list

    /camera/depth/points
    /downsampled
    /xlimited
    /ylimited
    /zfinal
```
