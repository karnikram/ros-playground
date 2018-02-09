Launch file and params for 2D occupancy grid mapping from 3D laser data. A local 2D CostMap in odom frame, that moves along with the robot, is used.

Tested to work with ROS Kinetic, and VLP 16.  

### Requirements

* [LOAM](https://github.com/laboshinl/loam_velodyne)
* [costmap_2d](http://wiki.ros.org/costmap_2d)

### Running

Replace the default loam_velodyne.rviz inside LOAM with the provided rviz file for easier visualization.


In a new terminal run,
```
roslaunch velo_cmap.launch
```

Run [this](https://drive.google.com/open?id=1dk7jnQ3JgCvo3QpXLILL_BvSZM921COA) bagfile in another terminal,
```
rosbag play overtaking2.bag --clock
```


<div align="center">
<img src="https://github.com/karnikram/visual-servoing/blob/master/screen-grab.png"/>
</div>

