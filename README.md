# `sst_local_planner`



This is a ROS Global Planner Plugin that implements the SST (Stable Sparse RRT) path planning algorithm.


## ROS Distros

Plugin tested in Noetic Distro.


## Install
Before using this package, you should install everthing needed for running a simulated nav enviroment in your computer. You can follow the tutorial of teb_local_planner to finish this.

http://wiki.ros.org/teb_local_planner/Tutorials

Put this package in your workspase and run

```
$ catkin_make
$ source devel/setup.bash
```
Check if the plugin "sst_star_local_planner" is successfully installed.
```
rospack plugins --attrib=plugin nav_core
```
You should see``sst_star_local_planner /home/wang/sim_nav/src/sst_star_local_planner/sst_star_planner_plugin.xml``




## Usage

You can use this plugin in move_base by adding this line in your launch file:

```
<!-- move_base.launch -->
<arg name="base_local_planner" default="sst_star_local_planner/SSTStarPlanner"/>
```

