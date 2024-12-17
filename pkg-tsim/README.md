# MobiSyst - TSim

This project is a ROS package that includes usefull elements for IMT Robot simulation (turtlebot2, ...).

Last system version: **Ubuntu 22.04 lts** / **ROS2 iron** 


## Installation

IMT MobiSyst simulation relies on _Gazebo Simulator_.
To notice the existance of an [install-gazebo](https://bitbucket.org/imt-mobisyst/mb6-space/src/master/bin) scrip in `mb6-space` workspace repository.

Considering that _gazebo_ is installed. there is nothong more to do, you can colcon build...


# Get started

tbot_sim packages propose some launch file configured for turtlebot2 robot.

For instance : 

```sh
ros2 launch tbot_sim challenge-1.launch.py
```
