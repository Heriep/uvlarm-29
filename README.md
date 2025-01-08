# Project UV-LARM Group OptimusPrime (29)

## Dependencies

### ROS

Install and configure ROS (Iron Irwini) : [ROS doc](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)

### Building tools

The building process rely on colcon and ament-cmake.

If not installed :
```bash
sudo apt install ament-cmake
sudo apt-get -y install colcon
```

### Simulation tools

The simulation uses Rviz2 and Gazebo, make sure to install both if you intent to use the simulation

## Setting up

Use the following to build the packages
```bash
colcon build
```

If you want to start the simulation environment :
```bash
ros2 launch grp_pibot29 simulation_launch.yaml
```

Or

If you want to start only the robot nodes :
```bash
ros2 launch grp_pibot29 tbot_launch.yaml
```