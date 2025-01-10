# Project UV-LARM Group OptimusPrime (29)

## Dependencies

### ROS

Install and configure ROS (Iron Irwini) : [ROS doc](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)

### Python

Install python 3 : 
```bash
sudo apt install python3
```

### Camera drivers

Install realsense drivers using vcpkg
```bash
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
./vcpkg integrate install
./vcpkg install realsense2
```

Install the python module using pip
```bash
pip install pyrealsense2
```

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

Don't forget to source the setup file if it is not included in your .bashrc file
```bash
source /opt/ros/iron/setup.bash
```

If you want to start the simulation environment :
```bash
ros2 launch grp_pibot29 simulation_launch.yaml
```
(make sure you have the tsim package in your ros workspace)

Or

If you want to start only the robot nodes :
```bash
ros2 launch grp_pibot29 tbot_launch.yaml
```