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

### Machine learning tools

Install the necessary dependencies to run yolov5 (to run outside the rosproject)
```bash
git clone https://github.com/ultralytics/yolov5
pip install -r yolov5/requirements.txt
```
There might be a compatibility issue with openCV and two other modules but does not seem to affect the program

## Setting up for challenge 1

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

## Setting up for challenge 2

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
ros2 launch grp_pibot29_2 simulation_v2_launch.yaml
```
(make sure you have the tsim package in your ros workspace)

If you want to start only the robot nodes (with yolo vision):
```bash
ros2 launch grp_pibot29_2 tbot_v2_launch.yaml
```

If you want to start yolo vision:
```bash
ros2 launch grp_pibot29_2 vision_mask_launch.yaml
```

If you want to start yolo vision:
```bash
ros2 launch grp_pibot29_2 vision_yolo_launch.yaml
```

If you want to start the operator configuration:
```bash
ros2 launch grp_pibot29_2 operator_launch.yaml
```