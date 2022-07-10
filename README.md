# my_controller Repository

This repository is a demo ROS package for UR3e with two demo nodes: a publisher nodes which publishes some joint positions which will be executed by the (fake) robot and a subscriber nodes which prints the joint states of the robot on the console. We assume you have followed the [ROS2 Foxy Tutorials](https://docs.ros.org/en/foxy/Tutorials.html) up to [writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) (and ideally also [Launch](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-Main.html)).

## Setup
We assume you have followed the [Installation Guide](https://github.com/opallch/ur3e_environment/wiki/Installation-Guide).
1. In `~/cocobots_ws/src`, clone this repository:
```bash
git clone https://github.com/opallch/my_controller.git
``` 
2. Build the package:
```bash
cd ~/cocobots_ws
colcon build --packages-select my_controller
``` 
3. On one terminal:
- If you are connected to a real robot:
```bash
ros2 launch ur_bringup ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.4 launch_rviz:=true reverse_ip:=<your IP address> limited:=true
```
- else:
```
ros2 launch ur_bringup ur_control.launch.py ur_type:=ur3e robot_ip:=yyy.yyy.yyy.yyy launch_rviz:=true use_fake_hardware:=true limited:=true
```

4. Launch the nodes in the `my_controller` package on another terminal:
```
ros2 launch my_controller my_controller.launch.py
```
Now, you should see the ur in rviz (as well as the real ur, if connected) moving and joint states being printed on the terminal.

5. (optional) If you want to see the connection between the running nodes, on a new terminal run:
```bash
rqt
```

## Package Structure
If your current location is the root of package:
- All self-written nodes are located in `my_controller/`
- Dependencies of the nodes should be included in `package.xml`
- The joint positions and other parameters used by `my_controller/publisher_joint_trajectory_controller.py` are defined in `config/publisher_config.yaml`.
- In `setup.py`, there are two important things to be defined:
    - entry points of your executables aka. nodes
    - files needed to be installed for running the nodes#
- Launch files can be found in `launch/`. There is only one launch file `my_controller.launch.py` comes with the repository.