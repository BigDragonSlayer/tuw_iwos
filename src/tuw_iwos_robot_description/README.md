# `tuw_iwos_robot_description`

## Description
Package with the robot description of the Independent Wheel Offset Steering robot prototype.
The robot description is reduced the most important components of the robot, which are the left and right fork as well as the left and right wheel.
The robot description is given in the [Unified Robot Description Format (URDF)](https://wiki.ros.org/urdf).
Additionally, this package also contains a launch file for the robot state publisher.

## Operation
It is recommended to start the robot description with the launch files of this package.
```bash
roslaunch tuw_iwos_robot_description robot_description_publisher.launch
```
If the robot is operated with `ros_control` the robot description should be started alongside with the robot state publisher.
This allows to publish the joint state for all joints operated with `ros_control` to the TF tree without any further configuration.
For this purpose start the according launch file with:
```bash
roslaunch tuw_iwos_robot_description robot_state_publisher.launch
```
In case it is necessary to make changes to the robot description the `robot_description_devel.launch` might be advantageous.
