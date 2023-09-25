# TUW IWOS
This is the main package for the ROS Noetic operation of the Independent Wheel Offset Steering robot prototype.
This prototype was created by Eugen Kaltenegger as part of the master thesis. Please check out the thesis for details on the Independent Wheel Offset Steering.

Packages outside of this repository that are required are:
- `tuw_geometry` (branch: `noetic`) 
- `tuw_msgs` (branch: `noetic`)
- `tuw_teleop` (branch: `master`)
- `tuw_sensor` (branch: `neotic`)
- `tuw_hardware_interface` (branch: `noetic`)

A Docker container containing this repository and all dependencies is available.
For more details please check out the `docker` repository of the `tuw-robotics` group.
In the folder `iwos/iwos-robot` more details on the docker container can be found.
Additionally, the setup in this repository may be applied to a local setup.

## Operating the Independent Wheel Offset Steering 
For more details on how to operate the Independent Wheel Offset Steering robot prototpye please check out the [ROBOT.md](./ROBOT.md) file.

## `tuw_iwos` - Metapackage
Metapackage for `tuw_iwos`.

## `tuw_iwos_hardware`
Package to operate the hardware of the Independent Wheel Offset Steering robot prototype.
This package depends on the `tuw_hardware_interface` package which utilizes the `ros_control` system for hardware operation.
For more details check [`tuw_iwos_hardware/README.md`](./tuw_iwos_hardware/README.md).

## `tuw_iwos_hardware_broker`
Package to distribute the joint specific command element of a [tuw_nav_msgs/JointsIWS](https://github.com/tuw-robotics/tuw_msgs/blob/master/tuw_nav_msgs/msg/JointsIWS.msg) message to the according topic for the `ros_controller` of the `ros_control` operating joint.
For more details check [`tuw_iwos_hardware_broker/README.md`](./tuw_iwos_hardware_broker/README.md).

## `tuw_iwos_launches`
Package containing launch files for the Independent Wheel Offset Steering robot prototype.
For more details check [`tuw_iwos_launches/README.md`](./tuw_iwos_launches/README.md).

## `tuw_iwos_motion_model`
Package containing the Motion Model and Odometry Motion Model for the Independent Wheel Offset Steering.
The algorithms are presented in the aforementioned master thesis.
For more details check [`tuw_iwos_motion_model/README.md`](./tuw_iwos_motion_model/README.md).

## `tuw_iwos_odometer`
Package containing an odometer approach for the Independent Wheel Offset Steering robot prototype.
For more details check [`tuw_iwos_motion_model/README.md`](./tuw_iwos_motion_model/README.md).

## `tuw_iwos_robot_description`
Package with the robot description (Unified Robot Description Format) of the Independent Wheel Offset Steering robot prototype.
For more details check [`tuw_iwos_robot_description`](./tuw_iwos_robot_description/README.md).

## `tuw_iwos_tools`
Package containing various tools for the other packages for the Independent Wheel Offset Steering robot prototype.
For more details check [`tuw_iwos_ros_control/README.md`](./tuw_iwos_ros_control/README.md).
