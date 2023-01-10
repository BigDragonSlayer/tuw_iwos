# TUW IWOS

## `tuw_iwos` - Metapackage
Metapackage for `tuw_iwos`.

## `tuw_iwos_command_converter`
Package containing a converter for [`tuw_geometry_msgs/TwistWithOrientation`](https://github.com/tuw-robotics/tuw_msgs/blob/noetic/tuw_geometry_msgs/msg/TwistWithOrientation.msg) messages to [`tuw_nav_msgs/JointIWS`]((https://github.com/tuw-robotics/tuw_msgs/blob/master/tuw_nav_msgs/msg/JointsIWS.msg)) messages.

For more details check [`tuw_iwos_command_converter/README.md`](./tuw_iwos_command_converter/README.md)

## `tuw_iwos_launches`
Package containing launch files for the IWOS robot.

For more details check [`tuw_iwos_launches/README.md`](./tuw_iwos_launches/README.md).

## `tuw_iwos_robot_description`
Package with the robot description of the IWOS robot.

For more details check [`tuw_iwos_robot_description`](./tuw_iwos_robot_description/README.md).

## `tuw_iwos_ros_control`
Package to combine the RobotHW interface of `ros_control` for the revolute joints and the steering joints of the IWOS robot.

For more details check [`tuw_iwos_ros_control/README.md`](./tuw_iwos_ros_control/README.md).

## `tuw_iwos_ros_control_distributor`
Package to distribute the joint specific command element of a [tuw_nav_msgs/JointsIWS](https://github.com/tuw-robotics/tuw_msgs/blob/master/tuw_nav_msgs/msg/JointsIWS.msg) message to the according topic for the `ros_controller` of the `ros_control` operating joint.

For more details check [`tuw_iwos_ros_control_distributor/README.md`](./tuw_iwos_ros_control_distributor/README.md).
