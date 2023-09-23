# `tuw_iwos_hardware_broker`

## Description
The main part of this package is the `tuw_iwos_hardware_broker_node`.
This node takes commands messages as input and splits them.
The command for each joint is then published for the according topic for each joint as an output.
This is required for joint operation with [`ros_controllers`][ros_controllers] and [`ros_control`][ros_control].

**Subscriptions:**
- `iwos_command_hardware`

  Type: `tuw_nav_msgs::JointsIWS`

  Units
    - revolute: m/s
    - steering: rad

**Publications:**
- `/hardware_command/revolute_command_left`
  
  Type: `std_msgs::Float64`

  Unit: m/s

- `/hardware_command/revolute_command_right`

  Type: `std_msgs::Float64`

  Unit: m/s


- `/hardware_command/steering_command_left`

  Type: `std_msgs::Float64`

  Unit: rad

- `/hardware_command/steering_command_right`

  Type: `std_msgs::Float64`

  Unit: rad

## Operation

In order to start the `tuw_iwos_hardware_broker_node` run the following command:
```bash
roslaunch tuw_iwos_hardware_broker tuw_iwos_hardware_broker_node
```
or 
```bash
rosrun tuw_iwos_hardware_broker tuw_iwos_hardware_broker_node
```
Note that some remappings might be necessary depending on the topic names.

[ros_control]: https://wiki.ros.org/ros_control
[ros_controllers]: https://wiki.ros.org/ros_controllers
