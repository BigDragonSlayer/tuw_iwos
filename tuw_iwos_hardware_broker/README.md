# `tuw_iwos_hardware_broker`

## Nodes

### `tuw_iwos_hardware_broker_node`

This node takes commands messages and splits them into a topic for each joint.
This is required for joint operation with [ros_controllers][ros_controllers] and [ros_control][ros_control].

**Subscriptions:**
- `iwos_command_hardware`

  Type: `tuw_nav_msgs::JointsIWS`

  Units
    - revolute: rad/s
    - steering: rad

**Publications:**
- `/hardware_command/revolute_command_left`
  
  Type: `std_msgs::Float64`

  Unit: rad/s

- `/hardware_command/revolute_command_right`

  Type: `std_msgs::Float64`

  Unit: rad/s


- `/hardware_command/steering_command_left`

  Type: `std_msgs::Float64`

  Unit: rad

- `/hardware_command/steering_command_right`

  Type: `std_msgs::Float64`

  Unit: rad

[ros_control]: https://wiki.ros.org/ros_control
[ros_controllers]: https://wiki.ros.org/ros_controllers