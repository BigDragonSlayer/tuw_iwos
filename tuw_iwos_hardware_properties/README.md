# `tuw_iwos_hardware_properties`

This is a package to manage hardware properties of an independent wheel steering robot on a high level.

## Nodes

### `tuw_iwos_hardware_properties_node`

This node takes command messages and limits revolute and steering values and converts velocities from meters per second to rad per second.
The limits for the revolute and steering as well as the parameters required for velocity conversation are configurable (via `rqt_reconfigure`).

**Subscriptions:**
- `iwos_command`

  Type: `tuw_nav_msgs::JointsIWS`

  Units
    - revolute: m/s
    - steering: rad

**Publications:**
- `iwos_command_hardware`

  Type: `tuw_nav_msgs::JointsIWS`

  Units
    - revolute: rad/s
    - steering: rad
