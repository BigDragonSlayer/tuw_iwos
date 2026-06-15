# `tuw_iwos_launches`

## Description
This package contains various launch files for the Independent Wheel Offset Steering robot prototype.

## Operation
The most important launch file in this repository can be run with:
```bash
roslaunch tuw_iwos_launches robot.launch
```

This launch file takes several parameters:
- `imu` (bool - default=`true`): decide whether IMU should be started or not
- `imu_filter` (bool - default=`false`): decide whether IMU filter should be started or not
- `laser` (bool - default=`true`): decide whether laser scanner should be started or not
- `gamepad` (bool - default=`true`): decide whether gamepad for motion commands should be started or not
- `odometer` (bool - default=`true`): decide whether odometer data should be published or not
- `robot_localization` (bool - default=`false`): decide whether odometer data should be enhanced with the `robot_localization` package or not
- `rviz` (bool - default=`false`): decide whether odometer should be started or not
