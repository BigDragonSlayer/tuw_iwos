# `tuw_iwos_odometer`

## Description
This package contains an odometer utilizing joint state information and optionally IMU data.
For more details on the odometer calculation based on joint state information for the Independent Wheel Offset Steering please refer to the master thesis of Eugen Kaltenegger.

## Operation
This package contains an executable for an odometer subscribing to input and publishing odometer data as well an executable providing a service for odometer calculation.

To start the node publishing odometer data run:
```bash
rosrun tuw_iwos_odometer tuw_iwos_odometer_node
```
or 
```bash
roslaunch tuw_iwos_odometer odometer.launch
```

In order to start the odometer service node run:
```bash
rosrun tuw_iwos_odometer tuw_iwos_odometer_service_node
```
For more details on the service check out the service definitions in `srv`.