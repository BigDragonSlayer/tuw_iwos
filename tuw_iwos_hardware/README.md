# `tuw_iwos_ros_control`

## Description
Package to combine the RobotHW interface of [`ros_control`][ros_control] for the revolute joints and the steering joints of the IWOS robot.
For this reason this package utilizes the `tuw_hardware_interface_dynamixel` package to control the steering joints and `tuw_hardware_interface_trianmic` üackage to control the revolute joints.
The interface configuration can be found in `config/interface/interface.yaml`.
A controller from the [`ros_controllers`][ros_controllers] package is required for each of the joints.
These controllers are defined in `config/controller/controller.yaml`.

The main part of this package is the `tuw_iwos_hardware` node.
The setup is passed to this node as in form of custom `yaml` files.
For this reason the parameters `trinamic_hardware_interface_setup` and `trinamic_hardware_interface_setup` containing the path to the according files are required.
These files contain the paths to further custom `yaml` files holding hardware specific information and configuration.
The hardware information and configuration files for the Independent Wheel Offset Steering robot prototype are located in `resources/hardware/` (hardware information) and `resources/config/` (hardware configuration).

## Operation
In order to operate the hardware for the Independent Wheel Offset Steering robot prototype run the following command:
```bash
roslaunch tuw_iwos_hardware hardware.launch
```
This sets all necessary parameters for operation and starts all necessary nodes.
Note that the files `config/setup/steering_setup.yaml` and `config/setup/revolute_setup.yaml` may need some changes depending on the desired motor port and baudrate.
Additionally changes to the files `resources/hardware` and `resources/config` can be made to change properties of the hardware.

[ros_control]: https://wiki.ros.org/ros_control
[ros_controllers]: https://wiki.ros.org/ros_controllers