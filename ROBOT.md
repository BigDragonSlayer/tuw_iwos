# Independent Wheel Offset Steering Prototype

This document is a guide on how to operate the Independent Wheel Offset Steering Prototype.

## Starting

The Robot requires two lead batteries, each with 12V (white Tamiya connector) connected in parallel resulting in a output of 24V (yellow XT60 connector).
Make sure the lead batteries are fully charged, connect them in parallel and then connect them to the robot.
Additionally, a XTPower battery provides power to the Intel NUC and the router on the robot.
Make sure the battery is fully charged and connect the Intel NUC to the DC output (black wire, both sides barrel) and connect the router to the 5V/2.1A output (whilte cable USB-a to USB-c).

Now that everything is supplied with power, it is time to start the Intel NUC (small round power button on the top).
The router will provide two networks (`roblab-iwos` and `roblab-iwos-5g`).
Both networks should provide internet access since the router is connecting to the `roblab` network.
Additionally, both networks grant access to the Intel NUC mounted to the robot.
The Intel NUC gets the IP address `192.168.8.210` assigned.
Connect to either `roblab-iwos` or `roblab-iwos-5g`, you can request the password from your supervisor.
Now you can connect to the robot via SSH, again you can request the username and the password from your supervisor.

_Note:_ the router is accessible on `http://192.168.8.1`.

When connected to the robot navigate to `~/projects/docker/iwos/iwos-robot/noetic`.
The next step is to start the docker container with all components required to run the robot.
For this purpose please check the details in the `README.md` in the [`tuw-robotics/docker` repository](https://github.com/tuw-robotics/docker/tree/master/iwos/iwos-robot) in the folder `iwos/iwos-robot`.

To get started fast you can run `make docker_run` which will start the required container (if present on the robot).
There you will be asked for a password which is `password` (what a secure password for this container).
Now start all the nodes to operate the robot with:
```bash
roslaunch tuw_iwos_launches robot.launch
```
This requires the controller (Logitech F710), the laser scanner and of course all actuators to be connected to the Intel NUC.

_Note:_ there may be some warnings or errors in that container regarding TF frames, these can be ignored.
Any warnings pointing out "invalid mode for IWOS" can also be ignored for now.
These simply point out that no ICC can be found with the current joint configuration.
Other warnings and errors should not be ignored.

## Troubleshooting

Some common mistakes might be:
- *The robot can not be controlled with the Logitech F710:*
  - Check that the left or right should button is pressed when using the joysticks.
  - Check that the switch on the back of the controller is set to "X".
  - Check that the correct receiver is attached to the Intel NUC.
- *The network is visible but the robot can not be accessed (or pinged):*
  - Check that you turned on the Intel NUC on the robot.
  - Check that the Intel NUC is connected via wire to the router on a lan port.
- *The network is visible but has no internet access:*
  - Check the `roblab` network and check that `roblab-iwos` is connected to `roblab`.
- *The actuators operate not as intended*
  - Check that all controller boards (Dynamixel U2D2 and Trinamic TCMC-1640) are powered and that no connection to the motors came loose.
  - Check for broken wires and connectors (especially for the sensor connectors of the TCMC-1640 to the wheels)
- *There are no sensor reading from the laser scanner*
  - Check that the laser scanner is connected via wire to the router on a lan port
  - Make sure the Intel NUC can communicate with the laser scanner.
    This requires a connection with an IP address of `192.168.0.15` for the Intel NUC and the gateway at `192.168.0.1`.
    