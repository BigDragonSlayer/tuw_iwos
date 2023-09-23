# `tuw_iwos_motion_model`

## Description
This package contains a node with a motion model for the Independent Wheel Offset Steering.
This motion model is based on the motion model by Sebastian Thrun designed for the Differential Drive.
For more details on the Motion Model from Sebastian Thrun please refer to the Book: Probabilistic Robotics by Sebastian Thrun, Wolfram Brugard and Dieter Fox.
For more details on the Motion Model for the Independent Wheel Offset Steering please refer to the master thesis of Eugen Kaltenegger.

## Operation
This package contains the `tuw_iwos_motion_model_service_node` node.
In order to start this node run:
```bash
rosrun tuw_iwos_motion_model tuw_iwos_motion_model_service_node
```
This node provides a service which takes input as the according algorith and provides either a probability or a sample drawn from the probability distribution.
For more details check out the service definitions in `srv` and the aforementioned Master Thesis.
