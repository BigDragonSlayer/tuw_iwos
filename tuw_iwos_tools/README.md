# `tuw_iwos_tools`

## Description
This package contains the `tuw_iwos_tools` library.
This library is contains tools that are useful when creating applications designed for the Independent Wheel Offset Steering robot.
This includes for example a function to calculate the ICC (in robot coordinates) and a function to calculate the offset between facing and driving direction.

## Application
The tools of this package are designed to be utilized in other ROS packages.
For this reason these other packages need to depend on this package.
As an example check out the `tuw_iwos_odometer` package.
Keep an eye on the `CMakeLists.txt` and the `package.xml` files to see how to import the `tuw_iwos_tools` library.
The functions of the library itself are for example used in the `odometer_motor` and `odometer_sensor` classes of the `tuw_iwos_odometer` package.
