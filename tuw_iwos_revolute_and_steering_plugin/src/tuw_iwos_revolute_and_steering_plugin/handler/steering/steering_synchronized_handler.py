#!/usr/bin/env python

from tuw_iwos_revolute_and_steering_plugin.tool.controller_tool import ControllerTool
from tuw_iwos_revolute_and_steering_plugin.tool.synchronized_controller import SynchronizedController


class SteeringSynchronizedHandler(SynchronizedController):
    """
    class holding and instantiating synchronized revolute controllers
    """
    def __init__(self, plugin, widget, unit, default_minimum, default_maximum):
        self._controller = ControllerTool(
            plugin=plugin,
            unit=unit,
            default_minimum=default_minimum,
            default_maximum=default_maximum,
            slider=widget.steering_control_synchronized_slider,
            decrease_button=widget.steering_control_synchronized_button_decrease,
            increase_button=widget.steering_control_synchronized_button_increase,
            reset_button=widget.steering_control_synchronized_button_reset,
            minimum=widget.steering_control_synchronized_value_minimum,
            maximum=widget.steering_control_synchronized_value_maximum,
            info=widget.steering_control_synchronized_value_current)

        super().__init__(controller=self._controller)
