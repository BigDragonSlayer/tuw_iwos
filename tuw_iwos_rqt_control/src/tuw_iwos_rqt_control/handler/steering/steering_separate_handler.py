#!/usr/bin/env python
from tuw_iwos_rqt_control.tool.controller_tool import ControllerTool
from tuw_iwos_rqt_control.tool.separate_controller import SeparateController


class SteeringSeparateHandler(SeparateController):

    def __init__(self, plugin, widget, unit, default_minimum, default_maximum):
        self._controller_left = ControllerTool(
            plugin=plugin,
            unit=unit,
            default_minimum=default_minimum,
            default_maximum=default_maximum,
            slider=widget.steering_control_separate_left_slider,
            decrease_button=widget.steering_control_separate_left_button_decrease,
            increase_button=widget.steering_control_separate_left_button_increase,
            reset_button=widget.steering_control_separate_button_left_reset,
            minimum=widget.steering_control_separate_left_value_minimum,
            maximum=widget.steering_control_separate_left_value_maximum,
            info=widget.steering_control_separate_left_value_current)

        self._controller_right = ControllerTool(
            plugin=plugin,
            unit=unit,
            default_minimum=default_minimum,
            default_maximum=default_maximum,
            slider=widget.steering_control_separate_right_slider,
            decrease_button=widget.steering_control_separate_right_button_decrease,
            increase_button=widget.steering_control_separate_right_button_increase,
            reset_button=widget.steering_control_separate_button_right_reset,
            minimum=widget.steering_control_separate_right_value_minimum,
            maximum=widget.steering_control_separate_right_value_maximum,
            info=widget.steering_control_separate_right_value_current)

        super().__init__(controller_left=self._controller_left, controller_right=self._controller_right)
