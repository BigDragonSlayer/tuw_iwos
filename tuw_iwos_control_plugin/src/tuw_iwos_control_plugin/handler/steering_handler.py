#!/usr/bin/env python

import rospy

from tuw_iwos_control_plugin.handler.steering.steering_separate_handler import SteeringSeparateHandler
from tuw_iwos_control_plugin.handler.steering.steering_synchronized_handler import SteeringSynchronizedHandler


class SteeringHandler:
    """
    class to handle steering controllers nested in tabs
    """
    unit = 'm/s'
    default_limit = float(1.0)

    def __init__(self, plugin, widget):
        self._plugin = plugin
        self._widget = widget

        default_minimum = rospy.get_param(param_name="iwos/steering_minimum", default=-SteeringHandler.default_limit)
        default_maximum = rospy.get_param(param_name="iwos/steering_maximum", default=SteeringHandler.default_limit)

        self._steering_separate_handler = SteeringSeparateHandler(
            plugin=self._plugin,
            widget=self._widget,
            unit=SteeringHandler.unit,
            default_minimum=default_minimum,
            default_maximum=default_maximum)
        self._steering_synchronized_handler = SteeringSynchronizedHandler(
            plugin=self._plugin,
            widget=self._widget,
            unit=SteeringHandler.unit,
            default_minimum=default_minimum,
            default_maximum=default_maximum)

        self._widget.steering_control_tab_widget.currentChanged.connect(self._on_tab_change)
        self._current_handler = self._current_widget()

    def _on_tab_change(self):
        current_values = self._current_handler.fetch_values()
        self._current_handler = self._current_widget()
        self._current_handler.update_values(current_values)

    def _current_widget(self):
        current_widget = self._widget.steering_control_tab_widget.currentWidget()
        if current_widget == self._widget.steering_control_separate_tab_widget:
            return self._steering_separate_handler
        if current_widget == self._widget.steering_control_synchronized_tab_widget:
            return self._steering_synchronized_handler

    def fetch_values(self):
        return self._current_handler.fetch_values()

    def enable(self):
        self._steering_separate_handler.enable()
        self._steering_synchronized_handler.enable()

    def disable(self):
        self._steering_separate_handler.disable()
        self._steering_synchronized_handler.disable()
