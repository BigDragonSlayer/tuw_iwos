#!/usr/bin/env python

import rospy

from tuw_iwos_control_plugin.handler.revolute.revolute_separate_handler import RevoluteSeparateHandler
from tuw_iwos_control_plugin.handler.revolute.revolute_synchronized_handler import RevoluteSynchronizedHandler


class RevoluteHandler:
    """
    class to handle revolute controllers nested in tabs
    """
    unit = 'rad'
    default_limit = float(0.1)

    def __init__(self, plugin, widget):
        self._plugin = plugin
        self._widget = widget

        default_minimum = rospy.get_param(param_name="iwos/revolute_minimum", default=-RevoluteHandler.default_limit)
        default_maximum = rospy.get_param(param_name="iwos/revolute_maximum", default=RevoluteHandler.default_limit)

        self._revolute_separate_handler = RevoluteSeparateHandler(
            plugin=self._plugin,
            widget=self._widget,
            unit=RevoluteHandler.unit,
            default_minimum=default_minimum,
            default_maximum=default_maximum)
        self._revolute_synchronized_handler = RevoluteSynchronizedHandler(
            plugin=self._plugin,
            widget=self._widget,
            unit=RevoluteHandler.unit,
            default_minimum=default_minimum,
            default_maximum=default_maximum)

        self._widget.revolute_control_tab_widget.currentChanged.connect(self._on_tab_change)
        self._current_handler = self._current_widget()

    def _on_tab_change(self):
        current_values = self._current_handler.fetch_values()
        self._current_handler = self._current_widget()
        self._current_handler.update_values(current_values)

    def _current_widget(self):
        current_widget = self._widget.revolute_control_tab_widget.currentWidget()
        if current_widget == self._widget.revolute_control_separate_tab_widget:
            return self._revolute_separate_handler
        if current_widget == self._widget.revolute_control_synchronized_tab_widget:
            return self._revolute_synchronized_handler

    def fetch_values(self):
        return self._current_handler.fetch_values()

    def enable(self):
        self._revolute_separate_handler.enable()
        self._revolute_synchronized_handler.enable()

    def disable(self):
        self._revolute_separate_handler.disable()
        self._revolute_synchronized_handler.disable()
