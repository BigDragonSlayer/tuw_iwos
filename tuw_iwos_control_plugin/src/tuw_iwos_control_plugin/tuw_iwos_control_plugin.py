#!/usr/bin/env python

import os
import rospkg
from tuw_iwos_control_plugin.handler.emergency_handler import EmergencyHandler
from tuw_iwos_control_plugin.handler.publisher_handler import PublisherHandler
from tuw_iwos_control_plugin.handler.revolute_handler import RevoluteHandler
from tuw_iwos_control_plugin.handler.steering_handler import SteeringHandler
from qt_gui.plugin import Plugin
from PyQt5.uic import loadUi
from PyQt5.QtWidgets import QWidget
from tuw_nav_msgs.msg import JointsIWS


class ControlPlugin(Plugin):
    """
    class to publish messages based on UI
    """
    def __init__(self, context):
        super(ControlPlugin, self).__init__(context)
        # setup plugin
        self.setObjectName('IWOSPlugin')
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('tuw_iwos_control_plugin'), 'resource', 'tuw_iwos_control_plugin.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('IWOSPluginUI')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        self.emergency_handler = EmergencyHandler(self, self._widget)
        self.publisher_handler = PublisherHandler(self, self._widget)
        self.revolute_handler = RevoluteHandler(self, self._widget)
        self.steering_handler = SteeringHandler(self, self._widget)
        return

    def publish_message(self):
        """
        creates and publishes a JointIWS message
        revolute and steering command are fetched from UI if emergency is disabled, or are zero if emergency is enabled
        :return:
        """
        message = JointsIWS()
        if self.emergency_handler.emergency() is False:
            message.type_revolute = "cmd_position"  # rad
            message.type_steering = "cmd_velocity"  # m/s
            message.revolute = self.revolute_handler.fetch_values()
            message.steering = self.steering_handler.fetch_values()

        if self.emergency_handler.emergency() is True:
            message.type_revolute = "cmd_torque"    # nm
            message.type_steering = "cmd_velocity"  # m/s
            message.revolute = [0.0, 0.0]
            message.steering = [0.0, 0.0]

        self.publisher_handler.publish(message)

    def enable_emergency_mode(self):
        """
        enables emergency mode
        in emergency mode emergency button turns red, other UI elements are disables, messages sent contain only zero
        :return:
        """
        self.publish_message()
        self.publisher_handler.disable()
        self.revolute_handler.disable()
        self.steering_handler.disable()

    def disable_emergency_mode(self):
        """
        disables emergency mode
        in emergency mode emergency button turns red, other UI elements are disables, messages sent contain only zero
        :return:
        """
        self.publisher_handler.enable()
        self.revolute_handler.enable()
        self.steering_handler.enable()

    def shutdown_plugin(self):
        self.publisher_handler.unregister_publisher()
        return
