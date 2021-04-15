#!/usr/bin/env python

import os
import rospkg
from tuw_iwos_rqt_control.handler.emergency_handler import EmergencyHandler
from tuw_iwos_rqt_control.handler.publisher_handler import PublisherHandler
from tuw_iwos_rqt_control.handler.revolute_handler import RevoluteHandler
from tuw_iwos_rqt_control.handler.steering_handler import SteeringHandler
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from tuw_nav_msgs.msg import JointsIWS


class IWOSControl(Plugin):

    def __init__(self, context):
        super(IWOSControl, self).__init__(context)
        # setup plugin
        self.setObjectName('IWOSPlugin')
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('tuw_iwos_rqt_control'), 'resource', 'tuw_iwos_rqt_control.ui')
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
        if self.emergency_handler.emergency() is False:
            message = JointsIWS()
            message.type_steering = "cmd_velocity"  # m/s
            message.type_revolute = "cmd_position"  # rad
            message.steering = self.steering_handler.fetch_values()
            message.revolute = self.revolute_handler.fetch_values()
            self.publisher_handler.publish(message)

    def publish_emergency_message(self):
        message = JointsIWS()
        message.type_steering = "cmd_velocity"  # m/s
        message.type_revolute = "cmd_torque"    # nm
        message.steering = [0.0, 0.0]
        message.revolute = [0.0, 0.0]
        self.publisher_handler.publish(message)

    def shutdown_plugin(self):
        self.publisher_handler.unregister_publisher()
        return
