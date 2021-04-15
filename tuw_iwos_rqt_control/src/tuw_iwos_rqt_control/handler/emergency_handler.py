#!/usr/bin/env python


class EmergencyHandler:

    def __init__(self, plugin, widget):
        self._plugin = plugin
        self._widget = widget
        self._widget.emergency_stop.clicked.connect(self._on_emergency_button)

        self._emergency = False

    def emergency(self):
        return self._emergency

    def _on_emergency_button(self):
        """
        callback for emergency button
        :return:
        """
        if self._emergency is False:
            self._start_emergency()
            return
        if self._emergency is True:
            self._end_emergency()
            return

    def _start_emergency(self):
        """
        sets emergency mode
        :return:
        """
        self._emergency = True
        self._widget.emergency_stop.setStyleSheet('QPushButton {background-color: #C80000;}')
        self._plugin.publish_emergency_message()
        return

    def _end_emergency(self):
        """
        unsets emergency mode
        :return:
        """
        self._emergency = False
        self._widget.emergency_stop.setStyleSheet('QPushButton {background-color: #FFFFFF;}')
        return
