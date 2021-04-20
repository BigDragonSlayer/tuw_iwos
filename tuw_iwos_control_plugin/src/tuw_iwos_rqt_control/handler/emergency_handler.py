#!/usr/bin/env python

class EmergencyHandler:
    """
    class to handle emergency button and emergency state
    """
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
            self._enable_emergency_mode()
            return
        if self._emergency is True:
            self._disable_emergency_mode()
            return

    def _enable_emergency_mode(self):
        """
        enables emergency mode
        in emergency mode control elements are disabled
        :return:
        """
        self._emergency = True
        self._widget.emergency_stop.setStyleSheet('QPushButton {background-color: #C80000;}')
        self._plugin.enable_emergency_mode()
        return

    def _disable_emergency_mode(self):
        """
        disables emergency mode
        in emergency mode control elements are disabled
        :return:
        """
        self._emergency = False
        self._widget.emergency_stop.setStyleSheet('QPushButton {background-color: #FFFFFF;}')
        self._plugin.disable_emergency_mode()
        return
