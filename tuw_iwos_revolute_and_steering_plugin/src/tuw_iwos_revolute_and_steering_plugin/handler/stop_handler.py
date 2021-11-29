#!/usr/bin/env python3

class StopHandler:
    """
    class to handle stop button and stop state
    """
    def __init__(self, plugin, widget):
        self._plugin = plugin
        self._widget = widget
        self._widget.stop.clicked.connect(self._on_stop_button)

        self._stop = False

    def stop(self):
        return self._stop

    def _on_stop_button(self):
        """
        callback for stop button
        :return:
        """
        if self._stop is False:
            self._enable_stop_mode()
            return
        if self._stop is True:
            self._disable_stop_mode()
            return

    def _enable_stop_mode(self):
        """
        enables stop mode
        in stop mode control elements are disabled
        :return:
        """
        self._stop = True
        self._widget.stop.setStyleSheet('QPushButton {background-color: #C80000;}')
        self._plugin.enable_stop_mode()
        return

    def _disable_stop_mode(self):
        """
        disables stop mode
        in stop mode control elements are disabled
        :return:
        """
        self._stop = False
        self._widget.stop.setStyleSheet('QPushButton {background-color: #FFFFFF;}')
        self._plugin.disable_stop_mode()
        return
