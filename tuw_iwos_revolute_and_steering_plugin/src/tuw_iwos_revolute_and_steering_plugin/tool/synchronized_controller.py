#!/usr/bin/env python

class SynchronizedController:
    """
    class to hold a combined controller for left and right
    """
    def __init__(self, controller):
        self._controlTool = controller

    def fetch_values(self):
        return [self._controlTool.fetch_value(),
                self._controlTool.fetch_value()]

    def update_values(self, values):
        average = (values[0] + values[1]) / 2
        self._controlTool.update_value(average)

    def enable(self):
        self._controlTool.enable()

    def disable(self):
        self._controlTool.disable()
