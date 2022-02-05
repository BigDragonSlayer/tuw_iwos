#!/usr/bin/env python3

class SeparateController:
    """
    class to hold a separate controllers for left and right
    """
    def __init__(self, controller_left, controller_right):
        self._controller_left = controller_left
        self._controller_right = controller_right

    def fetch_values(self):
        return [self._controller_left.fetch_value(),
                self._controller_right.fetch_value()]

    def update_values(self, values):
        self._controller_left.update_value(values[0])
        self._controller_right.update_value(values[1])

    def enable(self):
        self._controller_left.enable()
        self._controller_right.enable()

    def disable(self):
        self._controller_left.disable()
        self._controller_right.disable()
