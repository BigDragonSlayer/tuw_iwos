#!/usr/bin/env python


class SeparateController:

    def __init__(self, controller_left, controller_right):
        self._controller_left = controller_left
        self._controller_right = controller_right

    def fetch_values(self):
        return [self._controller_left.fetch_value(),
                self._controller_right.fetch_value()]

    def update_values(self, values):
        self._controller_left.update_value(values[0])
        self._controller_right.update_value(values[1])
