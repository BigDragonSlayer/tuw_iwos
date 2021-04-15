#!/usr/bin/env python


class ControllerTool:

    slider_factor = 1000
    number_of_slider_steps = 20

    def __init__(self, plugin, unit, default_minimum, default_maximum,
                 slider, decrease_button, increase_button, reset_button, minimum, maximum, info):
        self._plugin = plugin
        self._unit = unit

        self._slider = slider
        self._decrease_button = decrease_button
        self._increase_button = increase_button
        self._reset_button = reset_button
        self._minimum = minimum
        self._maximum = maximum
        self._info = info

        self._slider.valueChanged.connect(self._on_slider_change)
        self._decrease_button.clicked.connect(self._on_decrease)
        self._increase_button.clicked.connect(self._on_increase)
        self._reset_button.clicked.connect(self._on_reset)
        self._minimum.valueChanged.connect(self._on_minimum_change)
        self._maximum.valueChanged.connect(self._on_maximum_change)

        self._default(default_minimum=default_minimum, default_maximum=default_maximum)

    def fetch_value(self):
        return self._slider.value() / self.slider_factor

    def update_value(self, value):
        self._slider.setValue(value * self.slider_factor)

    def _default(self, default_minimum, default_maximum):
        self._minimum.setValue(default_minimum)
        self._maximum.setValue(default_maximum)
        self._update_info()

    def _publish_message(self):
        self._plugin.publish_message()

    def _on_slider_change(self):
        self._publish_message()
        self._update_info()

    def _update_info(self):
        value = self._slider.value() / self.slider_factor
        unit = self._unit

        minimum = self._minimum.value()
        maximum = self._maximum.value()
        scale = (abs(maximum) + abs(minimum)) / 2
        if scale > 1:
            self._info.setText('%0.2f %s' % (value, unit))
        if scale <= 1:
            self._info.setText('%0.3f %s' % (value, unit))

    def _on_increase(self):
        self._slider.setValue(self._slider.value() + self._slider.singleStep())

    def _on_decrease(self):
        self._slider.setValue(self._slider.value() - self._slider.singleStep())

    def _on_reset(self):
        if self._slider.value() != 0:
            self._slider.setValue(0)
            return
        if self._slider.value() == 0:
            self._publish_message()
            return

    def _on_minimum_change(self, value):
        self._slider.setMinimum(value)
        self._update_slider()

    def _on_maximum_change(self, value):
        self._slider.setMaximum(value)
        self._update_slider()

    def _update_slider(self):
        minimum = self._minimum.value()
        maximum = self._maximum.value()
        slider_range = abs(maximum) + abs(minimum)
        self._slider.setMinimum(minimum * self.slider_factor)
        self._slider.setMaximum(maximum * self.slider_factor)
        self._slider.setSingleStep(slider_range / self.number_of_slider_steps * self.slider_factor)
