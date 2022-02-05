#!/usr/bin/env python3

class ControllerTool:
    """
    class to operate a control unit
    a control unit consists:
    - slider
    - decrease and increase button
    - reset button
    - minimum and a maximum spinbox
    - info field
    """
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

        self._ui_elements = [self._slider,
                             self._decrease_button,
                             self._increase_button,
                             self._reset_button,
                             self._minimum,
                             self._maximum,
                             self._info]

        self._slider.valueChanged.connect(self._on_slider_drag)
        self._slider.sliderReleased.connect(self._on_slider_release)
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

    def _on_slider_drag(self):
        self._update_info()

    def _on_slider_release(self):
        self._publish_message()
        self._update_info()

    def _update_info(self):
        value = self._slider.value() / self.slider_factor
        unit = self._unit

        minimum = self._minimum.value()
        maximum = self._maximum.value()
        scale = (abs(maximum) + abs(minimum)) / 2
        if scale >= 1:
            self._info.setText('%0.2f %s' % (value, unit))
        if scale < 1:
            self._info.setText('%0.3f %s' % (value, unit))

    def _on_increase(self):
        self._slider.setValue(self._slider.value() + self._slider.singleStep())
        self._on_slider_release()

    def _on_decrease(self):
        self._slider.setValue(self._slider.value() - self._slider.singleStep())
        self._on_slider_release()

    def _on_reset(self):
        self._slider.setValue(0)
        self._on_slider_release()

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

    def enable(self):
        for element in self._ui_elements:
            element.setEnabled(True)

    def disable(self):
        for element in self._ui_elements:
            element.setEnabled(False)
