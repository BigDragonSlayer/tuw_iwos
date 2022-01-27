#!/usr/bin/env python3

from tuw_iwos_controller_revolute.config.abstract_default_config import AbstractDefaultConfig
from tuw_iwos_controller_revolute.config.abstract_dynamic_config import AbstractDynamicConfig
from tuw_iwos_controller_revolute.config.config_file_reader import ConfigFileReader


class RevoluteConfig(AbstractDefaultConfig, AbstractDynamicConfig):

    def __init__(self):
        self.exchange_wheels = False
        self.reverse_left_wheel = False
        self.reverse_right_wheel = False

    def from_file(self, config_file_path):
        config_content = ConfigFileReader.get_config_from_file(config_file_path=config_file_path)

        self.exchange_wheels = config_content['Exchange_Wheels']
        self.reverse_left_wheel = config_content['Reverse_Left_Wheel']
        self.reverse_right_wheel = config_content['Reverse_Right_Wheel']

        return self

    def to_dynamic_reconfigure(self):
        return {
            'exchange_wheels': self.exchange_wheels,
            'reverse_left_wheel': self.reverse_left_wheel,
            'reverse_right_wheel': self.reverse_right_wheel
        }

    def from_dynamic_reconfigure(self, dynamic_config):
        self.exchange_wheels = dynamic_config['exchange_wheels']
        self.reverse_left_wheel = dynamic_config['reverse_left_wheel']
        self.reverse_right_wheel = dynamic_config['reverse_right_wheel']
        return self
