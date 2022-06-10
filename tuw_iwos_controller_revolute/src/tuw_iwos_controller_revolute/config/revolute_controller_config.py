#!/usr/bin/env python3

from tuw_iwos_controller_revolute.config.abstract_default_config import AbstractDefaultConfig
from tuw_iwos_controller_revolute.config.abstract_dynamic_config import AbstractDynamicConfig
from tuw_iwos_controller_revolute.config.config_file_reader import ConfigFileReader


class RevoluteControllerConfig(AbstractDefaultConfig, AbstractDynamicConfig):

    def __init__(self):
        self.sensor_steps = None
        self.max_velocity = None
        self.velocity_scale = None
        self.exchange_wheels = None
        self.reverse_left_wheel = None
        self.reverse_right_wheel = None

    def from_file(self, config_file_path):
        config_content = ConfigFileReader.get_config_from_file(config_file_path=config_file_path)

        self.sensor_steps = config_content["Sensor_Steps"]
        self.max_velocity = config_content['Max_Velocity']
        self.velocity_scale = config_content['Velocity_Scale']
        self.exchange_wheels = config_content['Exchange_Wheels']
        self.reverse_left_wheel = config_content['Reverse_Left_Wheel']
        self.reverse_right_wheel = config_content['Reverse_Right_Wheel']

        return self

    def to_dynamic_reconfigure(self):
        return {
            'sensor_steps': self.sensor_steps,
            'max_velocity': self.max_velocity,
            'velocity_scale': self.velocity_scale,
            'exchange_wheels': self.exchange_wheels,
            'reverse_left_wheel': self.reverse_left_wheel,
            'reverse_right_wheel': self.reverse_right_wheel
        }

    def from_dynamic_reconfigure(self, dynamic_config):
        self.sensor_steps = dynamic_config['sensor_steps']
        self.max_velocity = dynamic_config['max_velocity']
        self.velocity_scale = dynamic_config['velocity_scale']
        self.exchange_wheels = dynamic_config['exchange_wheels']
        self.reverse_left_wheel = dynamic_config['reverse_left_wheel']
        self.reverse_right_wheel = dynamic_config['reverse_right_wheel']
        return self
