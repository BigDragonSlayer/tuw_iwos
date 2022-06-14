#!/usr/bin/env python3

from typing import Optional

from tuw_iwos_controller_revolute.config.abstract_default_config import AbstractDefaultConfig
from tuw_iwos_controller_revolute.config.abstract_dynamic_config import AbstractDynamicConfig
from tuw_iwos_controller_revolute.config.config_file_reader import ConfigFileReader


class SteeringControllerConfig(AbstractDefaultConfig, AbstractDynamicConfig):

    def __init__(self):
        self.joint_offset: Optional[int] = None
        self.exchange_joints: Optional[bool] = None
        self.reverse_left_joint: Optional[bool] = None
        self.reverse_right_joint: Optional[bool] = None

    def from_file(self, config_file_path):
        config_content = ConfigFileReader.get_config_from_file(config_file_path=config_file_path)

        self.joint_offset = config_content['Joint_Offset']
        self.exchange_joints = config_content['Exchange_Joints']
        self.reverse_left_joint = config_content['Reverse_Left_Joint']
        self.reverse_right_joint = config_content['Reverse_Right_Joint']

        return self

    def to_dynamic_reconfigure(self):
        return {
            'joint_offset': self.joint_offset,
            'exchange_joints': self.exchange_joints,
            'reverse_left_joint': self.reverse_left_joint,
            'reverse_right_joint': self.reverse_right_joint
        }

    def from_dynamic_reconfigure(self, dynamic_config):
        self.joint_offset = dynamic_config['joint_offset']
        self.exchange_joints = dynamic_config['exchange_joints']
        self.reverse_left_joint = dynamic_config['reverse_left_joint']
        self.reverse_right_joint = dynamic_config['reverse_right_joint']
        return self
