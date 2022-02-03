#!/usr/bin/env python3

import yaml

from tuw_iwos_controller_revolute.exception.invalid_file_exception import InvalidFileException
from tuw_iwos_controller_revolute.exception.invalid_path_exception import InvalidPathException


class ConfigFileReader:

    @staticmethod
    def open_config_file(config_file_path):
        yaml_file = open(file=config_file_path, mode='r')

        if yaml_file is None:
            raise InvalidPathException()
        else:
            return yaml_file

    @staticmethod
    def read_config_file(config_file):
        yaml_content = yaml.load(stream=config_file, Loader=yaml.FullLoader)

        if yaml_content is None:
            raise InvalidFileException()
        else:
            return yaml_content

    @staticmethod
    def get_config_from_file(config_file_path):
        config_file = ConfigFileReader.open_config_file(config_file_path=config_file_path)
        config_content = ConfigFileReader.read_config_file(config_file=config_file)
        return config_content
