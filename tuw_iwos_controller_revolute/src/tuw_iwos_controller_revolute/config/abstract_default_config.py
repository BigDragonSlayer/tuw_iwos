#!/usr/bin/env python3

from abc import ABC
from abc import abstractmethod

from typing import Self


class AbstractDefaultConfig(ABC):

    @abstractmethod
    def from_file(self, config_file_path: str) -> Self:
        """
        set all config values based on config file (yaml)
        :param config_file_path: path to config file (yaml)
        :return: self
        """
        pass
