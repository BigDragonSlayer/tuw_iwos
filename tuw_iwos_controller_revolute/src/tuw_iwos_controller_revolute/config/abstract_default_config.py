#!/usr/bin/env python3

from abc import ABC
from abc import abstractmethod

from typing import TypeVar

# workaround for return self typehint
TypeAbstractDefaultConfig = TypeVar("TypeAbstractDefaultConfig", bound="AbstractDefaultConfig")


class AbstractDefaultConfig(ABC):

    @abstractmethod
    def from_file(self, config_file_path: str) -> TypeAbstractDefaultConfig:
        """
        set all config values based on config file (yaml)
        :param config_file_path: path to config file (yaml)
        :return: self
        """
        pass
