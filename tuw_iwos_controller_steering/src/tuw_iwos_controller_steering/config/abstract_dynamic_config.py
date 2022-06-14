#!/usr/bin/env python3

from abc import ABC
from abc import abstractmethod

from typing import Dict
from typing import NewType
from typing import NoReturn
from typing import Union

DynamicReconfigureDict = NewType("DynamicReconfigureDict", Dict[str, Union[str, int, float]])


class AbstractDynamicConfig(ABC):

    @abstractmethod
    def to_dynamic_reconfigure(self):
        """
        convert config to dynamic reconfigure dict
        :return: dynamic reconfigure dict
        """
        pass

    @abstractmethod
    def from_dynamic_reconfigure(self, dynamic_reconfigure: DynamicReconfigureDict) -> NoReturn:
        """
        convert dynamic reconfigure dict to config
        :param dynamic_reconfigure: dynamic reconfigure dict
        :return: config instance
        """
        pass
