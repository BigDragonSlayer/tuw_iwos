#!/usr/bin/env python3

from abc import ABC
from abc import abstractmethod

from typing import Dict
from typing import NewType
from typing import TypeVar
from typing import Union

# workaround for return self typehint
TypeAbstractDynamicConfig = TypeVar("TypeAbstractDynamicConfig", bound="AbstractDynamicConfig")
DynamicReconfigureDict = NewType("DynamicReconfigureDict", Dict[str, Union[bool, int, float, None]])


class AbstractDynamicConfig(ABC):

    @abstractmethod
    def to_dynamic_reconfigure(self) -> DynamicReconfigureDict:
        """
        convert config to dynamic reconfigure dict
        :return: dynamic reconfigure dict
        """
        pass

    @abstractmethod
    def from_dynamic_reconfigure(self, dynamic_reconfigure: DynamicReconfigureDict) -> TypeAbstractDynamicConfig:
        """
        convert dynamic reconfigure dict to config
        :param dynamic_reconfigure: dynamic reconfigure dict
        :return: config instance
        """
        pass
