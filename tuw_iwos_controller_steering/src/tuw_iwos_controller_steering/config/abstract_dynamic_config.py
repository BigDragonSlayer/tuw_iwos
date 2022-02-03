#!/usr/bin/env python3

from abc import ABC, abstractmethod


class AbstractDynamicConfig(ABC):

    @abstractmethod
    def to_dynamic_reconfigure(self):
        """
        convert config to dynamic reconfigure dict
        :return: dynamic reconfigure dict
        """
        pass

    @abstractmethod
    def from_dynamic_reconfigure(self, dynamic_reconfigure):
        """
        convert dynamic reconfigure dict to config
        :param dynamic_reconfigure: dynamic reconfigure dict
        :return: config instance
        """
        pass
