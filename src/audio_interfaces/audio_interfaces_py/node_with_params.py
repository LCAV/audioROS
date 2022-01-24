#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
node_with_params.py: Base class to use when we want to define parameters that can easily be set through a startup file. 
"""

from abc import ABC, abstractmethod

from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
import rclpy


class NodeWithParams(Node, ABC):
    @property
    @abstractmethod
    def PARAMS_DICT(self):
        pass

    def __init__(self, *args):

        # will call only Node.__init__, which exists. Otherwise
        # it would search for ABC.__init__
        super().__init__(
            *args,
            automatically_declare_parameters_from_overrides=True,
            allow_undeclared_parameters=True,
        )
        self.add_on_set_parameters_callback(self.set_params)

        # Store the current parameters in a way that is
        # easy to access.
        # self.PARAMS_DICT will not be affected and keeps the
        # initial values.
        self.current_params = self.PARAMS_DICT

        # fill parameters with default values, unless otherwise specified
        # by parameters file.
        params = self.get_parameters(self.PARAMS_DICT.keys())
        self.set_parameters(params)

    @staticmethod
    def verify_validity(params):
        return True

    def custom_set_params(self):
        """ Called after set_params """
        return

    def set_params(self, params):
        new_params = self.current_params.copy()
        for param in params:
            # we need this in case this parameter was not set at startup.
            # then we use the default values.
            if param.type_ == param.Type.NOT_SET:
                param = rclpy.parameter.Parameter(
                    param.name, value=self.PARAMS_DICT[param.name]
                )
            new_params[param.name] = param.value

        if self.verify_validity(new_params):
            self.current_params = new_params
            self.custom_set_params()
            return SetParametersResult(successful=True)
        else:
            return SetParametersResult(successful=False)
