#!/usr/bin/env python3

# Copyright 2024 Proxima Technology Inc, TIER IV
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from autoware_adapi_v1_msgs.msg import OperationModeState
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


class DataCollectingBaseNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # common params
        self.declare_parameter(
            "NUM_BINS_V",
            10,
            ParameterDescriptor(description="Number of bins of velocity in heatmap"),
        )

        self.declare_parameter(
            "NUM_BINS_STEER",
            10,
            ParameterDescriptor(description="Number of bins of steer in heatmap"),
        )

        self.declare_parameter(
            "NUM_BINS_A",
            10,
            ParameterDescriptor(description="Number of bins of acceleration in heatmap"),
        )

        self.declare_parameter(
            "V_MIN",
            0.0,
            ParameterDescriptor(description="Minimum velocity in heatmap [m/s]"),
        )

        self.declare_parameter(
            "V_MAX",
            11.5,
            ParameterDescriptor(description="Maximum velocity in heatmap [m/s]"),
        )

        self.declare_parameter(
            "STEER_MIN",
            -1.0,
            ParameterDescriptor(description="Minimum steer in heatmap [rad]"),
        )

        self.declare_parameter(
            "STEER_MAX",
            1.0,
            ParameterDescriptor(description="Maximum steer in heatmap [rad]"),
        )

        self.declare_parameter(
            "A_MIN",
            -1.0,
            ParameterDescriptor(description="Minimum acceleration in heatmap [m/ss]"),
        )

        self.declare_parameter(
            "A_MAX",
            1.0,
            ParameterDescriptor(description="Maximum acceleration in heatmap [m/ss]"),
        )

        self.declare_parameter(
            "wheel_base",
            2.79,
            ParameterDescriptor(description="Wheel base [m]"),
        )

        self.sub_odometry_ = self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.onOdometry,
            1,
        )

        self.sub_acceleration_ = self.create_subscription(
            AccelWithCovarianceStamped,
            "/localization/acceleration",
            self.onAcceleration,
            1,
        )

        self.operation_mode_subscription_ = self.create_subscription(
            OperationModeState,
            "/system/operation_mode/state",
            self.subscribe_operation_mode,
            10,
        )
        self.operation_mode_subscription_

        self._present_kinematic_state = None
        self._present_acceleration = None
        self.present_operation_mode_ = None

        """
        velocity and acceleration grid
        velocity and steer grid
        """
        self.num_bins_v = self.get_parameter("NUM_BINS_V").get_parameter_value().integer_value
        self.num_bins_steer = (
            self.get_parameter("NUM_BINS_STEER").get_parameter_value().integer_value
        )
        self.num_bins_a = self.get_parameter("NUM_BINS_A").get_parameter_value().integer_value
        self.v_min, self.v_max = (
            self.get_parameter("V_MIN").get_parameter_value().double_value,
            self.get_parameter("V_MAX").get_parameter_value().double_value,
        )
        self.steer_min, self.steer_max = (
            self.get_parameter("STEER_MIN").get_parameter_value().double_value,
            self.get_parameter("STEER_MAX").get_parameter_value().double_value,
        )
        self.a_min, self.a_max = (
            self.get_parameter("A_MIN").get_parameter_value().double_value,
            self.get_parameter("A_MAX").get_parameter_value().double_value,
        )

        self.collected_data_counts_of_vel_acc = np.zeros(
            (self.num_bins_v, self.num_bins_a), dtype=np.int32
        )
        self.collected_data_counts_of_vel_steer = np.zeros(
            (self.num_bins_v, self.num_bins_steer), dtype=np.int32
        )

        self.v_bins = np.linspace(self.v_min, self.v_max, self.num_bins_v + 1)
        self.steer_bins = np.linspace(self.steer_min, self.steer_max, self.num_bins_steer + 1)
        self.a_bins = np.linspace(self.a_min, self.a_max, self.num_bins_a + 1)

        self.v_bin_centers = (self.v_bins[:-1] + self.v_bins[1:]) / 2
        self.steer_bin_centers = (self.steer_bins[:-1] + self.steer_bins[1:]) / 2
        self.a_bin_centers = (self.a_bins[:-1] + self.a_bins[1:]) / 2

    def onOdometry(self, msg):
        self._present_kinematic_state = msg

    def onAcceleration(self, msg):
        self._present_acceleration = msg

    def subscribe_operation_mode(self, msg):
        self.present_operation_mode_ = msg.mode
