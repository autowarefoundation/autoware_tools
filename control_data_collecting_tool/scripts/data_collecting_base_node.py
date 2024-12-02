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

import os

from ament_index_python.packages import get_package_share_directory
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_vehicle_msgs.msg import ControlModeReport
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
            "COURSE_NAME",
            "eight_course",
            ParameterDescriptor(
                description="Course name [`eight_course`, `u_shaped_return`, `straight_line_positive`, `straight_line_negative`, `reversal_loop_circle`, `along_road`]"
            ),
        )
        # set course name
        self.COURSE_NAME = self.get_parameter("COURSE_NAME").value

        self.declare_parameter(
            "MASK_NAME",
            "default",
            ParameterDescriptor(description="Masks for Data collection"),
        )

        self.declare_parameter(
            "wheel_base",
            2.79,
            ParameterDescriptor(description="Wheel base [m]"),
        )

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
            "NUM_BINS_STEER_RATE",
            5,
            ParameterDescriptor(description="Number of bins of steer in heatmap"),
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
            "STEER_RATE_MIN",
            0.0,
            ParameterDescriptor(description="Minimum steer in heatmap [rad]"),
        )

        self.declare_parameter(
            "STEER_RATE_MAX",
            0.3,
            ParameterDescriptor(description="Maximum steer in heatmap [rad]"),
        )

        self.ego_point = np.array([0.0, 0.0])
        self.goal_point = np.array([0.0, 0.0])

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

        self.control_mode_subscription_ = self.create_subscription(
            ControlModeReport,
            "/vehicle/status/control_mode",
            self.subscribe_control_mode,
            10,
        )

        self._present_kinematic_state = Odometry()
        self._present_acceleration = AccelWithCovarianceStamped()
        self.present_operation_mode_ = None
        self._present_control_mode_ = None

        """
        velocity and acceleration grid
        velocity and steer grid
        """
        self.num_bins_v = self.get_parameter("NUM_BINS_V").get_parameter_value().integer_value
        self.num_bins_steer = (
            self.get_parameter("NUM_BINS_STEER").get_parameter_value().integer_value
        )
        self.num_bins_a = self.get_parameter("NUM_BINS_A").get_parameter_value().integer_value
        self.num_bins_steer_rate = (
            self.get_parameter("NUM_BINS_STEER_RATE").get_parameter_value().integer_value
        )

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
        self.steer_rate_min, self.steer_rate_max = (
            self.get_parameter("STEER_RATE_MIN").get_parameter_value().double_value,
            self.get_parameter("STEER_RATE_MAX").get_parameter_value().double_value,
        )

        self.collected_data_counts_of_vel_acc = np.zeros(
            (self.num_bins_v, self.num_bins_a), dtype=np.int32
        )
        self.collected_data_counts_of_vel_steer = np.zeros(
            (self.num_bins_v, self.num_bins_steer), dtype=np.int32
        )
        self.collected_data_counts_of_vel_steer_rate = np.zeros(
            (self.num_bins_v, self.num_bins_steer_rate), dtype=np.int32
        )

        self.v_bins = np.linspace(self.v_min, self.v_max, self.num_bins_v + 1)
        self.steer_bins = np.linspace(self.steer_min, self.steer_max, self.num_bins_steer + 1)
        self.a_bins = np.linspace(self.a_min, self.a_max, self.num_bins_a + 1)
        self.steer_rate_bins = np.linspace(
            self.steer_rate_min, self.steer_rate_max, self.num_bins_steer_rate + 1
        )

        self.v_bin_centers = (self.v_bins[:-1] + self.v_bins[1:]) / 2
        self.steer_bin_centers = (self.steer_bins[:-1] + self.steer_bins[1:]) / 2
        self.a_bin_centers = (self.a_bins[:-1] + self.a_bins[1:]) / 2
        self.steer_rate_bin_centers = (self.steer_rate_bins[:-1] + self.steer_rate_bins[1:]) / 2

        """
        load mask (data collection range in heat map)
        """
        # set mask name
        MASK_NAME = self.get_parameter("MASK_NAME").value
        mask_directory_path = (
            get_package_share_directory("control_data_collecting_tool")
            + "/config/masks/"
            + MASK_NAME
        )

        mask_vel_acc_path = os.path.join(
            mask_directory_path, f"{MASK_NAME}_Velocity_Acceleration.txt"
        )
        self.mask_vel_acc = self.load_mask_from_txt(
            mask_vel_acc_path, self.num_bins_v, self.num_bins_a
        )

        mask_velocity_steering_path = os.path.join(
            mask_directory_path, f"{MASK_NAME}_Velocity_Steering.txt"
        )
        self.mask_vel_steer = self.load_mask_from_txt(
            mask_velocity_steering_path, self.num_bins_v, self.num_bins_steer
        )

        mask_velocity_steer_rate_path = os.path.join(
            mask_directory_path, f"{MASK_NAME}_Velocity_Steering_Rate.txt"
        )
        self.mask_vel_steer_rate = self.load_mask_from_txt(
            mask_velocity_steer_rate_path, self.num_bins_v, self.num_bins_steer_rate
        )

    def onOdometry(self, msg):
        self._present_kinematic_state = msg
        self.ego_point = np.array(
            [
                self._present_kinematic_state.pose.pose.position.x,
                self._present_kinematic_state.pose.pose.position.y,
            ]
        )

    def onAcceleration(self, msg):
        self._present_acceleration = msg

    def subscribe_operation_mode(self, msg):
        self.present_operation_mode_ = msg.mode

    def subscribe_control_mode(self, msg):
        self._present_control_mode_ = msg.mode

    def load_mask_from_txt(self, file_path, nx, ny):
        """
        Loads a numerical mask from a text file into a numpy array.

        Parameters:
        - file_path: Path to the text file to load.

        Returns:
        - A numpy array containing the loaded mask.
        """
        try:
            mask = np.loadtxt(file_path, dtype=int)
            if len(mask) == nx and len(mask[0]) == ny:
                return mask
        except Exception as e:
            pass

        return np.ones((nx, ny), dtype=int)
