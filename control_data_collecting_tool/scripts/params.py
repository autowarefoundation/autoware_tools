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

import numpy as np


class Params:
    def __init__(self, param_dict):
        self.acc_kp = param_dict["acc_kp"]
        self.max_lateral_accel = param_dict["max_lateral_accel"]

        """
        velocity and acceleration grid
        velocity and steer grid
        """
        self.num_bins_v = param_dict["NUM_BINS_V"]
        self.num_bins_steer = param_dict["NUM_BINS_STEER"]
        self.num_bins_a = param_dict["NUM_BINS_A"]
        self.v_min, self.v_max = (param_dict["V_MIN"], param_dict["V_MAX"])
        self.steer_min, self.steer_max = (param_dict["STEER_MIN"], param_dict["STEER_MAX"])
        self.a_min, self.a_max = (param_dict["A_MIN"], param_dict["A_MAX"])

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

        self.collecting_data_min_v, self.collecting_data_max_v = (
            param_dict["COLLECTING_DATA_V_MIN"],
            param_dict["COLLECTING_DATA_V_MAX"],
        )

        collecting_data_min_a, collecting_data_max_a = (
            param_dict["COLLECTING_DATA_A_MIN"],
            param_dict["COLLECTING_DATA_A_MAX"],
        )

        self.collecting_data_min_n_v = max(
            [np.digitize(self.collecting_data_min_v, self.v_bins) - 1, 0]
        )
        self.collecting_data_max_n_v = (
            min([np.digitize(self.collecting_data_max_v, self.v_bins) - 1, self.num_bins_v - 1]) + 1
        )

        self.collecting_data_min_n_a = max([np.digitize(collecting_data_min_a, self.a_bins) - 1, 0])
        self.collecting_data_max_n_a = (
            min([np.digitize(collecting_data_max_a, self.a_bins) - 1, self.num_bins_a - 1]) + 1
        )

        self.accel_pedal_input_min = param_dict["ACCEL_PEDAL_INPUT_MIN"]
        self.accel_pedal_input_max = param_dict["ACCEL_PEDAL_INPUT_MAX"]
        self.num_bins_accel_pedal_input = param_dict["NUM_BINS_ACCEL_PEDAL_INPUT"]
        self.accel_pedal_input_bins = np.linspace(
            self.accel_pedal_input_min,
            self.accel_pedal_input_max,
            self.num_bins_accel_pedal_input + 1,
        )
        self.accel_pedal_input_bin_centers = (
            self.accel_pedal_input_bins[:-1] + self.accel_pedal_input_bins[1:]
        ) / 2

        self.brake_pedal_input_min = param_dict["BRAKE_PEDAL_INPUT_MIN"]
        self.brake_pedal_input_max = param_dict["BRAKE_PEDAL_INPUT_MAX"]
        self.num_bins_brake_pedal_input = param_dict["NUM_BINS_BRAKE_PEDAL_INPUT"]
        self.brake_pedal_input_bins = np.linspace(
            self.brake_pedal_input_min,
            self.brake_pedal_input_max,
            self.num_bins_brake_pedal_input + 1,
        )
        self.brake_pedal_input_bin_centers = (
            self.brake_pedal_input_bins[:-1] + self.brake_pedal_input_bins[1:]
        ) / 2

        self.control_mode = param_dict["CONTROL_MODE"]
