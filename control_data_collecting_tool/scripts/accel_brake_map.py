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
import pandas as pd
from scipy.interpolate import RegularGridInterpolator


class MapConverter:
    def __init__(self, path_to_map):
        #
        map_df = pd.read_csv(path_to_map, delimiter=",", header=0)
        self.map_grid = map_df.to_numpy()[:, 1:]

        #
        self.velocity_grid_map = np.array(map_df.columns[1:], dtype=float)
        self.max_vel_of_map = self.velocity_grid_map[-1]
        self.min_vel_of_map = self.velocity_grid_map[0]

        #
        self.pedal_grid_map = map_df.to_numpy()[:, 0]
        self.max_pedal_of_map = self.pedal_grid_map[-1]
        self.min_pedal_of_map = self.pedal_grid_map[0]

        #
        self.map_grid_interpolator = RegularGridInterpolator(
            (self.pedal_grid_map, self.velocity_grid_map), self.map_grid
        )

    def pedal_to_accel_input(self, pedal, velocity):
        #
        pedal = np.clip(pedal, self.min_pedal_of_map, self.max_pedal_of_map)
        velocity = np.clip(velocity, self.min_vel_of_map, self.max_vel_of_map)
        accel_input = self.map_grid_interpolator([pedal, velocity])[0]

        #
        return accel_input

    def accel_input_to_pedal(self, accel_input, velocity):
        velocity = np.clip(velocity, self.min_vel_of_map, self.max_vel_of_map)

        vel_idx = 0
        alpha = 0.0
        for i in range(len(self.velocity_grid_map) - 1):
            vel_left_grid = self.velocity_grid_map[i]
            vel_right_grid = self.velocity_grid_map[i + 1]

            if vel_left_grid <= velocity <= vel_right_grid:
                vel_idx = i
                alpha = (velocity - vel_left_grid) / (vel_right_grid - vel_left_grid)

        accel_input_map = (
            alpha * self.map_grid[:, vel_idx] + (1 - alpha) * self.map_grid[:, vel_idx + 1]
        )

        min_accel_map = np.min([accel_input_map[0], accel_input_map[-1]])
        max_accel_map = np.max([accel_input_map[0], accel_input_map[-1]])
        accel_input = np.clip(accel_input, min_accel_map, max_accel_map)
        actuation_input = 0.0

        for i in range(len(accel_input_map) - 1):
            if accel_input_map[i] <= accel_input_map[i + 1]:
                if accel_input_map[i] <= accel_input and accel_input <= accel_input_map[i + 1]:
                    delta_accel_input = accel_input_map[i + 1] - accel_input_map[i]
                    if abs(delta_accel_input) < 1e-3:
                        actuation_input = self.pedal_grid_map[i]
                    else:
                        beta = (accel_input - accel_input_map[i]) / delta_accel_input
                        actuation_input = (
                            beta * self.pedal_grid_map[i + 1] + (1 - beta) * self.pedal_grid_map[i]
                        )

            elif accel_input_map[i + 1] < accel_input_map[i]:
                sign = -1
                if accel_input_map[i + 1] <= accel_input and accel_input <= accel_input_map[i]:
                    delta_accel_input = accel_input_map[i] - accel_input_map[i + 1]
                    if abs(delta_accel_input) < 1e-3:
                        actuation_input = sign * self.pedal_grid_map[i]
                    else:
                        beta = (accel_input - accel_input_map[i + 1]) / delta_accel_input
                        actuation_input = sign * (
                            beta * self.pedal_grid_map[i] + (1 - beta) * self.pedal_grid_map[i + 1]
                        )

        return actuation_input


class AccelBrakeMapConverter:
    def __init__(self, path_to_accel_map, path_to_brake_map):
        #
        self.accel_map_converter = MapConverter(path_to_accel_map)
        self.brake_map_converter = MapConverter(path_to_brake_map)

    def convert_accel_input_to_actuation_cmd(self, accel_input, velocity):
        #
        accel_actuation_cmd = self.accel_map_converter.accel_input_to_pedal(accel_input, velocity)
        brake_actuation_cmd = self.brake_map_converter.accel_input_to_pedal(accel_input, velocity)

        return accel_actuation_cmd - brake_actuation_cmd

    def convert_actuation_cmd_to_accel_input(self, actuation_cmd, velocity):
        #
        accel_input = 0.0
        if actuation_cmd > 0.0:
            accel_input = self.accel_map_converter.pedal_to_accel_input(
                abs(actuation_cmd), velocity
            )
        if actuation_cmd <= 0.0:
            accel_input = self.brake_map_converter.pedal_to_accel_input(
                abs(actuation_cmd), velocity
            )

        return accel_input
