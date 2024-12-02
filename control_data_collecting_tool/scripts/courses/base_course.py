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
from params import Params


class Base_Course:
    def __init__(self, step: float, param_dict):
        self.step = step
        self.trajectory_points = None
        self.yaw = None
        self.parts = None
        self.curvature = None
        self.achievement_rates = None

        self.boundary_points = None
        self.boundary_yaw = None

        self.closed = True

        self.A = np.array([0.0, 0.0])
        self.B = np.array([0.0, 0.0])
        self.C = np.array([0.0, 0.0])
        self.D = np.array([0.0, 0.0])

        self.params = Params(param_dict)

    def get_trajectory_points(
        self,
        long_side_length: float,
        short_side_length: float,
        ego_point=np.array([0.0, 0.0]),
        goal_point=np.array([0.0, 0.0]),
    ):
        pass

    def choose_target_velocity_acc(self, collected_data_counts_of_vel_acc):
        min_num_data = 1e12
        min_data_num_margin = 20
        min_index_list = []

        for i in range(self.params.collecting_data_min_n_v, self.params.collecting_data_max_n_v):
            for j in range(
                self.params.collecting_data_min_n_a, self.params.collecting_data_max_n_a
            ):
                if min_num_data - min_data_num_margin > collected_data_counts_of_vel_acc[i, j]:
                    min_num_data = collected_data_counts_of_vel_acc[i, j]
                    min_index_list.clear()
                    min_index_list.append((j, i))

                elif min_num_data + min_data_num_margin > collected_data_counts_of_vel_acc[i, j]:
                    min_index_list.append((j, i))

        return min_index_list[np.random.randint(0, len(min_index_list))]

    def get_target_velocity(
        self,
        nearestIndex,
        current_time,
        current_vel,
        current_acc,
        collected_data_counts_of_vel_acc,
        collected_data_counts_of_vel_steer,
    ):
        pass

    def set_vertices(self, A, B, C, D):
        self.A = A
        self.B = B
        self.C = C
        self.D = D

    def get_boundary_points(self):
        pass

    def check_in_boundary(self, current_position):
        x, y = current_position[0], current_position[1]
        polygon = self.boundary_points
        wn = 0

        for i in range(len(polygon)):
            x1, y1 = polygon[i][0], polygon[i][1]
            x2, y2 = polygon[(i + 1) % len(polygon)][0], polygon[(i + 1) % len(polygon)][1]

            # Calculate if the point is to the left of the edge
            is_left = (x2 - x1) * (y - y1) - (x - x1) * (y2 - y1) > 0

            if y1 <= y < y2 and is_left:  # Upward crossing
                wn += 1
            elif y2 <= y < y1 and not is_left:  # Downward crossing
                wn -= 1

        return wn != 0

    def return_trajectory_points(self, yaw_offset, rectangle_center_position):
        rot_matrix = np.array(
            [
                [np.cos(yaw_offset), -np.sin(yaw_offset)],
                [np.sin(yaw_offset), np.cos(yaw_offset)],
            ]
        )

        trajectory_position_data = (rot_matrix @ self.trajectory_points.T).T
        trajectory_position_data += rectangle_center_position
        trajectory_yaw_data = self.yaw + yaw_offset

        return (
            trajectory_position_data,
            trajectory_yaw_data,
            self.curvature,
            self.parts,
            self.achievement_rates,
        )

    def update_trajectory_points(
        self,
        nearestIndex,
        yaw_offset,
        rectangle_center_position,
        collected_data_counts_of_vel_acc,
        collected_data_counts_of_vel_steer,
    ):
        return nearestIndex, *self.return_trajectory_points(yaw_offset, rectangle_center_position)
