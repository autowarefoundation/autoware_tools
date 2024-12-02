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

from courses.base_course import Base_Course
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor


def computeTriangleArea(A, B, C):
    return 0.5 * abs(np.cross(B - A, C - A))


def declare_straight_line_negative_params(node):
    node.declare_parameter(
        "stopping_buffer_distance",
        10.0,
        ParameterDescriptor(description="The safety distance from end of the straight line [m]"),
    )


class Straight_Line_Negative(Base_Course):
    def __init__(self, step: float, param_dict):
        super().__init__(step, param_dict)
        self.closed = False

        self.target_vel_on_straight_line = 6.0
        self.target_acc_on_straight_line = 0.0
        self.vel_idx, self.acc_idx = 0, 0

        self.deceleration_rate = 1.0

        self.sine_period_for_velocity = 7.5

        self.stopping_buffer_distance = param_dict["stopping_buffer_distance"]

    def get_trajectory_points(
        self,
        long_side_length: float,
        short_side_length: float,
        ego_point=np.array([0.0, 0.0]),
        goal_point=np.array([0.0, 0.0]),
    ):
        total_distance = long_side_length
        t_array = np.arange(start=0.0, stop=total_distance, step=self.step).astype("float")

        self.yaw = np.zeros(len(t_array))
        self.parts = ["linear" for _ in range(len(t_array.copy()))]
        x = np.linspace(-total_distance / 2, total_distance / 2, len(t_array))
        y = np.zeros(len(t_array))

        self.trajectory_points = np.vstack((x, y)).T
        self.curvature = 1e-9 * np.ones(len(t_array))
        self.achievement_rates = np.linspace(0.0, 1.0, len(t_array))

        return self.trajectory_points, self.yaw, self.curvature, self.parts, self.achievement_rates

    def get_target_velocity(
        self,
        nearestIndex,
        current_time,
        current_vel,
        current_acc,
        collected_data_counts_of_vel_acc,
        collected_data_counts_of_vel_steer,
        mask_vel_acc,
        mask_vel_steer
    ):
        part = self.parts[nearestIndex]
        achievement_rate = self.achievement_rates[nearestIndex]
        acc_kp_of_pure_pursuit = self.params.acc_kp

        # Check and update target velocity on straight line
        if part == "straight" and achievement_rate < 0.05:
            self.acc_idx, self.vel_idx = self.choose_target_velocity_acc(
                collected_data_counts_of_vel_acc,
                mask_vel_acc
            )
            self.target_acc_on_straight_line = self.params.a_bin_centers[self.acc_idx]
            self.target_vel_on_straight_line = self.params.v_bin_centers[self.vel_idx]

            distance = len(self.parts) * self.step
            stop_distance = self.target_vel_on_straight_line**2 / (2 * self.params.a_max)
            self.deceleration_rate = (
                1.0 - stop_distance / distance - self.stopping_buffer_distance / distance
            )

        # Calculate sine wave and apply to velocity
        T = self.sine_period_for_velocity
        sine = np.sin(2 * np.pi * current_time / T) * np.sin(np.pi * current_time / T)

        if current_vel > self.target_vel_on_straight_line:
            target_vel = self.target_vel_on_straight_line + sine - 1.0
            target_vel = max(target_vel, 0.05)
        elif current_vel < self.target_vel_on_straight_line - 2.0 * abs(
            self.target_acc_on_straight_line
        ):
            target_vel = current_vel + self.params.a_max / acc_kp_of_pure_pursuit * (
                1.25 + 0.5 * sine
            )
        else:
            target_vel = current_vel + abs(
                self.target_acc_on_straight_line
            ) / acc_kp_of_pure_pursuit * (1.25 + 0.5 * sine)

        # Adjust for deceleration based on achievement rate
        if self.deceleration_rate - 0.05 <= achievement_rate < self.deceleration_rate:
            target_vel = current_vel - abs(
                self.target_acc_on_straight_line
            ) / acc_kp_of_pure_pursuit * (1.25 + 0.5 * sine)
        elif self.deceleration_rate <= achievement_rate:
            target_vel = (
                (current_vel - self.params.a_max / acc_kp_of_pure_pursuit)
                * (1.0 + 0.5 * sine)
                * (1.0 - achievement_rate)
            )

        return target_vel

    def get_boundary_points(self):
        self.boundary_points = np.vstack((self.A, self.B, self.C, self.D))

    def check_in_boundary(self, current_position):
        P = current_position[0:2]

        area_ABCD = computeTriangleArea(self.A, self.B, self.C) + computeTriangleArea(
            self.C, self.D, self.A
        )

        area_PAB = computeTriangleArea(P, self.A, self.B)
        area_PBC = computeTriangleArea(P, self.B, self.C)
        area_PCD = computeTriangleArea(P, self.C, self.D)
        area_PDA = computeTriangleArea(P, self.D, self.A)

        if area_PAB + area_PBC + area_PCD + area_PDA > area_ABCD * 1.001:
            return False
        else:
            return True
