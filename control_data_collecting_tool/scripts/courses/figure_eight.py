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


def computeTriangleArea(A, B, C):
    return 0.5 * abs(np.cross(B - A, C - A))


def declare_figure_eight_params(node):
    node.declare_parameter("smoothing_window", 400)
    node.declare_parameter("velocity_on_curve", 3.5)


class Figure_Eight(Base_Course):
    def __init__(self, step: float, param_dict):
        super().__init__(step, param_dict)

        self.window_size = param_dict["smoothing_window"]
        self.set_target_velocity_on_straight_line = False
        self.target_vel_on_straight_line = 6.0
        self.target_acc_on_straight_line = 0.0
        self.vel_idx, self.acc_idx = 0, 0
        self.previous_part = "curve"

        self.deceleration_rate = 1.0

        self.sine_period_for_velocity = 7.5
        self.velocity_on_curve = param_dict["velocity_on_curve"]

    def get_trajectory_points(
        self,
        long_side_length: float,
        short_side_length: float,
        ego_point=np.array([0.0, 0.0]),
        goal_point=np.array([0.0, 0.0]),
    ):
        a = short_side_length
        b = long_side_length

        C = [-(b / 2 - (1.0 - np.sqrt(3) / 2) * a), -a / 2]
        D = [(b / 2 - (1.0 - np.sqrt(3) / 2) * a), -a / 2]

        R = a  # radius of the circle
        OL = [-b / 2 + a, 0]  # center of the left circle
        OR = [b / 2 - a, 0]  # center of the right circle
        OB = np.sqrt(
            (b / 2 + (1.0 - np.sqrt(3) / 2) * a) ** 2 + (a / 2) ** 2
        )  # half length of the straight trajectory
        AD = 2 * OB
        BD = 2 * np.pi * R / 6  # the length of arc BD
        AC = BD
        CO = OB

        total_distance = 4 * OB + 2 * BD

        t_array = np.arange(start=0.0, stop=total_distance, step=self.step).astype("float")
        x = np.array([0.0 for i in range(len(t_array.copy()))])
        y = np.array([0.0 for i in range(len(t_array.copy()))])
        self.yaw = t_array.copy()
        self.curvature = t_array.copy()
        self.achievement_rates = t_array.copy()
        self.parts = ["part" for _ in range(len(t_array.copy()))]
        i_end = t_array.shape[0]

        for i, t in enumerate(t_array):
            if t > OB + BD + AD + AC + CO:
                i_end = i
                break

            if 0 <= t and t <= OB:
                x[i] = (b / 2 - (1.0 - np.sqrt(3) / 2) * a) * t / OB
                y[i] = a * t / (2 * OB)
                self.parts[i] = "straight_positive"
                self.achievement_rates[i] = t / (2 * OB) + 0.5

            if OB <= t and t <= OB + BD:
                t1 = t - OB
                t1_rad = t1 / R
                x[i] = OR[0] + R * np.cos(np.pi / 6 - t1_rad)
                y[i] = OR[1] + R * np.sin(np.pi / 6 - t1_rad)
                self.parts[i] = "curve"
                self.achievement_rates[i] = t1 / BD

            if OB + BD <= t and t <= OB + BD + AD:
                t2 = t - (OB + BD)
                x[i] = D[0] - (b / 2 - (1.0 - np.sqrt(3) / 2) * a) * t2 / OB
                y[i] = D[1] + a * t2 / (2 * OB)
                self.parts[i] = "straight"
                self.achievement_rates[i] = t2 / (2 * OB)

            if OB + BD + AD <= t and t <= OB + BD + AD + AC:
                t3 = t - (OB + BD + AD)
                t3_rad = t3 / R
                x[i] = OL[0] - R * np.cos(-np.pi / 6 + t3_rad)
                y[i] = OL[1] - R * np.sin(-np.pi / 6 + t3_rad)
                self.parts[i] = "curve"
                self.achievement_rates[i] = t3 / BD

            if OB + BD + AD + AC <= t and t <= OB + BD + AD + AC + CO:
                t4 = t - (OB + BD + AD + AC)
                x[i] = C[0] + (b / 2 - (1.0 - np.sqrt(3) / 2) * a) * t4 / OB
                y[i] = C[1] + a * t4 / (2 * OB)
                self.parts[i] = "straight_positive"
                self.achievement_rates[i] = t4 / (2 * OB)

        # drop rest
        x = x[:i_end]
        y = y[:i_end]
        self.parts = self.parts[:i_end]
        self.achievement_rates = self.achievement_rates[:i_end]

        x = np.concatenate((x, x))
        y = np.concatenate((y, y))

        self.parts = np.concatenate((self.parts, self.parts))
        self.achievement_rates = np.concatenate((self.achievement_rates, self.achievement_rates))
        window_size = self.window_size
        x = np.concatenate((x[-window_size // 2 :], x, x[: window_size // 2]))
        y = np.concatenate((y[-window_size // 2 :], y, y[: window_size // 2]))

        x_smoothed = np.convolve(x, np.ones(window_size) / window_size, mode="valid")[
            window_size // 2 : -window_size // 2
        ]
        y_smoothed = np.convolve(y, np.ones(window_size) / window_size, mode="valid")[
            window_size // 2 : -window_size // 2
        ]
        self.trajectory_points = 1.0 * np.array([x_smoothed, y_smoothed]).T

        dx = (x_smoothed[1:] - x_smoothed[:-1]) / self.step
        dy = (y_smoothed[1:] - y_smoothed[:-1]) / self.step

        ddx = (dx[1:] - dx[:-1]) / self.step
        ddy = (dy[1:] - dy[:-1]) / self.step

        self.yaw = np.arctan2(dy, dx)
        self.yaw = np.array(self.yaw.tolist() + [self.yaw[-1]])

        self.curvature = (
            1e-9 + abs(ddx * dy[:-1] - ddy * dx[:-1]) / (dx[:-1] ** 2 + dy[:-1] ** 2) ** 1.5
        )
        self.curvature = np.array(
            self.curvature.tolist() + [self.curvature[-2], self.curvature[-1]]
        )

        return self.trajectory_points, self.yaw, self.curvature, self.parts, self.achievement_rates

    def get_target_velocity(
        self,
        nearestIndex,
        current_time,
        current_vel,
        current_acc,
        collected_data_counts_of_vel_acc,
        collected_data_counts_of_vel_steer,
    ):
        part = self.parts[nearestIndex]
        achievement_rate = self.achievement_rates[nearestIndex]
        acc_kp_of_pure_pursuit = self.params.acc_kp

        # Check and update target velocity on straight line
        if (
            (part == "straight" and self.previous_part == "curve")
            or (part == "straight" and achievement_rate < 0.05)
        ) and not self.set_target_velocity_on_straight_line:
            self.acc_idx, self.vel_idx = self.choose_target_velocity_acc(
                collected_data_counts_of_vel_acc
            )
            self.target_acc_on_straight_line = self.params.a_bin_centers[self.acc_idx]
            self.target_vel_on_straight_line = self.params.v_bin_centers[self.vel_idx]

            i = 0
            while self.parts[i + nearestIndex] == "straight":
                i += 1

            distance = i * self.step
            stop_distance = self.target_vel_on_straight_line**2 / (2 * self.params.a_max)
            self.deceleration_rate = 1.0 - stop_distance / distance
            self.set_target_velocity_on_straight_line = True

        # Reset target velocity on line if entering a curve
        if part == "curve":
            self.set_target_velocity_on_straight_line = False

        self.previous_part = part

        # Calculate sine wave and apply to velocity
        T = self.sine_period_for_velocity
        sine = np.sin(2 * np.pi * current_time / T) * np.sin(np.pi * current_time / T)

        if current_vel > self.target_vel_on_straight_line:
            target_vel = self.target_vel_on_straight_line + sine + 1.5 * sine - 1.0
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
            target_vel = max(
                current_vel - self.params.a_max / acc_kp_of_pure_pursuit * (1.0 + 0.5 * sine),
                self.velocity_on_curve,
            )

        # Handle special conditions for curves or trajectory end
        if part == "curve":
            target_vel = self.velocity_on_curve

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
