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

from autoware_planning_msgs.msg import Trajectory
from autoware_planning_msgs.msg import TrajectoryPoint
from geometry_msgs.msg import AccelWithCovarianceStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PolygonStamped
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import numpy as np
from numpy import arctan
from numpy import cos
from numpy import pi
from numpy import sin
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
import seaborn as sns
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

debug_matplotlib_plot_flag = True
Differential_Smoothing_Flag = True
USE_CURVATURE_RADIUS_FLAG = False

#COURSE_NAME = "eight_course"
#COURSE_NAME = "u_shaped_return"
#COURSE_NAME = "straight_line_positive"
COURSE_NAME = "straight_line_negative"

def smooth_bounding(upper: np.ndarray, threshold: np.ndarray, x: np.ndarray):
    result = np.zeros(x.shape)
    for i in range(x.shape[0]):
        if x[i] <= threshold[i]:
            result[i] = x[i]
        else:
            z = np.exp(-(x[i] - threshold[i]) / (upper[i] - threshold[i]))
            result[i] = upper[i] * (1 - z) + threshold[i] * z
    return result


def getYaw(orientation_xyzw):
    return R.from_quat(orientation_xyzw.reshape(-1, 4)).as_euler("xyz")[:, 2]


def get_eight_course_trajectory_points(
    long_side_length: float, short_side_length: float, step: float, total_distance: float
):
    a = short_side_length
    b = long_side_length

    # Boundary points between circular and linear trajectory
    C = [-(b / 2 - (1.0 - np.sqrt(3) / 2) * a), -a / 2]
    D = [(b / 2 - (1.0 - np.sqrt(3) / 2) * a), -a / 2]

    R = a  # radius of the circle
    OL = [-b / 2 + a, 0]  # center of the left circle
    OR = [b / 2 - a, 0]  # center of the right circle
    OB = np.sqrt(
        (b / 2 + (1.0 - np.sqrt(3) / 2) * a) ** 2 + (a / 2) ** 2
    )  # half length of the linear trajectory
    AD = 2 * OB
    θB = np.arctan(
        a / 2 / (b / 2 + (1.0 - np.sqrt(3) / 2) * a)
    )  # Angle that OB makes with respect to x-axis
    BD = 2 * np.pi * R / 6  # the length of arc BD
    AC = BD
    CO = OB

    total_distance = 4 * OB + 2 * BD

    t_array = np.arange(start=0.0, stop=total_distance, step=step).astype("float")
    x = [0.0 for i in range(len(t_array.copy()))]
    y = [0.0 for i in range(len(t_array.copy()))]
    yaw = t_array.copy()
    curve = t_array.copy()
    achievement_rates = t_array.copy()
    parts = ["part" for _ in range(len(t_array.copy()))]
    i_end = t_array.shape[0]

    for i, t in enumerate(t_array):
        if t > OB + BD + AD + AC + CO:
            i_end = i
            break

        if 0 <= t and t <= OB:
            print(t)
            x[i] = (b / 2 - (1.0 - np.sqrt(3) / 2) * a) * t / OB
            y[i] = a * t / (2 * OB)
            yaw[i] = θB
            curve[i] = 1e-10
            parts[i] = "linear_positive"
            achievement_rates[i] = t / (2 * OB) + 0.5

        if OB <= t and t <= OB + BD:
            t1 = t - OB
            t1_rad = t1 / R
            x[i] = OR[0] + R * np.cos(np.pi / 6 - t1_rad)
            y[i] = OR[1] + R * np.sin(np.pi / 6 - t1_rad)
            yaw[i] = -t1_rad
            curve[i] = 1 / R
            parts[i] = "right_circle"
            achievement_rates[i] = t1 / BD

        if OB + BD <= t and t <= OB + BD + AD:
            t2 = t - (OB + BD)
            x[i] = D[0] - (b / 2 - (1.0 - np.sqrt(3) / 2) * a) * t2 / OB
            y[i] = D[1] + a * t2 / (2 * OB)
            yaw[i] = np.pi - θB
            curve[i] = 1e-10

            parts[i] = "linear_negative"
            achievement_rates[i] = t2 / (2 * OB)

        if OB + BD + AD <= t and t <= OB + BD + AD + AC:
            t3 = t - (OB + BD + AD)
            t3_rad = t3 / R
            x[i] = OL[0] - R * np.cos(-np.pi / 6 + t3_rad)
            y[i] = OL[1] - R * np.sin(-np.pi / 6 + t3_rad)
            yaw[i] = np.pi + t3_rad
            curve[i] = 1 / R

            parts[i] = "left_circle"
            achievement_rates[i] = t3 / BD

        if OB + BD + AD + AC <= t and t <= OB + BD + AD + AC + CO:
            t4 = t - (OB + BD + AD + AC)
            x[i] = C[0] + (b / 2 - (1.0 - np.sqrt(3) / 2) * a) * t4 / OB
            y[i] = C[1] + a * t4 / (2 * OB)
            yaw[i] = θB
            curve[i] = 1e-10

            parts[i] = "linear_positive"
            achievement_rates[i] = t4 / (2 * OB)

    # drop rest
    x = x[:i_end]
    y = y[:i_end]
    yaw = yaw[:i_end]
    curve = curve[:i_end]
    parts = parts[:i_end]
    achievement_rates = achievement_rates[:i_end]

    if USE_CURVATURE_RADIUS_FLAG:
        return np.array([x, y]).T, yaw, 1 / curve[:i_end], parts, achievement_rates
    else:
        return np.array([x, y]).T, yaw, curve[:i_end], parts, achievement_rates

def get_straight_line_course_trajectory_points(
    long_side_length: float, short_side_length: float, step: float
):
    
    a = short_side_length
    b = long_side_length

    total_distance = b

    t_array = np.arange(start=0.0, stop=total_distance, step=step).astype("float")

    if COURSE_NAME == "straight_line_positive":
        yaw = np.zeros( len(t_array) )
        parts = ["linear_positive" for _ in range(len(t_array.copy()))]
        x = np.linspace( -total_distance/2, total_distance/2, len(t_array))
        y = np.zeros( len(t_array) )

    elif COURSE_NAME == "straight_line_negative":
        yaw = pi * np.ones( len(t_array) )
        parts = ["linear_negative" for _ in range(len(t_array.copy()))]
        x = np.linspace( total_distance/2, -total_distance/2, len(t_array))
        y = np.zeros( len(t_array) )

    curve =  1e-9 * np.ones(len(t_array))
    achievement_rates = np.linspace( 0.0, 1.0 ,len(t_array))

    if USE_CURVATURE_RADIUS_FLAG:
        return np.vstack((x, y)).T, yaw, 1 / curve, parts, achievement_rates
    else:
        return np.vstack((x, y)).T, yaw, curve, parts, achievement_rates
    

def get_u_shaped_return_course_trajectory_points(
    long_side_length: float, short_side_length: float, step: float, total_distance: float
):
    a = short_side_length
    b = long_side_length

    t_array = np.arange(start=0.0, stop=total_distance, step=step).astype("float")
    x = t_array.copy()
    y = t_array.copy()
    yaw = t_array.copy()

    # Boundary points between circular and linear trajectory
    A = [-(b - a) / 2, a / 2]
    B = [(b - a) / 2, a / 2]
    C = [-(b - a) / 2, -a / 2]
    D = [(b - a) / 2, -a / 2]

    # _O = [0.0, 0.0]  # origin
    R = a / 2  # radius of the circle
    OL = [-(b - a) / 2, 0]  # center of the left circle
    OR = [(b - a) / 2, 0]  # center of the right circle

    AB = (b - a)
    arc_BD = pi * R
    DC = (b - a)
    arc_CA = pi * R
    
    total_distance = 2 * AB + 2 * np.pi * R

    t_array = np.arange(start=0.0, stop=total_distance, step=step).astype("float")
    x = [0.0 for i in range(len(t_array.copy()))]
    y = [0.0 for i in range(len(t_array.copy()))]
    yaw = t_array.copy()
    curve = t_array.copy()
    achievement_rates = t_array.copy()
    parts = ["part" for _ in range(len(t_array.copy()))]
    i_end = t_array.shape[0]

    for i, t in enumerate(t_array):

        if t > AB + arc_BD + DC + arc_CA:
            i_end = i
            break

        if 0 <= t and t <= AB:
            section_rate = t / AB
            x[i] = section_rate * B[0] + (1-section_rate) * A[0]
            y[i] = section_rate * B[1] + (1-section_rate) * A[1]
            yaw[i] = 0.0
            curve[i] = 1e-10
            parts[i] = "linear_positive"
            achievement_rates[i] = section_rate

        if AB <= t and t <= AB + arc_BD:
            section_rate = (t - AB) / arc_BD
            x[i] = OR[0] + R * cos( pi/2 - pi * section_rate)
            y[i] = OR[1] + R * sin( pi/2 - pi * section_rate)
            yaw[i] = pi/2 - pi * section_rate - pi/2
            curve[i] = 1.0 / R
            parts[i] = "right_circle"
            achievement_rates[i] = section_rate


        if AB + arc_BD <= t and t <= AB + arc_BD + DC:
            section_rate = (t - AB - arc_BD)/ DC
            x[i] = section_rate * C[0] + (1-section_rate) * D[0]
            y[i] = section_rate * C[1] + (1-section_rate) * D[1]
            yaw[i] = pi
            curve[i] = 1e-10
            parts[i] = "linear_negative"
            achievement_rates[i] = section_rate
            

        if AB + arc_BD + DC <= t and t <= AB + arc_BD + DC + arc_CA:
            section_rate = (t - AB - arc_BD - DC)/ arc_CA
            x[i] = OL[0] + R * cos( 3*pi/2 - pi * section_rate)
            y[i] = OL[1] + R * sin( 3*pi/2 - pi * section_rate)
            yaw[i] = 3*pi/2 - pi * section_rate  - pi/2
            curve[i] = 1.0 / R
            parts[i] = "left_circle"
            achievement_rates[i] = section_rate

    # drop rest
    x = x[:i_end]
    y = y[:i_end]
    yaw = yaw[:i_end]
    curve = curve[:i_end]
    parts = parts[:i_end]
    achievement_rates = achievement_rates[:i_end]

    if USE_CURVATURE_RADIUS_FLAG:
        return np.array([x, y]).T, yaw, 1 / curve[:i_end], parts, achievement_rates
    else:
        return np.array([x, y]).T, yaw, curve[:i_end], parts, achievement_rates

class DataCollectingTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("data_collecting_trajectory_publisher")

        # velocity and acceleration grid
        # velocity and steer grid
        self.num_bins = 10
        self.v_min, self.v_max = 0, 11.2
        self.steer_min, self.steer_max = -1.0, 1.0
        self.a_min, self.a_max = -1.5, 1.5

        self.collected_data_counts_of_vel_acc = np.zeros((self.num_bins, self.num_bins))
        self.collected_data_counts_of_vel_steer = np.zeros((self.num_bins, self.num_bins))

        self.v_bins = np.linspace(self.v_min, self.v_max, self.num_bins + 1)
        self.steer_bins = np.linspace(self.steer_min, self.steer_max, self.num_bins + 1)
        self.a_bins = np.linspace(self.a_min, self.a_max, self.num_bins + 1)

        self.v_bin_centers = (self.v_bins[:-1] + self.v_bins[1:]) / 2
        self.steer_bin_centers = (self.steer_bins[:-1] + self.steer_bins[1:]) / 2
        self.a_bin_centers = (self.a_bins[:-1] + self.a_bins[1:]) / 2

        self.fig, self.axs = plt.subplots(4, 1, figsize=(12, 20))
        self.grid_update_time_interval = 5.0
        self.last_grid_update_time = None
        plt.ion()

        self.trajectory_parts = None
        self.trajectory_achievement_rates = None

        self.prev_part = "left_circle"

        self.acc_idx = 0
        self.vel_idx = 0

        self.target_acc_on_line = 0.0
        self.target_vel_on_line = 0.0

        self.on_line_vel_flag = True

        self.previouse_target_vel = 2.0
        self.deceleration_rate = 0.6

        self.vel_hist = np.zeros(50)
        self.acc_hist = np.zeros(50)

        self.declare_parameter(
            "wheel_base",
            2.79,  # sample_vehicle_launch/sample_vehicle_description/config/vehicle_info.param.yaml
            ParameterDescriptor(description="Wheel base [m]"),
        )

        self.declare_parameter(
            "acc_kp",
            1.0,
            ParameterDescriptor(description="Pure pursuit accel command proportional gain"),
        )

        self.declare_parameter(
            "max_lateral_accel",
            0.5,
            ParameterDescriptor(description="Max lateral acceleration limit [m/ss]"),
        )

        self.declare_parameter(
            "lateral_error_threshold",
            5.0,
            ParameterDescriptor(
                description="Lateral error threshold where applying velocity limit [m/s]"
            ),
        )

        self.declare_parameter(
            "yaw_error_threshold",
            0.75,
            ParameterDescriptor(
                description="Yaw error threshold where applying velocity limit [rad]"
            ),
        )

        self.declare_parameter(
            "velocity_limit_by_tracking_error",
            1.0,
            ParameterDescriptor(
                description="Velocity limit when tracking error exceeds threshold [m/s]"
            ),
        )

        self.declare_parameter(
            "mov_ave_window",
            50,
            ParameterDescriptor(description="Moving average smoothing window size"),
        )

        self.declare_parameter(
            "target_longitudinal_velocity",
            6.0,
            ParameterDescriptor(description="Target longitudinal velocity [m/s]"),
        )

        self.declare_parameter(
            "longitudinal_velocity_noise_amp",
            0.0,
            ParameterDescriptor(
                description="Target longitudinal velocity additional sine noise amplitude [m/s]"
            ),
        )

        self.declare_parameter(
            "longitudinal_velocity_noise_min_period",
            5.0,
            ParameterDescriptor(
                description="Target longitudinal velocity additional sine noise minimum period [s]"
            ),
        )

        self.declare_parameter(
            "longitudinal_velocity_noise_max_period",
            20.0,
            ParameterDescriptor(
                description="Target longitudinal velocity additional sine noise maximum period [s]"
            ),
        )

        self.trajectory_for_collecting_data_pub_ = self.create_publisher(
            Trajectory,
            "/data_collecting_trajectory",
            1,
        )

        self.data_collecting_trajectory_marker_array_pub_ = self.create_publisher(
            MarkerArray,
            "/data_collecting_trajectory_marker_array",
            1,
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

        self.sub_data_collecting_area_ = self.create_subscription(
            PolygonStamped,
            "/data_collecting_area",
            self.onDataCollectingArea,
            1,
        )
        self.sub_data_collecting_area_

        self.timer_period_callback = 0.03  # 30ms
        self.traj_step = 0.1

        self.timer = self.create_timer(self.timer_period_callback, self.timer_callback)

        self._present_kinematic_state = None
        self._present_acceleration = None

        self.trajectory_position_data = None
        self.trajectory_yaw_data = None
        self.trajectory_longitudinal_velocity_data = None
        self.trajectory_curvature_data = None
        self.current_target_longitudinal_velocity = (
            self.get_parameter("target_longitudinal_velocity").get_parameter_value().double_value
        )
        self.current_window = (
            self.get_parameter("mov_ave_window").get_parameter_value().integer_value
        )

        self.one_round_progress_rate = None

        self.vel_noise_list = []

    def onOdometry(self, msg):
        self._present_kinematic_state = msg

    def onAcceleration(self, msg):
        self._present_acceleration = msg

    def onDataCollectingArea(self, msg):
        self._data_collecting_area_polygon = msg
        self.updateNominalTargetTrajectory()

    def get_target_velocity(self,nearestIndex):

            part = self.trajectory_parts[
                        nearestIndex
                    ]  # "left_circle", "right_circle", "linear_positive", "linear_negative"
            achievement_rate = self.trajectory_achievement_rates[nearestIndex]
            current_vel = self._present_kinematic_state.twist.twist.linear.x
            current_acc = self._present_acceleration.accel.accel.linear.x

            acc_kp_of_pure_pursuit = self.get_parameter("acc_kp").get_parameter_value().double_value
            N = self.num_bins

            self.acc_hist[:-1] = 1.0 * self.acc_hist[1:]
            self.acc_hist[-1] = current_acc
            self.vel_hist[:-1] = 1.0 * self.vel_hist[1:]
            self.vel_hist[-1] = current_vel

            max_lateral_accel = (
                self.get_parameter("max_lateral_accel").get_parameter_value().double_value
            )
            if USE_CURVATURE_RADIUS_FLAG:
                max_vel_from_lateral_acc = np.sqrt(
                    max_lateral_accel * self.trajectory_curvature_data[nearestIndex]
                )
            else:
                max_vel_from_lateral_acc = np.sqrt(
                    max_lateral_accel / self.trajectory_curvature_data[nearestIndex]
                )

            target_vel = min([self.v_max / N, max_vel_from_lateral_acc])

            # set self.target_acc_on_line and self.target_vel_on_line after vehicle from circle part to linear part
            min_data_num_margin = 5
            min_index_list = []
            if (self.prev_part == "left_circle" or self.prev_part == "right_circle") and (
                part == "linear_positive" or part == "linear_negative"
            ):
                self.on_line_vel_flag = True
                min_num_data = 1e12

                # do not collect data when velocity and acceleration are low
                exclude_idx_list = [(0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (0, 2)]
                # do not collect data when velocity and acceleration are high
                exclude_idx_list += [
                    (-1 + N, -1 + N),
                    (-2 + N, -1 + N),
                    (-3 + N, -1 + N),
                    (-1 + N, -2 + N),
                    (-2 + N, -2 + N),
                    (-1 + N, -3 + N),
                ]

                for i in range(0, N):
                    for j in range(0, N):
                        if (i, j) not in exclude_idx_list:
                            if (
                                min_num_data - min_data_num_margin
                                > self.collected_data_counts_of_vel_acc[i, j]
                            ):
                                min_num_data = self.collected_data_counts_of_vel_acc[i, j]
                                min_index_list.clear()
                                min_index_list.append((j, i))

                            elif (
                                min_num_data + min_data_num_margin
                                > self.collected_data_counts_of_vel_acc[i, j]
                            ):
                                min_index_list.append((j, i))

                self.acc_idx, self.vel_idx = min_index_list[
                    np.random.randint(0, len(min_index_list))
                ]
                self.target_acc_on_line = self.a_bin_centers[self.acc_idx]
                self.target_vel_on_line = self.v_bin_centers[self.vel_idx]

                if self.target_vel_on_line > self.v_max * 3.0 / 4.0:
                    self.deceleration_rate = 0.55
                elif self.target_vel_on_line > self.v_max / 2.0:
                    self.deceleration_rate = 0.65
                else:
                    self.deceleration_rate = 0.85

            # set target velocity on linear part
            if part == "linear_positive" or part == "linear_negative":
                if (
                    current_vel > self.target_vel_on_line - self.v_max / N / 8.0
                    and self.target_vel_on_line >= self.v_max / 2.0
                ):
                    self.on_line_vel_flag = False
                elif (
                    abs(current_vel - self.target_vel_on_line) < self.v_max / N / 4.0
                    and self.target_vel_on_line < self.v_max / 2.0
                ):
                    self.on_line_vel_flag = False

                # accelerate until vehicle reaches target_vel_on_line
                if 0.0 <= achievement_rate and achievement_rate < 0.45 and self.on_line_vel_flag:
                    target_vel = self.target_vel_on_line

                    if (
                        current_vel > self.target_vel_on_line - self.v_max / N * 0.5
                        and self.target_acc_on_line > 2.0 * self.a_max / N
                    ):
                        target_vel = current_vel + self.target_acc_on_line / acc_kp_of_pure_pursuit

                # collect target_acceleration data when current velcity is close to target_vel_on_line
                elif (
                    achievement_rate < self.deceleration_rate
                    or self.target_vel_on_line < self.v_max / 2.0
                ):
                    if self.collected_data_counts_of_vel_acc[self.vel_idx, self.acc_idx] > 50:
                        self.acc_idx = np.argmin(
                            self.collected_data_counts_of_vel_acc[self.vel_idx, :]
                        )
                        self.target_acc_on_line = self.a_bin_centers[self.acc_idx]

                    if (
                        current_vel
                        < max([self.target_vel_on_line - 1.0 * self.v_max / N, self.v_max / N / 2.0])
                        and self.target_acc_on_line < 0.0
                    ):
                        self.acc_idx = np.argmin(
                            self.collected_data_counts_of_vel_acc[self.vel_idx, int(N / 2.0) : N]
                        ) + int(N / 2)
                        self.target_acc_on_line = self.a_bin_centers[self.acc_idx]

                    elif (
                        current_vel > self.target_vel_on_line + 1.0 * self.v_max / N
                        and self.target_acc_on_line > 0.0
                    ):
                        self.acc_idx = np.argmin(
                            self.collected_data_counts_of_vel_acc[self.vel_idx, 0 : int(N / 2.0)]
                        )
                        self.target_acc_on_line = self.a_bin_centers[self.acc_idx]

                    target_vel = current_vel + self.target_acc_on_line / acc_kp_of_pure_pursuit

                # deceleration
                if (
                    self.deceleration_rate <= achievement_rate
                ):
                    if COURSE_NAME == "eight_course" or COURSE_NAME == "u_shaped_return":
                        target_vel = np.min([self.v_max / N, max_vel_from_lateral_acc / 2.0])
                    elif COURSE_NAME == "straight_line_positive" or COURSE_NAME == "straight_line_negative":
                        target_vel = 0.0

            # set target velocity on circle part
            if part == "left_circle" or part == "right_circle":
                if achievement_rate < 0.10 and self.target_vel_on_line > self.v_max / 2.0:
                    target_vel = np.min([self.v_max / N, max_vel_from_lateral_acc / 2.0])
                elif achievement_rate < 0.50:
                    target_vel = max_vel_from_lateral_acc / 2.0
                else:
                    target_vel = max_vel_from_lateral_acc

            self.prev_part = part

            return target_vel

    def updateNominalTargetTrajectory(self):
        data_collecting_area = np.array(
            [
                np.array(
                    [
                        self._data_collecting_area_polygon.polygon.points[i].x,
                        self._data_collecting_area_polygon.polygon.points[i].y,
                        self._data_collecting_area_polygon.polygon.points[i].z,
                    ]
                )
                for i in range(4)
            ]
        )

        # [1] compute an approximate rectangle
        l1 = np.sqrt(((data_collecting_area[0, :2] - data_collecting_area[1, :2]) ** 2).sum())
        l2 = np.sqrt(((data_collecting_area[1, :2] - data_collecting_area[2, :2]) ** 2).sum())
        l3 = np.sqrt(((data_collecting_area[2, :2] - data_collecting_area[3, :2]) ** 2).sum())
        l4 = np.sqrt(((data_collecting_area[3, :2] - data_collecting_area[0, :2]) ** 2).sum())
        la = (l1 + l3) / 2
        lb = (l2 + l4) / 2
        if np.abs(la - lb) < 1e-6:
            la += 0.1  # long_side_length must not be equal to short_side_length
        ld = np.sqrt(la**2 + lb**2)
        rectangle_center_position = np.zeros(2)
        for i in range(4):
            rectangle_center_position[0] += data_collecting_area[i, 0] / 4.0
            rectangle_center_position[1] += data_collecting_area[i, 1] / 4.0

        vec_from_center_to_point0_data = data_collecting_area[0, :2] - rectangle_center_position
        vec_from_center_to_point1_data = data_collecting_area[1, :2] - rectangle_center_position
        unit_vec_from_center_to_point0_data = vec_from_center_to_point0_data / (
            np.sqrt((vec_from_center_to_point0_data**2).sum()) + 1e-10
        )
        unit_vec_from_center_to_point1_data = vec_from_center_to_point1_data / (
            np.sqrt((vec_from_center_to_point1_data**2).sum()) + 1e-10
        )

        # [2] compute whole trajectory
        # [2-1] generate figure eight path
        if la > lb:
            long_side_length = la
            short_side_length = lb
            vec_long_side = (
                -unit_vec_from_center_to_point0_data + unit_vec_from_center_to_point1_data
            )
        else:
            long_side_length = lb
            short_side_length = la
            vec_long_side = (
                unit_vec_from_center_to_point0_data + unit_vec_from_center_to_point1_data
            )
        unit_vec_long_side = vec_long_side / np.sqrt((vec_long_side**2).sum())
        if unit_vec_long_side[1] < 0:
            unit_vec_long_side *= -1
        yaw_offset = np.arccos(unit_vec_long_side[0])
        if yaw_offset > pi / 2:
            yaw_offset -= pi

        long_side_margin = 5
        long_side_margin = 5
        total_distance = ld * (1 + np.pi) * 2

        actual_long_side = max(long_side_length - long_side_margin, 1.1)
        actual_short_side = max(short_side_length - long_side_margin, 1.0)

        if COURSE_NAME == "eight_course":
            (
                trajectory_position_data,
                trajectory_yaw_data,
                trajectory_curvature_data,
                self.trajectory_parts,
                self.trajectory_achievement_rates,
            ) = get_eight_course_trajectory_points(
                actual_long_side,
                actual_short_side,
                self.traj_step,
                total_distance,
            )
        
        elif COURSE_NAME == "straight_line_positive" or COURSE_NAME == "straight_line_negative":
            (
                trajectory_position_data,
                trajectory_yaw_data,
                trajectory_curvature_data,
                self.trajectory_parts,
                self.trajectory_achievement_rates,
            ) = get_straight_line_course_trajectory_points(
                actual_long_side,
                actual_short_side,
                self.traj_step,
            )
        
        elif COURSE_NAME == "u_shaped_return":
            (
                trajectory_position_data,
                trajectory_yaw_data,
                trajectory_curvature_data,
                self.trajectory_parts,
                self.trajectory_achievement_rates,
            ) = get_u_shaped_return_course_trajectory_points(
                actual_long_side,
                actual_short_side,
                self.traj_step,
                total_distance,
            )

        else:
            self.trajectory_position_data = None
            self.trajectory_yaw_data = None
            self.trajectory_longitudinal_velocity_data = None
            self.trajectory_curvature_data = None
        

        for i in range(len(trajectory_yaw_data)):
            if trajectory_yaw_data[i] > np.pi:
                trajectory_yaw_data[i] -= 2 * np.pi
            if trajectory_yaw_data[i] < -np.pi:
                trajectory_yaw_data[i] += 2 * np.pi

        # [2-2] translation and rotation of origin
        rot_matrix = np.array(
            [
                [np.cos(yaw_offset), -np.sin(yaw_offset)],
                [np.sin(yaw_offset), np.cos(yaw_offset)],
            ]
        )
        trajectory_position_data = (rot_matrix @ trajectory_position_data.T).T
        trajectory_position_data += rectangle_center_position
        trajectory_yaw_data += yaw_offset

        # [2-3] smoothing figure eight path
        window = self.get_parameter("mov_ave_window").get_parameter_value().integer_value
        self.current_window = 1 * window
        if window < len(trajectory_position_data):
            w = np.ones(window) / window
            augmented_position_data = np.vstack(
                [
                    trajectory_position_data[-window:],
                    trajectory_position_data,
                    trajectory_position_data[:window],
                ]
            )
            trajectory_position_data[:, 0] = (
                1 * np.convolve(augmented_position_data[:, 0], w, mode="same")[window:-window]
            )
            trajectory_position_data[:, 1] = (
                1 * np.convolve(augmented_position_data[:, 1], w, mode="same")[window:-window]
            )
            augmented_yaw_data = np.hstack(
                [
                    trajectory_yaw_data[-window:],
                    trajectory_yaw_data,
                    trajectory_yaw_data[:window],
                ]
            )
            smoothed_trajectory_yaw_data = trajectory_yaw_data.copy()
            for i in range(len(trajectory_yaw_data)):
                tmp_yaw = trajectory_yaw_data[i]
                tmp_data = (
                    augmented_yaw_data[window + (i - window // 2) : window + (i + window // 2)]
                    - tmp_yaw
                )
                for j in range(len(tmp_data)):
                    if tmp_data[j] > np.pi:
                        tmp_data[j] -= 2 * np.pi
                    if tmp_data[j] < -np.pi:
                        tmp_data[j] += 2 * np.pi
                tmp_data = np.convolve(tmp_data, w, mode="same")
                smoothed_trajectory_yaw_data[i] = (
                    tmp_yaw + np.convolve(tmp_data, w, mode="same")[window // 2]
                )
                if smoothed_trajectory_yaw_data[i] > np.pi:
                    smoothed_trajectory_yaw_data[i] -= 2 * np.pi
                if smoothed_trajectory_yaw_data[i] < -np.pi:
                    smoothed_trajectory_yaw_data[i] += 2 * np.pi

            trajectory_yaw_data = smoothed_trajectory_yaw_data.copy()

            if not USE_CURVATURE_RADIUS_FLAG:
                augmented_curvature_data = np.hstack(
                    [
                        trajectory_curvature_data[-window:],
                        trajectory_curvature_data,
                        trajectory_curvature_data[:window],
                    ]
                )
                trajectory_curvature_data = (
                    1 * np.convolve(augmented_curvature_data, w, mode="same")[window:-window]
                )
        # [2-4] nominal velocity
        target_longitudinal_velocity = (
            self.get_parameter("target_longitudinal_velocity").get_parameter_value().double_value
        )

        trajectory_longitudinal_velocity_data = target_longitudinal_velocity * np.zeros(
            len(trajectory_position_data)
        )
        self.current_target_longitudinal_velocity = 1 * target_longitudinal_velocity

        self.trajectory_position_data = trajectory_position_data.copy()
        self.trajectory_yaw_data = trajectory_yaw_data.copy()
        self.trajectory_longitudinal_velocity_data = trajectory_longitudinal_velocity_data.copy()
        self.trajectory_curvature_data = trajectory_curvature_data.copy()

        self.get_logger().info("update nominal target trajectory")

    def count_observations(self, v, a, steer):
        v_bin = np.digitize(v, self.v_bins) - 1
        steer_bin = np.digitize(steer, self.steer_bins) - 1
        a_bin = np.digitize(a, self.a_bins) - 1

        if 0 <= v_bin < self.num_bins and 0 <= a_bin < self.num_bins:
            self.collected_data_counts_of_vel_acc[v_bin, a_bin] += 1

        if 0 <= v_bin < self.num_bins and 0 <= steer_bin < self.num_bins:
            self.collected_data_counts_of_vel_steer[v_bin, steer_bin] += 1

    def plot_data_collection_grid(self):
        # do not update if enough time has not passed
        if self.last_grid_update_time is not None:
            time_elapsed = self.get_clock().now() - self.last_grid_update_time
            if self.grid_update_time_interval > time_elapsed.nanoseconds / 1e9:
                return

        # update collected acceleration and velocity grid
        for collection in self.axs[2].collections:
            if collection.colorbar is not None:
                collection.colorbar.remove()
        self.axs[2].cla()

        self.heatmap = sns.heatmap(
            self.collected_data_counts_of_vel_acc.T,
            annot=True,
            cmap="coolwarm",
            xticklabels=np.round(self.v_bin_centers, 2),
            yticklabels=np.round(self.a_bin_centers, 2),
            ax=self.axs[2],
            linewidths=0.1,
            linecolor="gray",
        )

        self.axs[2].set_xlabel("Velocity bins")
        self.axs[2].set_ylabel("Acceleration bins")

        for collection in self.axs[3].collections:
            if collection.colorbar is not None:
                collection.colorbar.remove()
        self.axs[3].cla()

        self.heatmap = sns.heatmap(
            self.collected_data_counts_of_vel_steer.T,
            annot=True,
            cmap="coolwarm",
            xticklabels=np.round(self.v_bin_centers, 2),
            yticklabels=np.round(self.steer_bin_centers, 2),
            ax=self.axs[3],
            linewidths=0.1,
            linecolor="gray",
        )

        # update collected steer and velocity grid
        self.axs[3].set_xlabel("Velocity bins")
        self.axs[3].set_ylabel("Steer bins")

        self.axs[1].cla()
        self.axs[1].scatter(self.acc_hist, self.vel_hist)
        self.axs[1].plot(self.acc_hist, self.vel_hist)
        self.axs[1].set_xlim([-2.0, 2.0])
        self.axs[1].set_ylim([0.0, self.v_max + 1.0])
        self.axs[1].set_xlabel("Acceleration")
        self.axs[1].set_ylabel("Velocity")
        self.last_grid_update_time = self.get_clock().now()

        data_collecting_area = np.array(
            [
                np.array(
                    [
                        self._data_collecting_area_polygon.polygon.points[i].x,
                        self._data_collecting_area_polygon.polygon.points[i].y,
                        self._data_collecting_area_polygon.polygon.points[i].z,
                    ]
                )
                for i in range(4)
            ]
        )

        l1 = np.sqrt(((data_collecting_area[0, :2] - data_collecting_area[1, :2]) ** 2).sum())
        l2 = np.sqrt(((data_collecting_area[1, :2] - data_collecting_area[2, :2]) ** 2).sum())
        l3 = np.sqrt(((data_collecting_area[2, :2] - data_collecting_area[3, :2]) ** 2).sum())
        l4 = np.sqrt(((data_collecting_area[3, :2] - data_collecting_area[0, :2]) ** 2).sum())
        la = (l1 + l3) / 2
        lb = (l2 + l4) / 2
        title_of_fig = "rectangle : (la,lb) = " + str((la, lb))
        title_of_fig += "total num of data = " + str( np.sum(self.collected_data_counts_of_vel_acc) )
        self.fig.suptitle(title_of_fig)

        self.fig.canvas.draw()

    def timer_callback(self):
        if (
            self._present_kinematic_state is not None
            and self._present_acceleration is not None
            and self.trajectory_position_data is not None
        ):
            yaw = getYaw(
                np.array(
                    [
                        self._present_kinematic_state.pose.pose.orientation.x,
                        self._present_kinematic_state.pose.pose.orientation.y,
                        self._present_kinematic_state.pose.pose.orientation.z,
                        self._present_kinematic_state.pose.pose.orientation.w,
                    ]
                )
            )
            wheel_base = self.get_parameter("wheel_base").get_parameter_value().double_value
            steer = np.arctan2(wheel_base * yaw, self._present_kinematic_state.twist.twist.linear.x)

            # update velocity and acceleration bin if ego vehicle is moving
            if self._present_kinematic_state.twist.twist.linear.x > 1e-3:
                self.count_observations(
                    self._present_kinematic_state.twist.twist.linear.x,
                    self._present_acceleration.accel.accel.linear.x,
                    steer,
                )

            # [0] update nominal target trajectory if changing related ros2 params
            target_longitudinal_velocity = (
                self.get_parameter("target_longitudinal_velocity")
                .get_parameter_value()
                .double_value
            )

            window = self.get_parameter("mov_ave_window").get_parameter_value().integer_value

            if (
                np.abs(target_longitudinal_velocity - self.current_target_longitudinal_velocity)
                > 1e-6
                or window != self.current_window
            ):
                True  # self.updateNominalTargetTrajectory()

            # [1] receive observation from topic
            present_position = np.array(
                [
                    self._present_kinematic_state.pose.pose.position.x,
                    self._present_kinematic_state.pose.pose.position.y,
                    self._present_kinematic_state.pose.pose.position.z,
                ]
            )
            present_orientation = np.array(
                [
                    self._present_kinematic_state.pose.pose.orientation.x,
                    self._present_kinematic_state.pose.pose.orientation.y,
                    self._present_kinematic_state.pose.pose.orientation.z,
                    self._present_kinematic_state.pose.pose.orientation.w,
                ]
            )
            present_linear_velocity = np.array(
                [
                    self._present_kinematic_state.twist.twist.linear.x,
                    self._present_kinematic_state.twist.twist.linear.y,
                    self._present_kinematic_state.twist.twist.linear.z,
                ]
            )
            present_yaw = getYaw(present_orientation)

            # [2] get whole trajectory data
            trajectory_position_data = self.trajectory_position_data.copy()
            trajectory_yaw_data = self.trajectory_yaw_data.copy()
            trajectory_longitudinal_velocity_data = (
                self.trajectory_longitudinal_velocity_data.copy()
            )
            trajectory_curvature_data = self.trajectory_curvature_data.copy()

            # [3] prepare velocity noise
            while True:
                if len(self.vel_noise_list) > len(trajectory_longitudinal_velocity_data) * 2:
                    break
                else:
                    tmp_noise_vel = (
                        np.random.rand()
                        * self.get_parameter("longitudinal_velocity_noise_amp")
                        .get_parameter_value()
                        .double_value
                    )
                    noise_min_period = (
                        self.get_parameter("longitudinal_velocity_noise_min_period")
                        .get_parameter_value()
                        .double_value
                    )
                    noise_max_period = (
                        self.get_parameter("longitudinal_velocity_noise_max_period")
                        .get_parameter_value()
                        .double_value
                    )
                    tmp_noise_period = noise_min_period + np.random.rand() * (
                        noise_max_period - noise_min_period
                    )
                    dt = self.timer_period_callback
                    noise_data_num = max(
                        4, int(tmp_noise_period / dt)
                    )  # 4 is minimum noise_data_num
                    for i in range(noise_data_num):
                        self.vel_noise_list.append(
                            tmp_noise_vel * np.sin(2.0 * np.pi * i / noise_data_num)
                        )
            self.vel_noise_list.pop(0)

            # [4] find near point index for local trajectory
            distance = np.sqrt(((trajectory_position_data - present_position[:2]) ** 2).sum(axis=1))
            index_array_near = np.argsort(distance)

            nearestIndex = None
            if (self.one_round_progress_rate is None) or (present_linear_velocity[0] < 0.1):
                # if initializing, or if re-initialize while stopping
                nearestIndex = index_array_near[0]
            else:
                for i in range(len(index_array_near)):
                    progress_rate_diff = (
                        1.0 * index_array_near[i] / len(trajectory_position_data)
                    ) - self.one_round_progress_rate
                    if progress_rate_diff > 0.5:
                        progress_rate_diff -= 1.0
                    if progress_rate_diff < -0.5:
                        progress_rate_diff += 1.0
                    near_progress_rate_threshold = 0.2
                    if np.abs(progress_rate_diff) < near_progress_rate_threshold:
                        nearestIndex = 1 * index_array_near[i]
                        break
                if nearestIndex is None:
                    nearestIndex = index_array_near[0]

            self.one_round_progress_rate = 1.0 * nearestIndex / len(trajectory_position_data)

            # set target velocity
            target_vel = self.get_target_velocity(nearestIndex)

            trajectory_longitudinal_velocity_data = np.array(
                [target_vel for _ in range(len(trajectory_longitudinal_velocity_data))]
            )

            # [5] modify target velocity
            # [5-1] add noise
            aug_data_length = len(trajectory_position_data) // 4
            trajectory_position_data = np.vstack(
                [trajectory_position_data, trajectory_position_data[:aug_data_length]]
            )
            trajectory_yaw_data = np.hstack(
                [trajectory_yaw_data, trajectory_yaw_data[:aug_data_length]]
            )
            trajectory_longitudinal_velocity_data = np.hstack(
                [
                    trajectory_longitudinal_velocity_data,
                    trajectory_longitudinal_velocity_data[:aug_data_length],
                ]
            )
            trajectory_longitudinal_velocity_data[nearestIndex:] += np.array(self.vel_noise_list)[
                : len(trajectory_longitudinal_velocity_data[nearestIndex:])
            ]
            trajectory_longitudinal_velocity_data_without_limit = (
                trajectory_longitudinal_velocity_data.copy()
            )

            # [5-2] apply lateral accel limitself.acc_hist[-1] = current_acc
            max_lateral_accel = (
                self.get_parameter("max_lateral_accel").get_parameter_value().double_value
            )
            if USE_CURVATURE_RADIUS_FLAG:
                lateral_acc_limit = np.sqrt(max_lateral_accel * trajectory_curvature_data)
            else:
                lateral_acc_limit = np.sqrt(max_lateral_accel / trajectory_curvature_data)
            lateral_acc_limit = np.hstack(
                [
                    lateral_acc_limit,
                    lateral_acc_limit[:aug_data_length],
                ]
            )
            if Differential_Smoothing_Flag:
                trajectory_longitudinal_velocity_data = smooth_bounding(
                    lateral_acc_limit,
                    0.9 * lateral_acc_limit,
                    trajectory_longitudinal_velocity_data,
                )
            else:
                trajectory_longitudinal_velocity_data = np.minimum(
                    trajectory_longitudinal_velocity_data, lateral_acc_limit
                )
            # [5-3] apply limit by lateral error
            velocity_limit_by_tracking_error = (
                self.get_parameter("velocity_limit_by_tracking_error")
                .get_parameter_value()
                .double_value
            )

            lateral_error_threshold = (
                self.get_parameter("lateral_error_threshold").get_parameter_value().double_value
            )

            yaw_error_threshold = (
                self.get_parameter("yaw_error_threshold").get_parameter_value().double_value
            )

            tmp_lateral_error = np.sqrt(
                ((trajectory_position_data[nearestIndex] - present_position[:2]) ** 2).sum()
            )

            tmp_yaw_error = np.abs(present_yaw - trajectory_yaw_data[nearestIndex])

            if lateral_error_threshold < tmp_lateral_error or yaw_error_threshold < tmp_yaw_error:
                if Differential_Smoothing_Flag:
                    velocity_limit_by_tracking_error_array = (
                        velocity_limit_by_tracking_error
                        * np.ones(trajectory_longitudinal_velocity_data.shape)
                    )
                    trajectory_longitudinal_velocity_data = smooth_bounding(
                        velocity_limit_by_tracking_error_array,
                        0.9 * velocity_limit_by_tracking_error_array,
                        trajectory_longitudinal_velocity_data,
                    )
                else:
                    trajectory_longitudinal_velocity_data = np.minimum(
                        trajectory_longitudinal_velocity_data, velocity_limit_by_tracking_error
                    )

            # [6] publish
            # [6-1] publish trajectory
            pub_traj_len = min(int(50 / self.traj_step), aug_data_length)
            tmp_traj = Trajectory()
            for i in range(pub_traj_len):
                tmp_traj_point = TrajectoryPoint()
                tmp_traj_point.pose.position.x = trajectory_position_data[i + nearestIndex, 0]
                tmp_traj_point.pose.position.y = trajectory_position_data[i + nearestIndex, 1]
                tmp_traj_point.pose.position.z = present_position[2]

                tmp_traj_point.pose.orientation.x = 0.0
                tmp_traj_point.pose.orientation.y = 0.0
                tmp_traj_point.pose.orientation.z = np.sin(
                    trajectory_yaw_data[i + nearestIndex] / 2
                )
                tmp_traj_point.pose.orientation.w = np.cos(
                    trajectory_yaw_data[i + nearestIndex] / 2
                )

                tmp_traj_point.longitudinal_velocity_mps = trajectory_longitudinal_velocity_data[
                    i + nearestIndex
                ]
                tmp_traj.points.append(tmp_traj_point)

            self.trajectory_for_collecting_data_pub_.publish(tmp_traj)

            # [6-2] publish marker_array
            marker_array = MarkerArray()

            # [6-2a] local trajectory
            marker_traj1 = Marker()
            marker_traj1.type = 4
            marker_traj1.id = 1
            marker_traj1.header.frame_id = "map"

            marker_traj1.action = marker_traj1.ADD

            marker_traj1.scale.x = 0.4
            marker_traj1.scale.y = 0.0
            marker_traj1.scale.z = 0.0

            marker_traj1.color.a = 1.0
            marker_traj1.color.r = 1.0
            marker_traj1.color.g = 0.0
            marker_traj1.color.b = 0.0

            marker_traj1.lifetime.nanosec = 500000000
            marker_traj1.frame_locked = True

            marker_traj1.points = []
            for i in range(len(tmp_traj.points)):
                tmp_marker_point = Point()
                tmp_marker_point.x = tmp_traj.points[i].pose.position.x
                tmp_marker_point.y = tmp_traj.points[i].pose.position.y
                tmp_marker_point.z = 0.0
                marker_traj1.points.append(tmp_marker_point)

            marker_array.markers.append(marker_traj1)

            # [6-2b] whole trajectory
            marker_traj2 = Marker()
            marker_traj2.type = 4
            marker_traj2.id = 0
            marker_traj2.header.frame_id = "map"

            marker_traj2.action = marker_traj2.ADD

            marker_traj2.scale.x = 0.2
            marker_traj2.scale.y = 0.0
            marker_traj2.scale.z = 0.0

            marker_traj2.color.a = 1.0
            marker_traj2.color.r = 0.0
            marker_traj2.color.g = 0.0
            marker_traj2.color.b = 1.0

            marker_traj2.lifetime.nanosec = 500000000
            marker_traj2.frame_locked = True

            marker_traj2.points = []
            marker_downsampling = 5
            for i in range((len(trajectory_position_data) // marker_downsampling)):
                tmp_marker_point = Point()
                tmp_marker_point.x = trajectory_position_data[i * marker_downsampling, 0]
                tmp_marker_point.y = trajectory_position_data[i * marker_downsampling, 1]
                tmp_marker_point.z = 0.0
                marker_traj2.points.append(tmp_marker_point)

            marker_array.markers.append(marker_traj2)

            self.data_collecting_trajectory_marker_array_pub_.publish(marker_array)

            if debug_matplotlib_plot_flag:
                self.axs[0].cla()
                step_size_array = np.sqrt(
                    ((trajectory_position_data[1:] - trajectory_position_data[:-1]) ** 2).sum(
                        axis=1
                    )
                )
                distance = np.zeros(len(trajectory_position_data))
                for i in range(1, len(trajectory_position_data)):
                    distance[i] = distance[i - 1] + step_size_array[i - 1]
                distance -= distance[nearestIndex]
                time_width_array = step_size_array / (
                    trajectory_longitudinal_velocity_data[:-1] + 0.01
                )
                timestamp = np.zeros(len(trajectory_position_data))
                for i in range(1, len(trajectory_position_data)):
                    timestamp[i] = timestamp[i - 1] + time_width_array[i - 1]
                timestamp -= timestamp[nearestIndex]

                self.axs[0].plot(0, present_linear_velocity[0], "o", label="current vel")

                self.axs[0].plot(
                    timestamp[nearestIndex : nearestIndex + pub_traj_len],
                    trajectory_longitudinal_velocity_data_without_limit[
                        nearestIndex : nearestIndex + pub_traj_len
                    ],
                    "--",
                    label="target vel before applying limit",
                )
                self.axs[0].plot(
                    timestamp[nearestIndex : nearestIndex + pub_traj_len],
                    lateral_acc_limit[nearestIndex : nearestIndex + pub_traj_len],
                    "--",
                    label="lateral acc limit (always)",
                )
                self.axs[0].plot(
                    timestamp[nearestIndex : nearestIndex + pub_traj_len],
                    velocity_limit_by_tracking_error * np.ones(pub_traj_len),
                    "--",
                    label="vel limit by tracking error (only when exceeding threshold)",
                )
                self.axs[0].plot(
                    timestamp[nearestIndex : nearestIndex + pub_traj_len],
                    trajectory_longitudinal_velocity_data[
                        nearestIndex : nearestIndex + pub_traj_len
                    ],
                    label="actual target vel",
                )
                self.axs[0].set_xlim([-0.5, 10.5])
                self.axs[0].set_ylim([-0.5, 12.5])

                self.axs[0].set_xlabel("future timestamp [s]")
                self.axs[0].set_ylabel("longitudinal_velocity [m/s]")
                self.axs[0].legend(fontsize=8)
                self.plot_data_collection_grid()
                plt.pause(0.01)




def main(args=None):
    rclpy.init(args=args)

    data_collecting_trajectory_publisher = DataCollectingTrajectoryPublisher()

    rclpy.spin(data_collecting_trajectory_publisher)

    data_collecting_trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
