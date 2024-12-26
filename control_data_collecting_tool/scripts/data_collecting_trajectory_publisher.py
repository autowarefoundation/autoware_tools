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

from accel_brake_map import AccelBrakeMapConverter
from autoware_planning_msgs.msg import Trajectory
from autoware_planning_msgs.msg import TrajectoryPoint
from courses.load_course import declare_course_params
from courses.load_course import load_course
from data_collecting_base_node import DataCollectingBaseNode
from geometry_msgs.msg import Point
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np
from numpy import pi
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

debug_matplotlib_plot_flag = False
Differential_Smoothing_Flag = True
USE_CURVATURE_RADIUS_FLAG = False


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


# inherits from DataCollectingBaseNode
class DataCollectingTrajectoryPublisher(DataCollectingBaseNode):
    def __init__(self):
        super().__init__("data_collecting_trajectory_publisher")

        self.declare_parameter(
            "acc_kp",
            1.0,
            ParameterDescriptor(description="Pure pursuit accel command proportional gain"),
        )

        self.declare_parameter(
            "max_lateral_accel",
            0.5,
            ParameterDescriptor(description="Max lateral acceleration limit [m/s^2]"),
        )

        self.declare_parameter(
            "lateral_error_threshold",
            2.0,
            ParameterDescriptor(
                description="Lateral error threshold where applying velocity limit [m/s]"
            ),
        )

        self.declare_parameter(
            "yaw_error_threshold",
            0.50,
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
            0.01,
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

        self.declare_parameter(
            "COLLECTING_DATA_V_MIN",
            0.0,
            ParameterDescriptor(description="Minimum velocity for data collection [m/s]"),
        )

        self.declare_parameter(
            "COLLECTING_DATA_V_MAX",
            11.5,
            ParameterDescriptor(description="Maximum velocity for data collection [m/s]"),
        )

        self.declare_parameter(
            "COLLECTING_DATA_A_MIN",
            -1.0,
            ParameterDescriptor(description="Minimum velocity for data collection [m/s^2]"),
        )

        self.declare_parameter(
            "COLLECTING_DATA_A_MAX",
            1.0,
            ParameterDescriptor(description="Maximum velocity for data collection [m/s^2]"),
        )

        self.pedal_input_pub_ = self.create_publisher(
            Float32,
            "/data_collecting_pedal_input",
            1,
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

        self.pub_stop_request_ = self.create_publisher(
            Bool,
            "/data_collecting_stop_request",
            1,
        )

        self.sub_data_collecting_area_ = self.create_subscription(
            PolygonStamped,
            "/data_collecting_area",
            self.onDataCollectingArea,
            1,
        )
        self.sub_data_collecting_area_
        self._data_collecting_area_polygon = None

        self.sub_data_collecting_gaol_pose_ = self.create_subscription(
            PoseStamped,
            "/data_collecting_goal_pose",
            self.onGoalPose,
            1,
        )
        self.sub_data_collecting_area_

        """
        Declare course specific parameters
        """
        declare_course_params(self.COURSE_NAME, self)

        # obtain ros params as dictionary
        param_names = self._parameters
        params = self.get_parameters(param_names)
        params_dict = {param.name: param.value for param in params}

        self.trajectory_step = 0.1
        self.course = load_course(self.COURSE_NAME, self.trajectory_step, params_dict)

        self.timer_period_callback = 0.03  # 30ms
        self.timer_trajectory_generator = self.create_timer(
            self.timer_period_callback, self.callback_trajectory_generator
        )

        self.trajectory_position_data = None
        self.trajectory_yaw_data = None
        self.trajectory_longitudinal_velocity_data = None
        self.trajectory_curvature_data = None
        self.trajectory_parts = None
        self.trajectory_achievement_rates = None

        self.nearestIndex = 0
        self.yaw_offset = 0.0
        self.rectangle_center_position = np.zeros(2)

        self.current_target_longitudinal_velocity = (
            self.get_parameter("target_longitudinal_velocity").get_parameter_value().double_value
        )
        self.current_window = (
            self.get_parameter("mov_ave_window").get_parameter_value().integer_value
        )

        self.vel_noise_list = []

        # subscriptions of data counter
        self.collected_data_counts_of_vel_acc_subscription_ = self.create_subscription(
            Int32MultiArray,
            "/control_data_collecting_tools/collected_data_counts_of_vel_acc",
            self.subscribe_collected_data_counts_of_vel_acc,
            10,
        )
        self.collected_data_counts_of_vel_acc_subscription_

        self.collected_data_counts_of_vel_steer_subscription_ = self.create_subscription(
            Int32MultiArray,
            "/control_data_collecting_tools/collected_data_counts_of_vel_steer",
            self.subscribe_collected_data_counts_of_vel_steer,
            10,
        )
        self.collected_data_counts_of_vel_steer_subscription_

        self.collected_data_counts_of_vel_accel_pedal_input_subscription_ = (
            self.create_subscription(
                Int32MultiArray,
                "/control_data_collecting_tools/collected_data_counts_of_vel_accel_pedal_input",
                self.subscribe_collected_data_counts_of_vel_accel_pedal_input,
                10,
            )
        )
        self.collected_data_counts_of_vel_accel_pedal_input_subscription_

        self.collected_data_counts_of_vel_brake_pedal_input_subscription_ = (
            self.create_subscription(
                Int32MultiArray,
                "/control_data_collecting_tools/collected_data_counts_of_vel_brake_pedal_input",
                self.subscribe_collected_data_counts_of_vel_brake_pedal_input,
                10,
            )
        )
        self.collected_data_counts_of_vel_brake_pedal_input_subscription_

        if debug_matplotlib_plot_flag:
            self.fig, self.axs = plt.subplots(4, 1, figsize=(12, 20))
            plt.ion()

    def subscribe_collected_data_counts_of_vel_acc(self, msg):
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        self.collected_data_counts_of_vel_acc = np.array(msg.data).reshape((rows, cols))

    def subscribe_collected_data_counts_of_vel_steer(self, msg):
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        self.collected_data_counts_of_vel_steer = np.array(msg.data).reshape((rows, cols))

    def subscribe_collected_data_counts_of_vel_accel_pedal_input(self, msg):
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        self.collected_data_counts_of_vel_accel_pedal_input = np.array(msg.data).reshape(
            (rows, cols)
        )

    def subscribe_collected_data_counts_of_vel_brake_pedal_input(self, msg):
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        self.collected_data_counts_of_vel_brake_pedal_input = np.array(msg.data).reshape(
            (rows, cols)
        )

    def onDataCollectingArea(self, msg):
        self._data_collecting_area_polygon = msg
        self.updateNominalTargetTrajectory()

    def onGoalPose(self, msg):
        self.goal_point[0] = msg.pose.position.x
        self.goal_point[1] = msg.pose.position.y

        self.trajectory_position_data = None
        self.trajectory_yaw_data = None
        self.updateNominalTargetTrajectory()

    def updateNominalTargetTrajectory(self):
        if self._data_collecting_area_polygon is not None:
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
        else:
            data_collecting_area = np.array([np.array([i, i**2]) for i in range(4)])

        A, B, C, D = (
            data_collecting_area[0][0:2],
            data_collecting_area[1][0:2],
            data_collecting_area[2][0:2],
            data_collecting_area[3][0:2],
        )

        # [1] compute an approximate rectangle
        l1 = np.sqrt(((data_collecting_area[0, :2] - data_collecting_area[1, :2]) ** 2).sum())
        l2 = np.sqrt(((data_collecting_area[1, :2] - data_collecting_area[2, :2]) ** 2).sum())
        l3 = np.sqrt(((data_collecting_area[2, :2] - data_collecting_area[3, :2]) ** 2).sum())
        l4 = np.sqrt(((data_collecting_area[3, :2] - data_collecting_area[0, :2]) ** 2).sum())
        la = (l1 + l3) / 2
        lb = (l2 + l4) / 2
        if np.abs(la - lb) < 1e-6:
            la += 0.1  # long_side_length must not be equal to short_side_length7

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
        # [2-1] generate path
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

        actual_long_side = max(long_side_length - long_side_margin, 1.1)
        actual_short_side = max(short_side_length - long_side_margin, 1.0)

        self.yaw_offset = yaw_offset
        self.rectangle_center_position = rectangle_center_position

        if self.COURSE_NAME is not None:
            self.course.set_vertices(A, B, C, D)

            self.course.get_trajectory_points(
                actual_long_side, actual_short_side, self.ego_point, self.goal_point
            )
            (
                self.trajectory_position_data,
                self.trajectory_yaw_data,
                self.trajectory_curvature_data,
                self.trajectory_parts,
                self.trajectory_achievement_rates,
            ) = self.course.return_trajectory_points(
                self.yaw_offset, self.rectangle_center_position
            )

            self.course.get_boundary_points()
            self.boundary_points = self.course.boundary_points

        else:
            self.trajectory_position_data = None
            self.trajectory_yaw_data = None
            self.trajectory_longitudinal_velocity_data = None
            self.trajectory_curvature_data = None

        # [2-2] nominal velocity

        if self.trajectory_position_data is not None:
            target_longitudinal_velocity = (
                self.get_parameter("target_longitudinal_velocity")
                .get_parameter_value()
                .double_value
            )

            self.trajectory_longitudinal_velocity_data = target_longitudinal_velocity * np.zeros(
                len(self.trajectory_position_data)
            )
            self.current_target_longitudinal_velocity = 1 * target_longitudinal_velocity

        if self.trajectory_position_data is not None:
            self.nearestIndex = 0
            self.get_logger().info("update nominal target trajectory")
        else:
            self.get_logger().error("Fail to generate trajectory")

    def callback_trajectory_generator(self):
        if (
            (
                self._present_kinematic_state.pose is not None
                and self._present_acceleration.accel is not None
                and self.trajectory_position_data is not None
            )
            or self.trajectory_position_data is not None
            or self.trajectory_yaw_data is not None
        ):
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
                self.updateNominalTargetTrajectory()

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

            if np.linalg.norm(present_orientation) < 1e-6:
                present_yaw = self.previous_yaw
            else:
                present_yaw = getYaw(present_orientation)[0]
                self.previous_yaw = present_yaw

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
            if self.course.closed:
                index_range = np.arange(
                    self.nearestIndex - len(trajectory_position_data) // 4,
                    self.nearestIndex + len(trajectory_position_data) // 4,
                ) % len(trajectory_position_data)
            else:
                min_index_range = max(0, self.nearestIndex - len(trajectory_position_data) // 4)
                max_index_range = min(
                    len(trajectory_position_data),
                    self.nearestIndex + len(trajectory_position_data) // 4,
                )
                index_range = np.arange(min_index_range, max_index_range)
            distance = np.sqrt(
                ((trajectory_position_data[index_range] - present_position[:2]) ** 2).sum(axis=1)
            )
            index_array_near = np.argsort(distance)
            if self.present_operation_mode_ == 3:
                self.nearestIndex = index_range[index_array_near[0]]
            # set target velocity
            present_vel = present_linear_velocity[0]
            present_acc = self._present_acceleration.accel.accel.linear.x
            current_time = self.get_clock().now().nanoseconds / 1e9

            target_vel = 0.0
            target_pedal_input = 0.0
            if self.CONTROL_MODE == "acceleration_cmd":
                target_vel = self.course.get_target_velocity(
                    self.nearestIndex,
                    current_time,
                    present_vel,
                    present_acc,
                    self.collected_data_counts_of_vel_acc,
                    self.collected_data_counts_of_vel_steer,
                    self.mask_vel_acc,
                    self.mask_vel_steer,
                )
            elif self.CONTROL_MODE == "actuation_cmd":
                target_pedal_input = self.course.get_target_pedal_input(
                    self.nearestIndex,
                    current_time,
                    present_vel,
                    self.collected_data_counts_of_vel_accel_pedal_input,
                    self.collected_data_counts_of_vel_brake_pedal_input,
                )
            elif (
                self.CONTROL_MODE == "external_acceleration_cmd"
                or self.CONTROL_MODE == "external_actuation_cmd"
            ):
                pass  # do nothing
            else:
                self.get_logger().error(f"Invalid control mode : {self.CONTROL_MODE}")

            trajectory_longitudinal_velocity_data = np.array(
                [target_vel for _ in range(len(trajectory_position_data))]
            )

            # [5] modify target velocity
            # [5-1] add noise
            aug_data_length = len(trajectory_position_data) // 4
            trajectory_longitudinal_velocity_data = np.hstack(
                [
                    trajectory_longitudinal_velocity_data,
                    trajectory_longitudinal_velocity_data[:aug_data_length],
                ]
            )
            trajectory_longitudinal_velocity_data[self.nearestIndex :] += np.array(
                self.vel_noise_list
            )[: len(trajectory_longitudinal_velocity_data[self.nearestIndex :])]
            trajectory_longitudinal_velocity_data_without_limit = (
                trajectory_longitudinal_velocity_data.copy()
            )

            # [5-2] apply lateral accel limit
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
                ((trajectory_position_data[self.nearestIndex] - present_position[:2]) ** 2).sum()
            )

            tmp_yaw_error = np.abs(
                (present_yaw - trajectory_yaw_data[self.nearestIndex] + np.pi) % (2 * np.pi) - np.pi
            )

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

            # [6-1] publish pedal input
            if self.CONTROL_MODE == "actuation_cmd":
                self.pedal_input_pub_.publish(Float32(data=float(target_pedal_input)))

            # [6-2] publish trajectory
            if self.course.closed:
                pub_trajectory_index = np.arange(
                    self.nearestIndex, self.nearestIndex + int(50 / self.trajectory_step)
                ) % len(trajectory_position_data)
            else:
                pub_trajectory_index = np.arange(
                    self.nearestIndex,
                    np.min(
                        [
                            self.nearestIndex + int(50 / self.trajectory_step),
                            len(trajectory_position_data),
                        ]
                    ),
                )

            pub_trajectory_length = len(pub_trajectory_index)
            tmp_trajectory = Trajectory()
            for i in pub_trajectory_index:
                tmp_trajectory_point = TrajectoryPoint()
                tmp_trajectory_point.pose.position.x = trajectory_position_data[i, 0]
                tmp_trajectory_point.pose.position.y = trajectory_position_data[i, 1]
                tmp_trajectory_point.pose.position.z = present_position[2]

                tmp_trajectory_point.pose.orientation.x = 0.0
                tmp_trajectory_point.pose.orientation.y = 0.0
                tmp_trajectory_point.pose.orientation.z = np.sin(trajectory_yaw_data[i] / 2)
                tmp_trajectory_point.pose.orientation.w = np.cos(trajectory_yaw_data[i] / 2)

                tmp_trajectory_point.longitudinal_velocity_mps = (
                    trajectory_longitudinal_velocity_data[i]
                )
                tmp_trajectory.points.append(tmp_trajectory_point)

            self.trajectory_for_collecting_data_pub_.publish(tmp_trajectory)

            # [6-3] publish marker_array
            marker_array = MarkerArray()

            # [6-3a] local trajectory
            marker_trajectory_1 = Marker()
            marker_trajectory_1.type = 4
            marker_trajectory_1.id = 1
            marker_trajectory_1.header.frame_id = "map"

            marker_trajectory_1.action = marker_trajectory_1.ADD

            marker_trajectory_1.scale.x = 0.4
            marker_trajectory_1.scale.y = 0.0
            marker_trajectory_1.scale.z = 0.0

            marker_trajectory_1.color.a = 1.0
            marker_trajectory_1.color.r = 1.0
            marker_trajectory_1.color.g = 0.0
            marker_trajectory_1.color.b = 0.0

            marker_trajectory_1.lifetime.nanosec = 500000000
            marker_trajectory_1.frame_locked = True

            marker_trajectory_1.points = []
            for i in range(len(tmp_trajectory.points)):
                tmp_marker_point = Point()
                tmp_marker_point.x = tmp_trajectory.points[i].pose.position.x
                tmp_marker_point.y = tmp_trajectory.points[i].pose.position.y
                tmp_marker_point.z = 0.0
                marker_trajectory_1.points.append(tmp_marker_point)

            marker_array.markers.append(marker_trajectory_1)

            # boundary
            boundary_marker_trajectory_1 = Marker()
            boundary_marker_trajectory_1.type = 4
            boundary_marker_trajectory_1.id = 3
            boundary_marker_trajectory_1.header.frame_id = "map"

            boundary_marker_trajectory_1.action = boundary_marker_trajectory_1.ADD

            boundary_marker_trajectory_1.scale.x = 0.4
            boundary_marker_trajectory_1.scale.y = 0.0
            boundary_marker_trajectory_1.scale.z = 0.0

            boundary_marker_trajectory_1.color.a = 0.5
            boundary_marker_trajectory_1.color.r = 0.0
            boundary_marker_trajectory_1.color.g = 1.0
            boundary_marker_trajectory_1.color.b = 0.0

            boundary_marker_trajectory_1.lifetime.nanosec = 500000000
            boundary_marker_trajectory_1.frame_locked = True
            boundary_marker_trajectory_1.points = []

            down_sampling_ = 5
            for i in range(len(self.boundary_points) // down_sampling_):
                boundary_tmp_marker_point = Point()
                boundary_tmp_marker_point.x = self.boundary_points[i * down_sampling_][0]
                boundary_tmp_marker_point.y = self.boundary_points[i * down_sampling_][1]
                boundary_tmp_marker_point.z = 0.0
                boundary_marker_trajectory_1.points.append(boundary_tmp_marker_point)

            boundary_tmp_marker_point = Point()
            boundary_tmp_marker_point.x = self.boundary_points[0][0]
            boundary_tmp_marker_point.y = self.boundary_points[0][1]
            boundary_tmp_marker_point.z = 0.0
            boundary_marker_trajectory_1.points.append(boundary_tmp_marker_point)

            marker_array.markers.append(boundary_marker_trajectory_1)

            # [6-3b] whole trajectory
            marker_trajectory_2 = Marker()
            marker_trajectory_2.type = 4
            marker_trajectory_2.id = 0
            marker_trajectory_2.header.frame_id = "map"

            marker_trajectory_2.action = marker_trajectory_2.ADD

            marker_trajectory_2.scale.x = 0.2
            marker_trajectory_2.scale.y = 0.0
            marker_trajectory_2.scale.z = 0.0

            marker_trajectory_2.color.a = 1.0
            marker_trajectory_2.color.r = 0.0
            marker_trajectory_2.color.g = 0.0
            marker_trajectory_2.color.b = 1.0

            marker_trajectory_2.lifetime.nanosec = 500000000
            marker_trajectory_2.frame_locked = True

            marker_trajectory_2.points = []
            marker_down_sampling = 5
            for i in range(len(trajectory_position_data) // marker_down_sampling):
                tmp_marker_point = Point()
                tmp_marker_point.x = trajectory_position_data[i * marker_down_sampling, 0]
                tmp_marker_point.y = trajectory_position_data[i * marker_down_sampling, 1]
                tmp_marker_point.z = 0.0
                marker_trajectory_2.points.append(tmp_marker_point)

            marker_array.markers.append(marker_trajectory_2)

            # [6-3c] add arrow
            marker_arrow = Marker()
            marker_arrow.type = marker_arrow.ARROW
            marker_arrow.id = 2
            marker_arrow.header.frame_id = "map"

            marker_arrow.action = marker_arrow.ADD

            marker_arrow.scale.x = 0.5
            marker_arrow.scale.y = 2.5
            marker_arrow.scale.z = 0.0

            marker_arrow.color.a = 1.0
            marker_arrow.color.r = 1.0
            marker_arrow.color.g = 0.0
            marker_arrow.color.b = 1.0

            marker_arrow.lifetime.nanosec = 500000000
            marker_arrow.frame_locked = True

            tangent_vec = np.array(
                [
                    np.cos(trajectory_yaw_data[self.nearestIndex]),
                    np.sin(trajectory_yaw_data[self.nearestIndex]),
                ]
            )

            marker_arrow.points = []

            start_marker_point = Point()
            start_marker_point.x = tmp_trajectory.points[0].pose.position.x
            start_marker_point.y = tmp_trajectory.points[0].pose.position.y
            start_marker_point.z = 0.0
            marker_arrow.points.append(start_marker_point)

            end_marker_point = Point()
            end_marker_point.x = tmp_trajectory.points[0].pose.position.x + 5.0 * tangent_vec[0]
            end_marker_point.y = tmp_trajectory.points[0].pose.position.y + 5.0 * tangent_vec[1]
            end_marker_point.z = 0.0
            marker_arrow.points.append(end_marker_point)

            marker_array.markers.append(marker_arrow)

            self.data_collecting_trajectory_marker_array_pub_.publish(marker_array)

            # [6-4] stop request
            # self.get_logger().info("self.course.check_in_boundary(present_position): {}".format(self.course.check_in_boundary(present_position)))
            if not self.course.check_in_boundary(present_position):
                msg = Bool()
                msg.data = True
                self.pub_stop_request_.publish(msg)

            # [6-5] update trajectory data if necessary
            (
                self.nearestIndex,
                self.trajectory_position_data,
                self.trajectory_yaw_data,
                self.trajectory_curvature_data,
                self.trajectory_parts,
                self.trajectory_achievement_rates,
            ) = self.course.update_trajectory_points(
                self.nearestIndex,
                self.yaw_offset,
                self.rectangle_center_position,
                self.collected_data_counts_of_vel_acc,
                self.collected_data_counts_of_vel_steer,
            )

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
                distance -= distance[self.nearestIndex]
                time_width_array = step_size_array / (
                    trajectory_longitudinal_velocity_data[:-1] + 0.01
                )
                timestamp = np.zeros(len(trajectory_position_data))
                for i in range(1, len(trajectory_position_data)):
                    timestamp[i] = timestamp[i - 1] + time_width_array[i - 1]
                timestamp -= timestamp[self.nearestIndex]

                self.axs[0].plot(0, present_linear_velocity[0], "o", label="current vel")

                self.axs[0].plot(
                    timestamp[self.nearestIndex : self.nearestIndex + pub_trajectory_length],
                    trajectory_longitudinal_velocity_data_without_limit[
                        self.nearestIndex : self.nearestIndex + pub_trajectory_length
                    ],
                    "--",
                    label="target vel before applying limit",
                )
                self.axs[0].plot(
                    timestamp[self.nearestIndex : self.nearestIndex + pub_trajectory_length],
                    lateral_acc_limit[
                        self.nearestIndex : self.nearestIndex + pub_trajectory_length
                    ],
                    "--",
                    label="lateral acc limit (always)",
                )
                self.axs[0].plot(
                    timestamp[self.nearestIndex : self.nearestIndex + pub_trajectory_length],
                    velocity_limit_by_tracking_error * np.ones(pub_trajectory_length),
                    "--",
                    label="vel limit by tracking error (only when exceeding threshold)",
                )
                self.axs[0].plot(
                    timestamp[self.nearestIndex : self.nearestIndex + pub_trajectory_length],
                    trajectory_longitudinal_velocity_data[
                        self.nearestIndex : self.nearestIndex + pub_trajectory_length
                    ],
                    label="actual target vel",
                )
                self.axs[0].set_xlim([-0.5, 10.5])
                self.axs[0].set_ylim([-0.5, 12.5])

                self.axs[0].set_xlabel("future timestamp [s]")
                self.axs[0].set_ylabel("longitudinal_velocity [m/s]")
                self.axs[0].legend(fontsize=8)

                self.fig.canvas.draw()
                plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)

    data_collecting_trajectory_publisher = DataCollectingTrajectoryPublisher()

    rclpy.spin(data_collecting_trajectory_publisher)

    data_collecting_trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
