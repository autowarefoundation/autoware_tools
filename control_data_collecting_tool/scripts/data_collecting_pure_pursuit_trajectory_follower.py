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
from autoware_control_msgs.msg import Control as AckermannControlCommand
from autoware_planning_msgs.msg import Trajectory
from autoware_vehicle_msgs.msg import GearCommand
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

debug_matplotlib_plot_flag = False
if debug_matplotlib_plot_flag:
    import matplotlib.pyplot as plt

    plt.rcParams["figure.figsize"] = [8, 8]


def getYaw(orientation_xyzw):
    return R.from_quat(orientation_xyzw.reshape(-1, 4)).as_euler("xyz")[:, 2]


class DataCollectingPurePursuitTrajectoryFollower(Node):
    def __init__(self):
        super().__init__("data_collecting_pure_pursuit_trajectory_follower")

        self.declare_parameter(
            "pure_pursuit_type",
            "linearized",
            ParameterDescriptor(
                description="Pure pursuit type (`naive` or `linearized` steer control law"
            ),
        )

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
            "lookahead_time",
            2.0,
            ParameterDescriptor(description="Pure pursuit lookahead length coef [m/(m/s)]"),
        )

        self.declare_parameter(
            "min_lookahead",
            2.0,
            ParameterDescriptor(description="Pure pursuit lookahead length intercept [m]"),
        )

        self.declare_parameter(
            "linearized_pure_pursuit_steer_kp_param",
            2.0,
            ParameterDescriptor(description="Linearized pure pursuit steering P gain parameter"),
        )

        self.declare_parameter(
            "linearized_pure_pursuit_steer_kd_param",
            2.0,
            ParameterDescriptor(description="Linearized pure pursuit steering D gain parameter"),
        )

        self.declare_parameter(
            "stop_acc",
            -2.0,
            ParameterDescriptor(
                description="Accel command for stopping data collecting driving [m/ss]"
            ),
        )

        self.declare_parameter(
            "stop_jerk_lim",
            2.0,
            ParameterDescriptor(
                description="Jerk limit for stopping data collecting driving [m/sss]"
            ),
        )

        # lim default values are taken from https://github.com/autowarefoundation/autoware.universe/blob/e90d3569bacaf64711072a94511ccdb619a59464/control/autoware_vehicle_cmd_gate/config/vehicle_cmd_gate.param.yaml
        self.declare_parameter(
            "lon_acc_lim",
            2.0,
            ParameterDescriptor(description="Longitudinal acceleration limit [m/ss]"),
        )

        self.declare_parameter(
            "lon_jerk_lim",
            5.0,
            ParameterDescriptor(description="Longitudinal jerk limit [m/sss]"),
        )

        self.declare_parameter(
            "steer_lim",
            1.0,
            ParameterDescriptor(description="Steering angle limit [rad]"),
        )

        self.declare_parameter(
            "steer_rate_lim",
            1.0,
            ParameterDescriptor(description="Steering angle rate limit [rad/s]"),
        )

        self.declare_parameter(
            "acc_noise_amp",
            0.01,
            ParameterDescriptor(description="Accel cmd additional sine noise amplitude [m/ss]"),
        )

        self.declare_parameter(
            "acc_noise_min_period",
            5.0,
            ParameterDescriptor(description="Accel cmd additional sineW noise minimum period [s]"),
        )

        self.declare_parameter(
            "acc_noise_max_period",
            20.0,
            ParameterDescriptor(description="Accel cmd additional sine noise maximum period [s]"),
        )

        self.declare_parameter(
            "steer_noise_amp",
            0.01,
            ParameterDescriptor(description="Steer cmd additional sine noise amplitude [rad]"),
        )

        self.declare_parameter(
            "steer_noise_min_period",
            5.0,
            ParameterDescriptor(description="Steer cmd additional sine noise minimum period [s]"),
        )

        self.declare_parameter(
            "steer_noise_max_period",
            20.0,
            ParameterDescriptor(description="Steer cmd additional sine noise maximum period [s]"),
        )

        self.sub_odometry_ = self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.onOdometry,
            1,
        )
        self.sub_odometry_

        self.sub_trajectory_ = self.create_subscription(
            Trajectory,
            "/data_collecting_trajectory",
            self.onTrajectory,
            1,
        )
        self.sub_trajectory_

        self.sub_operation_mode_ = self.create_subscription(
            OperationModeState,
            "/system/operation_mode/state",
            self.onOperationMode,
            1,
        )
        self.sub_operation_mode_

        self.sub_stop_request_ = self.create_subscription(
            Bool,
            "/data_collecting_stop_request",
            self.onStopRequest,
            1,
        )
        self.sub_stop_request_

        self.control_cmd_pub_ = self.create_publisher(
            AckermannControlCommand,
            "/external/selected/control_cmd",
            1,
        )

        self.gear_cmd_pub_ = self.create_publisher(
            GearCommand,
            "/external/selected/gear_cmd",
            1,
        )

        self.data_collecting_lookahead_marker_array_pub_ = self.create_publisher(
            MarkerArray,
            "/data_collecting_lookahead_marker_array",
            1,
        )

        self.timer_period_callback = 0.03  # 30ms
        self.timer = self.create_timer(self.timer_period_callback, self.timer_callback)

        self._present_kinematic_state = None
        self._present_trajectory = None
        self._present_operation_mode = None
        self.stop_request = False
        self._previous_cmd = np.zeros(2)

        self.acc_noise_list = []
        self.steer_noise_list = []
        self.acc_history = []
        self.steer_history = []
        self.acc_noise_history = []
        self.steer_noise_history = []

    def onOdometry(self, msg):
        self._present_kinematic_state = msg

    def onTrajectory(self, msg):
        self._present_trajectory = msg

    def onOperationMode(self, msg):
        self._present_operation_mode = msg

    def onStopRequest(self, msg):
        self.stop_request = msg.data
        self.get_logger().info("receive " + str(msg))

    def timer_callback(self):
        if (self._present_trajectory is not None) and (self._present_kinematic_state is not None):
            self.control()

    def pure_pursuit_control(
        self,
        pos_xy_obs,
        pos_yaw_obs,
        longitudinal_vel_obs,
        pos_xy_ref_target,
        longitudinal_vel_ref_nearest,
    ):
        # naive pure pursuit steering control law
        wheel_base = self.get_parameter("wheel_base").get_parameter_value().double_value
        acc_kp = self.get_parameter("acc_kp").get_parameter_value().double_value
        longitudinal_vel_err = longitudinal_vel_obs - longitudinal_vel_ref_nearest
        pure_pursuit_acc_cmd = -acc_kp * longitudinal_vel_err

        alpha = (
            np.arctan2(pos_xy_ref_target[1] - pos_xy_obs[1], pos_xy_ref_target[0] - pos_xy_obs[0])
            - pos_yaw_obs[0]
        )
        ang_z = 2.0 * longitudinal_vel_ref_nearest * np.sin(alpha) / wheel_base
        steer = np.arctan(ang_z * wheel_base / longitudinal_vel_ref_nearest)

        return np.array([pure_pursuit_acc_cmd, steer])

    def linearized_pure_pursuit_control(
        self,
        pos_xy_obs,
        pos_yaw_obs,
        longitudinal_vel_obs,
        pos_xy_ref_nearest,
        pos_yaw_ref_nearest,
        longitudinal_vel_ref_nearest,
    ):
        # control law equal to simple_trajectory_follower in autoware
        wheel_base = self.get_parameter("wheel_base").get_parameter_value().double_value
        acc_kp = self.get_parameter("acc_kp").get_parameter_value().double_value

        # Currently, the following params are not declared as ROS 2 params.
        lookahead_coef = self.get_parameter("lookahead_time").get_parameter_value().double_value
        lookahead_intercept = self.get_parameter("min_lookahead").get_parameter_value().double_value

        linearized_pure_pursuit_steer_kp_param = (
            self.get_parameter("linearized_pure_pursuit_steer_kp_param")
            .get_parameter_value()
            .double_value
        )
        linearized_pure_pursuit_steer_kd_param = (
            self.get_parameter("linearized_pure_pursuit_steer_kd_param")
            .get_parameter_value()
            .double_value
        )

        longitudinal_vel_err = longitudinal_vel_obs - longitudinal_vel_ref_nearest
        pure_pursuit_acc_cmd = -acc_kp * longitudinal_vel_err

        cos_yaw = np.cos(pos_yaw_ref_nearest)
        sin_yaw = np.sin(pos_yaw_ref_nearest)
        diff_position = pos_xy_obs - pos_xy_ref_nearest
        lat_err = -sin_yaw * diff_position[0] + cos_yaw * diff_position[1]
        yaw_err = pos_yaw_obs - pos_yaw_ref_nearest
        lat_err = np.array([lat_err]).flatten()[0]
        yaw_err = np.array([yaw_err]).flatten()[0]
        while True:
            if yaw_err > np.pi:
                yaw_err -= 2.0 * np.pi
            if yaw_err < (-np.pi):
                yaw_err += 2.0 * np.pi
            if np.abs(yaw_err) < np.pi:
                break

        lookahead = lookahead_intercept + lookahead_coef * np.abs(longitudinal_vel_obs)
        linearized_pure_pursuit_steer_kp = (
            linearized_pure_pursuit_steer_kp_param * wheel_base / (lookahead * lookahead)
        )
        linearized_pure_pursuit_steer_kd = (
            linearized_pure_pursuit_steer_kd_param * wheel_base / lookahead
        )
        pure_pursuit_steer_cmd = (
            -linearized_pure_pursuit_steer_kp * lat_err - linearized_pure_pursuit_steer_kd * yaw_err
        )
        return np.array([pure_pursuit_acc_cmd, pure_pursuit_steer_cmd])

    def control(self):
        # [0] receive topic
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

        trajectory_position = []
        trajectory_orientation = []
        trajectory_longitudinal_velocity = []
        points = self._present_trajectory.points
        for i in range(len(points)):
            trajectory_position.append(
                [points[i].pose.position.x, points[i].pose.position.y, points[i].pose.position.z]
            )
            trajectory_orientation.append(
                [
                    points[i].pose.orientation.x,
                    points[i].pose.orientation.y,
                    points[i].pose.orientation.z,
                    points[i].pose.orientation.w,
                ]
            )
            trajectory_longitudinal_velocity.append(points[i].longitudinal_velocity_mps)
        trajectory_position = np.array(trajectory_position)
        trajectory_orientation = np.array(trajectory_orientation)
        trajectory_longitudinal_velocity = np.array(trajectory_longitudinal_velocity)

        is_applying_control = False
        if self._present_operation_mode is not None:
            if (
                self._present_operation_mode.mode == 3
                and self._present_operation_mode.is_autoware_control_enabled
            ):
                is_applying_control = True

        nearestIndex = (
            ((trajectory_position[:, :2] - present_position[:2]) ** 2).sum(axis=1).argmin()
        )

        # prepare noise
        if len(self.acc_noise_list) == 0:
            tmp_noise_amp = (
                np.random.rand()
                * self.get_parameter("acc_noise_amp").get_parameter_value().double_value
            )
            noise_min_period = (
                self.get_parameter("acc_noise_min_period").get_parameter_value().double_value
            )
            noise_max_period = (
                self.get_parameter("acc_noise_max_period").get_parameter_value().double_value
            )
            tmp_noise_period = noise_min_period + np.random.rand() * (
                noise_max_period - noise_min_period
            )
            dt = self.timer_period_callback
            noise_data_num = max(4, int(tmp_noise_period / dt))
            for i in range(noise_data_num):
                self.acc_noise_list.append(tmp_noise_amp * np.sin(2.0 * np.pi * i / noise_data_num))
        if len(self.steer_noise_list) == 0:
            tmp_noise_amp = (
                np.random.rand()
                * self.get_parameter("steer_noise_amp").get_parameter_value().double_value
            )
            noise_min_period = (
                self.get_parameter("steer_noise_min_period").get_parameter_value().double_value
            )
            noise_max_period = (
                self.get_parameter("steer_noise_max_period").get_parameter_value().double_value
            )
            tmp_noise_period = noise_min_period + np.random.rand() * (
                noise_max_period - noise_min_period
            )

            dt = self.timer_period_callback
            noise_data_num = max(4, int(tmp_noise_period / dt))
            for i in range(noise_data_num):
                self.steer_noise_list.append(
                    tmp_noise_amp * np.sin(2.0 * np.pi * i / noise_data_num)
                )

        # [1] compute control
        targetIndex = 1 * nearestIndex
        lookahead_coef = self.get_parameter("lookahead_time").get_parameter_value().double_value
        lookahead_intercept = self.get_parameter("min_lookahead").get_parameter_value().double_value
        pure_pursuit_lookahead_length = (
            lookahead_coef * present_linear_velocity[0] + lookahead_intercept
        )

        while True:
            tmp_distance = np.sqrt(
                ((trajectory_position[targetIndex][:2] - present_position[:2]) ** 2).sum()
            )
            if tmp_distance > pure_pursuit_lookahead_length:
                break
            if targetIndex == (len(trajectory_position) - 1):
                break
            targetIndex += 1

        pure_pursuit_type = (
            self.get_parameter("pure_pursuit_type").get_parameter_value().string_value
        )

        cmd = np.zeros(2)
        if pure_pursuit_type == "naive":
            cmd = self.pure_pursuit_control(
                present_position[:2],
                present_yaw,
                present_linear_velocity[0],
                trajectory_position[targetIndex][:2],
                trajectory_longitudinal_velocity[nearestIndex],
            )
        elif pure_pursuit_type == "linearized":
            cmd = self.linearized_pure_pursuit_control(
                present_position[:2],
                present_yaw,
                present_linear_velocity[0],
                trajectory_position[targetIndex][:2],
                getYaw(trajectory_orientation[targetIndex]),
                trajectory_longitudinal_velocity[nearestIndex],
            )
        else:
            self.get_logger().info(
                'pure_pursuit_type should be "naive" or "linearized" but is set to "%s" '
                % pure_pursuit_type
            )

        cmd_without_noise = 1 * cmd

        tmp_acc_noise = self.acc_noise_list.pop(0)
        tmp_steer_noise = self.steer_noise_list.pop(0)

        cmd[0] += tmp_acc_noise
        cmd[1] += tmp_steer_noise

        # overwrite control_cmd if received stop request
        if not self.stop_request:
            pass
        else:
            stop_acc = self.get_parameter("stop_acc").get_parameter_value().double_value
            if stop_acc > 0.0:
                self.get_logger().info("stop_acc should be negative! Force set to -1.0")
                stop_acc = -1.0
            stop_jerk_lim = self.get_parameter("stop_jerk_lim").get_parameter_value().double_value
            cmd[0] = max(
                stop_acc, self._previous_cmd[0] - stop_jerk_lim * self.timer_period_callback
            )
            cmd[1] = 1 * self._previous_cmd[1]

        # apply control_cmd limit
        lon_acc_lim = self.get_parameter("lon_acc_lim").get_parameter_value().double_value
        steer_lim = self.get_parameter("steer_lim").get_parameter_value().double_value
        cmd[0] = np.clip(cmd[0], -lon_acc_lim, lon_acc_lim)
        cmd[1] = np.clip(cmd[1], -steer_lim, steer_lim)
        acc_diff_limit = (
            self.get_parameter("lon_jerk_lim").get_parameter_value().double_value
            * self.timer_period_callback
        )
        steer_diff_limit = (
            self.get_parameter("steer_rate_lim").get_parameter_value().double_value
            * self.timer_period_callback
        )
        cmd[0] = np.clip(
            cmd[0], self._previous_cmd[0] - acc_diff_limit, self._previous_cmd[0] + acc_diff_limit
        )
        cmd[1] = np.clip(
            cmd[1],
            self._previous_cmd[1] - steer_diff_limit,
            self._previous_cmd[1] + steer_diff_limit,
        )

        if is_applying_control:
            self._previous_cmd = cmd.copy()
        else:
            self._previous_cmd = np.zeros(2)

        # [2] publish cmd
        control_cmd_msg = AckermannControlCommand()
        control_cmd_msg.stamp = control_cmd_msg.lateral.stamp = (
            control_cmd_msg.longitudinal.stamp
        ) = (self.get_clock().now().to_msg())
        control_cmd_msg.longitudinal.velocity = trajectory_longitudinal_velocity[nearestIndex]
        control_cmd_msg.longitudinal.acceleration = cmd[0]
        control_cmd_msg.lateral.steering_tire_angle = cmd[1]

        self.control_cmd_pub_.publish(control_cmd_msg)

        gear_cmd_msg = GearCommand()
        gear_cmd_msg.stamp = control_cmd_msg.lateral.stamp
        gear_cmd_msg.command = GearCommand.DRIVE
        self.gear_cmd_pub_.publish(gear_cmd_msg)

        # [3] publish marker
        marker_array = MarkerArray()

        marker_traj = Marker()
        marker_traj.type = 4
        marker_traj.id = 1
        marker_traj.header.frame_id = "map"

        marker_traj.action = marker_traj.ADD

        marker_traj.scale.x = 0.6
        marker_traj.scale.y = 0.0
        marker_traj.scale.z = 0.0

        marker_traj.color.a = 1.0
        marker_traj.color.r = 0.0
        marker_traj.color.g = 1.0
        marker_traj.color.b = 0.0

        marker_traj.lifetime.nanosec = 500000000
        marker_traj.frame_locked = True

        marker_traj.points = []
        tmp_marker_point = Point()
        tmp_marker_point.x = present_position[0]
        tmp_marker_point.y = present_position[1]
        tmp_marker_point.z = 0.0
        marker_traj.points.append(tmp_marker_point)
        tmp_marker_point = Point()
        tmp_marker_point.x = trajectory_position[targetIndex][0]
        tmp_marker_point.y = trajectory_position[targetIndex][1]
        tmp_marker_point.z = 0.0
        marker_traj.points.append(tmp_marker_point)

        marker_array.markers.append(marker_traj)
        self.data_collecting_lookahead_marker_array_pub_.publish(marker_array)

        # [99] debug plot
        if debug_matplotlib_plot_flag:
            self.acc_history.append(1 * cmd[0])
            self.steer_history.append(1 * cmd[1])
            self.acc_noise_history.append(tmp_acc_noise)
            self.steer_noise_history.append(tmp_steer_noise)
            max_plot_len = 666
            if len(self.acc_history) > max_plot_len:
                self.acc_history.pop(0)
                self.steer_history.pop(0)
                self.acc_noise_history.pop(0)
                self.steer_noise_history.pop(0)
            dt = self.timer_period_callback
            timestamp = -dt * np.array(range(len(self.steer_history)))[::-1]
            plt.cla()
            plt.clf()
            plt.subplot(2, 1, 1)
            plt.plot(0, cmd[0], "o", label="current_acc")
            plt.plot(0, cmd_without_noise[0], "o", label="acc cmd without noise")
            plt.plot(timestamp, self.acc_history, "-", label="acc cmd history")
            plt.plot(timestamp, self.acc_noise_history, "-", label="acc noise history")
            plt.xlim([-20.5, 0.5])
            plt.ylim([-1, 3])
            plt.ylabel("acc [m/ss]")
            plt.legend()
            plt.subplot(2, 1, 2)
            plt.plot(0, cmd[1], "o", label="current_steer")
            plt.plot(0, cmd_without_noise[1], "o", label="steer without noise")
            plt.plot(timestamp, self.steer_history, "-", label="steer cmd history")
            plt.plot(timestamp, self.steer_noise_history, "-", label="steer noise history")
            plt.xlim([-20.5, 0.5])
            plt.ylim([-1.25, 1.25])
            plt.xlabel("future timestamp [s]")
            plt.ylabel("steer [rad]")
            plt.legend()
            plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)

    data_collecting_pure_pursuit_trajectory_follower = DataCollectingPurePursuitTrajectoryFollower()

    rclpy.spin(data_collecting_pure_pursuit_trajectory_follower)

    data_collecting_pure_pursuit_trajectory_follower.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
