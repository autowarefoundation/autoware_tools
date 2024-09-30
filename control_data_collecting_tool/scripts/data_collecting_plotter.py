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

import threading

from geometry_msgs.msg import AccelWithCovarianceStamped
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import numpy as np
from numpy import arctan2
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import seaborn as sns


class DataCollectingPlotter(Node):
    def __init__(self):
        super().__init__("data_collecting_plotter")

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
            ParameterDescriptor(description="Maximum velocity in heatmap [m/s]"),
        )

        self.declare_parameter(
            "V_MAX",
            1.0,
            ParameterDescriptor(description="Minimum steer in heatmap [m/s]"),
        )

        self.declare_parameter(
            "STEER_MIN",
            -1.0,
            ParameterDescriptor(description="Maximum steer in heatmap [rad]"),
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
            2.79,  # sample_vehicle_launch/sample_vehicle_description/config/vehicle_info.param.yaml
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

        self._present_kinematic_state = None
        self._present_acceleration = None

        # velocity and acceleration grid
        # velocity and steer grid
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

        self.collected_data_counts_of_vel_acc = np.zeros((self.num_bins_v, self.num_bins_a))
        self.collected_data_counts_of_vel_steer = np.zeros((self.num_bins_v, self.num_bins_steer))

        self.v_bins = np.linspace(self.v_min, self.v_max, self.num_bins_v + 1)
        self.steer_bins = np.linspace(self.steer_min, self.steer_max, self.num_bins_steer + 1)
        self.a_bins = np.linspace(self.a_min, self.a_max, self.num_bins_a + 1)

        self.v_bin_centers = (self.v_bins[:-1] + self.v_bins[1:]) / 2
        self.steer_bin_centers = (self.steer_bins[:-1] + self.steer_bins[1:]) / 2
        self.a_bin_centers = (self.a_bins[:-1] + self.a_bins[1:]) / 2

        self.vel_hist = np.zeros(200)
        self.acc_hist = np.zeros(200)

        self.vel_hist_for_plot = np.zeros(200)
        self.acc_hist_for_plot = np.zeros(200)

        # create callback groups
        self.callback_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()

        # callback for data count
        self.timer_period_callback = 0.033  # 30ms
        self.timer_counter = self.create_timer(
            self.timer_period_callback,
            self.timer_callback_counter,
            callback_group=self.callback_group,
        )

        # callback for plot
        self.grid_update_time_interval = 5.0
        self.timer_plotter = self.create_timer(
            self.grid_update_time_interval,
            self.timer_callback_plotter,
            callback_group=self.callback_group,
        )

        self.fig, self.axs = plt.subplots(3, 1, figsize=(12, 20))
        plt.ion()

        self.collected_data_counts_of_vel_acc_for_plot = np.zeros(
            (self.num_bins_v, self.num_bins_a)
        )
        self.collected_data_counts_of_vel_steer_for_plot = np.zeros(
            (self.num_bins_v, self.num_bins_steer)
        )

        self.v_bin_centers_for_plot = (self.v_bins[:-1] + self.v_bins[1:]) / 2
        self.steer_bin_centers_for_plot = (self.steer_bins[:-1] + self.steer_bins[1:]) / 2
        self.a_bin_centers_for_plot = (self.a_bins[:-1] + self.a_bins[1:]) / 2

    def onOdometry(self, msg):
        self._present_kinematic_state = msg

    def onAcceleration(self, msg):
        self._present_acceleration = msg

    def count_observations(self, v, a, steer):
        v_bin = np.digitize(v, self.v_bins) - 1
        steer_bin = np.digitize(steer, self.steer_bins) - 1
        a_bin = np.digitize(a, self.a_bins) - 1

        if 0 <= v_bin < self.num_bins_v and 0 <= a_bin < self.num_bins_a:
            self.collected_data_counts_of_vel_acc[v_bin, a_bin] += 1

        if 0 <= v_bin < self.num_bins_v and 0 <= steer_bin < self.num_bins_steer:
            self.collected_data_counts_of_vel_steer[v_bin, steer_bin] += 1

    def timer_callback_plotter(self):
        with self.lock:
            self.collected_data_counts_of_vel_acc_for_plot = (
                self.collected_data_counts_of_vel_acc.copy()
            )
            self.collected_data_counts_of_vel_steer_for_plot = (
                self.collected_data_counts_of_vel_steer.copy()
            )

            self.acc_hist_for_plot = self.acc_hist.copy()
            self.vel_hist_for_plot = self.vel_hist.copy()

            self.v_bin_centers_for_plot = self.v_bin_centers
            self.steer_bin_centers_for_plot = self.steer_bin_centers
            self.a_bin_centers_for_plot = self.a_bin_centers

        self.plot_data_collection_grid()
        plt.pause(0.01)

    def timer_callback_counter(self):
        if self._present_kinematic_state is not None and self._present_acceleration is not None:
            # calculate steer
            angular_z = self._present_kinematic_state.twist.twist.angular.z
            wheel_base = self.get_parameter("wheel_base").get_parameter_value().double_value
            current_steer = arctan2(
                wheel_base * angular_z, self._present_kinematic_state.twist.twist.linear.x
            )

            current_vel = self._present_kinematic_state.twist.twist.linear.x
            current_acc = self._present_acceleration.accel.accel.linear.x

            # update velocity and acceleration bin if ego vehicle is moving
            if self._present_kinematic_state.twist.twist.linear.x > 1e-3:
                with self.lock:
                    self.count_observations(
                        current_vel,
                        current_acc,
                        current_steer,
                    )

                    self.acc_hist[:-1] = 1.0 * self.acc_hist[1:]
                    self.acc_hist[-1] = current_acc
                    self.vel_hist[:-1] = 1.0 * self.vel_hist[1:]
                    self.vel_hist[-1] = current_vel

    def plot_data_collection_grid(self):
        self.axs[0].cla()
        self.axs[0].scatter(self.acc_hist_for_plot, self.vel_hist_for_plot)
        self.axs[0].plot(self.acc_hist_for_plot, self.vel_hist_for_plot)
        self.axs[0].set_xlim([-2.0, 2.0])
        self.axs[0].set_ylim([0.0, self.v_max + 1.0])
        self.axs[0].set_xlabel("Acceleration")
        self.axs[0].set_ylabel("Velocity")

        # update collected acceleration and velocity grid
        for collection in self.axs[1].collections:
            if collection.colorbar is not None:
                collection.colorbar.remove()
        self.axs[1].cla()

        self.heatmap = sns.heatmap(
            self.collected_data_counts_of_vel_acc_for_plot.T,
            annot=True,
            cmap="coolwarm",
            xticklabels=np.round(self.v_bin_centers_for_plot, 2),
            yticklabels=np.round(self.a_bin_centers_for_plot, 2),
            ax=self.axs[1],
            linewidths=0.1,
            linecolor="gray",
        )

        self.axs[1].set_xlabel("Velocity bins")
        self.axs[1].set_ylabel("Acceleration bins")

        for collection in self.axs[2].collections:
            if collection.colorbar is not None:
                collection.colorbar.remove()
        self.axs[2].cla()

        self.heatmap = sns.heatmap(
            self.collected_data_counts_of_vel_steer_for_plot.T,
            annot=True,
            cmap="coolwarm",
            xticklabels=np.round(self.v_bin_centers_for_plot, 2),
            yticklabels=np.round(self.steer_bin_centers_for_plot, 2),
            ax=self.axs[2],
            linewidths=0.1,
            linecolor="gray",
        )

        # update collected steer and velocity grid
        self.axs[2].set_xlabel("Velocity bins")
        self.axs[2].set_ylabel("Steer bins")

        self.fig.canvas.draw()

        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)

    data_collecting_plot = DataCollectingPlotter()
    rclpy.spin(data_collecting_plot)

    data_collecting_plot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
