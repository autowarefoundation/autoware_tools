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

from data_collecting_base_node import DataCollectingBaseNode
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import seaborn as sns
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray


class DataCollectingPlotter(DataCollectingBaseNode):
    def __init__(self):
        super().__init__("data_collecting_plotter")

        # callback for plot
        self.grid_update_time_interval = 5.0
        self.timer_plotter = self.create_timer(
            self.grid_update_time_interval,
            self.timer_callback_plotter,
        )

        self.fig, self.axs = plt.subplots(3, 1, figsize=(12, 20))
        plt.ion()

        self.collected_data_counts_of_vel_acc_subscriber_ = self.create_subscription(
            Int32MultiArray,
            "/control_data_collecting_tools/collected_data_counts_of_vel_acc",
            self.subscribe_collected_data_counts_of_vel_acc,
            10,
        )
        self.collected_data_counts_of_vel_acc_subscriber_

        self.collected_data_counts_of_vel_steer_subscriber_ = self.create_subscription(
            Int32MultiArray,
            "/control_data_collecting_tools/collected_data_counts_of_vel_steer",
            self.subscribe_collected_data_counts_of_vel_steer,
            10,
        )
        self.collected_data_counts_of_vel_steer_subscriber_

        self.acc_hist_subscriber_ = self.create_subscription(
            Float32MultiArray,
            "/control_data_collecting_tools/acc_hist",
            self.subscribe_acc_hist,
            10,
        )
        self.acc_hist_subscriber_

        self.vel_hist_subscriber_ = self.create_subscription(
            Float32MultiArray,
            "/control_data_collecting_tools/vel_hist",
            self.subscribe_vel_hist,
            10,
        )
        self.vel_hist_subscriber_

        self.acc_hist = [0.0] * 200
        self.vel_hist = [0.0] * 200

    def subscribe_collected_data_counts_of_vel_acc(self, msg):
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        self.collected_data_counts_of_vel_acc = np.array(msg.data).reshape((rows, cols))

    def subscribe_collected_data_counts_of_vel_steer(self, msg):
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        self.collected_data_counts_of_vel_steer = np.array(msg.data).reshape((rows, cols))

    def subscribe_acc_hist(self, msg):
        self.acc_hist = msg.data

    def subscribe_vel_hist(self, msg):
        self.vel_hist = msg.data

    def timer_callback_plotter(self):
        self.plot_data_collection_grid()
        plt.pause(0.1)

    def plot_data_collection_grid(self):
        self.axs[0].cla()
        self.axs[0].scatter(self.acc_hist, self.vel_hist)
        self.axs[0].plot(self.acc_hist, self.vel_hist)
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
            self.collected_data_counts_of_vel_acc.T,
            annot=True,
            cmap="coolwarm",
            xticklabels=np.round(self.v_bin_centers, 2),
            yticklabels=np.round(self.a_bin_centers, 2),
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
            self.collected_data_counts_of_vel_steer.T,
            annot=True,
            cmap="coolwarm",
            xticklabels=np.round(self.v_bin_centers, 2),
            yticklabels=np.round(self.steer_bin_centers, 2),
            ax=self.axs[2],
            linewidths=0.1,
            linecolor="gray",
        )

        # update collected steer and velocity grid
        self.axs[2].set_xlabel("Velocity bins")
        self.axs[2].set_ylabel("Steer bins")

        self.fig.canvas.draw()


def main(args=None):
    rclpy.init(args=args)

    data_collecting_plot = DataCollectingPlotter()
    rclpy.spin(data_collecting_plot)

    data_collecting_plot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
