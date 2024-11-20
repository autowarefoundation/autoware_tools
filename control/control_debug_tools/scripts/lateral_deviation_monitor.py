#!/usr/bin/env python3

# Copyright 2024 TIER IV, Inc.
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

import time

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import SteeringReport
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from termcolor import colored
from tier4_metric_msgs.msg import MetricArray


class SteeringAndLateralDeviationMonitor(Node):
    def __init__(self, plot=False):
        super().__init__("steering_and_lateral_deviation_monitor")

        self.create_subscription(
            Control, "/control/command/control_cmd", self.control_cmd_callback, 10
        )

        self.create_subscription(
            SteeringReport, "/vehicle/status/steering_status", self.steering_status_callback, 10
        )

        self.create_subscription(
            MetricArray, "/control/control_evaluator/metrics", self.metrics_callback, 10
        )

        self.control_steering_angle = None
        self.vehicle_steering_angle = None
        self.lateral_deviation = None
        self.steering_diff = None

        # For tracking max values
        self.max_control_steering_angle = 0
        self.max_vehicle_steering_angle = 0
        self.max_lateral_deviation = 0
        self.max_steering_diff = 0

        # For plotting
        self.plot = plot
        if self.plot:
            self.start_time = time.time()
            self.time_data = []
            self.control_steering_data = []
            self.vehicle_steering_data = []
            self.lateral_deviation_data = []
            self.steering_diff_data = []
            self.fig, self.ax = plt.subplots(3, 1, figsize=(10, 10))

            # Set the window title
            self.fig.canvas.manager.set_window_title("Steering and Lateral Deviation Monitor")

    def control_cmd_callback(self, msg):
        self.control_steering_angle = msg.lateral.steering_tire_angle
        self.update_steering_diff()
        self.update_max_values()
        self.display_values()

    def steering_status_callback(self, msg):
        self.vehicle_steering_angle = msg.steering_tire_angle
        self.update_steering_diff()
        self.update_max_values()

    def metrics_callback(self, msgs):
        for msg in msgs.metric_array:
            if msg.name == "lateral_deviation":
                self.lateral_deviation = float(msg.value)
                self.update_max_values()

    def update_steering_diff(self):
        if self.control_steering_angle is not None and self.vehicle_steering_angle is not None:
            self.steering_diff = self.control_steering_angle - self.vehicle_steering_angle

    def update_max_values(self):
        if self.control_steering_angle is not None:
            self.max_control_steering_angle = max(
                self.max_control_steering_angle, abs(self.control_steering_angle)
            )
        if self.vehicle_steering_angle is not None:
            self.max_vehicle_steering_angle = max(
                self.max_vehicle_steering_angle, abs(self.vehicle_steering_angle)
            )
        if self.lateral_deviation is not None:
            self.max_lateral_deviation = max(
                self.max_lateral_deviation, abs(self.lateral_deviation)
            )
        if self.steering_diff is not None:
            self.max_steering_diff = max(self.max_steering_diff, abs(self.steering_diff))

    def display_values(self):
        if (
            self.control_steering_angle is not None
            and self.vehicle_steering_angle is not None
            and self.lateral_deviation is not None
            and self.steering_diff is not None
        ):
            self.get_logger().info(
                f"\n{'-'*40}\n"
                f"{colored('Control Steering Angle: ', 'red')}{self.control_steering_angle:.2f} [rad] (Max: {self.max_control_steering_angle:.2f} [rad])\n"
                f"{colored('Vehicle Steering Angle: ', 'green')}{self.vehicle_steering_angle:.2f} [rad] (Max: {self.max_vehicle_steering_angle:.2f} [rad])\n"
                f"{colored('Steering Diff: ', 'magenta')}{self.steering_diff:.2f} [rad] (Max: {self.max_steering_diff:.2f} [rad])\n"
                f"{colored('Lateral Deviation: ', 'blue')}{self.lateral_deviation:.4f} [m] (Max: {self.max_lateral_deviation:.4f} [m])\n"
                f"{'-'*40}\n"
            )
            if self.plot:
                self.update_plot()

    def update_plot(self):
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        self.control_steering_data.append(self.control_steering_angle)
        self.vehicle_steering_data.append(self.vehicle_steering_angle)
        self.lateral_deviation_data.append(self.lateral_deviation)
        self.steering_diff_data.append(self.steering_diff)

        # Keep only the last specified seconds of data
        if current_time > 60:
            while self.time_data and self.time_data[0] < current_time - 30:
                self.time_data.pop(0)
                self.control_steering_data.pop(0)
                self.vehicle_steering_data.pop(0)
                self.lateral_deviation_data.pop(0)
                self.steering_diff_data.pop(0)

        # Clear previous data
        self.ax[0].clear()
        self.ax[1].clear()
        self.ax[2].clear()

        # Plot steering angles on the same graph
        self.ax[0].plot(
            self.time_data, self.control_steering_data, label="Control Steering Angle", color="red"
        )
        self.ax[0].plot(
            self.time_data,
            self.vehicle_steering_data,
            label="Vehicle Steering Angle",
            color="green",
        )
        self.ax[0].set_ylabel("Steering Angle [rad]")
        self.ax[0].set_xlabel("Time [s]")
        self.ax[0].legend()

        # Plot steering difference on a separate graph
        self.ax[1].plot(
            self.time_data, self.steering_diff_data, label="Steering Diff", color="magenta"
        )
        self.ax[1].set_ylabel("Steering Diff [rad]")
        self.ax[1].set_xlabel("Time [s]")
        self.ax[1].legend()

        # Plot lateral deviation on another graph
        self.ax[2].plot(
            self.time_data, self.lateral_deviation_data, label="Lateral Deviation", color="blue"
        )
        self.ax[2].set_ylabel("Lateral Deviation [m]")
        self.ax[2].set_xlabel("Time [s]")
        self.ax[2].legend()

        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    monitor = SteeringAndLateralDeviationMonitor(plot=True)
    if monitor.plot:
        plt.ion()  # Interactive mode on for real-time plot updates
    rclpy.spin(monitor)

    monitor.destroy_node()
    rclpy.shutdown()
    if monitor.plot:
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
