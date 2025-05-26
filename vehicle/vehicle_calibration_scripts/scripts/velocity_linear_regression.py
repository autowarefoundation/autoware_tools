#!/usr/bin/env python3

# Copyright 2024 Tier IV, Inc. All rights reserved.
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

import argparse
import glob
import math
import os
import pprint
import re
import sys

import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.widgets import RangeSlider
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from sklearn import linear_model

# Script parameters
CAN_VELOCITY_TOPIC_NAME = "/vehicle/status/velocity_status"
DOPPLER_VELOCITY_TOPIC_NAME = "/kistler/e0_status"

ERROR_VELOCITY_FOR_RANGE_DETECTION = 2.0  # [km/h]
ERROR_VELOCITY_RATE_FOR_RANGE_DETECTION = 0.1  # [-]
COUNTER_FOR_RANGE_DETECTION = 5  # [times]


def get_topics(rosbag_path, topic_names):
    storage_options = rosbag2_py.StorageOptions(uri=rosbag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_filter = rosbag2_py.StorageFilter(topics=topic_names)
    reader.set_filter(topic_filter)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    topics = {}
    for topic_name in topic_names:
        topics[topic_name] = []

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        topics[topic].append(msg)

    for topic_name in topic_names:
        if not topics[topic_name]:
            print("Required topic is empty. Exit this program")
            print(f"topic name: {topic_name}")
            sys.exit()

    return topics


def average_within_time_range(value_array, time_array, start_time, end_time):
    filtered_value_array = []
    for value, time in zip(value_array, time_array):
        if start_time <= time <= end_time:
            filtered_value_array.append(value)

    if filtered_value_array:
        average_value = sum(filtered_value_array) / len(filtered_value_array)
        return average_value
    else:
        print("No values found within the specified time range.")
        sys.exit()


class VelocityLinearRegressor:
    def __init__(self, data_path, report_path, vehicle_name):
        self.velocity_data_paths = glob.glob(data_path + "/*.db3")
        if not self.velocity_data_paths:
            print("Velocity data is empty. Exit this program.")
            sys.exit()
        print("===== Velocity data =====")
        pprint.pprint(self.velocity_data_paths)

        self.report_path = report_path
        self.vehicle_name = vehicle_name

        self.can_velocity_array = []
        self.doppler_velocity_array = []

    def run(self):
        for velocity_data_path in self.velocity_data_paths:
            print(f"Start: {velocity_data_path}")

            topics = get_topics(
                velocity_data_path, [CAN_VELOCITY_TOPIC_NAME, DOPPLER_VELOCITY_TOPIC_NAME]
            )

            # Plot to set the processing range
            self.plot(topics, velocity_data_path)

        # Plot the result of linear regression
        self.linear_regression()
        self.plot_linear_regression()

        # Accuracy verification after applying the coefficients
        self.plot_accuracy_verification()

        # Output report
        self.report()

    def plot(self, topics, velocity_data_path):
        # Ready plot data
        can_velocity_time = []
        can_velocity_mps = []
        can_velocity_kph = []
        doppler_velocity_time = []
        doppler_velocity_mps = []
        doppler_velocity_kph = []

        for velocity_status_msg in topics[CAN_VELOCITY_TOPIC_NAME]:
            can_velocity_time.append(
                round(
                    velocity_status_msg.header.stamp.sec
                    + velocity_status_msg.header.stamp.nanosec * 1e-9,
                    3,
                )
            )
            can_velocity_mps.append(velocity_status_msg.longitudinal_velocity)
            can_velocity_kph.append(velocity_status_msg.longitudinal_velocity * 3.6)

        for doppler_velocity_msg in topics[DOPPLER_VELOCITY_TOPIC_NAME]:
            doppler_velocity_time.append(
                round(doppler_velocity_msg.stamp.sec + doppler_velocity_msg.stamp.nanosec * 1e-9, 2)
            )
            doppler_velocity_mps.append(doppler_velocity_msg.velocity_x / 3.6)
            doppler_velocity_kph.append(doppler_velocity_msg.velocity_x)

        self.start_time = can_velocity_time[0]
        self.end_time = can_velocity_time[-1]

        # Plot
        fig, ax = plt.subplots()
        plt.subplots_adjust(left=0.1, bottom=0.3)

        (can_velocity_line,) = ax.plot(can_velocity_time, can_velocity_kph, label="CAN velocity")
        (doppler_velocity_line,) = ax.plot(
            doppler_velocity_time, doppler_velocity_kph, label="Doppler velocity"
        )
        ax.set_title(os.path.basename(velocity_data_path), fontsize=11)
        ax.set_xlabel("Time [sec]")
        ax.set_ylabel("Velocity [km/h]")

        start_line = ax.axvline(x=self.start_time, color="r", linestyle="--", label="Start")
        end_line = ax.axvline(x=self.end_time, color="b", linestyle="--", label="End")

        ax.legend()

        # Range slider to select processing range
        slider_ax = fig.add_axes([0.20, 0.1, 0.60, 0.03])
        slider = RangeSlider(
            slider_ax,
            "Range",
            can_velocity_time[0],
            can_velocity_time[-1],
            valinit=(can_velocity_time[0], can_velocity_time[-1]),
        )
        slider.valtext.set_visible(False)

        def callback_slider_changed(val):
            self.start_time = val[0]
            self.end_time = val[1]
            start_line.set_xdata([self.start_time, self.start_time])
            end_line.set_xdata([self.end_time, self.end_time])

            fig.canvas.draw_idle()

        slider.on_changed(callback_slider_changed)

        # Button to start the process
        ax_button = plt.axes([0.8, 0.025, 0.1, 0.04])
        button = Button(ax_button, "Process", color="lightblue", hovercolor="0.975")

        def callback_button_clicked(event):
            self.can_velocity_array.append(
                average_within_time_range(
                    can_velocity_kph, can_velocity_time, self.start_time, self.end_time
                )
            )
            self.doppler_velocity_array.append(
                average_within_time_range(
                    doppler_velocity_kph, doppler_velocity_time, self.start_time, self.end_time
                )
            )

            filename = os.path.splitext(os.path.basename(velocity_data_path))[0] + "png"
            fig.savefig(os.path.join(self.report_path, filename))

            plt.close(fig)

        button.on_clicked(callback_button_clicked)

        plt.show()

    def linear_regression(self):
        x = np.array(self.can_velocity_array)
        y = np.array(self.doppler_velocity_array)

        # Linear regression without intercept
        model = linear_model.LinearRegression(fit_intercept=False)
        model.fit(x.reshape(-1, 1), y.reshape(-1, 1))

        self.linear_regression_coef = model.coef_[0][0]

        # Linear regression with intercept
        model = linear_model.LinearRegression()
        model.fit(x.reshape(-1, 1), y.reshape(-1, 1))

        self.linear_regression_coef_with_intercept = model.coef_[0][0]
        self.linear_regression_intercept = model.intercept_[0]

    def plot_linear_regression(self):
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8.0, 6.0))
        plt.subplots_adjust(left=0.1, bottom=0.2, wspace=0.4)

        ax1.scatter(
            self.can_velocity_array, self.doppler_velocity_array, label="velocity data", zorder=2
        )
        x = np.arange(0, 45, 5)
        y = self.linear_regression_coef * x
        ax1.plot(x, y, label="linear regression", color="r", linestyle="dashed", zorder=1)
        ax1.set_title("Linear regression without intercept", fontsize=11)
        ax1.set_xlabel("CAN velocity [km/h]")
        ax1.set_ylabel("Doppler velocity [km/h]")
        ax1.text(0.5, 0.1, f"y = {round(self.linear_regression_coef, 3)} * x")
        ax1.legend()

        ax2.scatter(
            self.can_velocity_array, self.doppler_velocity_array, label="velocity data", zorder=2
        )
        x = np.arange(0, 45, 5)
        y = self.linear_regression_coef_with_intercept * x + self.linear_regression_intercept
        ax2.plot(x, y, label="linear regression", color="r", linestyle="dashed", zorder=1)
        ax2.set_title("Linear regression with intercept", fontsize=11)
        ax2.set_xlabel("CAN velocity [km/h]")
        ax2.set_ylabel("Doppler velocity [km/h]")
        ax2.text(
            0.5,
            0.1,
            f"y = {round(self.linear_regression_coef_with_intercept, 3)} * x + {round(self.linear_regression_intercept, 3)}",
        )
        ax2.legend()

        ax_button = plt.axes([0.8, 0.025, 0.1, 0.04])
        button = Button(ax_button, "Confirm", color="lightblue", hovercolor="0.975")

        def callback_button_clicked(event):
            filename = "linear_regression.png"
            fig.savefig(os.path.join(self.report_path, filename))

            plt.close(fig)

        button.on_clicked(callback_button_clicked)

        plt.show()

    def plot_accuracy_verification(self):
        # Ready plot data
        collected_can_velocity = np.array(self.can_velocity_array) * self.linear_regression_coef
        collected_can_velocity_with_intercept = (
            np.array(self.can_velocity_array) * self.linear_regression_coef_with_intercept
            + self.linear_regression_intercept
        )
        y_error = np.array(self.doppler_velocity_array) * 0.03

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8.0, 6.0))
        plt.subplots_adjust(left=0.1, bottom=0.2, wspace=0.4)

        ax1.errorbar(
            self.can_velocity_array,
            self.doppler_velocity_array,
            yerr=y_error,
            label="criteria",
            color="g",
            capsize=3,
            linestyle="none",
            zorder=1,
        )
        ax1.scatter(
            self.can_velocity_array,
            collected_can_velocity,
            color="b",
            label="collected",
            s=5,
            zorder=3,
        )
        ax1.scatter(
            self.can_velocity_array, self.can_velocity_array, color="r", label="raw", s=5, zorder=2
        )
        ax1.set_title("Linear regression without intercept", fontsize=11)
        ax1.set_xlabel("CAN velocity [km/h]")
        ax1.set_ylabel("Collected velocity [km/h]")
        ax1.legend()

        ax2.errorbar(
            self.can_velocity_array,
            self.doppler_velocity_array,
            yerr=y_error,
            label="criteria",
            color="g",
            capsize=3,
            linestyle="none",
            zorder=1,
        )
        ax2.scatter(
            self.can_velocity_array,
            collected_can_velocity_with_intercept,
            color="b",
            label="collected",
            s=5,
            zorder=3,
        )
        ax2.scatter(
            self.can_velocity_array, self.can_velocity_array, color="r", label="raw", s=5, zorder=2
        )
        ax2.set_title("Linear regression with intercept", fontsize=11)
        ax2.set_xlabel("CAN velocity [km/h]")
        ax2.set_ylabel("Collected velocity [km/h]")
        ax2.legend()

        ax_button = plt.axes([0.8, 0.025, 0.1, 0.04])
        button = Button(ax_button, "Confirm", color="lightblue", hovercolor="0.975")

        def callback_button_clicked(event):
            filename = "accuracy_verification.png"
            fig.savefig(os.path.join(self.report_path, filename))

            plt.close(fig)

        button.on_clicked(callback_button_clicked)

        plt.show()

    def report(self):
        content = f"""## Result

### Linear regression without intercept
coefficient = {round(self.linear_regression_coef, 3)}

### Linear regression with intercept
coefficient = {round(self.linear_regression_coef_with_intercept, 3)}
intercept = {round(self.linear_regression_intercept, 3)}
"""

        filename = f"velocity_linear_regression_report_{self.vehicle_name}.md"
        with open(os.path.join(self.report_path, filename), "w") as f:
            f.write(content)


def main():
    print("===== Velocity linear regressor begin! =====")
    parser = argparse.ArgumentParser(description="Velocity linear regressor")
    parser.add_argument("path", help="path to data")
    parser.add_argument("report_path", help="path to report")
    parser.add_argument("vehicle_name", help="vehicle name in (0, 0) of output csv files")
    args = parser.parse_args()
    print(f"Data path: {args.path}")
    print(f"Report path: {args.report_path}")

    regressor = VelocityLinearRegressor(args.path, args.report_path, args.vehicle_name)
    regressor.run()


if __name__ == "__main__":
    main()
