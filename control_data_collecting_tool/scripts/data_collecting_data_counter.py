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

from collections import deque
import os

from data_collecting_base_node import DataCollectingBaseNode
import numpy as np
from numpy import arctan2
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
import rosbag_play
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import MultiArrayLayout


def publish_Int32MultiArray(publisher_, array_data):
    msg = Int32MultiArray()

    flattened_data = array_data.flatten().tolist()
    msg.data = flattened_data

    layout = MultiArrayLayout()

    dim1 = MultiArrayDimension()
    dim1.label = "rows"
    dim1.size = len(array_data)
    dim1.stride = len(flattened_data)

    dim2 = MultiArrayDimension()
    dim2.label = "cols"
    dim2.size = len(array_data[0])
    dim2.stride = len(array_data[0])

    layout.dim.append(dim1)
    layout.dim.append(dim2)

    msg.layout = layout

    publisher_.publish(msg)


# inherits from DataCollectingBaseNode
class DataCollectingDataCounter(DataCollectingBaseNode):
    def __init__(self):
        super().__init__("data_collecting_data_counter")

        self.vel_hist = deque([float(0.0)] * 200, maxlen=200)
        self.acc_hist = deque([float(0.0)] * 200, maxlen=200)
        self.previous_steer = 0.0
        self.previous_acc = 0.0

        self.timer_period_callback = 0.033
        self.timer_counter = self.create_timer(
            self.timer_period_callback,
            self.timer_callback_counter,
        )

        self.collected_data_counts_of_vel_acc_publisher_ = self.create_publisher(
            Int32MultiArray, "/control_data_collecting_tools/collected_data_counts_of_vel_acc", 10
        )
        self.collected_data_counts_of_vel_steer_publisher_ = self.create_publisher(
            Int32MultiArray, "/control_data_collecting_tools/collected_data_counts_of_vel_steer", 10
        )
        self.collected_data_counts_of_vel_abs_steer_rate_publisher_ = self.create_publisher(
            Int32MultiArray,
            "/control_data_collecting_tools/collected_data_counts_of_vel_abs_steer_rate",
            10,
        )
        self.collected_data_counts_of_vel_jerk_publisher_ = self.create_publisher(
            Int32MultiArray, "/control_data_collecting_tools/collected_data_counts_of_vel_jerk", 10
        )

        self.collected_data_counts_of_vel_accel_pedal_input_publisher_ = self.create_publisher(
            Int32MultiArray,
            "/control_data_collecting_tools/collected_data_counts_of_vel_accel_pedal_input",
            10,
        )

        self.collected_data_counts_of_vel_brake_pedal_input_publisher_ = self.create_publisher(
            Int32MultiArray,
            "/control_data_collecting_tools/collected_data_counts_of_vel_brake_pedal_input",
            10,
        )

        self.collected_data_counts_of_vel_jerk_publisher_ = self.create_publisher(
            Int32MultiArray, "/control_data_collecting_tools/collected_data_counts_of_vel_jerk", 10
        )

        self.vel_hist_publisher_ = self.create_publisher(
            Float32MultiArray, "/control_data_collecting_tools/vel_hist", 10
        )
        self.acc_hist_publisher_ = self.create_publisher(
            Float32MultiArray, "/control_data_collecting_tools/acc_hist", 10
        )

        self.declare_parameter(
            "LOAD_ROSBAG2_FILES",
            True,
            ParameterDescriptor(
                description="Flag that determines whether to load rosbag2 data or not"
            ),
        )

        load_rosbag2_files = (
            self.get_parameter("LOAD_ROSBAG2_FILES").get_parameter_value().bool_value
        )

        self.declare_parameter(
            "STEER_THRESHOLD_FOR_PEDAL_INPUT_COUNT",
            0.2,
            ParameterDescriptor(
                description="Threshold of steering angle to count pedal input data"
            ),
        )

        self.steer_threshold_for_pedal_count = self.get_parameter("STEER_THRESHOLD_FOR_PEDAL_INPUT_COUNT").get_parameter_value().double_value

        if load_rosbag2_files:
            # candidates referencing the rosbag data
            rosbag2_dir_list = [d for d in os.listdir("./") if os.path.isdir(os.path.join("./", d))]
            # load rosbag data
            self.load_rosbag_data(rosbag2_dir_list)

    def load_rosbag_data(self, rosbag2_dir_list):
        for rosbag2_dir in rosbag2_dir_list:
            # try to fetch /localization/acceleration and /localization/kinematic_state from rosbag2_file
            rosbag2_file = "./" + rosbag2_dir + "/" + rosbag2_dir + "_0.db3"
            db3reader = rosbag_play.db3Reader(rosbag2_file)

            load_acc_topic = db3reader.load_db3("/localization/acceleration")
            load_kinematic_topic = db3reader.load_db3("/localization/kinematic_state")

            # if /localization/acceleration or /localization/kinematic_state is not include in the data base skip counting data points
            if load_acc_topic and load_kinematic_topic:
                acceleration, kinematic_state = db3reader.read_msg(
                    "/localization/acceleration"
                ), db3reader.read_msg("/localization/kinematic_state")
                if acceleration is None or kinematic_state is None:
                    continue

                previous_acc, previous_vel, previous_ang_vel_z = (
                    acceleration.accel.accel.linear.x,
                    kinematic_state.twist.twist.linear.x,
                    kinematic_state.twist.twist.angular.z,
                )
                previous_acc_time, previous_kinematic_time = (
                    acceleration.header.stamp.sec + 1e-9 * acceleration.header.stamp.nanosec,
                    kinematic_state.header.stamp.sec + 1e-9 * kinematic_state.header.stamp.nanosec,
                )
                current_time = max([previous_acc_time, previous_kinematic_time])

                acceleration, kinematic_state = db3reader.read_msg(
                    "/localization/acceleration"
                ), db3reader.read_msg("/localization/kinematic_state")
                if acceleration is None or kinematic_state is None:
                    break
                current_acc_time, current_kinematic_time = (
                    acceleration.header.stamp.sec + 1e-9 * acceleration.header.stamp.nanosec,
                    kinematic_state.header.stamp.sec + 1e-9 * kinematic_state.header.stamp.nanosec,
                )

                # A while loop for counting data points
                while True:
                    # interpolate acceleration if necessary
                    while current_time > current_acc_time:
                        previous_acc = acceleration.accel.accel.linear.x
                        acceleration = db3reader.read_msg("/localization/acceleration")
                        if acceleration is None:
                            break
                        previous_acc_time = current_acc_time
                        current_acc_time = (
                            acceleration.header.stamp.sec + 1e-9 * acceleration.header.stamp.nanosec
                        )

                    if acceleration is None:
                        break
                    acc = (current_acc_time - current_time) * acceleration.accel.accel.linear.x + (
                        current_time - previous_acc_time
                    ) * previous_acc
                    acc /= current_acc_time - previous_acc_time

                    # interpolate kinematic state if necessary
                    while current_time > current_kinematic_time:
                        previous_vel, previous_ang_vel_z = (
                            kinematic_state.twist.twist.linear.x,
                            kinematic_state.twist.twist.angular.z,
                        )
                        kinematic_state = db3reader.read_msg("/localization/kinematic_state")
                        if kinematic_state is None:
                            break
                        previous_kinematic_time = current_kinematic_time
                        current_kinematic_time = (
                            kinematic_state.header.stamp.sec
                            + 1e-9 * kinematic_state.header.stamp.nanosec
                        )

                    if kinematic_state is None:
                        break
                    vel = (
                        current_kinematic_time - current_time
                    ) * kinematic_state.twist.twist.linear.x + (
                        current_time - previous_kinematic_time
                    ) * previous_vel
                    vel /= current_kinematic_time - previous_kinematic_time

                    ang_vel_z = (
                        current_kinematic_time - current_time
                    ) * kinematic_state.twist.twist.angular.z + (
                        current_time - previous_kinematic_time
                    ) * previous_ang_vel_z
                    ang_vel_z /= current_kinematic_time - previous_kinematic_time

                    # calculation of steer
                    wheel_base = self.get_parameter("wheel_base").get_parameter_value().double_value
                    steer = arctan2(wheel_base * ang_vel_z, kinematic_state.twist.twist.linear.x)

                    # count number of data
                    if kinematic_state.twist.twist.linear.x > 1e-3:
                        self.count_observations(
                            vel,
                            acc,
                            steer,
                            steer_rate=(steer - self.previous_steer) / 0.033,
                            jerk=(acc - self.previous_acc) / 0.033,
                        )
                    self.previous_steer = steer
                    self.previous_acc = acc
                    current_time += self.timer_period_callback

                self.previous_steer = 0.0
                self.previous_acc = 0.0

    def count_observations(self, v, a, steer, steer_rate, jerk):
        v_bin = np.digitize(v, self.v_bins) - 1
        steer_bin = np.digitize(steer, self.steer_bins) - 1
        a_bin = np.digitize(a, self.a_bins) - 1
        abs_steer_rate_bin = np.digitize(abs(steer_rate), self.abs_steer_rate_bins) - 1
        jerk_bin = np.digitize(jerk, self.jerk_bins) - 1

        if 0 <= v_bin < self.num_bins_v and 0 <= a_bin < self.num_bins_a:
            self.collected_data_counts_of_vel_acc[v_bin, a_bin] += 1

        if 0 <= v_bin < self.num_bins_v and 0 <= steer_bin < self.num_bins_steer:
            self.collected_data_counts_of_vel_steer[v_bin, steer_bin] += 1

        if 0 <= v_bin < self.num_bins_v and 0 <= abs_steer_rate_bin < self.num_bins_abs_steer_rate:
            self.collected_data_counts_of_vel_abs_steer_rate[v_bin, abs_steer_rate_bin] += 1

        if 0 <= v_bin < self.num_bins_v and 0 <= jerk_bin < self.num_bins_jerk:
            self.collected_data_counts_of_vel_jerk[v_bin, jerk_bin] += 1

    def count_pedal_input_observation(self, actuation_cmd, current_vel):
        if actuation_cmd is not None:
            accel_pedal_input = actuation_cmd.actuation.accel_cmd
            brake_pedal_input = actuation_cmd.actuation.brake_cmd
            accel_pedal_input_bin = (
                np.digitize(accel_pedal_input, self.accel_pedal_input_bin_centers) - 1
            )
            brake_pedal_input_bin = (
                np.digitize(brake_pedal_input, self.brake_pedal_input_bin_centers) - 1
            )
            v_bin = np.digitize(current_vel, self.v_bins) - 1

            if accel_pedal_input > 1e-3:
                if (
                    0 <= v_bin < self.num_bins_v
                    and 0 <= accel_pedal_input_bin < self.num_bins_accel_pedal_input
                ):
                    self.collected_data_counts_of_vel_accel_pedal_input[
                        v_bin, accel_pedal_input_bin
                    ] += 1

            if brake_pedal_input > 1e-3:
                if (
                    0 <= v_bin < self.num_bins_v
                    and 0 <= brake_pedal_input_bin < self.num_bins_brake_pedal_input
                ):
                    self.collected_data_counts_of_vel_brake_pedal_input[
                        v_bin, brake_pedal_input_bin
                    ] += 1

    # call back for counting data points
    def timer_callback_counter(self):
        if (
            self._present_kinematic_state is not None
            and self._present_acceleration is not None
            and self.present_operation_mode_ == 3
            and self._present_control_mode_ == 1
        ):
            # calculate steer
            angular_z = self._present_kinematic_state.twist.twist.angular.z
            wheel_base = self.get_parameter("wheel_base").get_parameter_value().double_value
            current_steer = arctan2(
                wheel_base * angular_z, self._present_kinematic_state.twist.twist.linear.x
            )
            current_vel = self._present_kinematic_state.twist.twist.linear.x
            current_acc = self._present_acceleration.accel.accel.linear.x
            current_steer_rate = (current_steer - self.previous_steer) / 0.033
            current_jerk = (current_acc - self.previous_acc) / 0.033
            pedal_input = self._present_actuation_cmd

            self.previous_steer = current_steer
            self.previous_acc = current_acc

            if self._present_kinematic_state.twist.twist.linear.x > 1e-3:
                self.count_observations(
                    current_vel, current_acc, current_steer, current_steer_rate, current_jerk
                )
                if abs(current_steer) < self.steer_threshold_for_pedal_count:
                    self.count_pedal_input_observation(pedal_input, current_vel)

                self.acc_hist.append(float(current_acc))
                self.vel_hist.append(float(current_vel))

        # publish collected_data_counts_of_vel_acc
        publish_Int32MultiArray(
            self.collected_data_counts_of_vel_acc_publisher_, self.collected_data_counts_of_vel_acc
        )

        # publish collected_data_counts_of_vel_steer
        publish_Int32MultiArray(
            self.collected_data_counts_of_vel_steer_publisher_,
            self.collected_data_counts_of_vel_steer,
        )

        # publish collected_data_counts_of_vel_abs_steer_rate
        publish_Int32MultiArray(
            self.collected_data_counts_of_vel_abs_steer_rate_publisher_,
            self.collected_data_counts_of_vel_abs_steer_rate,
        )

        # publish collected_data_counts_of_vel_jerk
        publish_Int32MultiArray(
            self.collected_data_counts_of_vel_jerk_publisher_,
            self.collected_data_counts_of_vel_jerk,
        )

        #
        publish_Int32MultiArray(
            self.collected_data_counts_of_vel_accel_pedal_input_publisher_,
            self.collected_data_counts_of_vel_accel_pedal_input,
        )

        publish_Int32MultiArray(
            self.collected_data_counts_of_vel_brake_pedal_input_publisher_,
            self.collected_data_counts_of_vel_brake_pedal_input,
        )

        # publish acc_hist
        msg = Float32MultiArray()
        msg.data = list(self.acc_hist)
        self.acc_hist_publisher_.publish(msg)

        # publish vel_hist
        msg = Float32MultiArray()
        msg.data = list(self.vel_hist)
        self.vel_hist_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    data_collecting_data_counter = DataCollectingDataCounter()
    rclpy.spin(data_collecting_data_counter)

    data_collecting_data_counter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
