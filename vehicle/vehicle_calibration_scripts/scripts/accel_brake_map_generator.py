#!/usr/bin/env python3

# Copyright 2023 Tier IV, Inc. All rights reserved.
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
import copy
import csv
import glob
import pprint
import sys

from autoware_auto_vehicle_msgs.msg import ControlModeReport
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

MAP_AXIS_VELOCITY_KPH = [0, 5, 10, 15, 20, 25, 30, 35]
MIN_VELOCITY_FOR_CALC_ACCEL_KPH = 3
TOPIC_NAMES = [
    "/vehicle/status/velocity_status",
    "/control/command/actuation_cmd",
    "/imu/tamagawa/imu_raw",
    "/vehicle/status/control_mode",
]
VELOCITY_MARGIN_KPH = 0.1
NEAREST_VELOCITY_THRESHOLD_KPH = 0.3

TOPIC_NAMES_FOR_ACCURACY_CHECK = [
    "/vehicle/status/velocity_status",
    "/control/command/control_cmd",
    "/sensing/imu/tamagawa/imu_raw",
    "/vehicle/status/control_mode",
]


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

    return topics


def get_velocity_for_calculating_acceleration():
    axis_velocity_step = MAP_AXIS_VELOCITY_KPH[-1] - MAP_AXIS_VELOCITY_KPH[-2]
    map_axis_velocity_kph = copy.copy(MAP_AXIS_VELOCITY_KPH)
    map_axis_velocity_kph.append(MAP_AXIS_VELOCITY_KPH[-1] + axis_velocity_step)

    velocity_for_calculating_acceleration = []
    for i in range(len(map_axis_velocity_kph) - 1):
        velocity = np.average([map_axis_velocity_kph[i], map_axis_velocity_kph[i + 1]])
        if i == 0:
            velocity = max(velocity, MIN_VELOCITY_FOR_CALC_ACCEL_KPH)
        velocity_for_calculating_acceleration.append(velocity * 1000.0 / 60.0 / 60.0)

    return velocity_for_calculating_acceleration


def get_auto_driving_mode_stamps(topics):
    stamp_auto_driving_start = 0
    stamp_auto_driving_end = int(1e20)
    is_auto_driving = False

    for control_mode_msg in topics["/vehicle/status/control_mode"]:
        if control_mode_msg.mode != ControlModeReport.MANUAL and not is_auto_driving:
            if stamp_auto_driving_start != 0:
                print("===== error!: driving auto twice in one rosbag =====")
                sys.exit()
            stamp_auto_driving_start = (
                control_mode_msg.stamp.sec + control_mode_msg.stamp.nanosec * 1e-9
            )
            is_auto_driving = True
        if control_mode_msg.mode == ControlModeReport.MANUAL and is_auto_driving:
            stamp_auto_driving_end = (
                control_mode_msg.stamp.sec + control_mode_msg.stamp.nanosec * 1e-9
            )
            is_auto_driving = False

    return (stamp_auto_driving_start, stamp_auto_driving_end)


def get_actuation_cmd(topics, actuation_type):
    buffer = []
    for actuation_cmd_msg in topics["/control/command/actuation_cmd"]:
        if actuation_type == "accel" and actuation_cmd_msg.actuation.accel_cmd != 0:
            buffer.append(actuation_cmd_msg.actuation.accel_cmd)
        if actuation_type == "brake" and actuation_cmd_msg.actuation.brake_cmd != 0:
            buffer.append(actuation_cmd_msg.actuation.brake_cmd)

    if not buffer:
        return 0
    else:
        return round(np.average(buffer), 1)


def get_control_cmd(topics):
    buffer = []
    for control_cmd_msg in topics["/control/command/control_cmd"]:
        if control_cmd_msg.longitudinal.acceleration != 0:
            buffer.append(control_cmd_msg.longitudinal.acceleration)

    if not buffer:
        return 0
    else:
        return round(np.average(buffer), 2)


def get_actuation_stamps(
    topics, actuation_type, target_actuation_cmd, stamp_auto_driving_start, stamp_auto_driving_end
):
    stamp_actuation_start = 0
    stamp_actuation_end = int(1e20)
    is_start = False

    for actuation_cmd_msg in topics["/control/command/actuation_cmd"]:
        stamp = actuation_cmd_msg.header.stamp.sec + actuation_cmd_msg.header.stamp.nanosec * 1e-9
        if stamp < stamp_auto_driving_start or stamp_auto_driving_end < stamp:
            continue

        if actuation_type == "accel":
            actuation_cmd = actuation_cmd_msg.actuation.accel_cmd
        elif actuation_type == "brake":
            actuation_cmd = actuation_cmd_msg.actuation.brake_cmd

        if (
            target_actuation_cmd * 0.9 < actuation_cmd
            and actuation_cmd < target_actuation_cmd * 1.1
        ):
            if is_start == False:
                stamp_actuation_start = stamp
                is_start = True

            stamp_actuation_end = stamp

    return (stamp_actuation_start, stamp_actuation_end)


def get_stamp_when_target_velocity(
    topics, target_velocity, stamp_actuation_start, stamp_actuation_end
):
    nearest_diff_velocity = 10000
    nearest_stamp = 0.0
    stamps_in_velocity_margin = []

    for velocity_status_msg in topics["/vehicle/status/velocity_status"]:
        stamp = (
            velocity_status_msg.header.stamp.sec + velocity_status_msg.header.stamp.nanosec * 1e-9
        )
        if stamp < stamp_actuation_start or stamp_actuation_end < stamp:
            continue

        diff_velocity = velocity_status_msg.longitudinal_velocity - target_velocity
        if abs(diff_velocity) < abs(nearest_diff_velocity):
            nearest_diff_velocity = diff_velocity
            nearest_stamp = stamp

        if abs(diff_velocity) < VELOCITY_MARGIN_KPH / 3.6:
            stamps_in_velocity_margin.append(stamp)

    if stamps_in_velocity_margin:
        return np.average(stamps_in_velocity_margin)
    if abs(nearest_diff_velocity) < NEAREST_VELOCITY_THRESHOLD_KPH / 3.6:
        return nearest_stamp

    print(f"No data corresponding to target velocity: {target_velocity * 3.6:.1f} km/h")
    return 0


class MapGenerator:
    def __init__(self, data_path, report_path, vehicle_name):
        self.accel_data_paths = glob.glob(data_path + "/accel/*.db3")
        self.brake_data_paths = glob.glob(data_path + "/brake/*.db3")
        if not self.accel_data_paths:
            print("accel data is empty. exit this program.")
            sys.exit()
        if not self.brake_data_paths:
            print("brake data is empty. exit this program.")
            sys.exit()
        print("===== target accel data =====")
        pprint.pprint(self.accel_data_paths)
        print("===== target brake data =====")
        pprint.pprint(self.brake_data_paths)

        self.report_path = report_path
        self.vehicle_name = vehicle_name

        self.velocity_for_calculating_acceleration = get_velocity_for_calculating_acceleration()

        self.accel_accelerations = {}
        self.brake_accelerations = {}

    def run(self):
        for accel_data_path in self.accel_data_paths:
            topics = get_topics(accel_data_path, TOPIC_NAMES)
            self.calc_acceleration(topics, "accel")
        for brake_data_path in self.brake_data_paths:
            topics = get_topics(brake_data_path, TOPIC_NAMES)
            self.calc_acceleration(topics, "brake")

    def calc_acceleration(self, topics, actuation_type):
        print("===== calc acceleration =====")

        # Obtain timestamps during autonomous driving mode
        stamp_auto_driving_start, stamp_auto_driving_end = get_auto_driving_mode_stamps(topics)
        print(
            f"auto driving start: {stamp_auto_driving_start}, auto driving end: {stamp_auto_driving_end}"
        )

        # Obtain actuation_cmd
        actuation_cmd = get_actuation_cmd(topics, actuation_type)
        print(f"actuation_type: {actuation_type}, actuation_cmd: {actuation_cmd}")

        # Obtain timestamps during applying the target actuation command
        stamp_actuation_start, stamp_actuation_end = get_actuation_stamps(
            topics, actuation_type, actuation_cmd, stamp_auto_driving_start, stamp_auto_driving_end
        )

        # Search for the timestamp when the velocity is reached for acceleration calculation velocity on automatic driving
        stamps_for_calculating_acceleration = []
        for velocity in self.velocity_for_calculating_acceleration:
            stamp = get_stamp_when_target_velocity(
                topics, velocity, stamp_actuation_start, stamp_auto_driving_end
            )
            print(f"velocity: {velocity:.2f}, stamp: {stamp}")
            stamps_for_calculating_acceleration.append(stamp)

        # Calculate acceleration from velocity and timestamps for calculating acceleration
        accelerations = []
        for i in range(len(self.velocity_for_calculating_acceleration) - 1):
            if (
                stamps_for_calculating_acceleration[i + 1] == 0
                or stamps_for_calculating_acceleration[i] == 0
            ):
                accelerations.append(0)
                continue
            diff_velocity = (
                self.velocity_for_calculating_acceleration[i + 1]
                - self.velocity_for_calculating_acceleration[i]
            )
            diff_stamp = (
                stamps_for_calculating_acceleration[i + 1] - stamps_for_calculating_acceleration[i]
            )
            accelerations.append(diff_velocity / diff_stamp)
        pprint.pprint(accelerations)

        # Update accel/brake map
        if actuation_type == "accel":
            if actuation_cmd in self.accel_accelerations:
                for i in range(len(accelerations)):
                    if self.accel_accelerations[actuation_cmd][i] == 0:
                        self.accel_accelerations[actuation_cmd][i] = accelerations[i]
                    elif accelerations[i] == 0:
                        pass
                    else:
                        self.accel_accelerations[actuation_cmd][i] = (
                            self.accel_accelerations[actuation_cmd][i] + accelerations[i]
                        ) / 2.0
            else:
                self.accel_accelerations[actuation_cmd] = accelerations
        if actuation_type == "brake":
            if actuation_cmd in self.brake_accelerations:
                for i in range(len(accelerations)):
                    if self.brake_accelerations[actuation_cmd][i] == 0:
                        self.brake_accelerations[actuation_cmd][i] = accelerations[i]
                    elif accelerations[i] == 0:
                        pass
                    else:
                        self.brake_accelerations[actuation_cmd][i] = (
                            self.brake_accelerations[actuation_cmd][i] + accelerations[i]
                        ) / 2.0
            else:
                self.brake_accelerations[actuation_cmd] = accelerations

    def output_graph(self):
        pass

    def output_map(self):
        print("===== output accel_map.csv and brake_map.csv =====")
        self.accel_accelerations = dict(sorted(self.accel_accelerations.items()))
        self.brake_accelerations = dict(sorted(self.brake_accelerations.items(), reverse=True))

        with open(self.report_path + "/accel_map.csv", "w") as f:
            writer = csv.writer(f)

            map_axis_velocity = [
                round(velocity_kph / 3.6, 2) for velocity_kph in MAP_AXIS_VELOCITY_KPH
            ]
            header = [self.vehicle_name] + map_axis_velocity
            writer.writerow(header)

            for actuation_cmd, accelerations in self.accel_accelerations.items():
                accelerations = [round(acceleration, 2) for acceleration in accelerations]
                row = [actuation_cmd] + [accelerations[0]] + accelerations
                writer.writerow(row)

        with open(self.report_path + "/brake_map.csv", "w") as f:
            writer = csv.writer(f)

            map_axis_velocity = [
                round(velocity_kph / 3.6, 2) for velocity_kph in MAP_AXIS_VELOCITY_KPH
            ]
            header = [self.vehicle_name] + map_axis_velocity
            writer.writerow(header)

            for actuation_cmd, accelerations in self.brake_accelerations.items():
                accelerations = [round(acceleration, 2) for acceleration in accelerations]
                row = [actuation_cmd] + [accelerations[0]] + accelerations
                writer.writerow(row)


class AccelerationAccuracyChecker:
    def __init__(self, data_path, report_path, vehicle_name):
        self.data_paths = glob.glob(data_path + "/*.db3")
        if not self.data_paths:
            print("acceleration data is empty. exit this program.")
            sys.exit()
        print("===== target acceleration data ======")
        pprint.pprint(self.data_paths)

        self.report_path = report_path
        self.vehicle_name = vehicle_name
        self.velocity_for_calculating_acceleration = get_velocity_for_calculating_acceleration()

        self.accelerations = {}

    def run(self):
        for data_path in self.data_paths:
            topics = get_topics(data_path, TOPIC_NAMES_FOR_ACCURACY_CHECK)
            self.calc_acceleration(topics)

    def calc_acceleration(self, topics):
        print("===== calc acceleration =====")

        # Obtain timestamps during autonomous driving mode
        stamp_auto_driving_start, stamp_auto_driving_end = get_auto_driving_mode_stamps(topics)
        print(
            f"auto driving start: {stamp_auto_driving_start}, auto driving end: {stamp_auto_driving_end}"
        )

        # Obtain control_cmd
        control_cmd = get_control_cmd(topics)
        print(f"control_cmd: {control_cmd}")

        # Search for the timestamp when the velocity is reached for acceleration calculation velocity on automatic driving
        stamps_for_calculating_acceleration = []
        for velocity in self.velocity_for_calculating_acceleration:
            stamp = get_stamp_when_target_velocity(
                topics, velocity, stamp_auto_driving_start, stamp_auto_driving_end
            )
            print(f"velocity: {velocity:.2f}, stamp: {stamp}")
            stamps_for_calculating_acceleration.append(stamp)

        # Calculate acceleration from velocity and timestamps for calculating acceleration
        accelerations = []
        for i in range(len(self.velocity_for_calculating_acceleration) - 1):
            if (
                stamps_for_calculating_acceleration[i + 1] == 0
                or stamps_for_calculating_acceleration[i] == 0
            ):
                accelerations.append(0)
                continue
            diff_velocity = (
                self.velocity_for_calculating_acceleration[i + 1]
                - self.velocity_for_calculating_acceleration[i]
            )
            diff_stamp = (
                stamps_for_calculating_acceleration[i + 1] - stamps_for_calculating_acceleration[i]
            )
            accelerations.append(diff_velocity / diff_stamp)
        pprint.pprint(accelerations)

        # Update accelerations
        if control_cmd in self.accelerations:
            for i in range(len(accelerations)):
                if self.accelerations[control_cmd][i] == 0:
                    self.accelerations[control_cmd][i] = accelerations[i]
                elif accelerations[i] == 0:
                    pass
                else:
                    self.accelerations[control_cmd][i] = (
                        self.accelerations[control_cmd][i] + accelerations[i]
                    ) / 2.0
        else:
            self.accelerations[control_cmd] = accelerations

    def output_report(self):
        print("===== output acceleration accuracy report =====")
        self.accelerations = dict(sorted(self.accelerations.items()))

        with open(self.report_path + "/acceleration_accuracy_report.csv", "w") as f:
            writer = csv.writer(f)

            map_axis_velocity = [
                round(velocity_kph / 3.6, 2) for velocity_kph in MAP_AXIS_VELOCITY_KPH
            ]
            header = [self.vehicle_name] + map_axis_velocity + ["average", "error"]
            writer.writerow(header)

            errors_by_control_cmd = {}
            for control_cmd, accelerations in self.accelerations.items():
                accelerations = [round(acceleration, 2) for acceleration in accelerations]
                accelerations_nonzero = [acc for acc in accelerations if acc != 0]
                if accelerations_nonzero:
                    average = round(np.average(accelerations_nonzero), 2)
                else:
                    average = 0
                if not control_cmd == 0:
                    error = round(average / control_cmd - 1.0, 2)
                else:
                    error = "-"
                row = [control_cmd] + ["-"] + accelerations + [average, error]
                writer.writerow(row)

                if not control_cmd == 0:
                    errors_by_control_cmd[control_cmd] = [
                        round(acceleration / control_cmd - 1.0, 2) for acceleration in accelerations
                    ]
                else:
                    errors_by_control_cmd[control_cmd] = []

            writer.writerow([])

            errors_by_control_cmd = dict(sorted(errors_by_control_cmd.items()))
            for control_cmd, errors in errors_by_control_cmd.items():
                row = [control_cmd] + ["-"] + errors
                writer.writerow(row)


def main():
    print("===== accel/brake map generator begin! =====")
    parser = argparse.ArgumentParser(description="Accel and brake map generator")
    parser.add_argument("path", help="path to data")
    parser.add_argument("report_path", help="path to report")
    parser.add_argument("vehicle_name", help="vehicle name in (0, 0) of output csv files")
    parser.add_argument("--generate", action="store_true", help="map generator mode")
    parser.add_argument("--check", action="store_true", help="acceleration accuracy checker mode")
    args = parser.parse_args()
    print(f"data path: {args.path}")
    print(f"report path: {args.report_path}")

    if args.generate is True:
        generator = MapGenerator(args.path, args.report_path, args.vehicle_name)
        generator.run()
        generator.output_map()
    elif args.check is True:
        checker = AccelerationAccuracyChecker(args.path, args.report_path, args.vehicle_name)
        checker.run()
        checker.output_report()
    else:
        print("please select mode. --generate or --check")
        sys.exit()


if __name__ == "__main__":
    main()
