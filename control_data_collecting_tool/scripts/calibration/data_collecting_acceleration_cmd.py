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

from datetime import datetime
import sys
import os
import yaml

from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import ControlModeReport
from autoware_vehicle_msgs.msg import GearCommand
from autoware_vehicle_msgs.msg import GearReport
from autoware_vehicle_msgs.msg import VelocityReport
from autoware_vehicle_msgs.srv import ControlModeCommand
import lib.command
import lib.cui
import lib.rosbag
import lib.system
import rclpy
from rclpy.node import Node
from tier4_vehicle_msgs.msg import ActuationCommandStamped

COUNTDOWN_TIME = 3  # [sec]

class DataCollectingAccelerationCmd(Node):
    def __init__(self):
        super().__init__("data_collecting_acceleration_cmd")

        package_share_directory = get_package_share_directory("control_data_collecting_tool")
        topic_file_path = os.path.join(package_share_directory, "config", "cmd_param.yaml")
        with open(topic_file_path, "r") as file:
            topic_data = yaml.safe_load(file)

        self.TARGET_VELOCITY = topic_data["data_collecting_acceleration_cmd"]["ros__parameters"]["TARGET_VELOCITY"]
        self.TARGET_ACCELERATION_FOR_DRIVE = topic_data["data_collecting_acceleration_cmd"]["ros__parameters"]["TARGET_ACCELERATION_FOR_DRIVE"]
        self.TARGET_ACCELERATION_FOR_BRAKE = topic_data["data_collecting_acceleration_cmd"]["ros__parameters"]["TARGET_ACCELERATION_FOR_BRAKE"]
        self.TARGET_JERK_FOR_DRIVE = topic_data["data_collecting_acceleration_cmd"]["ros__parameters"]["TARGET_JERK_FOR_DRIVE"]
        self.TARGET_JERK_FOR_BRAKE = topic_data["data_collecting_acceleration_cmd"]["ros__parameters"]["TARGET_JERK_FOR_BRAKE"]
        self.MIN_ACCEL = topic_data["data_collecting_acceleration_cmd"]["ros__parameters"]["MIN_ACCEL"]
        self.MAX_ACCEL = topic_data["data_collecting_acceleration_cmd"]["ros__parameters"]["MAX_ACCEL"]
        self.TOPIC_LIST_FOR_VALIDATION = topic_data["data_collecting_acceleration_cmd"]["ros__parameters"]["topics"]
        self.NODE_LIST_FOR_VALIDATION = topic_data["data_collecting_acceleration_cmd"]["ros__parameters"]["validation_nodes"]

        self.client_control_mode = self.create_client(
            ControlModeCommand, "/control/control_mode_request"
        )

        while not self.client_control_mode.wait_for_service(timeout_sec=1.0):
            print("Waiting for the control mode service to become available...")

        self.pub_data_collecting_control_cmd = self.create_publisher(Float32, "/data_collecting_accel_cmd", 1)
        self.pub_control_cmd = self.create_publisher(Control, "/control/command/control_cmd", 1)
        self.pub_gear_cmd = self.create_publisher(GearCommand, "/control/command/gear_cmd", 1)

        self.sub_velocity_status = self.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status", self.on_velocity_status, 1
        )
        self.sub_control_mode = self.create_subscription(
            ControlModeReport, "/vehicle/status/control_mode", self.on_control_mode, 1
        )
        self.sub_gear_status = self.create_subscription(
            GearReport, "/vehicle/status/gear_status", self.on_gear_status, 1
        )

        self.current_velocity = 0.0
        self.current_control_mode = ControlModeReport.MANUAL
        self.current_gear = GearReport.NONE

        # For commands reset
        self.pub_data_collecting_pedal_input = self.create_publisher(
            Float32, "/data_collecting_pedal_input", 1
        )
        self.pub_actuation_cmd = self.create_publisher(
            ActuationCommandStamped, "/control/command/actuation_cmd", 1
        )

    def on_velocity_status(self, msg):
        self.current_velocity = msg.longitudinal_velocity

    def on_control_mode(self, msg):
        self.current_control_mode = msg.mode

    def on_gear_status(self, msg):
        self.current_gear = msg.report

    def run(self):
        print("===== Start map accuracy tester =====")
        lib.system.check_service_active("autoware.service")
        lib.system.check_node_active(self.NODE_LIST_FOR_VALIDATION)

        print(f"===== Set Acceleration to {self.TARGET_ACCELERATION_FOR_BRAKE} =====")
        lib.command.accelerate(
            self, self.TARGET_ACCELERATION_FOR_BRAKE, 1e-3, "brake", self.TARGET_JERK_FOR_BRAKE
        )

        print("===== Start checking accel map =====")
        lib.cui.do_check("Do you want to check accel map?", lambda: self.check("accel"))

        print("===== Start checking brake map =====")
        lib.cui.do_check("Do you want to check brake map?", lambda: self.check("brake"))

        print("===== Successfully finished! =====")

    def check(self, mode):
        is_finished = False
        while not is_finished:
            print("===== Input target acceleration =====")
            min_acceleration, max_acceleration = self.get_min_max_acceleration(mode)
            target_acceleration = lib.cui.input_target_value(
                "acceleration", min_acceleration, max_acceleration, "m/s^2"
            )

            if mode == "accel":
                print(
                    f"===== Drive to {self.TARGET_VELOCITY} km/h with acceleration {target_acceleration} ====="
                )
                lib.command.change_gear(self, "drive")
                lib.cui.ready_check("Ready to drive?")
                lib.cui.countdown(COUNTDOWN_TIME)
                print("===== Record rosbag =====")
                filename = self.get_rosbag_name(mode, target_acceleration)
                process = lib.rosbag.record_ros2_bag(filename, lib.rosbag.TOPIC_LIST)
                print(f"record rosbag: {filename}")
                lib.command.accelerate(self, target_acceleration, self.TARGET_VELOCITY, "drive",break_time=60.0)
                print("===== End rosbag record =====")
                process.terminate()
                lib.command.accelerate(
                    self, self.TARGET_ACCELERATION_FOR_BRAKE, 1e-3, "brake", self.TARGET_JERK_FOR_BRAKE
                )
                
            elif mode == "brake":
                print(
                    f"===== Drive to {TARGET_VELOCITY} km/h and brake with {target_acceleration} ====="
                )
                lib.command.change_gear(self, "drive")
                lib.cui.ready_check("Ready to drive?")
                lib.cui.countdown(COUNTDOWN_TIME)
                lib.command.accelerate(
                    self,
                    self.TARGET_ACCELERATION_FOR_DRIVE,
                    self.TARGET_VELOCITY,
                    "drive",
                    self.TARGET_JERK_FOR_DRIVE,
                )
                print("===== Record rosbag =====")
                filename = self.get_rosbag_name(mode, target_acceleration)
                process = lib.rosbag.record_ros2_bag(filename, lib.rosbag.TOPIC_LIST)
                print(f"record rosbag: {filename}")
                lib.command.accelerate(self, target_acceleration, 1e-3, "brake")
                print("===== End rosbag record =====")
                process.terminate()
                
            else:
                print(f"Invalid mode: {mode}")
                sys.exit(1)

            process.wait()
            print("===== Validate rosbag =====")
            is_rosbag_valid = lib.rosbag.validate(filename, self.TOPIC_LIST_FOR_VALIDATION)
            if not is_rosbag_valid:
                print(f"Rosag validation error: {filename}")
                sys.exit(1)

            is_finished = lib.cui.finish_check(f"Will you continue to check {mode} map?")

        print(f"===== Successfully {mode} map checking finished! =====")

    def get_min_max_acceleration(self, mode):
        if mode == "accel":
            return 0.0, self.MAX_ACCEL
        if mode == "brake":
            return self.MIN_ACCEL, 0.0
        if mode == "sub_brake":
            return self.MIN_ACCEL_SUB_BRAKE, 0.0

    def get_rosbag_name(self, mode, target_acceleration):
        current_time = datetime.now().strftime("%Y%m%d-%H%M%S")
        filename = "_".join(
            [
                "acceleration_accuracy",
                mode,
                str(target_acceleration),
                current_time,
            ]
        )
        return filename


def main(args=None):
    rclpy.init(args=args)

    tester = DataCollectingAccelerationCmd()
    tester.run()

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
