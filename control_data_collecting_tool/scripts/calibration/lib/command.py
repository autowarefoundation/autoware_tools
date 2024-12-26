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

import sys
import time

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import ControlModeReport
from autoware_vehicle_msgs.msg import GearCommand
from autoware_vehicle_msgs.msg import GearReport
from autoware_vehicle_msgs.srv import ControlModeCommand
import rclpy
from std_msgs.msg import Float32
from tier4_vehicle_msgs.msg import ActuationCommandStamped


def call_control_mode_request(node, mode):
    request = ControlModeCommand.Request()
    request.mode = mode

    future = node.client_control_mode.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        print(f"Response from control mode service: {future.result()}")
    else:
        print("Control mode service call failed.")
        sys.exit()


def change_mode(node, mode):
    print(f"Change mode to {mode}")
    if mode == "autonomous":
        request = ControlModeCommand.Request.AUTONOMOUS
        report = ControlModeReport.AUTONOMOUS
    elif mode == "autonomous_velocity_only":
        request = ControlModeCommand.Request.AUTONOMOUS_VELOCITY_ONLY
        report = ControlModeReport.AUTONOMOUS_VELOCITY_ONLY
    elif mode == "autonomous_steering_only":
        request = ControlModeCommand.Request.AUTONOMOUS_STEER_ONLY
        report = ControlModeReport.AUTONOMOUS_STEER_ONLY
    elif mode == "manual":
        request = ControlModeCommand.Request.MANUAL
        report = ControlModeReport.MANUAL
    else:
        print(f"Invalid mode: {mode}")
        sys.exit(1)

    call_control_mode_request(node, request)

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.current_control_mode == report:
            break


def change_gear(node, target_gear):
    print(f"Change gear to {target_gear}")
    if target_gear == "neutral":
        command = GearCommand.NEUTRAL
        report = GearReport.NEUTRAL
    elif target_gear == "drive":
        command = GearCommand.DRIVE
        report = GearReport.DRIVE
    elif target_gear == "reverse":
        command = GearCommand.REVERSE
        report = GearReport.REVERSE
    else:
        print(f"Invalid gear: {target_gear}")
        sys.exit(1)

    gear_cmd_msg = GearCommand()
    gear_cmd_msg.stamp = node.get_clock().now().to_msg()
    gear_cmd_msg.command = command
    node.pub_gear_cmd.publish(gear_cmd_msg)

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.current_gear == report:
            print(f"Current gear is {target_gear}")
            break


def accelerate(node, target_acceleration, target_velocity, mode, target_jerk=None, break_time=120.0):
    print(f"Accelerate with {target_acceleration} m/s^2.")
    start_time = time.time()
    
    if target_jerk == None:
        acceleration_cmd = target_acceleration
    else:
        acceleration_cmd = 0.0

    condition = (
        lambda: acceleration_cmd < target_acceleration - 1e-3
        if mode == "drive"
        else acceleration_cmd > target_acceleration + 1e-3
    )
    while condition():
        acceleration_cmd += target_jerk / 10.0
        data_collecting_control_cmd = acceleration_cmd
        node.pub_data_collecting_control_cmd.publish(Float32(data=float(data_collecting_control_cmd)))
        time.sleep(0.1)

    data_collecting_control_cmd = target_acceleration
    node.pub_data_collecting_control_cmd.publish(Float32(data=float(data_collecting_control_cmd)))

    while rclpy.ok():
        rclpy.spin_once(node)
        if (mode == "drive" and node.current_velocity * 3.6 >= target_velocity) or (
            mode == "brake" and node.current_velocity * 3.6 <= target_velocity
        ):
            print(f"Reached {target_velocity} km/h.")
            data_collecting_control_cmd = 0.0 if mode == "drive" else -2.5
            node.pub_data_collecting_control_cmd.publish(
                Float32(data=float(data_collecting_control_cmd))
            )
            break

        if time.time() - start_time > break_time:
            print("break : " + str(break_time) + " has passed.")
            break

    time.sleep(1)


def actuate(node, mode, target_command, target_velocity, break_time=120.0):
    print(f"Actuate with {mode} command: {target_command}.")
    start_time = time.time()

    data_collecting_pedal_input = 0.0
    if mode == "accel":
        data_collecting_pedal_input = target_command
    elif mode == "brake":
        data_collecting_pedal_input = -target_command
    else:
        print(f"Invalid mode: {mode}")
        sys.exit(1)

    node.pub_control_data_pedal_input.publish(Float32(data=float(data_collecting_pedal_input)))

    while rclpy.ok():
        rclpy.spin_once(node)
        if (
            (mode == "accel" and node.current_velocity * 3.6 >= target_velocity)
            or (mode == "brake" and node.current_velocity * 3.6 <= target_velocity)
        ):
            print(f"Reached {target_velocity} km/h.")
            data_collecting_pedal_input = 0.0
            if mode == "accel":
                data_collecting_pedal_input = 0.0
            elif mode == "brake":
                data_collecting_pedal_input = -0.8
            node.pub_control_data_pedal_input.publish(
                Float32(data=float(data_collecting_pedal_input))
            )
            break

        if time.time() - start_time > break_time:
            print("break : " + str(break_time) + " has passed.")
            break

    time.sleep(1)


def reset_commands(node):
    control_cmd_msg = Control()
    control_cmd_msg.stamp = node.get_clock().now().to_msg()
    control_cmd_msg.longitudinal.acceleration = 0.0
    control_cmd_msg.lateral.steering_tire_angle = 0.0
    node.pub_control_cmd.publish(control_cmd_msg)
    print("Reset control command.")

    actuation_cmd_msg = ActuationCommandStamped()
    actuation_cmd_msg.header.stamp = node.get_clock().now().to_msg()
    actuation_cmd_msg.actuation.accel_cmd = 0.0
    actuation_cmd_msg.actuation.brake_cmd = 0.0
    node.pub_actuation_cmd.publish(actuation_cmd_msg)
    print("Reset actuation command.")

    gear_cmd_msg = GearCommand()
    gear_cmd_msg.stamp = node.get_clock().now().to_msg()
    gear_cmd_msg.command = GearCommand.NEUTRAL
    node.pub_gear_cmd.publish(gear_cmd_msg)
    print("Reset gear command.")
