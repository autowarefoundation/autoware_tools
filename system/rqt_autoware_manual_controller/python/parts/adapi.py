# Copyright 2025 The Autoware Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from enum import Enum

from autoware_adapi_v1_msgs.msg import Gear as GearMsg
from autoware_adapi_v1_msgs.msg import GearCommand
from autoware_adapi_v1_msgs.msg import HazardLights as HazardLightsMsg
from autoware_adapi_v1_msgs.msg import HazardLightsCommand
from autoware_adapi_v1_msgs.msg import ManualControlMode
from autoware_adapi_v1_msgs.msg import ManualControlModeStatus
from autoware_adapi_v1_msgs.msg import ManualOperatorStatus
from autoware_adapi_v1_msgs.msg import PedalsCommand
from autoware_adapi_v1_msgs.msg import SteeringCommand
from autoware_adapi_v1_msgs.msg import TurnIndicators as TurnIndicatorsMsg
from autoware_adapi_v1_msgs.msg import TurnIndicatorsCommand
from autoware_adapi_v1_msgs.srv import ListManualControlMode
from autoware_adapi_v1_msgs.srv import SelectManualControlMode
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile


class ManualMode(Enum):
    Disabled = ManualControlMode.DISABLED
    Pedals = ManualControlMode.PEDALS
    Acceleration = ManualControlMode.ACCELERATION
    Velocity = ManualControlMode.VELOCITY


class Gear(Enum):
    Unknown = GearMsg.UNKNOWN
    Neutral = GearMsg.NEUTRAL
    Drive = GearMsg.DRIVE
    Reverse = GearMsg.REVERSE
    Park = GearMsg.PARK
    Low = GearMsg.LOW


class TurnIndicators(Enum):
    Disable = TurnIndicatorsMsg.DISABLE
    Left = TurnIndicatorsMsg.LEFT
    Right = TurnIndicatorsMsg.RIGHT


class HazardLights(Enum):
    Disable = HazardLightsMsg.DISABLE
    Enable = HazardLightsMsg.ENABLE


class Adapi:
    def __init__(self, node: Node, target: str):
        # interfaces
        durable_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._node = node
        # fmt: off
        self._cli_mode_list        = node.create_client(ListManualControlMode,         f"/api/manual/{target}/control_mode/list")                                       # noqa: E221
        self._cli_mode_select      = node.create_client(SelectManualControlMode,       f"/api/manual/{target}/control_mode/select")                                     # noqa: E221
        self._sub_mode_status      = node.create_subscription(ManualControlModeStatus, f"/api/manual/{target}/control_mode/status", self._on_mode_status, durable_qos)  # noqa: E221
        self._pub_pedals           = node.create_publisher(PedalsCommand,              f"/api/manual/{target}/command/pedals", 1)                                       # noqa: E221
        self._pub_steering         = node.create_publisher(SteeringCommand,            f"/api/manual/{target}/command/steering", 1)                                     # noqa: E221
        self._pub_gear             = node.create_publisher(GearCommand,                f"/api/manual/{target}/command/gear", durable_qos)                               # noqa: E221
        self._pub_turn_indicators  = node.create_publisher(TurnIndicatorsCommand,      f"/api/manual/{target}/command/turn_indicators", durable_qos)                    # noqa: E221
        self._pub_hazard_lights    = node.create_publisher(HazardLightsCommand,        f"/api/manual/{target}/command/hazard_lights", durable_qos)                      # noqa: E221
        self._pub_heartbeat        = node.create_publisher(ManualOperatorStatus,       f"/api/manual/{target}/heartbeat", durable_qos)                                  # noqa: E221
        # fmt: on
        self._timer = node.create_timer(1.0, self._on_timer)
        self._command_timer = node.create_timer(0.1, self._on_command_timer)

        # variables
        self._is_mode_listed = False
        self._msg_pedals = PedalsCommand()
        self._msg_steering = SteeringCommand()
        self._msg_gear = GearCommand()
        self._msg_turn_indicators = TurnIndicatorsCommand()
        self._msg_hazard_lights = HazardLightsCommand()
        self._msg_heartbeat = ManualOperatorStatus()
        # callbacks
        self.on_mode_list = lambda _: None
        self.on_mode_status = lambda _: None
        # init
        self.set_gear(Gear.Drive)
        self.set_turn_indicators(TurnIndicators.Disable)
        self.set_hazard_lights(HazardLights.Disable)

    def select_mode(self, mode: ManualMode):
        self._cli_mode_select.call_async(
            SelectManualControlMode.Request(mode=ManualControlMode(mode=mode.value))
        )

    def _on_mode_status(self, status):
        self.on_mode_status(ManualMode(status.mode.mode))

    def _on_mode_list(self, future):
        result = future.result()
        self.on_mode_list([ManualMode(mode.mode) for mode in result.modes])
        self._is_mode_listed = True

    def _on_timer(self):
        if not self._is_mode_listed and self._cli_mode_list.service_is_ready():
            future = self._cli_mode_list.call_async(ListManualControlMode.Request())
            future.add_done_callback(self._on_mode_list)

    def set_pedals(self, accel: float, brake: float):
        self._msg_pedals.accelerator = accel
        self._msg_pedals.brake = brake

    def set_steering(self, steering: float):
        self._msg_steering.steering_tire_angle = steering

    def set_gear(self, gear: Gear):
        self._msg_gear.command.status = gear.value

    def set_turn_indicators(self, turn_indicators: TurnIndicators):
        self._msg_turn_indicators.command.status = turn_indicators.value

    def set_hazard_lights(self, hazard_lights: HazardLights):
        self._msg_hazard_lights.command.status = hazard_lights.value

    def set_heartbeat(self, ready: bool):
        self._msg_heartbeat.ready = ready

    def _on_command_timer(self):
        stamp = self._node.get_clock().now().to_msg()
        self._msg_pedals.stamp = stamp
        self._msg_steering.stamp = stamp
        self._msg_gear.stamp = stamp
        self._msg_turn_indicators.stamp = stamp
        self._msg_hazard_lights.stamp = stamp
        self._msg_heartbeat.stamp = stamp
        self._pub_pedals.publish(self._msg_pedals)
        self._pub_steering.publish(self._msg_steering)
        self._pub_gear.publish(self._msg_gear)
        self._pub_turn_indicators.publish(self._msg_turn_indicators)
        self._pub_hazard_lights.publish(self._msg_hazard_lights)
        self._pub_heartbeat.publish(self._msg_heartbeat)
