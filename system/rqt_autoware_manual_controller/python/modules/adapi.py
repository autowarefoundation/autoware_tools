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

from autoware_adapi_v1_msgs.msg import Gear
from autoware_adapi_v1_msgs.msg import GearCommand
from autoware_adapi_v1_msgs.msg import HazardLights
from autoware_adapi_v1_msgs.msg import HazardLightsCommand
from autoware_adapi_v1_msgs.msg import ManualControlMode
from autoware_adapi_v1_msgs.msg import ManualControlModeStatus
from autoware_adapi_v1_msgs.msg import ManualOperatorHeartbeat
from autoware_adapi_v1_msgs.msg import PedalsCommand
from autoware_adapi_v1_msgs.msg import SteeringCommand
from autoware_adapi_v1_msgs.msg import TurnIndicators
from autoware_adapi_v1_msgs.msg import TurnIndicatorsCommand
from autoware_adapi_v1_msgs.msg import VehicleStatus
from autoware_adapi_v1_msgs.srv import ListManualControlMode
from autoware_adapi_v1_msgs.srv import SelectManualControlMode
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


class ManualMode(Enum):
    Disabled = ManualControlMode.DISABLED
    Pedals = ManualControlMode.PEDALS
    Acceleration = ManualControlMode.ACCELERATION
    Velocity = ManualControlMode.VELOCITY


class GearEnum(Enum):
    Unknown = Gear.UNKNOWN
    Neutral = Gear.NEUTRAL
    Drive = Gear.DRIVE
    Reverse = Gear.REVERSE
    Park = Gear.PARK
    Low = Gear.LOW


class TurnIndicatorsEnum(Enum):
    Unknown = TurnIndicators.UNKNOWN
    Disable = TurnIndicators.DISABLE
    Left = TurnIndicators.LEFT
    Right = TurnIndicators.RIGHT


class HazardLightsEnum(Enum):
    Unknown = HazardLights.UNKNOWN
    Disable = HazardLights.DISABLE
    Enable = HazardLights.ENABLE


class HeartbeatEnum(Enum):
    NotReady = False
    Ready = True


class Adapi:
    def __init__(self, node: Node, target: str):
        # interfaces
        qos_notification = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        qos_realtime = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._node = node
        # fmt: off
        self._cli_mode_list        = node.create_client(ListManualControlMode,         f"/api/manual/{target}/control_mode/list")                                            # noqa: E221
        self._cli_mode_select      = node.create_client(SelectManualControlMode,       f"/api/manual/{target}/control_mode/select")                                          # noqa: E221
        self._sub_mode_status      = node.create_subscription(ManualControlModeStatus, f"/api/manual/{target}/control_mode/status", self._on_mode_status, qos_notification)  # noqa: E221
        self._pub_pedals           = node.create_publisher(PedalsCommand,              f"/api/manual/{target}/command/pedals", 1)                                            # noqa: E221
        self._pub_steering         = node.create_publisher(SteeringCommand,            f"/api/manual/{target}/command/steering", 1)                                          # noqa: E221
        self._pub_gear             = node.create_publisher(GearCommand,                f"/api/manual/{target}/command/gear", qos_notification)                               # noqa: E221
        self._pub_turn_indicators  = node.create_publisher(TurnIndicatorsCommand,      f"/api/manual/{target}/command/turn_indicators", qos_notification)                    # noqa: E221
        self._pub_hazard_lights    = node.create_publisher(HazardLightsCommand,        f"/api/manual/{target}/command/hazard_lights", qos_notification)                      # noqa: E221
        self._pub_heartbeat        = node.create_publisher(ManualOperatorHeartbeat,    f"/api/manual/{target}/operator/heartbeat", qos_realtime)                             # noqa: E221
        self._sub_vehicle_status   = node.create_subscription(VehicleStatus,            "/api/vehicle/status", self._on_vehicle_status, qos_realtime)                        # noqa: E221
        # fmt: on
        self._timer = node.create_timer(1.0, self._on_timer)
        self._command_timer = node.create_timer(0.1, self._on_command_timer)

        # variables
        self._mode_list = None
        self._mode_status = None
        self._gear_status = None
        self._turn_indicators_status = None
        self._hazard_lights_status = None
        # message
        self._msg_pedals = PedalsCommand()
        self._msg_steering = SteeringCommand()
        self._msg_gear = GearCommand()
        self._msg_turn_indicators = TurnIndicatorsCommand()
        self._msg_hazard_lights = HazardLightsCommand()
        self._msg_heartbeat = ManualOperatorHeartbeat()
        # callbacks
        self._callback_mode_list = lambda _: None
        self._callback_mode_status = lambda _: None
        self._callback_gear_status = lambda _: None
        self._callback_turn_indicators_status = lambda _: None
        self._callback_hazard_lights_status = lambda _: None
        # init
        self.send_pedals(0.0, 0.5)
        self.send_steering(0.0)
        self.send_gear(GearEnum.Park)
        self.send_turn_indicators(TurnIndicatorsEnum.Disable)
        self.send_hazard_lights(HazardLightsEnum.Disable)
        self.send_heartbeat(HeartbeatEnum.NotReady)

    def select_mode(self, mode: ManualMode):
        self._cli_mode_select.call_async(
            SelectManualControlMode.Request(mode=ManualControlMode(mode=mode.value))
        )

    def set_on_mode_list(self, callback):
        self._callback_mode_list = callback
        if self._mode_list is not None:
            self._callback_mode_list(self._mode_list)

    def set_on_mode_status(self, callback):
        self._callback_mode_status = callback
        if self._mode_status is not None:
            self._callback_mode_status(self._mode_status)

    def set_on_gear_status(self, callback):
        self._callback_gear_status = callback
        if self._gear_status is not None:
            self._callback_gear_status(self._gear_status)

    def set_on_turn_indicators_status(self, callback):
        self._callback_turn_indicators_status = callback
        if self._turn_indicators_status is not None:
            self._callback_turn_indicators_status(self._turn_indicators_status)

    def set_on_hazard_lights_status(self, callback):
        self._callback_hazard_lights_status = callback
        if self._hazard_lights_status is not None:
            self._callback_hazard_lights_status(self._hazard_lights_status)

    def _on_mode_list(self, future):
        result = future.result()
        self._mode_list = [ManualMode(mode.mode) for mode in result.modes]
        self._callback_mode_list(self._mode_list)

    def _on_mode_status(self, status):
        self._mode_status = ManualMode(status.mode.mode)
        self._callback_mode_status(self._mode_status)

    def _on_timer(self):
        if self._mode_list is None and self._cli_mode_list.service_is_ready():
            future = self._cli_mode_list.call_async(ListManualControlMode.Request())
            future.add_done_callback(self._on_mode_list)

    def _on_vehicle_status(self, status: VehicleStatus):
        self._gear_status = GearEnum(status.gear.status)
        self._turn_indicators_status = TurnIndicatorsEnum(status.turn_indicators.status)
        self._hazard_lights_status = HazardLightsEnum(status.hazard_lights.status)
        self._callback_gear_status(self._gear_status)
        self._callback_turn_indicators_status(self._turn_indicators_status)
        self._callback_hazard_lights_status(self._hazard_lights_status)

    def send_pedals(self, throttle: float, brake: float):
        self._msg_pedals.throttle = throttle
        self._msg_pedals.brake = brake

    def send_steering(self, steering: float):
        self._msg_steering.steering_tire_angle = steering

    def send_gear(self, gear: GearEnum):
        self._msg_gear.command.status = gear.value

    def send_turn_indicators(self, turn_indicators: TurnIndicatorsEnum):
        self._msg_turn_indicators.command.status = turn_indicators.value

    def send_hazard_lights(self, hazard_lights: HazardLightsEnum):
        self._msg_hazard_lights.command.status = hazard_lights.value

    def send_heartbeat(self, ready: HeartbeatEnum):
        self._msg_heartbeat.ready = ready.value

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
