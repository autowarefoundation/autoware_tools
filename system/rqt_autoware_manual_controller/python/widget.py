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

from python_qt_binding import QtWidgets
from rqt_autoware_manual_controller.modules.adapi import Adapi
from rqt_autoware_manual_controller.modules.gear import GearControl
from rqt_autoware_manual_controller.modules.hazard_lights import HazardLightsControl
from rqt_autoware_manual_controller.modules.heartbeat import HeartbeatControl
from rqt_autoware_manual_controller.modules.mode_select import ManualModeSelect
from rqt_autoware_manual_controller.modules.mode_select import ManualModeStatus
from rqt_autoware_manual_controller.modules.mouse_control import MouseControl
from rqt_autoware_manual_controller.modules.turn_indicators import TurnIndicatorsControl


class ControllerWidget(QtWidgets.QSplitter):
    def __init__(self, adapi: Adapi):
        super().__init__()
        self.adapi = adapi
        self.mode_select = ManualModeSelect(self.adapi)
        self.mode_status = ManualModeStatus(self.adapi)
        self.mouse = MouseControl(self.adapi)
        self.gear = GearControl(self.adapi)
        self.turn_indicators = TurnIndicatorsControl(self.adapi)
        self.hazard_lights = HazardLightsControl(self.adapi)
        self.heartbeat = HeartbeatControl(self.adapi)
        row = 0
        layout = QtWidgets.QGridLayout()
        layout.addWidget(QtWidgets.QLabel("Item"), row, 0)
        layout.addWidget(QtWidgets.QLabel("Status"), row, 1)
        layout.addWidget(QtWidgets.QLabel("Command"), row, 2)
        row += 1
        layout.addWidget(QtWidgets.QLabel("Mode"), row, 0)
        layout.addWidget(self.mode_status, row, 1)
        layout.addLayout(self.mode_select, row, 2)
        row += 1
        layout.addWidget(QtWidgets.QLabel("Velocity"), row, 0)
        layout.addWidget(QtWidgets.QLabel("---"), row, 1)
        layout.addWidget(QtWidgets.QLabel("---"), row, 2)
        row += 1
        layout.addWidget(QtWidgets.QLabel("Acceleration"), row, 0)
        layout.addWidget(QtWidgets.QLabel("---"), row, 1)
        layout.addWidget(QtWidgets.QLabel("---"), row, 2)
        row += 1
        layout.addWidget(QtWidgets.QLabel("Throttle"), row, 0)
        layout.addWidget(QtWidgets.QLabel("---"), row, 1)
        layout.addWidget(self.mouse.accel, row, 2)
        row += 1
        layout.addWidget(QtWidgets.QLabel("Brake"), row, 0)
        layout.addWidget(QtWidgets.QLabel("---"), row, 1)
        layout.addWidget(self.mouse.brake, row, 2)
        row += 1
        layout.addWidget(QtWidgets.QLabel("Steering"), row, 0)
        layout.addWidget(QtWidgets.QLabel("---"), row, 1)
        layout.addWidget(self.mouse.steer, row, 2)
        row += 1
        layout.addWidget(QtWidgets.QLabel("Gear"), row, 0)
        layout.addWidget(self.gear.status, row, 1)
        layout.addLayout(self.gear.command, row, 2)
        row += 1
        layout.addWidget(QtWidgets.QLabel("Turn Indicator"), row, 0)
        layout.addWidget(self.turn_indicators.status, row, 1)
        layout.addLayout(self.turn_indicators.command, row, 2)
        row += 1
        layout.addWidget(QtWidgets.QLabel("Hazard Lights"), row, 0)
        layout.addWidget(self.hazard_lights.status, row, 1)
        layout.addLayout(self.hazard_lights.command, row, 2)
        row += 1
        layout.addWidget(QtWidgets.QLabel("Heartbeat"), row, 0)
        layout.addWidget(self.heartbeat.status, row, 1)
        layout.addLayout(self.heartbeat.command, row, 2)
        row += 1
        layout.setRowStretch(row, 1)

        widget = QtWidgets.QWidget()
        widget.setLayout(layout)
        self.addWidget(self.mouse.capture)
        self.addWidget(widget)

    def shutdown(self):
        pass
