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

from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
from rqt_autoware_manual_controller.parts.adapi import Adapi
from rqt_autoware_manual_controller.parts.mode_select import ManualModeSelect
from rqt_autoware_manual_controller.parts.mode_select import ManualModeStatus


class ControllerWidget(QtWidgets.QSplitter):
    def __init__(self, adapi: Adapi):
        super().__init__()
        self.adapi = adapi
        self.mode_select = ManualModeSelect(self.adapi)
        self.mode_status = ManualModeStatus(self.adapi)

        layout = QtWidgets.QGridLayout()
        layout.addWidget(QtWidgets.QLabel("Item"), 0, 0)
        layout.addWidget(QtWidgets.QLabel("Status"), 0, 1)
        layout.addWidget(QtWidgets.QLabel("Command"), 0, 2)
        layout.addWidget(QtWidgets.QLabel("Mode"), 1, 0)
        layout.addWidget(self.mode_status, 1, 1)
        layout.addLayout(self.mode_select, 1, 2)
        layout.addWidget(QtWidgets.QLabel("Accel Pedal"), 2, 0)
        layout.addWidget(QtWidgets.QLabel("Brake Pedal"), 3, 0)
        layout.addWidget(QtWidgets.QLabel("Steering"), 4, 0)
        layout.addWidget(QtWidgets.QLabel("Gear"), 5, 0)
        layout.addWidget(QtWidgets.QLabel("Turn Indicator"), 6, 0)
        layout.addWidget(QtWidgets.QLabel("Hazard Lights"), 7, 0)
        layout.setRowStretch(8, 1)
        widget = QtWidgets.QWidget()
        widget.setLayout(layout)

        self.mouse = MouseCapture(self.adapi)
        self.addWidget(self.mouse)
        self.addWidget(widget)

    def shutdown(self):
        pass


class MouseCapture(QtWidgets.QLabel):
    def __init__(self, adapi: Adapi):
        super().__init__()
        self.adapi = adapi
        self.setText("+")
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setStyleSheet("background-color: gray;")
        self.setMouseTracking(False)

    def mouseMoveEvent(self, event):
        w = self.size().width() / 2
        h = self.size().height() / 2
        x = (event.pos().x() - w) / w
        y = (event.pos().y() - h) / h
        x = -max(-1.0, min(1.0, x))
        y = -max(-1.0, min(1.0, y))
        steer = x
        accel = max(0.0, +y)
        brake = max(0.0, -y)
        print("accel", accel, "brake", brake, "steer", steer)
        self.adapi.set_pedals(accel, brake)
        self.adapi.set_steering(steer)
