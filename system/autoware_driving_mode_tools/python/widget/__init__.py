# Copyright 2026 The Autoware Contributors
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

from .driving_mode import DrivingModeControl
from .gate_status import CommandFilterDisplay
from .gate_status import CommandSourceDisplay
from .gate_status import TrajectorySourceDisplay
from .operation_mode import OperationModeControl
from .vehicle_interface import VehicleInterfaceWidget


class MainWidget(QtWidgets.QWidget):
    def __init__(self, node):
        super().__init__()
        modes = [
            (1001, "Stop"),
            (1002, "Autonomous"),
            (1003, "Local"),
            (1004, "Remote"),
            (2001, "Emergency"),
            (2002, "Comfortable"),
        ]
        self.operation_mode_control = OperationModeControl(node, modes)
        self.modes_control = DrivingModeControl(node, modes)
        self.trajectory_source = TrajectorySourceDisplay(node)
        self.command_source = CommandSourceDisplay(node)
        self.command_filter = CommandFilterDisplay(node)
        self.vehicle_interface = VehicleInterfaceWidget(node)

        layout = QtWidgets.QGridLayout()
        self.setLayout(layout)

        self._add_row(layout, 0, "Operation Mode", self.operation_mode_control)
        self._add_row(layout, 1, "Driving Mode", self.modes_control)
        self._add_row(layout, 2, "Trajectory Source", self.trajectory_source)
        self._add_row(layout, 3, "Command Source", self.command_source)
        self._add_row(layout, 4, "Command Filter", self.command_filter)
        self._add_row(layout, 5, "Vehicle Interface", self.vehicle_interface)

    @staticmethod
    def _add_row(layout, row, label, widget):
        if isinstance(widget, QtWidgets.QWidget):
            layout.addWidget(QtWidgets.QLabel(label), row, 0)
            layout.addWidget(widget, row, 1)
        if isinstance(widget, QtWidgets.QLayout):
            layout.addWidget(QtWidgets.QLabel(label), row, 0)
            layout.addLayout(widget, row, 1)
