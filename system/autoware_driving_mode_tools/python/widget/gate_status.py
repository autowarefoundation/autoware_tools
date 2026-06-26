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


from autoware_driving_mode_tools.utils import durable_qos
from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
from tier4_system_msgs.msg import CommandFilterStatus
from tier4_system_msgs.msg import CommandSourceStatus
from tier4_system_msgs.msg import TrajectorySourceStatus


class TrajectorySourceDisplay(QtWidgets.QLabel):
    def __init__(self, node):
        super().__init__("No Data")
        self.subscription = node.create_subscription(
            TrajectorySourceStatus,
            "/planning/trajectory_gate/source/status",
            self.on_msg,
            durable_qos(1),
        )
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setStyleSheet("border: 1px solid black;")

    def on_msg(self, msg):
        self.setText(str(msg.source))


class CommandSourceDisplay(QtWidgets.QLabel):
    def __init__(self, node):
        super().__init__("No Data")
        self.subscription = node.create_subscription(
            CommandSourceStatus,
            "/control/control_command_gate/source/status",
            self.on_msg,
            durable_qos(1),
        )
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setStyleSheet("border: 1px solid black;")

    def on_msg(self, msg):
        self.setText(str(msg.source))


class CommandFilterDisplay(QtWidgets.QLabel):
    def __init__(self, node):
        super().__init__("No Data")
        self.subscription = node.create_subscription(
            CommandFilterStatus,
            "/control/control_command_gate/filter/status",
            self.on_msg,
            durable_qos(1),
        )
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setStyleSheet("border: 1px solid black;")

    def on_msg(self, msg):
        self.setText(str(msg.filter))
