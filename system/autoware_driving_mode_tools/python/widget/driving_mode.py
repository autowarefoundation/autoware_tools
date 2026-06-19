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

from enum import Enum
from itertools import product

from autoware_driving_mode_manager.msg import DebugModeFlag
from autoware_driving_mode_tools.utils import default_qos
from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
from tier4_system_msgs.msg import DrivingModeFlag
from tier4_system_msgs.msg import DrivingModeFlagItem


def centered_label(text):
    label = QtWidgets.QLabel(text)
    label.setAlignment(QtCore.Qt.AlignCenter)
    return label


class FlagType(Enum):
    Available = 1
    Active = 2
    Stable = 3
    Continuable = 4


class FlagStatus:
    def __init__(self):
        self.send = False
        self.data = False


class FlagButton:
    def __init__(self):
        self.send = None
        self.data = None


class DrivingModeControl(QtWidgets.QWidget):
    def __init__(self, node, modes):
        super().__init__()
        self.modes = [mode for mode, name in modes]
        self.clock = node.get_clock()
        self.status = {pair: FlagStatus() for pair in product(self.modes, FlagType)}
        self.button = {pair: FlagButton() for pair in product(self.modes, FlagType)}
        self.flags = {}
        self.timer = node.create_timer(0.5, self.on_timer)

        self.publishers = {}
        self.publishers[FlagType.Available] = node.create_publisher(
            DrivingModeFlag, "/system/driving_mode/available", default_qos(1)
        )
        self.publishers[FlagType.Active] = node.create_publisher(
            DrivingModeFlag, "/system/driving_mode/active", default_qos(1)
        )
        self.publishers[FlagType.Stable] = node.create_publisher(
            DrivingModeFlag, "/system/driving_mode/stable", default_qos(1)
        )
        self.publishers[FlagType.Continuable] = node.create_publisher(
            DrivingModeFlag, "/system/driving_mode/continuable", default_qos(1)
        )
        self.subscription = node.create_subscription(
            DebugModeFlag,
            "/system/driving_mode_manager/debug/status",
            self.on_msg,
            default_qos(1),
        )
        self.create_widget(modes)

    def on_msg(self, msg):
        for item in msg.items:
            text = ""
            text += "A" if item.available else "-"
            text += "R" if item.active else "-"
            text += "S" if item.stable else "-"
            text += "C" if item.continuable else "-"
            self.flags[item.mode].setText(text)

    def on_timer(self):
        self.publish(FlagType.Available)
        self.publish(FlagType.Active)
        self.publish(FlagType.Stable)
        self.publish(FlagType.Continuable)

    def publish(self, flag):
        items = []
        for mode in self.modes:
            status = self.status.get((mode, flag))
            if status.send:
                items.append(DrivingModeFlagItem(mode=mode, flag=status.data))
        msg = DrivingModeFlag(stamp=self.clock.now().to_msg(), items=items)
        self.publishers[flag].publish(msg)

    def set_flag_send(self, mode, flag, send):
        self.status[(mode, flag)].send = send
        self.publish(flag)

    def set_flag_data(self, mode, flag, data):
        self.status[(mode, flag)].data = data
        self.publish(flag)

    def set_flag_send_all(self, flag, send):
        for mode in self.modes:
            self.button[(mode, flag)].send.setChecked(send)
            self.status[(mode, flag)].send = send
        self.publish(flag)

    def set_flag_data_all(self, flag, data):
        print(flag, data)
        for mode in self.modes:
            self.button[(mode, flag)].data.setChecked(data)
            self.status[(mode, flag)].data = data
        self.publish(flag)

    def create_button(self, flag, mode, layout, row, col):
        button = self.button[(mode, flag)]
        button.send = QtWidgets.QPushButton("Send")
        button.data = QtWidgets.QPushButton("Data")
        button.send.clicked.connect(lambda clicked: self.set_flag_send(mode, flag, clicked))
        button.data.clicked.connect(lambda clicked: self.set_flag_data(mode, flag, clicked))
        for index, button in enumerate((button.send, button.data)):
            button.setCheckable(True)
            layout.addWidget(button, row, col + index)

    def create_all_buttons(self, flag, layout, row, col):
        button = FlagButton()
        button.send = QtWidgets.QPushButton("Send")
        button.data = QtWidgets.QPushButton("Data")
        button.send.clicked.connect(lambda clicked: self.set_flag_send_all(flag, clicked))
        button.data.clicked.connect(lambda clicked: self.set_flag_data_all(flag, clicked))
        for index, button in enumerate((button.send, button.data)):
            button.setCheckable(True)
            layout.addWidget(button, row, col + index)

    def create_widget(self, modes):
        layout = QtWidgets.QGridLayout()
        layout.setSpacing(0)
        layout.addWidget(QtWidgets.QLabel("Autoware Mode"), 1, 0)
        layout.addWidget(centered_label("  Flags  "), 1, 1)
        layout.addWidget(centered_label("Available"), 1, 2, 1, 2)
        layout.addWidget(centered_label("Active"), 1, 4, 1, 2)
        layout.addWidget(centered_label("Stable"), 1, 6, 1, 2)
        layout.addWidget(centered_label("Continuable"), 1, 8, 1, 2)
        self.setLayout(layout)
        self.create_all_buttons(FlagType.Available, layout, 0, 2)
        self.create_all_buttons(FlagType.Active, layout, 0, 4)
        self.create_all_buttons(FlagType.Stable, layout, 0, 6)
        self.create_all_buttons(FlagType.Continuable, layout, 0, 8)
        for row, (mode, name) in enumerate(modes, start=2):
            layout.addWidget(QtWidgets.QLabel(f"{name} ({mode})"), row, 0)
            self.create_button(FlagType.Available, mode, layout, row, 2)
            self.create_button(FlagType.Active, mode, layout, row, 4)
            self.create_button(FlagType.Stable, mode, layout, row, 6)
            self.create_button(FlagType.Continuable, mode, layout, row, 8)
            label = centered_label("----")
            layout.addWidget(label, row, 1)
            self.flags[mode] = label
