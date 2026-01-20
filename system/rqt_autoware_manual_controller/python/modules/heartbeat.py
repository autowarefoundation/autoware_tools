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
from rqt_autoware_manual_controller.modules.adapi import HeartbeatEnum


class HeartbeatControl:
    def __init__(self, adapi: Adapi):
        self.adapi = adapi
        self.command = QtWidgets.QVBoxLayout()
        self.status = QtWidgets.QLabel("---")

        modes = [HeartbeatEnum.NotReady, HeartbeatEnum.Ready]
        self.buttons = {mode: QtWidgets.QPushButton(mode.name) for mode in modes}
        for mode, button in self.buttons.items():
            button.clicked.connect(lambda _, mode=mode: self.adapi.send_heartbeat(mode))
            self.command.addWidget(button)
