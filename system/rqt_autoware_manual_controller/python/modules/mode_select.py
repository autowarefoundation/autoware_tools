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
from rqt_autoware_manual_controller.modules.adapi import ManualMode


class ManualModeSelect(QtWidgets.QVBoxLayout):
    def __init__(self, adapi: Adapi):
        super().__init__()
        adapi.set_on_mode_list(self.on_mode_list)
        self.adapi = adapi
        self.buttons = {mode: QtWidgets.QPushButton(mode.name) for mode in ManualMode}
        for mode, button in self.buttons.items():
            button.setEnabled(mode is ManualMode.Disabled)
            button.clicked.connect(lambda clicked, mode=mode: self.adapi.select_mode(mode))
            self.addWidget(button)

    def on_mode_list(self, modes):
        for mode in modes:
            self.buttons[mode].setEnabled(True)


class ManualModeStatus(QtWidgets.QLabel):
    def __init__(self, adapi: Adapi):
        super().__init__()
        self.setText("Unknown")
        adapi.set_on_mode_status(self.on_mode_status)

    def on_mode_status(self, mode: ManualMode):
        self.setText(mode.name)
