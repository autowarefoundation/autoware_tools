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
from rqt_autoware_manual_controller.modules.adapi import TurnIndicatorsEnum


class TurnIndicatorsControl:
    def __init__(self, adapi: Adapi):
        self.adapi = adapi
        self.adapi.set_on_turn_indicators_status(self.on_turn_indicators_status)

        self.command = QtWidgets.QVBoxLayout()
        self.status = QtWidgets.QLabel()

        turn_indicators = [
            TurnIndicatorsEnum.Disable,
            TurnIndicatorsEnum.Left,
            TurnIndicatorsEnum.Right,
        ]
        self.buttons = {
            turn_indicator: QtWidgets.QPushButton(turn_indicator.name)
            for turn_indicator in turn_indicators
        }
        for turn_indicator, button in self.buttons.items():
            button.clicked.connect(
                lambda _, turn_indicator=turn_indicator: self.adapi.send_turn_indicators(
                    turn_indicator
                )
            )
            self.command.addWidget(button)

    def on_turn_indicators_status(self, turn_indicators: TurnIndicatorsEnum):
        self.status.setText(turn_indicators.name)
