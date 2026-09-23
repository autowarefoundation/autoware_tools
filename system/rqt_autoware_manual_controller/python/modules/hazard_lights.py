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
from rqt_autoware_manual_controller.modules.adapi import HazardLightsEnum


class HazardLightsControl:
    def __init__(self, adapi: Adapi):
        self.adapi = adapi
        self.adapi.set_on_hazard_lights_status(self.on_hazard_lights_status)

        self.command = QtWidgets.QVBoxLayout()
        self.status = QtWidgets.QLabel()

        hazard_lights = [HazardLightsEnum.Disable, HazardLightsEnum.Enable]
        self.buttons = {
            hazard_light: QtWidgets.QPushButton(hazard_light.name) for hazard_light in hazard_lights
        }
        for hazard_light, button in self.buttons.items():
            button.clicked.connect(
                lambda _, hazard_light=hazard_light: self.adapi.send_hazard_lights(hazard_light)
            )
            self.command.addWidget(button)

    def on_hazard_lights_status(self, hazard_lights: HazardLightsEnum):
        self.status.setText(hazard_lights.name)
