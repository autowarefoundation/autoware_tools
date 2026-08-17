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

from rqt_autoware_manual_controller.modules.adapi import Adapi
from rqt_autoware_manual_controller.widget import ControllerWidget
from rqt_gui_py.plugin import Plugin


class ControllerPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.widget = ControllerWidget(Adapi(context.node, "local"))
        context.add_widget(self.widget)

    def shutdown_plugin(self):
        self.widget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        plugin_settings.set_value("SplitterState", self.widget.saveState())

    def restore_settings(self, plugin_settings, instance_settings):
        if plugin_settings.contains("SplitterState"):
            self.widget.restoreState(plugin_settings.value("SplitterState"))
