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


from autoware_system_msgs.srv import ChangeAutowareControl
from autoware_system_msgs.srv import ChangeOperationMode
from python_qt_binding import QtCore
from python_qt_binding import QtWidgets


class OperationModeControl(QtWidgets.QWidget):
    def __init__(self, node, modes):
        super().__init__()
        self.create_widget(modes)
        self.client_mode = node.create_client(
            ChangeOperationMode, "/system/operation_mode/change_operation_mode"
        )
        self.client_ctrl = node.create_client(
            ChangeAutowareControl, "/system/operation_mode/change_autoware_control"
        )

    def create_widget(self, modes):
        modes = [
            (ChangeOperationMode.Request.STOP, "Stop"),
            (ChangeOperationMode.Request.AUTONOMOUS, "Autonomous"),
            (ChangeOperationMode.Request.LOCAL, "Local"),
            (ChangeOperationMode.Request.REMOTE, "Remote"),
        ]
        ctrls = [
            (True, "Autoware"),
            (False, "Manual"),
        ]
        self.result = QtWidgets.QLabel("response")
        self.result.setAlignment(QtCore.Qt.AlignCenter)
        self.result.setStyleSheet("border: 1px solid black;")
        layout = QtWidgets.QGridLayout()
        layout.setSpacing(0)
        for col, (ctrl, name) in enumerate(ctrls, start=0):
            button = QtWidgets.QPushButton(name)
            button.clicked.connect(lambda _, ctrl=ctrl: self.change_ctrl(ctrl))
            layout.addWidget(button, 0, col * 2, 1, 2)
        for col, (mode, name) in enumerate(modes, start=0):
            button = QtWidgets.QPushButton(name)
            button.clicked.connect(lambda _, mode=mode: self.change_mode(mode))
            layout.addWidget(button, 1, col)
        layout.addWidget(self.result, 2, 0, 1, len(modes))
        self.setLayout(layout)

    def change_mode(self, mode):
        request = ChangeOperationMode.Request()
        request.mode = mode
        self.result.setText("waiting for response...")
        self.client_mode.call_async(request).add_done_callback(self.on_response)

    def change_ctrl(self, ctrl):
        request = ChangeAutowareControl.Request()
        request.autoware_control = ctrl
        self.result.setText("waiting for response...")
        self.client_ctrl.call_async(request).add_done_callback(self.on_response)

    def on_response(self, future):
        result = future.result()
        status = result.status
        self.result.setText(
            f"success: {status.success}, code: {status.code}, message: {status.message}"
        )
