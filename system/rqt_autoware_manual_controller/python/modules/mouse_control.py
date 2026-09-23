from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
from rqt_autoware_manual_controller.modules.adapi import Adapi


class MouseControl:
    def __init__(self, adapi: Adapi):
        self.adapi = adapi
        self.capture = MouseCapture(self)
        self.capture.setText("+")
        self.capture.setAlignment(QtCore.Qt.AlignCenter)
        self.capture.setStyleSheet("background-color: gray;")
        self.capture.setMouseTracking(False)
        self.accel = QtWidgets.QLabel()
        self.brake = QtWidgets.QLabel()
        self.steer = QtWidgets.QLabel()

    def update_pedals(self, accel: float, brake: float):
        self.accel.setText(f"{accel:+0.2f}")
        self.brake.setText(f"{brake:+0.2f}")
        self.adapi.send_pedals(accel, brake)

    def update_steer(self, steer: float):
        self.steer.setText(f"{steer:+0.2f}")
        self.adapi.send_steering(steer)


class MouseCapture(QtWidgets.QLabel):
    def __init__(self, control: MouseControl):
        super().__init__()
        self.control = control

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
        self.control.update_pedals(accel, brake)
        self.control.update_steer(steer)
