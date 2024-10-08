from datetime import datetime
from enum import Enum


class DebugMessageType(Enum):
    INFO = {"value": "INFO", "color": "\033[94m"}  # Blue
    ERROR = {"value": "ERROR", "color": "\033[91m"}  # Red
    SUCCESS = {"value": "SUCCESS", "color": "\033[92m"}  # Green


class Debug:
    @classmethod
    def log(cls, message, message_type=DebugMessageType.INFO):
        current_time = datetime.now().strftime("%H:%M:%S")
        color = message_type.value["color"]
        reset_color = "\033[0m"  # Reset color to default
        formatted_message = (
            f"[{current_time} - {color}{message_type.value['value']}{reset_color}] {message}"
        )
        print(formatted_message)

    @classmethod
    def get_log(cls, message, message_type=DebugMessageType.INFO):
        current_time = datetime.now().strftime("%H:%M:%S")
        color = message_type.value["color"]
        reset_color = "\033[0m"  # Reset color to default
        formatted_message = (
            f"[{current_time} - {color}{message_type.value['value']}{reset_color}] {message}"
        )
        return formatted_message
