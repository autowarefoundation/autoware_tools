from datetime import datetime
from enum import Enum


class DebugMessageType(Enum):
    """
    Enum representing different types of debug messages.

    Attributes:
        INFO (dict): Message type for informational messages.
        ERROR (dict): Message type for error messages.
        SUCCESS (dict): Message type for success messages.
    """

    INFO = {"value": "INFO", "color": "\033[94m"}  # Blue
    ERROR = {"value": "ERROR", "color": "\033[91m"}  # Red
    SUCCESS = {"value": "SUCCESS", "color": "\033[92m"}  # Green


class Debug:
    """
    A class for logging debug messages with different types.

    Methods:
        log(message, message_type=DebugMessageType.INFO):
            Logs a message to the console with a timestamp and message type.

        get_log(message, message_type=DebugMessageType.INFO):
            Returns a formatted log message string without printing it.
    """

    @classmethod
    def log(cls, message, message_type=DebugMessageType.INFO):
        """
        Logs a message to the console with a timestamp and message type.

        Parameters:
            message (str): The message to log.
            message_type (DebugMessageType): The type of the message (default: INFO).

        Returns:
            None
        """
        current_time = datetime.now().strftime("%H:%M:%S")
        color = message_type.value["color"]
        reset_color = "\033[0m"  # Reset color to default
        formatted_message = f"[{current_time} - {color}{message_type.value['value']}{reset_color}] {message}"
        print(formatted_message)

    @classmethod
    def get_log(cls, message, message_type=DebugMessageType.INFO):
        """
        Returns a formatted log message string without printing it.

        Parameters:
            message (str): The message to format.
            message_type (DebugMessageType): The type of the message (default: INFO).

        Returns:
            str: The formatted log message string.
        """
        current_time = datetime.now().strftime("%H:%M:%S")
        color = message_type.value["color"]
        reset_color = "\033[0m"  # Reset color to default
        formatted_message = f"[{current_time} - {color}{message_type.value['value']}{reset_color}] {message}"
        return formatted_message
