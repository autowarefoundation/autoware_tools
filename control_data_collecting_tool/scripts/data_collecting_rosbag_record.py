#!/usr/bin/env python3

# Copyright 2024 Proxima Technology Inc, TIER IV
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from datetime import datetime
from functools import partial
import os

from ament_index_python.packages import get_package_share_directory
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_vehicle_msgs.msg import ControlModeReport
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rosbag2_py import ConverterOptions
from rosbag2_py import SequentialWriter
from rosbag2_py import StorageOptions
from rosbag2_py._storage import TopicMetadata
from rosidl_runtime_py.utilities import get_message
import yaml


class MessageWriter:
    def __init__(self, topics, node):
        self.topics = topics
        self.node = node
        self.message_writer = None
        self.message_subscriptions_ = []

    def create_writer(self):
        self.message_writer = SequentialWriter()

    def subscribe_topics(self):
        topic_type_list = []
        unsubscribed_topic = []

        # Get topic types from the list of topics and store them
        for topic_name in self.topics:
            topic_type = self.get_topic_type(topic_name)
            topic_type_list.append(topic_type)
            if topic_type is None:
                # If a topic type is not found, mark it as unsubscribed
                unsubscribed_topic.append(topic_name)

        # If some topics are not found, log a message and skip the subscription
        if len(unsubscribed_topic) > 0:
            self.node.get_logger().info(f"Failed to get type for topic: {unsubscribed_topic}")
            return

        # Generate a unique directory and filename based on the current time for the rosbag file
        now = datetime.now()
        formatted_time = now.strftime("%Y_%m_%d-%H_%M_%S")
        bag_dir = f"./rosbag2_{formatted_time}"
        storage_options = StorageOptions(uri=bag_dir, storage_id="sqlite3")
        converter_options = ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )

        # Open the rosbag for writing
        try:
            self.message_writer.open(storage_options, converter_options)
        except Exception as e:
            self.node.get_logger().error(f"Failed to open bag: {e}")
            return

        # Create topics in the rosbag for recording
        for topic_name, topic_type in zip(self.topics, topic_type_list):
            self.node.get_logger().info(f"Recording topic: {topic_name} of type: {topic_type}")
            topic_metadata = TopicMetadata(
                name=topic_name, type=topic_type, serialization_format="cdr"
            )
            self.message_writer.create_topic(topic_metadata)

    def get_topic_type(self, topic_name):
        # Get the list of topics and their types from the ROS node
        topic_names_and_types = self.node.get_topic_names_and_types()
        for name, types in topic_names_and_types:
            if name == topic_name:
                return types[0]
        return None

    def start_record(self):
        # Subscribe to the topics and start recording messages
        for topic_name in self.topics:
            topic_type = self.get_topic_type(topic_name)
            if topic_type:
                msg_module = get_message(topic_type)
                subscription_ = self.node.create_subscription(
                    msg_module, topic_name, partial(self.callback_write_message, topic_name), 10
                )
                self.message_subscriptions_.append(subscription_)
        self.node.get_logger().info("start recording rosbag")

    # call back function called in start recording
    def callback_write_message(self, topic_name, message):
        try:
            serialized_message = serialize_message(message)
            current_time = rclpy.clock.Clock().now()
            self.message_writer.write(topic_name, serialized_message, current_time.nanoseconds)
        except Exception as e:
            self.node.get_logger().error(f"Failed to write message: {e}")

    def stop_record(self):
        # Stop recording by destroying the subscriptions and deleting the writer
        for subscription_ in self.message_subscriptions_:
            self.node.destroy_subscription(subscription_)
        del self.message_writer
        self.node.get_logger().info("stop recording rosbag")


class DataCollectingRosbagRecord(Node):
    def __init__(self):
        super().__init__("data_collecting_rosbag_record")
        package_share_directory = get_package_share_directory("control_data_collecting_tool")
        topic_file_path = os.path.join(package_share_directory, "config", "topics.yaml")
        with open(topic_file_path, "r") as file:
            topic_data = yaml.safe_load(file)
        self.topics = topic_data["topics"]
        self.writer = MessageWriter(self.topics, self)
        self.subscribed = False
        self.recording = False

        self.present_operation_mode_ = None
        self.operation_mode_subscription_ = self.create_subscription(
            OperationModeState,
            "/system/operation_mode/state",
            self.subscribe_operation_mode,
            10,
        )
        
        self._present_control_mode_ = None
        self.control_mode_subscription_ = self.create_subscription(
            ControlModeReport,
            "/vehicle/status/control_mode",
            self.subscribe_control_mode,
            10,
        )

        self.timer_period_callback = 1.0
        self.timer_callback = self.create_timer(
            self.timer_period_callback,
            self.record_message,
        )

    def subscribe_operation_mode(self, msg):
        self.present_operation_mode_ = msg.mode

    def subscribe_control_mode(self, msg):
        self._present_control_mode_ = msg.mode

    def record_message(self):
        # Start subscribing to topics and recording if the operation mode is 3(LOCAL) and control mode is 1(AUTONOMOUS)
        if self.present_operation_mode_ == 3 and self._present_control_mode_ == 1 and not self.subscribed and not self.recording:
            self.writer.create_writer()
            self.writer.subscribe_topics()
            self.subscribed = True

        # Start recording if topics are subscribed and the operation mode is 3(LOCAL)
        if self.present_operation_mode_ == 3 and self._present_control_mode_ == 1 and self.subscribed and not self.recording:
            self.writer.start_record()
            self.recording = True

        # Stop recording if the operation mode changes from 3(LOCAL) 
        if (self.present_operation_mode_ != 3 or self._present_control_mode_ != 1) and self.subscribed and self.recording:
            self.writer.stop_record()
            self.subscribed = False
            self.recording = False


def main(args=None):
    rclpy.init(args=args)

    data_collecting_rosbag_record = DataCollectingRosbagRecord()
    rclpy.spin(data_collecting_rosbag_record)
    data_collecting_rosbag_record.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
