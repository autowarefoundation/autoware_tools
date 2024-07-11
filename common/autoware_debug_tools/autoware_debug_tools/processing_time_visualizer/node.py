import curses
import time
import uuid

import rclpy
import rclpy.executors
from rclpy.node import Node
from tier4_debug_msgs.msg import ProcessingTimeTree as ProcessingTimeTreeMsg

from .print_tree import print_tree
from .topic_selector import select_topic
from .tree import ProcessingTimeTree
from .utils import exit_curses
from .utils import init_curses


class ProcessingTimeVisualizer(Node):
    def __init__(self):
        super().__init__("processing_time_visualizer" + str(uuid.uuid4()).replace("-", "_"))

        self.subscriber = self.subscribe_processing_time_tree()
        self.tree = ProcessingTimeTree()
        self.worst_case_tree = self.tree
        self.stdcscr = init_curses()
        print_tree("🌲 Processing Time Tree 🌲", self.topic_name, self.tree, self.stdcscr)

    def subscribe_processing_time_tree(self):
        topics = []

        s = time.time()
        while True:
            for topic_name, topic_types in self.get_topic_names_and_types():
                for topic_type in topic_types:
                    if (
                        topic_type == "tier4_debug_msgs/msg/ProcessingTimeTree"
                        and topic_name not in topics
                    ):
                        topics.append(topic_name)

            if time.time() - s > 1.0:
                break

        if len(topics) == 0:
            self.get_logger().info("No ProcessingTimeTree topic found")
            self.get_logger().info("Exiting...")
            exit(1)
        else:
            self.topic_name = curses.wrapper(select_topic, topics)
            subscriber = self.create_subscription(
                ProcessingTimeTreeMsg,
                self.topic_name,
                self.callback,
                10,
            )

        return subscriber

    def callback(self, msg: ProcessingTimeTreeMsg):
        self.tree = ProcessingTimeTree.from_msg(msg)
        if self.tree.processing_time > self.worst_case_tree.processing_time:
            self.worst_case_tree = self.tree
        print_tree("🌲 Processing Time Tree 🌲", self.topic_name, self.tree, self.stdcscr)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ProcessingTimeVisualizer()
    except KeyboardInterrupt:
        exit_curses()
        return
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.destroy_node()
        exit_curses()
        print("⏰ Worst Case Execution Time ⏰")
        print(node.worst_case_tree)


if __name__ == "__main__":
    main()
