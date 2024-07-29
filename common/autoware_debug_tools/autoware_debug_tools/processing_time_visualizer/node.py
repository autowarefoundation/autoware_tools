import curses
import json
import time
from typing import Dict
import uuid

import pyperclip
import rclpy
import rclpy.executors
from rclpy.node import Node
from tier4_debug_msgs.msg import ProcessingTimeTree as ProcessingTimeTreeMsg

from .print_tree import print_trees
from .topic_selector import select_topic
from .tree import ProcessingTimeTree
from .utils import exit_curses
from .utils import init_curses


class ProcessingTimeVisualizer(Node):
    def __init__(self):
        super().__init__("processing_time_visualizer" + str(uuid.uuid4()).replace("-", "_"))
        self.subscriber = self.subscribe_processing_time_tree()
        self.quit_option = None
        self.trees: Dict[str, ProcessingTimeTree] = {}
        self.worst_case_tree: Dict[str, ProcessingTimeTree] = {}
        self.stdcscr = init_curses()
        self.show_comment = False
        print_trees("🌲 Processing Time Tree 🌲", self.topic_name, self.trees, self.stdcscr)

        self.create_timer(0.1, self.update_screen)

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

    def update_screen(self):
        key = self.stdcscr.getch()

        self.show_comment = not self.show_comment if key == ord("c") else self.show_comment
        logs = print_trees(
            "🌲 Processing Time Tree 🌲",
            self.topic_name,
            self.trees.values(),
            self.stdcscr,
            self.show_comment,
        )
        if key == ord("y"):
            pyperclip.copy(logs)
        if key == ord("q"):
            self.quit_option = "q"
            raise KeyboardInterrupt
        if key == ord("r"):
            self.quit_option = "r"
            raise KeyboardInterrupt

    def callback(self, msg: ProcessingTimeTreeMsg):
        tree = ProcessingTimeTree.from_msg(msg)
        self.trees[tree.name] = tree
        if tree.name not in self.worst_case_tree:
            self.worst_case_tree[tree.name] = tree
        else:
            self.worst_case_tree[tree.name] = (
                tree
                if tree.processing_time > self.worst_case_tree[tree.name].processing_time
                else self.worst_case_tree[tree.name]
            )


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
        if node.quit_option == "r":
            pyperclip.copy(json.dumps([v.__dict__() for v in node.worst_case_tree.values()]))
        if len(node.worst_case_tree) == 0:
            exit(1)
        print("⏰ Worst Case Execution Time ⏰")
        for tree in node.worst_case_tree.values():
            print(tree, end=None)


if __name__ == "__main__":
    main()
