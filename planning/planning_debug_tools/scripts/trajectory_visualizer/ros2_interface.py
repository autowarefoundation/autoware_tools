import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from typing import Callable, List, Tuple, Any, Optional

class ROS2Interface(Node):
    """
    A class to interface with ROS 2, providing functionalities to find specific topics
    and subscribe to them with callbacks.
    """
    TARGET_MSG_TYPE_STR = "autoware_planning_msgs/msg/Trajectory"

    def __init__(self, node_name: str = "ros2_interface_node"):
        super().__init__(node_name)
        self._subs = {} # Stores active subscriptions: {topic_name: subscription_object}

    def get_trajectory_topics(self) -> List[Tuple[str, str]]:
        topic_names_and_types = self.get_topic_names_and_types()
        
        trajectory_topics = []
        for topic_name, msg_types in topic_names_and_types:
            if self.TARGET_MSG_TYPE_STR in msg_types:
                trajectory_topics.append(topic_name)
        return trajectory_topics

    def add_callback(self, 
                     topic_name: str, 
                     msg_type: Any, 
                     callback_function: Callable[[Any], None]):
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE 
        )

        if topic_name in self._subs:
            self.destroy_subscription(self._subs[topic_name])
            del self._subs[topic_name]

        try:
            subscription = self.create_subscription(
                msg_type,
                topic_name,
                callback_function,
                qos_profile
            )
            self._subs[topic_name] = subscription
        except Exception as e:
            self.get_logger().error(f"Failed to create subscription for topic '{topic_name}': {e}")

    def remove_callback(self, topic_name: str):
        """
        Removes a callback (subscription) for a given topic.

        Args:
            topic_name (str): The name of the topic to unsubscribe from.
        """
        if topic_name in self._subs:
            self.destroy_subscription(self._subs[topic_name])
            del self._subs[topic_name]
            self.get_logger().info(f"Unsubscribed from topic '{topic_name}'.")

    def cleanup(self):
        """
        Cleans up all active subscriptions. Call this before destroying the node
        if you want to explicitly manage subscription cleanup.
        The node's destroy_node() will also handle this.
        """
        topics_to_remove = list(self._subs.keys())
        for topic_name in topics_to_remove:
            self.remove_callback(topic_name)