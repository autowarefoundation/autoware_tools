from typing import Any
from typing import Callable
from typing import List
from typing import Tuple

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


class ROS2Interface(Node):
    """Class to interface with ROS 2."""

    TARGET_MSG_TYPE_STR = "autoware_planning_msgs/msg/Trajectory"
    EGO_ODOM_TOPIC = "/localization/kinematic_state"

    def __init__(self, node_name: str = "ros2_interface_node"):
        super().__init__(node_name)
        self._subs = {}  # Stores active subscriptions: {topic_name: subscription_object}
        self._ego_odom_sub = self.create_subscription(
            Odometry, self.EGO_ODOM_TOPIC, self.ego_odom_cb, 1
        )
        self.ego_odom = None

    def get_trajectory_topics(self) -> List[Tuple[str, str]]:
        topic_names_and_types = self.get_topic_names_and_types()

        trajectory_topics = []
        for topic_name, msg_types in topic_names_and_types:
            if self.TARGET_MSG_TYPE_STR in msg_types:
                trajectory_topics.append(topic_name)
        return trajectory_topics

    def add_callback(
        self, topic_name: str, msg_type: Any, callback_function: Callable[[Any], None]
    ):
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        if topic_name in self._subs:
            self.destroy_subscription(self._subs[topic_name])
            del self._subs[topic_name]

        try:
            subscription = self.create_subscription(
                msg_type, topic_name, callback_function, qos_profile
            )
            self._subs[topic_name] = subscription
        except Exception as e:
            self.get_logger().error(f"Failed to create subscription for topic '{topic_name}': {e}")

    def remove_callbacks(self):
        for sub in self._subs.values():
            self.destroy_subscription(sub)
        self._subs.clear()

    def ego_odom_cb(self, msg):
        self.ego_odom = msg

    def cleanup(self):
        self.remove_callbacks()
