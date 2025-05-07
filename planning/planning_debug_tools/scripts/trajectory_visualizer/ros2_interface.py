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
        """
        Initializes the ROS2Interface node.

        Args:
            node_name (str): The name for the ROS 2 node.
        """
        super().__init__(node_name)
        self.get_logger().info(f"Node '{self.get_name()}' started.")
        self._subs = {} # Stores active subscriptions: {topic_name: subscription_object}

    def get_trajectory_topics(self) -> List[Tuple[str, str]]:
        """
        Gets a list of all topic names and their types that match
        'autoware_planning_msgs.msg.Trajectory'.

        Returns:
            List[str]]: A list of topic name
        """
        self.get_logger().info(f"Searching for topics with type '{self.TARGET_MSG_TYPE_STR}'...")
        topic_names_and_types = self.get_topic_names_and_types()
        
        trajectory_topics = []
        for topic_name, msg_types in topic_names_and_types:
            if self.TARGET_MSG_TYPE_STR in msg_types:
                trajectory_topics.append(topic_name)
        return trajectory_topics

    def add_callback(self, 
                     topic_name: str, 
                     msg_type: Any, 
                     callback_function: Callable[[Any], None], 
                     qos_profile: Optional[QoSProfile] = None) -> bool:
        """
        Adds a callback function to a given topic. If a subscription already exists
        for this topic, it will be replaced.

        Args:
            topic_name (str): The name of the topic to subscribe to.
            msg_type (Any): The Python class of the message type (e.g., from an import like
                            `from std_msgs.msg import String` then pass `String`).
            callback_function (Callable[[Any], None]): The function to call when a message
                                                       is received. It should accept one
                                                       argument (the message).
            qos_profile (Optional[QoSProfile]): The QoS profile for the subscription.
                                                If None, a default reliable, keep-last profile
                                                with depth 1 is used.

        Returns:
            bool: True if the subscription was successfully created, False otherwise.
        """
        if not topic_name:
            self.get_logger().error("Topic name cannot be empty.")
            return False
        if not msg_type:
            self.get_logger().error(f"Message type for topic '{topic_name}' cannot be None.")
            return False
        if not callable(callback_function):
            self.get_logger().error(f"Callback for topic '{topic_name}' is not a callable function.")
            return False

        if qos_profile is None:
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.VOLATILE 
            )
            self.get_logger().info(f"Using default QoS profile for topic '{topic_name}'.")

        if topic_name in self._subs:
            self.get_logger().info(f"Replacing existing subscription for topic '{topic_name}'.")
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
            self.get_logger().info(f"Successfully subscribed to topic '{topic_name}' with type '{msg_type.__name__}'.")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to create subscription for topic '{topic_name}': {e}")
            return False

    def remove_callback(self, topic_name: str) -> bool:
        """
        Removes a callback (subscription) for a given topic.

        Args:
            topic_name (str): The name of the topic to unsubscribe from.
        
        Returns:
            bool: True if the subscription was found and destroyed, False otherwise.
        """
        if topic_name in self._subs:
            self.destroy_subscription(self._subs[topic_name])
            del self._subs[topic_name]
            self.get_logger().info(f"Unsubscribed from topic '{topic_name}'.")
            return True
        else:
            self.get_logger().warn(f"No active subscription found for topic '{topic_name}' to remove.")
            return False

    def cleanup(self):
        """
        Cleans up all active subscriptions. Call this before destroying the node
        if you want to explicitly manage subscription cleanup.
        The node's destroy_node() will also handle this.
        """
        self.get_logger().info("Cleaning up all active subscriptions...")
        topics_to_remove = list(self._subs.keys())
        for topic_name in topics_to_remove:
            self.remove_callback(topic_name)
        self.get_logger().info("All subscriptions cleaned up.")

# Example Usage (Illustrative - requires a ROS 2 environment and messages)
# You would run this within a ROS 2 Python script.

# 1. Define your callback function
def my_trajectory_processor(msg):
    """
    This is an example callback function.
    It will be called with the received message object.
    """
    # Assuming msg is an instance of autoware_planning_msgs.msg.Trajectory
    # You can access its fields here, e.g., msg.points
    ros_interface_node.get_logger().info(f"Received trajectory with {len(msg.points)} points.")
    # Add your processing logic here

# Global variable for the node instance, to be accessible by callbacks if needed,
# or pass the logger/node instance into the callback if preferred.
ros_interface_node = None 

def main(args=None):
    global ros_interface_node
    rclpy.init(args=args)
    
    ros_interface_node = ROS2Interface(node_name="my_trajectory_listener_node")

    try:
        # 2. Get list of trajectory topics
        trajectory_topics = ros_interface_node.get_trajectory_topics()

        if trajectory_topics:
            ros_interface_node.get_logger().info("Available trajectory topics:")
            for topic_name, topic_type in trajectory_topics:
                ros_interface_node.get_logger().info(f"  - {topic_name} ({topic_type})")

            # 3. Add a callback to the first found trajectory topic (example)
            # IMPORTANT: You MUST have autoware_planning_msgs installed and built
            # in your ROS 2 workspace for this import to work.
            try:
                from autoware_planning_msgs.msg import Trajectory as TrajectoryMsg
                
                # Select a topic to subscribe to, e.g., the first one found
                topic_to_subscribe = trajectory_topics[0][0] 
                
                ros_interface_node.add_callback(
                    topic_name=topic_to_subscribe,
                    msg_type=TrajectoryMsg, # Pass the actual message class
                    callback_function=my_trajectory_processor
                )
                
                # You can add more callbacks to other topics or the same topic with different
                # message types if needed by calling add_callback multiple times.

            except ImportError:
                ros_interface_node.get_logger().error(
                    "Could not import 'autoware_planning_msgs.msg.Trajectory'. "
                    "Make sure the Autoware messages are installed and sourced."
                )
            except IndexError:
                ros_interface_node.get_logger().info("No trajectory topics found to subscribe to.")

        else:
            ros_interface_node.get_logger().info("No topics of type 'autoware_planning_msgs.msg.Trajectory' found.")

        # 4. Spin the node to process callbacks
        # rclpy.spin() keeps the node alive and processing callbacks until Ctrl+C
        ros_interface_node.get_logger().info("Spinning node... Press Ctrl+C to exit.")
        rclpy.spin(ros_interface_node)

    except KeyboardInterrupt:
        ros_interface_node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        # 5. Clean up
        if ros_interface_node:
            ros_interface_node.cleanup() # Explicitly clean up subscriptions
            ros_interface_node.destroy_node()
        rclpy.shutdown()
        print("ROS 2 Interface shut down cleanly.")

if __name__ == '__main__':
    main()
