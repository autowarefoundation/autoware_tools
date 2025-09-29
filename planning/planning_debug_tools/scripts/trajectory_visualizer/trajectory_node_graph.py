#!/usr/bin/env python3

from dataclasses import dataclass
from dataclasses import field
from pathlib import Path
from typing import Dict
from typing import Set

import rclpy
from rclpy.node import Node
import yaml


@dataclass
class GraphNode:
    """Represents a ROS 2 node in the graph with its publishers and subscribers."""

    name: str  # Full node name with namespace
    publishers: Dict[str, str] = field(default_factory=dict)  # topic_name -> message_type
    subscribers: Dict[str, str] = field(default_factory=dict)  # topic_name -> message_type

    def __str__(self):
        return f"Node: {self.name}\n  Publishers: {list(self.publishers.keys())}\n  Subscribers: {list(self.subscribers.keys())}"


class TrajectoryNodeGraph:
    """Library class for analyzing ROS 2 trajectory node graphs."""

    def __init__(self, node=None):
        """Initialize TrajectoryNodeGraph.

        Args:
            node: ROS 2 Node instance (optional, will create one if not provided)
        """
        self._external_node = node is not None
        self.node = node

        # Target message types to filter
        self.target_msg_types = [
            "autoware_internal_planning_msgs/msg/PathWithLaneId",
            "autoware_planning_msgs/msg/Path",
            "autoware_planning_msgs/msg/Trajectory",
        ]

        # Initialize result containers
        self.graph = {}
        self.main_flow_topics = []
        self.connection_topics = []
        self.nodes_in_order = []

    def analyze(self):
        """Analyze the ROS 2 graph and collect data.

        Returns:
            dict: Analysis results containing graph, topics, and connections
        """
        if not self.node:
            raise RuntimeError("No ROS 2 node available for analysis")

        # Build the graph
        self.graph = self.build_graph()

        # Analyze connections from behavior_path_planner
        start_node = (
            "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner"
        )
        if start_node in self.graph:
            # Collect main flow topics
            self.main_flow_topics = []
            self._trace_connections_and_collect(self.graph, start_node, visited=set(), depth=0)

            # Collect all topics including debug
            self.connection_topics = []
            self._collect_connection_topics(self.graph, start_node, visited=set())

            # Collect nodes in connection order
            self.nodes_in_order = []
            self._collect_nodes_in_order(self.graph, start_node, visited=set())

        return {
            "graph": self.graph,
            "main_flow_topics": self._get_unique_main_flow_topics(),
            "main_flow_topics_with_types": self._get_main_flow_topics_with_types(),
            "connection_topics": self.connection_topics,
            "nodes_in_order": self.nodes_in_order,
            "all_topics": self._get_all_topics(),
            "all_topics_with_types": self._get_all_topics_with_types(),
            "topics_by_node": self._get_topics_by_node(),
        }

    def _get_all_topics(self):
        """Get all unique topics from the graph."""
        all_topics = set()
        for node in self.graph.values():
            all_topics.update(node.publishers.keys())
            all_topics.update(node.subscribers.keys())
        return sorted(all_topics)

    def _get_unique_main_flow_topics(self):
        """Get unique main flow topics in order."""
        unique_main_flow = []
        seen = set()
        for topic in self.main_flow_topics:
            if topic not in seen:
                unique_main_flow.append(topic)
                seen.add(topic)
        return unique_main_flow

    def _get_main_flow_topics_with_types(self):
        """Get unique main flow topics with their message types."""
        unique_main_flow = []
        seen = set()
        for topic in self.main_flow_topics:
            if topic not in seen:
                # Find message type from graph
                msg_type = self._find_topic_type(topic)
                unique_main_flow.append((topic, msg_type))
                seen.add(topic)
        return unique_main_flow

    def _get_all_topics_with_types(self):
        """Get all unique topics with their message types."""
        all_topics = {}
        for node in self.graph.values():
            for topic, msg_type in node.publishers.items():
                all_topics[topic] = msg_type
            for topic, msg_type in node.subscribers.items():
                if topic not in all_topics:
                    all_topics[topic] = msg_type
        return [(topic, msg_type) for topic, msg_type in sorted(all_topics.items())]

    def _find_topic_type(self, topic_name):
        """Find the message type for a given topic."""
        for node in self.graph.values():
            if topic_name in node.publishers:
                return node.publishers[topic_name]
            if topic_name in node.subscribers:
                return node.subscribers[topic_name]
        return None

    def _get_topics_by_node(self):
        """Get topics organized by node in connection order with message types."""
        main_flow_set = set(self.main_flow_topics)
        topics_by_node = {}

        # Process nodes in connection order
        for node_name in self.nodes_in_order:
            if node_name in self.graph:
                node = self.graph[node_name]
                node_topics = []

                # First add main flow topics with message type
                for topic, msg_type in node.publishers.items():
                    if topic in main_flow_set:
                        node_topics.append(("main", topic, msg_type))

                # Then add other topics with message type
                for topic, msg_type in node.publishers.items():
                    if topic not in main_flow_set:
                        node_topics.append(("other", topic, msg_type))

                if node_topics:
                    topics_by_node[node_name] = node_topics

        # Add remaining nodes
        for node_name, node in self.graph.items():
            if node_name not in self.nodes_in_order:
                node_topics = []
                for topic, msg_type in node.publishers.items():
                    node_topics.append(("other", topic, msg_type))
                if node_topics:
                    topics_by_node[node_name] = node_topics

        return topics_by_node

    def save_to_yaml(self, filename="graph_config.yaml"):
        """Save the minimal graph configuration to a YAML file."""
        # Prepare minimal data for YAML
        yaml_data = {"main_flow_topics": self._get_unique_main_flow_topics(), "topics_by_node": {}}

        # Save topics by node with priority (minimal format)
        topics_by_node = self._get_topics_by_node()
        for node_name, topics in topics_by_node.items():
            yaml_data["topics_by_node"][node_name] = []
            for priority, topic_name, msg_type in topics:
                yaml_data["topics_by_node"][node_name].append(
                    {"priority": priority, "topic": topic_name, "msg_type": msg_type}
                )

        # Save to file in the same directory as the script
        script_dir = Path(__file__).parent
        yaml_path = script_dir / filename

        with open(yaml_path, "w") as f:
            yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)

        return str(yaml_path)

    @staticmethod
    def load_from_yaml(filename="graph_config.yaml"):
        """Load graph configuration from a YAML file."""
        # Try to find the YAML file in the script directory
        script_dir = Path(__file__).parent
        yaml_path = script_dir / filename

        if not yaml_path.exists():
            return None

        with open(yaml_path, "r") as f:
            yaml_data = yaml.safe_load(f)

        return yaml_data

    def filter_graph_with_active_topics(self, yaml_data, active_topics):
        """Filter the loaded graph data to only include currently active topics."""
        if not yaml_data:
            return None

        filtered_result = {
            "graph": {},  # Keep empty for compatibility
            "main_flow_topics": [],
            "main_flow_topics_with_types": [],
            "connection_topics": [],
            "nodes_in_order": [],  # Keep empty for compatibility
            "all_topics": [],
            "all_topics_with_types": [],
            "topics_by_node": {},
        }

        # Convert active topics to a set for faster lookup
        active_topic_names = {t[0] for t in active_topics}
        active_topics_dict = {t[0]: t[1] for t in active_topics}

        # Filter topics by node
        for node_name, topics in yaml_data.get("topics_by_node", {}).items():
            filtered_topics = []
            for topic_info in topics:
                topic_name = topic_info["topic"]
                if topic_name in active_topic_names:
                    filtered_topics.append(
                        (topic_info["priority"], topic_name, topic_info["msg_type"])
                    )

            if filtered_topics:
                filtered_result["topics_by_node"][node_name] = filtered_topics

        # Filter main flow topics
        for topic in yaml_data.get("main_flow_topics", []):
            if topic in active_topic_names:
                filtered_result["main_flow_topics"].append(topic)
                # Add with type for compatibility
                msg_type = active_topics_dict.get(topic, "Unknown")
                filtered_result["main_flow_topics_with_types"].append((topic, msg_type))

        # Build all_topics lists from filtered topics_by_node
        all_topics = set()
        all_topics_with_types = {}

        for node_name, topics in filtered_result["topics_by_node"].items():
            for _, topic_name, msg_type in topics:
                all_topics.add(topic_name)
                if topic_name not in all_topics_with_types:
                    all_topics_with_types[topic_name] = msg_type

        filtered_result["all_topics"] = sorted(all_topics)
        filtered_result["all_topics_with_types"] = [
            (t, msg_type) for t, msg_type in sorted(all_topics_with_types.items())
        ]

        return filtered_result

    def build_graph(self) -> Dict[str, GraphNode]:
        """Build a graph of nodes with their publishers and subscribers."""
        graph = {}

        # Get all nodes
        node_names_and_namespaces = self.node.get_node_names_and_namespaces()

        for name, namespace in node_names_and_namespaces:
            # Format full node name
            if namespace.endswith("/"):
                full_name = namespace + name
            else:
                full_name = namespace + "/" + name if namespace != "/" else "/" + name

            # Only process /planning/ nodes
            if not full_name.startswith("/planning/") and not full_name.startswith("/control/"):
                continue

            # Get publishers and subscribers for this node
            publishers = self._get_node_publishers(full_name)
            subscribers = self._get_node_subscribers(full_name)

            # Only add to graph if node has publishers or subscribers of target types
            if publishers or subscribers:
                graph_node = GraphNode(
                    name=full_name, publishers=publishers, subscribers=subscribers
                )
                graph[full_name] = graph_node

        return graph

    def _get_node_publishers(self, node_name) -> Dict[str, str]:
        """Get publishers for a node (filtered by target message types)."""
        publishers = {}
        try:
            topic_info_list = self.node.get_topic_names_and_types()

            for topic_name, topic_types in topic_info_list:
                # Check if this topic has any of our target message types
                has_target_type = any(msg_type in self.target_msg_types for msg_type in topic_types)
                if not has_target_type:
                    continue

                # Get publishers for this topic
                pub_info_list = self.node.get_publishers_info_by_topic(topic_name)
                for pub_info in pub_info_list:
                    full_node_name = f"{pub_info.node_namespace}{'/' if not pub_info.node_namespace.endswith('/') else ''}{pub_info.node_name}"
                    if full_node_name == node_name:
                        # Only add target message types
                        for t in topic_types:
                            if t in self.target_msg_types:
                                publishers[topic_name] = t
                        break
        except Exception:
            pass  # Silently ignore errors in library mode

        return publishers

    def _get_node_subscribers(self, node_name) -> Dict[str, str]:
        """Get subscribers for a node (filtered by target message types)."""
        subscribers = {}
        try:
            topic_info_list = self.node.get_topic_names_and_types()

            for topic_name, topic_types in topic_info_list:
                # Check if this topic has any of our target message types
                has_target_type = any(msg_type in self.target_msg_types for msg_type in topic_types)
                if not has_target_type:
                    continue

                # Get subscribers for this topic
                sub_info_list = self.node.get_subscriptions_info_by_topic(topic_name)
                for sub_info in sub_info_list:
                    full_node_name = f"{sub_info.node_namespace}{'/' if not sub_info.node_namespace.endswith('/') else ''}{sub_info.node_name}"
                    if full_node_name == node_name:
                        # Only add target message types
                        for t in topic_types:
                            if t in self.target_msg_types:
                                subscribers[topic_name] = t
                        break
        except Exception:
            pass  # Silently ignore errors in library mode

        return subscribers

    def _trace_connections_and_collect(
        self, graph: Dict[str, GraphNode], node_name: str, visited: Set[str], depth: int
    ):
        """Trace connections from a starting node and collect main flow topics."""
        if node_name in visited or depth > 10:  # Increased limit to capture full flow
            return

        visited.add(node_name)
        # indent = "  " * depth  # Not used currently

        if node_name not in graph:
            return

        node = graph[node_name]
        # print(f"{indent}{node_name}")

        # Find connected nodes through published topics
        # Track which topics are actually used in connections
        connected_topics = []
        for pub_topic in node.publishers.keys():
            # Find nodes that subscribe to this topic
            for other_name, other_node in graph.items():
                if other_name != node_name and pub_topic in other_node.subscribers:
                    connected_topics.append(pub_topic)
                    if pub_topic not in self.main_flow_topics:
                        self.main_flow_topics.append(pub_topic)
                    # print(f"{indent}  └─> publishes '{pub_topic}' to:")
                    self._trace_connections_and_collect(graph, other_name, visited, depth + 1)

    def _collect_connection_topics(
        self, graph: Dict[str, GraphNode], node_name: str, visited: Set[str]
    ):
        """Collect topics in connection order starting from a node."""
        if node_name in visited or node_name not in graph:
            return

        visited.add(node_name)
        node = graph[node_name]

        # Add published topics from this node
        for pub_topic in node.publishers.keys():
            if pub_topic not in self.connection_topics:
                self.connection_topics.append(pub_topic)

            # Find nodes that subscribe to this topic
            for other_name, other_node in graph.items():
                if other_name != node_name and pub_topic in other_node.subscribers:
                    self._collect_connection_topics(graph, other_name, visited)

    def _collect_nodes_in_order(
        self, graph: Dict[str, GraphNode], node_name: str, visited: Set[str]
    ):
        """Collect nodes in connection order."""
        if node_name in visited or node_name not in graph:
            return

        visited.add(node_name)

        # Add this node if not already added
        if node_name not in self.nodes_in_order:
            self.nodes_in_order.append(node_name)

        node = graph[node_name]
        # Find connected nodes through published topics
        for pub_topic in node.publishers.keys():
            for other_name, other_node in graph.items():
                if other_name != node_name and pub_topic in other_node.subscribers:
                    self._collect_nodes_in_order(graph, other_name, visited)


class TrajectoryNodeGraphRunner(Node):
    """ROS 2 Node for running TrajectoryNodeGraph analysis."""

    def __init__(self):
        super().__init__("trajectory_node_graph")
        self.get_logger().info("Trajectory Node Graph started")
        self.timer = self.create_timer(1.5, self.timer_callback)
        self.executed = False
        self.analyzer = TrajectoryNodeGraph(node=self)

    def timer_callback(self):
        """Timer callback to analyze and display results after waiting."""
        if not self.executed:
            try:
                # Analyze the graph
                results = self.analyzer.analyze()

                # Display results
                self.display_results(results)

                self.executed = True
                self.timer.cancel()
                raise SystemExit(0)
            except Exception as e:
                self.get_logger().error(f"Analysis failed: {e}")
                raise SystemExit(1)

    def display_results(self, results):
        """Display analysis results to stdout."""
        # Display filtered nodes with publishers/subscribers
        self.display_filtered_nodes()

        print("\n" + "=" * 80)

        # Display graph
        if results["graph"]:
            self.display_graph(results["graph"])
        else:
            print("WARNING: Graph is empty!")

        # Display YAML outputs
        self.display_yaml_topics(results)

        # Display sorted topics by node
        self.display_sorted_topics_by_node(results)

    def display_filtered_nodes(self):
        """Display filtered nodes with their publishers and subscribers."""
        print("\n=== ROS 2 Nodes under /planning/ namespace with Publishers ===")

        node_names_and_namespaces = self.get_node_names_and_namespaces()
        planning_nodes = []

        for name, namespace in node_names_and_namespaces:
            if namespace.endswith("/"):
                full_name = namespace + name
            else:
                full_name = namespace + "/" + name if namespace != "/" else "/" + name

            if full_name.startswith("/planning/") or full_name.startswith("/control/"):
                planning_nodes.append(full_name)
                print(f"\nNode: {full_name}")
                self.show_node_details(full_name)

        print("\n=== Summary ===")
        print(f"Total /planning/ nodes found: {len(planning_nodes)}")
        print(f"Total nodes in system: {len(node_names_and_namespaces)}")

    def show_node_details(self, node_name):
        """Show publishers and subscribers for a node."""
        # Show publishers
        self.show_node_publishers(node_name)
        # Show subscribers
        self.show_node_subscribers(node_name)

    def show_node_publishers(self, node_name):
        """Show filtered publishers for a node."""
        try:
            topic_info_list = self.node.get_topic_names_and_types()
            publishers = []

            for topic_name, topic_types in topic_info_list:
                has_target_type = any(
                    msg_type in self.analyzer.target_msg_types for msg_type in topic_types
                )
                if not has_target_type:
                    continue

                pub_info_list = self.get_publishers_info_by_topic(topic_name)
                for pub_info in pub_info_list:
                    full_node_name = f"{pub_info.node_namespace}{'/' if not pub_info.node_namespace.endswith('/') else ''}{pub_info.node_name}"
                    if full_node_name == node_name:
                        filtered_types = [
                            t for t in topic_types if t in self.analyzer.target_msg_types
                        ]
                        if filtered_types:
                            publishers.append((topic_name, filtered_types))
                        break

            if publishers:
                print("  Publishers (filtered):")
                for topic_name, topic_types in publishers:
                    for topic_type in topic_types:
                        print(f"    - Topic: {topic_name}, Type: {topic_type}")
            else:
                print("  No publishers (for target message types)")
        except Exception as e:
            print(f"  Error getting publishers: {e}")

    def show_node_subscribers(self, node_name):
        """Show filtered subscribers for a node."""
        try:
            topic_info_list = self.node.get_topic_names_and_types()
            subscribers = []

            for topic_name, topic_types in topic_info_list:
                has_target_type = any(
                    msg_type in self.analyzer.target_msg_types for msg_type in topic_types
                )
                if not has_target_type:
                    continue

                sub_info_list = self.get_subscriptions_info_by_topic(topic_name)
                for sub_info in sub_info_list:
                    full_node_name = f"{sub_info.node_namespace}{'/' if not sub_info.node_namespace.endswith('/') else ''}{sub_info.node_name}"
                    if full_node_name == node_name:
                        filtered_types = [
                            t for t in topic_types if t in self.analyzer.target_msg_types
                        ]
                        if filtered_types:
                            subscribers.append((topic_name, filtered_types))
                        break

            if subscribers:
                print("  Subscribers (filtered):")
                for topic_name, topic_types in subscribers:
                    for topic_type in topic_types:
                        print(f"    - Topic: {topic_name}, Type: {topic_type}")
            else:
                print("  No subscribers (for target message types)")
        except Exception as e:
            print(f"  Error getting subscribers: {e}")

    def display_graph(self, graph):
        """Display the graph with connections."""
        print("\n=== Graph Nodes (with target message types only) ===")
        print(f"Total nodes in graph: {len(graph)}\n")

        for node_name, node in graph.items():
            print(f"Node: {node_name}")
            if node.publishers:
                print("  Publishers:")
                for topic, msg_type in node.publishers.items():
                    print(f"    - {topic} [{msg_type.split('/')[-1]}]")
            if node.subscribers:
                print("  Subscribers:")
                for topic, msg_type in node.subscribers.items():
                    print(f"    - {topic} [{msg_type.split('/')[-1]}]")
            print()

        # Display connections
        start_node = (
            "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner"
        )
        if start_node in graph:
            print("\n=== Connections starting from behavior_path_planner ===")
            self.trace_connections(graph, start_node, visited=set(), depth=0)
        else:
            print(f"\nWarning: Start node {start_node} not found in graph")

    def trace_connections(self, graph, node_name, visited, depth):
        """Trace and display connections."""
        if node_name in visited or depth > 10:
            return

        if node_name not in graph:
            return

        visited.add(node_name)
        indent = "  " * depth
        node = graph[node_name]
        print(f"{indent}{node_name}")

        for pub_topic in node.publishers.keys():
            for other_name, other_node in graph.items():
                if other_name != node_name and pub_topic in other_node.subscribers:
                    print(f"{indent}  └─> publishes '{pub_topic}' to:")
                    self.trace_connections(graph, other_name, visited, depth + 1)

    def display_yaml_topics(self, results):
        """Display topics in various YAML formats."""
        # All topics
        print("\n=== Topics in YAML format ===")
        all_topics = results["all_topics"]
        yaml_data = {"topics": all_topics, "topic_count": len(all_topics)}
        print(yaml.dump(yaml_data, default_flow_style=False, sort_keys=False))

        # Topics with types
        print("\n=== Topics with types in YAML format ===")
        topics_with_types = {}
        for node in results["graph"].values():
            topics_with_types.update(node.publishers)
            topics_with_types.update(node.subscribers)

        sorted_topics_with_types = dict(sorted(topics_with_types.items()))
        yaml_data = {
            "topics_with_types": sorted_topics_with_types,
            "topic_count": len(sorted_topics_with_types),
        }
        print(yaml.dump(yaml_data, default_flow_style=False, sort_keys=False))

        # Connection order topics
        print("\n=== Topics in connection order (from behavior_path_planner) ===")
        yaml_data = {
            "topics_in_connection_order": results["connection_topics"],
            "topic_count": len(results["connection_topics"]),
        }
        print(yaml.dump(yaml_data, default_flow_style=False, sort_keys=False))

        # Main flow topics
        print("\n=== Main Flow Topics in YAML format (Connected Topics Only) ===")
        yaml_data = {
            "topics": results["main_flow_topics"],
            "topic_count": len(results["main_flow_topics"]),
        }
        print(yaml.dump(yaml_data, default_flow_style=False, sort_keys=False))

    def display_sorted_topics_by_node(self, results):
        """Display topics sorted by node with main flow highlighted."""
        print(
            "\n=== All Topics Sorted by Graph Order with Node Names (Main Flow Topics Marked with *) ==="
        )
        print("topics_by_node:")

        topics_by_node = results["topics_by_node"]
        for node_name, topics in topics_by_node.items():
            print(f"  [{node_name}]:")
            for topic_type, topic, _ in topics:
                if topic_type == "main":
                    print(f"    - '* {topic}'")
                else:
                    print(f"    - '  {topic}'")

        # Count statistics
        all_topics = []
        main_count = 0
        for topics in topics_by_node.values():
            for topic_type, topic, _ in topics:
                all_topics.append(topic)
                if topic_type == "main":
                    main_count += 1

        # print(f"\ntotal_topics: {len(all_topics)}")
        # print(f"main_flow_topics_count: {main_count}")
        # print(f"total_nodes: {len(topics_by_node)}")


def main(args=None):
    """Execute main function for standalone execution."""
    rclpy.init(args=args)
    node = None

    try:
        node = TrajectoryNodeGraphRunner()
        rclpy.spin(node)
    except SystemExit:
        # Save the graph configuration to YAML file after successful analysis
        if node and hasattr(node, "analyzer") and node.analyzer.graph:
            yaml_path = node.analyzer.save_to_yaml()
            print(f"\n{'='*60}")
            print(f"Graph configuration saved to: {yaml_path}")
            print(f"{'='*60}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node and rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
