import json
from typing import List

class LaunchTreeNode:
    """Each node in the launch tree is a LaunchTreeNode. It represents a launch file or a ros node."""

    def __init__(self, name: str, **kwargs):
        self.name = name
        self.children: List[LaunchTreeNode] = []
        self.parameters = kwargs

    def add_child(self, child: "LaunchTreeNode"):
        self.children.append(child)

    def jsonify(self):
        return {
            "name": self.name,
            "children": [child.jsonify() for child in self.children],
            "parameters": self.parameters,
        }


class LaunchTree:
    """Tree Structure to store the launch file structure."""

    def __init__(self):
        self.root = None
        self.edges_manager = []
        self.nodes_manager = {}

    def get_node(self, node_name):
        return self.nodes_manager[node_name]

    def add_root(self, root_name, **kwargs):
        if self.root is None:
            self.root = LaunchTreeNode(root_name)
            self.nodes_manager[root_name] = self.root
        else:
            print("Root already exists")

    def add_child(self, parent_name, child_name, **kwargs):
        if self.root is None:
            self.root = LaunchTreeNode(parent_name)
            self.nodes_manager[parent_name] = self.root

        if parent_name not in self.nodes_manager:
            # print(f"Parent node {parent_name} not found")
            return

        if child_name in self.nodes_manager:
            # print(f"Child node {child_name} already exists")
            return

        child = LaunchTreeNode(child_name, **kwargs)
        self.nodes_manager[child_name] = child
        self.nodes_manager[parent_name].add_child(child)
        self.edges_manager.append((parent_name, child_name))

    def add_argument(self, node_name, argument_name, argument_value):
        if node_name not in self.nodes_manager:
            print(f"Node {node_name} not found")
            return

        self.nodes_manager[node_name].arguments[argument_name] = argument_value

    def jsonify(self):
        json_object = self.root.jsonify()
        return json_object

    def __repr__(self) -> str:
        json_object = self.jsonify()
        return json.dumps(json_object, indent=4)


def find_unset_parameters(tree: LaunchTree):
    """Find all unset parameters in the launch tree."""
    unset_parameters = []
    for node_name, node in tree.nodes_manager.items():
        for parameter_name, parameter_value in node.parameters.items():
            if parameter_value == "ERROR_PARAMETER_NOT_SET":
                unset_parameters.append((node_name, parameter_name))
    return unset_parameters
