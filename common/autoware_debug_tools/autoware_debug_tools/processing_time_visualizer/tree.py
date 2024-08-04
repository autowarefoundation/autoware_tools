from typing import Dict

from tier4_debug_msgs.msg import ProcessingTimeTree as ProcessingTimeTreeMsg


class ProcessingTimeTree:
    def __init__(
        self,
        name: str = "",
        processing_time: float = 0.0,
        comment: str = "",
        id: int = 1,  # noqa
        parent_id: int = 0,
        run_count: int = 1
    ):
        self.name = name
        self.processing_time = processing_time
        self.comment = comment
        self.id = id
        self.parent_id = parent_id
        self.run_count = run_count
        self.children = []
        # Dict of {node name: node id}
        self.children_node_name_to_id: Dict[str, int] = {}

    @classmethod
    def from_msg(cls, msg: ProcessingTimeTreeMsg, summarize: bool = False) -> "ProcessingTimeTree":
        if summarize:
            node_dict: Dict[int, ProcessingTimeTree] = {}
            # Mapping for children whose parent gets merged
            parent_alias_dict: Dict[int, int] = {}

            for node in msg.nodes:
                parent_id = node.parent_id
                aliased_parent_id = parent_alias_dict.get(node.parent_id)

                if aliased_parent_id or parent_id in node_dict:
                    if aliased_parent_id:
                        parent_id = aliased_parent_id

                    # If node name already exist, use that node for aggregation
                    agg_node_id = node_dict[parent_id].children_node_name_to_id.get(node.name)
                    if agg_node_id:
                        agg_node = node_dict[agg_node_id]

                        # Create alias from current node to agg_node for its child nodes
                        if node.id not in parent_alias_dict:
                            parent_alias_dict[node.id] = agg_node_id

                        agg_node.processing_time += node.processing_time
                        agg_node.run_count += 1
                    else:
                        # If it is not in parent's children_name_to_id, it is not in node_dict
                        node_dict[node.id] = ProcessingTimeTree(
                            node.name, node.processing_time, node.comment, node.id, parent_id
                        )

                        node_dict[parent_id].children_node_name_to_id[node.name] = node.id
                else:
                    node_dict[node.id] = ProcessingTimeTree(
                        node.name, node.processing_time, node.comment, node.id, node.parent_id
                    )

            # Build the tree structure
            for node in list(node_dict.values()):
                parent = node_dict.get(node.parent_id)
                if parent is None:
                    aliased_parent_id = parent_alias_dict.get(node.parent_id)
                    parent = node_dict.get(aliased_parent_id)

                if parent:
                    # Checking the case child came first
                    agg_node_id = parent.children_node_name_to_id.get(node.name)
                    if agg_node_id:
                        # Avoid aggregated node itself
                        if agg_node_id != node.id:
                            agg_node = node_dict[agg_node_id]
                            agg_node.processing_time += node.processing_time
                            agg_node.run_count += 1
                    else:
                        parent.children_node_name_to_id[node.name] = node.id

                    parent.children.append(node)
        else:
            # Create a dictionary to map node IDs to ProcessingTimeTree objects
            node_dict: Dict[int, ProcessingTimeTree] = {
                node.id: ProcessingTimeTree(
                    node.name, node.processing_time, node.comment, node.id, node.parent_id
                )
                for node in msg.nodes
            }

            # Build the tree structure
            for node in list(node_dict.values()):
                parent = node_dict.get(node.parent_id)
                if parent:
                    parent.children.append(node)

        root = node_dict[1]

        return root

    def to_lines(self, show_comment: bool = True, summarize: bool = False) -> str:
        def construct_string(
            node: "ProcessingTimeTree",
            lines: list,
            prefix: str,
            is_last: bool,
            is_root: bool,
        ) -> None:
            # If not the root, append the prefix and the node information
            line = ""
            if not is_root:
                line += prefix + ("└── " if is_last else "├── ")
            line += f"{node.name}: {node.processing_time:.2f} [ms]"
            line += f", run count: {node.run_count}" if summarize else ""
            line += f": {node.comment}" if show_comment and node.comment else ""
            lines.append(line)
            # Recur for each child node
            for i, child in enumerate(node.children):
                construct_string(
                    child,
                    lines,
                    prefix + ("    " if is_last else "│   "),
                    i == len(node.children) - 1,
                    False,
                )

        lines = []
        # Start the recursive string construction with the root node
        construct_string(self, lines, "", True, True)
        return lines

    def __str__(self) -> str:
        return "".join([line + "\n" for line in self.to_lines()])

    def __eq__(self, other: "ProcessingTimeTree") -> bool:
        return self.name == other.name
