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
    ):
        self.name = name
        self.processing_time = processing_time
        self.comment = comment
        self.id = id
        self.parent_id = parent_id
        self.children = []

    @classmethod
    def from_msg(cls, msg: ProcessingTimeTreeMsg) -> "ProcessingTimeTree":
        # Create a dictionary to map node IDs to ProcessingTimeTree objects
        node_dict: Dict[int, ProcessingTimeTree] = {
            node.id: ProcessingTimeTree(
                node.name, node.processing_time, node.comment, node.id, node.parent_id
            )
            for node in msg.nodes
        }

        # Build the tree structure
        root = node_dict[1]
        for node in list(node_dict.values()):
            parent = node_dict.get(node.parent_id)
            if parent:
                parent.children.append(node)

        return root

    def to_lines(self, show_comment: bool = True) -> str:
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
