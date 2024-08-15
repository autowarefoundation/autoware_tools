from typing import Dict, List, Optional
from abc import ABC, abstractmethod

from tier4_debug_msgs.msg import ProcessingTimeTree as ProcessingTimeTreeMsg
from tier4_debug_msgs.msg import ProcessingTimeNode as ProcessingTimeNodeMsg


class TreeBase(ABC):
    def __init__(self, name: str = "", comment: str = ""):
        self.name = name
        self.comment = comment
        self.children: List["TreeBase"] = []

    @abstractmethod
    def to_lines(self, show_comment: bool = True) -> List[str]:
        pass

    @property
    def has_children(self) -> bool:
        return bool(self.children)

    def __str__(self) -> str:
        return "\n".join(self.to_lines())


class ProcessingTimeTree(TreeBase):
    def __init__(
        self,
        name: str = "",
        processing_time: float = 0.0,
        comment: str = "",
        id: int = 1,  # noqa
        parent_id: int = 0,
    ):
        super().__init__(name, comment)
        self.processing_time = processing_time
        self.id = id
        self.parent_id = parent_id

    @classmethod
    def from_msg(cls, msg: ProcessingTimeTreeMsg) -> "ProcessingTimeTree":

        nodes: List[ProcessingTimeNodeMsg] = msg.nodes

        node_dict = {
            node.id: cls(
                node.name, node.processing_time, node.comment, node.id, node.parent_id
            )
            for node in nodes
        }

        for node in list(node_dict.values()):
            parent = node_dict.get(node.parent_id)
            if parent:
                parent.children.append(node)

        return node_dict[1]  # Return the root node

    def to_lines(self, show_comment: bool = True) -> List[str]:
        def construct_string(
            node: "ProcessingTimeTree",
            lines: List[str],
            prefix: str,
            is_last: bool,
            is_root: bool,
        ) -> None:
            line = ""
            if not is_root:
                line += prefix + ("└── " if is_last else "├── ")
            line += f"{node.name}: {node.processing_time:.2f} [ms]"
            line += f": {node.comment}" if show_comment and node.comment else ""
            lines.append(line)
            for i, child in enumerate(node.children):
                construct_string(
                    child,
                    lines,
                    prefix + ("    " if is_last else "│   "),
                    i == len(node.children) - 1,
                    False,
                )

        lines: List[str] = []
        construct_string(self, lines, "", True, True)
        return lines


class SummarizedProcessingTimeTree(TreeBase):
    def __init__(
        self,
        name: str = "",
        total_processing_time: float = 0.0,
        run_count: int = 0,
        comment: str = "",
    ):
        super().__init__(name, comment)
        self.total_processing_time = total_processing_time
        self.run_count = run_count

    @property
    def avg_processing_time(self) -> float:
        return (
            self.total_processing_time / self.run_count if self.run_count > 0 else 0.0
        )

    @classmethod
    def from_processing_time_tree(
        cls, tree: ProcessingTimeTree
    ) -> "SummarizedProcessingTimeTree":
        def convert_to_summarized_tree_without_summarizing(
            node: ProcessingTimeTree,
        ) -> SummarizedProcessingTimeTree:
            summarized_node = cls(
                name=node.name,
                total_processing_time=node.processing_time,
                run_count=1,
                comment=node.comment,
            )
            children: List[SummarizedProcessingTimeTree] = [
                convert_to_summarized_tree_without_summarizing(child)
                for child in node.children
            ]
            summarized_node.children = children
            return summarized_node

        tree = convert_to_summarized_tree_without_summarizing(tree)

        def summarize_tree(node: SummarizedProcessingTimeTree) -> None:
            summarized_children: Dict[str, SummarizedProcessingTimeTree] = {}
            for child in node.children:
                if child.name in summarized_children:
                    summarized_children[
                        child.name
                    ].total_processing_time += child.total_processing_time
                    summarized_children[child.name].run_count += child.run_count
                    summarized_children[child.name].children.extend(child.children)
                else:
                    summarized_children[child.name] = child
            node.children = list(summarized_children.values())
            for child in node.children:
                summarize_tree(child)

        summarize_tree(tree)
        return tree

    def to_lines(self, show_comment: bool = True) -> List[str]:
        def construct_string(
            node: "SummarizedProcessingTimeTree",
            lines: List[str],
            prefix: str,
            is_last: bool,
            is_root: bool,
        ) -> None:
            line = ""
            if not is_root:
                line += prefix + ("└── " if is_last else "├── ")
            percentage = 100.0 * node.total_processing_time / self.total_processing_time
            line += (
                f"{percentage:.2f}% {node.name}: total {node.total_processing_time:.2f} [ms], "
                f"avg. {node.avg_processing_time:.2f} [ms], "
                f"run count: {node.run_count}"
            )
            line += f": {node.comment}" if show_comment and node.comment else ""
            lines.append(line)
            for i, child in enumerate(node.children):
                construct_string(
                    child,
                    lines,
                    prefix + ("    " if is_last else "│   "),
                    i == len(node.children) - 1,
                    False,
                )
            # rest_processing_time = node.total_processing_time - sum(
            #     child.processing_time for child in node.children
            # )

        lines: List[str] = []
        construct_string(self, lines, "", True, True)
        return lines
