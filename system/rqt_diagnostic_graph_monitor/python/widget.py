# Copyright 2023 The Autoware Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from python_qt_binding import QtCore
from python_qt_binding import QtWidgets

from .graph import Graph
from .items import MonitorItem
from .utils import foreach


class MonitorWidget(QtWidgets.QSplitter):
    def __init__(self):
        super().__init__(QtCore.Qt.Orientation.Vertical)
        self.graph = None
        self.items = []

        self.root_widget = QtWidgets.QTreeWidget()
        self.tree_widget = QtWidgets.QTreeWidget()
        self.root_widget.setHeaderLabels(["Top-level"])
        self.tree_widget.setHeaderLabels(["Subtrees"])

        self.info_widget = QtWidgets.QTextEdit()

        self.splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        self.splitter.addWidget(self.root_widget)
        self.splitter.addWidget(self.tree_widget)
        self.addWidget(self.splitter)
        self.addWidget(self.info_widget)

        self.root_widget.itemClicked.connect(self.on_item_selected)
        self.tree_widget.itemClicked.connect(self.on_item_selected)

        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self.on_timer)
        self._timer.start(500)

    def on_item_selected(self, item, column):
        item = item.data(0, QtCore.Qt.UserRole)
        node = item.node
        print(node.path, node.kind, node.diag)

    def shutdown(self):
        self.clear_graph()

    def on_timer(self):
        foreach(self.items, lambda item: item.update())

    def on_graph(self, graph: Graph):
        self.clear_graph()
        self.build_graph(graph)

    def clear_graph(self):
        self.graph = None
        self.items = []
        self.root_widget.clear()
        self.tree_widget.clear()

    def build_graph(self, graph: Graph):
        root_nodes = filter(self.is_root_node, graph.nodes)
        tree_nodes = filter(self.is_tree_node, graph.nodes)
        root_items = [MonitorItem(None, node) for node in root_nodes]
        tree_items = [MonitorItem(None, node) for node in tree_nodes]
        link_items = [MonitorItem(link, link.child) for link in graph.links]
        self.graph = graph
        self.items = root_items + tree_items + link_items

        # Note: overwrite link items with root/tree items if there is more than one.
        parents = {}
        for items in [link_items, tree_items, root_items]:
            parents.update({item.node: item.item for item in items})

        # Connect tree widget items.
        root_widget_item = self.root_widget.invisibleRootItem()
        tree_widget_item = self.tree_widget.invisibleRootItem()
        for item in root_items:
            root_widget_item.addChild(item.item)
        for item in tree_items:
            tree_widget_item.addChild(item.item)
        for item in link_items:
            parents[item.link.parent].addChild(item.item)

    @staticmethod
    def is_root_node(node):
        return len(node.parents) == 0

    @staticmethod
    def is_tree_node(node):
        return len(node.parents) >= 2 and len(node.children) != 0
