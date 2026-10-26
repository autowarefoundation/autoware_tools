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


class ItemInspector(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.item = None

        self.opts = QtWidgets.QCheckBox("Realtime Update")
        self.tree = QtWidgets.QTreeWidget()
        self.tree.setHeaderLabels(["Item", "Data"])

        self.item_node_group = QtWidgets.QTreeWidgetItem(["Node"])
        self.item_node_path = QtWidgets.QTreeWidgetItem(["Path"])
        self.item_node_type = QtWidgets.QTreeWidgetItem(["Type"])
        self.item_node_level = QtWidgets.QTreeWidgetItem(["Level"])
        self.item_node_group.addChild(self.item_node_path)
        self.item_node_group.addChild(self.item_node_type)
        self.item_node_group.addChild(self.item_node_level)

        self.item_diag_group = QtWidgets.QTreeWidgetItem(["Diag"])
        self.item_diag_name = QtWidgets.QTreeWidgetItem(["Name"])
        self.item_diag_hardware = QtWidgets.QTreeWidgetItem(["Hardware ID"])
        self.item_diag_level = QtWidgets.QTreeWidgetItem(["Level"])
        self.item_diag_message = QtWidgets.QTreeWidgetItem(["Message"])
        self.item_diag_group.addChild(self.item_diag_name)
        self.item_diag_group.addChild(self.item_diag_hardware)
        self.item_diag_group.addChild(self.item_diag_level)
        self.item_diag_group.addChild(self.item_diag_message)
        self.item_kv_group = QtWidgets.QTreeWidgetItem(["KeyValue"])

        self.tree.addTopLevelItem(self.item_node_group)
        self.tree.addTopLevelItem(self.item_diag_group)
        self.tree.addTopLevelItem(self.item_kv_group)
        self.item_node_group.setExpanded(True)
        self.item_diag_group.setExpanded(True)
        self.item_kv_group.setExpanded(True)

        self.setLayout(QtWidgets.QVBoxLayout())
        self.layout().addWidget(self.opts)
        self.layout().addWidget(self.tree)

    def select(self, item: MonitorItem):
        self.item = item
        self._update_unit_items()

    def update(self):
        if self.item and self.opts.isChecked():
            self._update_unit_items()

    def _update_unit_items(self):
        if self.item.node.diag is None:
            self._update_node_items(self.item.node)
            self._remove_diag_items()
            self._remove_kv_items()
        else:
            self._update_node_items(self.item.node)
            self._update_diag_items(self.item.node.diag)
            self._update_kv_items(self.item.node.diag.values)

    def _update_node_items(self, node):
        self.item_node_path.setText(1, node.path)
        self.item_node_type.setText(1, node.kind)
        self.item_node_level.setText(1, node.level.name)

    def _update_diag_items(self, diag):
        self.item_diag_name.setText(1, diag.name)
        self.item_diag_hardware.setText(1, diag.hardware)
        self.item_diag_level.setText(1, diag.level.name)
        self.item_diag_message.setText(1, diag.message)

    def _remove_diag_items(self):
        self.item_diag_name.setText(1, "")
        self.item_diag_hardware.setText(1, "")
        self.item_diag_level.setText(1, "")
        self.item_diag_message.setText(1, "")

    def _update_kv_items(self, values):
        root = self.item_kv_group
        for index in range(root.childCount(), len(values)):
            root.addChild(QtWidgets.QTreeWidgetItem())
        for index in range(len(values)):
            root.child(index).setText(0, values[index].key)
            root.child(index).setText(1, values[index].value)
        for index in range(len(values), root.childCount()):
            root.child(index).setText(0, "")
            root.child(index).setText(1, "")

    def _remove_kv_items(self):
        self._update_kv_items([])


class MonitorWidget(QtWidgets.QSplitter):
    def __init__(self):
        super().__init__(QtCore.Qt.Orientation.Vertical)
        self.graph = None
        self.items = []

        self.root_widget = QtWidgets.QTreeWidget()
        self.tree_widget = QtWidgets.QTreeWidget()
        self.root_widget.setHeaderLabels(["Top-level"])
        self.tree_widget.setHeaderLabels(["Subtrees"])

        self.info_widget = ItemInspector()

        self.splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        self.splitter.addWidget(self.root_widget)
        self.splitter.addWidget(self.tree_widget)
        self.addWidget(self.splitter)
        self.addWidget(self.info_widget)

        self.root_widget.itemActivated.connect(self.on_item_selected)
        self.tree_widget.itemActivated.connect(self.on_item_selected)

        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self.on_timer)
        self._timer.start(500)

    def shutdown(self):
        self.clear_graph()

    def on_timer(self):
        foreach(self.items, lambda item: item.update())

    def on_item_selected(self, item, column):
        item = item.data(0, QtCore.Qt.UserRole)
        self.info_widget.select(item)

    def on_graph_created(self, graph: Graph):
        self.clear_graph()
        self.build_graph(graph)

    def on_graph_updated(self, graph: Graph):
        self.info_widget.update()

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
