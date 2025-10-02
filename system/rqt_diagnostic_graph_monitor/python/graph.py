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

from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.time import Time


class DummyStatus:
    def __init__(self):
        self.level = DiagnosticStatus.STALE


class BaseUnit:
    def __init__(self, struct, status=DummyStatus()):
        self._parents = []
        self._children = []
        self._struct = struct
        self._status = status

    def update(self, status):
        self._status = status

    @property
    def parents(self):
        return self._parents

    @property
    def children(self):
        return self._children


class NodeUnit(BaseUnit):
    def __init__(self, struct):
        super().__init__(struct)
        self._diag = None

    @property
    def level(self):
        return self._status.level

    @property
    def path(self):
        return self._struct.path

    @property
    def kind(self):
        return self._struct.type

    @property
    def diag(self):
        return self._diag


class DiagUnit(BaseUnit):
    def __init__(self, struct):
        super().__init__(struct)
        self._node = None

    @property
    def level(self):
        return self._status.level

    @property
    def name(self):
        return self._struct.name

    @property
    def parent(self):
        return self._struct.parent


class UnitLink:
    def __init__(self, parent: BaseUnit, child: BaseUnit):
        self._parent = parent
        self._child = child
        parent._children.append(self)
        child._parents.append(self)

    def update(self, status):
        self.status = status

    @property
    def parent(self):
        return self._parent

    @property
    def child(self):
        return self._child


class Graph:
    def __init__(self, msg):
        self._struct_stamp = Time.from_msg(msg.stamp)
        self._status_stamp = None
        self._id = msg.id
        self._nodes = [NodeUnit(struct) for struct in msg.nodes]
        self._diags = [DiagUnit(struct) for struct in msg.diags]
        self._links = []
        for struct in msg.links:
            self._links.append(UnitLink(self._nodes[struct.parent], self._nodes[struct.child]))
        for diag in self._diags:
            node = self._nodes[diag.parent]
            node._diag = diag
            diag._node = node

    def update(self, msg):
        if msg.id == self._id:
            self._status_stamp = Time.from_msg(msg.stamp)
            for node, status in zip(self._nodes, msg.nodes):
                node.update(status)
            for diag, status in zip(self._diags, msg.diags):
                diag.update(status)
            for link, status in zip(self._links, msg.links):
                link.update(status)

    @property
    def nodes(self):
        return self._nodes

    @property
    def links(self):
        return self._links
