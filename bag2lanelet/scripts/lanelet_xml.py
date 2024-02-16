import xml.etree.ElementTree as ET
from xml.dom.minidom import parseString
from mgrspy import mgrs


class LaneletMap:
    def __init__(self, mgrs="53SPU"):
        self.mgrs = mgrs
        self.element_num = 0
        self.node_list = []
        self.way_list = []
        self.relation_list = []

        self.root = ET.Element("osm", {"generator": "bag2lanelet"})
        self.meta = ET.SubElement(
            self.root, "MetaInfo", {"format_version": "1", "map_version": "2"}
        )

    def get_latlon(self, x, y, z):
        mgrs_code = self.mgrs + "%05d%05d" % (x, y)
        latlon = mgrs.toWgs(mgrs_code)
        return latlon

    def add_node(self, x, y, z):
        self.element_num += 1
        mgrs_code = self.mgrs + "%05d%05d" % (x, y)
        latlon = mgrs.toWgs(mgrs_code)
        mgrs_code = self.mgrs + ("%05d" % x)[:3] + ("%05d" % y)[:3]
        node = ET.SubElement(self.root, 'node', {'id': str(self.element_num), 'lat': str(latlon[0]), 'lon': str(latlon[1])})
        tag = [
            {"k": "type", "v": ""},
            {"k": "subtype", "v": ""},
            {"k": "mgrs_code", "v": mgrs_code},
            {"k": "local_x", "v": str(x)},
            {"k": "local_y", "v": str(y)},
            {"k": "ele", "v": str(z)},
        ]
        for t in tag:
            ET.SubElement(node, "tag", t)

        return self.element_num

    def add_way(self, node_list):
        self.element_num += 1

        way = ET.SubElement(self.root, "way", {"id": str(self.element_num)})
        tag = [
            {"k": "type", "v": "line_thin"},
            {"k": "subtype", "v": "solid"},
        ]

        for nd in node_list:
            ET.SubElement(way, "nd", {"ref": str(nd)})

        for t in tag:
            ET.SubElement(way, "tag", t)

        return self.element_num

    def add_relation(self, left_id, right_id, center_id=None):
        self.element_num += 1
        relation = ET.SubElement(self.root, "relation", {"id": str(self.element_num)})

        ET.SubElement(relation, "member", {"type": "way", "ref": str(left_id), "role": "left"})
        ET.SubElement(relation, "member", {"type": "way", "ref": str(right_id), "role": "right"})

        if center_id:
            ET.SubElement(
                relation, "member", {"type": "way", "ref": str(center_id), "role": "centerline"}
            )

        tag = [
            {"k": "type", "v": "lanelet"},
            {"k": "subtype", "v": "road"},
            {"k": "location", "v": "urban"},
            {"k": "participant:vehicle", "v": "yes"},
            {"k": "one_way", "v": "yes"},
            {"k": "speed_limit", "v": str(20)},
        ]
        for t in tag:
            ET.SubElement(relation, "tag", t)
        return self.element_num

    def save(self, filename):
        tree = ET.ElementTree(self.root)
        parsed = parseString(ET.tostring(self.root, "utf-8")).toprettyxml(indent="  ")
        with open(filename, "w") as f:
            f.write(parsed)
