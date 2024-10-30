import os
import xml.etree.ElementTree as ET

from autoware_lanelet2_divider.debug import Debug
from autoware_lanelet2_divider.debug import DebugMessageType
from tqdm import tqdm


def select_tags(root, node_list, way_list, relation_list) -> None:
    """
    Populates lists for 'node', 'way', and 'relation' XML elements found in the given root element.

    Parameters:
        root (ElementTree): The root XML element to parse.
        node_list (list): List to store 'node' elements.
        way_list (list): List to store 'way' elements.
        relation_list (list): List to store 'relation' elements.

    Returns:
        None
    """
    node_list.clear()
    way_list.clear()
    relation_list.clear()
    for child in root:
        if child.tag == "node":
            node_list.append(child)
        elif child.tag == "way":
            way_list.append(child)
        elif child.tag == "relation":
            relation_list.append(child)


def complete_missing_elements(
    input_osm_file_path: str, input_extracted_osm_folder: str
) -> bool:
    """
    Completes missing elements (nodes, ways, relations) in the divided OSM files by checking against the whole map.

    Parameters:
        input_osm_file_path (str): Path to the whole map OSM file.
        input_extracted_osm_folder (str): Directory containing divided OSM files.

    Returns:
        bool: True if the process completes successfully.
    """
    Debug.log(
        f"Completing missing elements in osm file: {input_osm_file_path}",
        DebugMessageType.INFO,
    )

    whole_map_xml = ET.parse(input_osm_file_path)
    whole_map_root = whole_map_xml.getroot()

    node_list = []
    way_list = []
    relation_list = []

    select_tags(whole_map_root, node_list, way_list, relation_list)

    osm_files = [
        f for f in os.listdir(input_extracted_osm_folder) if f.endswith(".osm")
    ]
    for osm_file in tqdm(
        osm_files,
        desc=Debug.get_log(
            "Completing missing elements in osm file", DebugMessageType.INFO
        ),
    ):
        divided_map_xml = ET.parse(input_extracted_osm_folder + "/" + osm_file)
        divided_map_root = divided_map_xml.getroot()

        divided_node_list = []
        divided_way_list = []
        divided_relation_list = []

        select_tags(
            divided_map_root, divided_node_list, divided_way_list, divided_relation_list
        )

        for relation in divided_relation_list:
            relation_refs = [member for member in relation.iter("member")]
            for r in relation_refs:
                if r.attrib["type"] == "way":
                    if r.attrib["ref"] not in [
                        way.attrib["id"] for way in divided_way_list
                    ]:
                        for way in way_list:
                            if way.attrib["id"] == r.attrib["ref"]:
                                divided_map_root.append(way)
                        select_tags(
                            divided_map_root,
                            divided_node_list,
                            divided_way_list,
                            divided_relation_list,
                        )
                elif r.attrib["type"] == "relation":
                    if r.attrib["ref"] not in [
                        rela.attrib["id"] for rela in divided_relation_list
                    ]:
                        for rel in relation_list:
                            if rel.attrib["id"] == r.attrib["ref"]:
                                divided_map_root.append(rel)
                        select_tags(
                            divided_map_root,
                            divided_node_list,
                            divided_way_list,
                            divided_relation_list,
                        )

        # Iterate on divided map's ways and find missing nodes
        for way in divided_way_list:
            nd = [nd.attrib["ref"] for nd in way.iter("nd")]
            for n in nd:
                if n not in [node.attrib["id"] for node in divided_node_list]:
                    # find the node in the whole map and add it to the divided map
                    for node in node_list:
                        if node.attrib["id"] == n:
                            divided_map_root.append(node)
                    select_tags(
                        divided_map_root,
                        divided_node_list,
                        divided_way_list,
                        divided_relation_list,
                    )

        divided_map_xml.write(input_extracted_osm_folder + "/" + osm_file)

    return True


def complete_missing_version_tag(input_osm_file_path: str):
    """
    Adds missing version attributes to the root and its child elements in an OSM file, if needed.

    Parameters:
        input_osm_file_path (str): Path to the OSM file to update with version tags.

    Returns:
        None
    """
    Debug.log(
        f"Completing missing version tags in osm file: {input_osm_file_path}",
        DebugMessageType.INFO,
    )

    whole_map_xml = ET.parse(input_osm_file_path)
    whole_map_root = whole_map_xml.getroot()

    add_version = any(
        root_element != "version" for root_element in whole_map_root.attrib
    )

    if add_version:
        whole_map_root.set("version", "0.6")

        for element in tqdm(
            whole_map_root,
            desc=Debug.get_log(
                "Completing missing version tags in osm file", DebugMessageType.INFO
            ),
        ):
            add_version = False
            for root_element in element.attrib:
                if root_element != "version":
                    add_version = True
                else:
                    add_version = False
                    break
            if add_version:
                element.set("version", "1")

    whole_map_xml.write(input_osm_file_path, encoding="utf-8", xml_declaration=True)
