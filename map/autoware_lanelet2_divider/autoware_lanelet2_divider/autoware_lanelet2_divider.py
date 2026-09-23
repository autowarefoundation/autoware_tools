import os
import shutil

import autoware_lanelet2_divider.data_preparation.data_preparation as data_preparation
from autoware_lanelet2_divider.debug import Debug
from autoware_lanelet2_divider.debug import DebugMessageType
import autoware_lanelet2_divider.osmium_tool.osmium_tool as osmium_tool
import autoware_lanelet2_divider.xml_tool.xml_tool as xml_tool
import rclpy
from rclpy.node import Node


class AutowareLanelet2Divider(Node):
    def __init__(self):
        super().__init__("autoware_lanelet2_divider")
        Debug.log("Autoware Lanelet2 Divider Node has been started.", DebugMessageType.INFO)

        self.declare_params()

        self.input_lanelet2_map_path = (
            self.get_parameter("input_lanelet2_map_path").get_parameter_value().string_value
        )
        self.output_folder_path = (
            self.get_parameter("output_folder_path").get_parameter_value().string_value
        )
        self.mgrs_grid = self.get_parameter("mgrs_grid").get_parameter_value().string_value
        self.grid_edge_size = (
            self.get_parameter("grid_edge_size").get_parameter_value().integer_value
        )

        Debug.log(
            "Input Lanelet2 Map Path: %s" % self.input_lanelet2_map_path, DebugMessageType.INFO
        )
        Debug.log("Output Folder Path: %s" % self.output_folder_path, DebugMessageType.INFO)
        Debug.log("MGRS Grid: %s" % self.mgrs_grid, DebugMessageType.INFO)
        Debug.log("Grid Edge Size: %d" % self.grid_edge_size, DebugMessageType.INFO)

        # Create copy of osm file
        shutil.copy(
            self.input_lanelet2_map_path, self.input_lanelet2_map_path.replace(".osm", "_temp.osm")
        )
        self.input_lanelet2_map_path = self.input_lanelet2_map_path.replace(".osm", "_temp.osm")

        # Complete if missing "version" element in lanelet2_map.osm
        xml_tool.complete_missing_version_tag(self.input_lanelet2_map_path)

        # Create config file to extract osm file
        config_files = data_preparation.data_preparation(
            self.mgrs_grid,
            self.grid_edge_size,
            self.input_lanelet2_map_path,
            self.output_folder_path,
        )
        # Extract osm file
        for config_file_path in config_files:
            is_extracted = osmium_tool.extract_osm_file(
                self.input_lanelet2_map_path,
                config_file_path,
                os.path.join(self.output_folder_path, "lanelet2_map.osm"),
            )
            if not is_extracted:
                Debug.log("Failed to extract osm file.\n", DebugMessageType.ERROR)
                rclpy.shutdown()

        # Complete missing elements in osm file
        xml_tool.complete_missing_elements(
            self.input_lanelet2_map_path, os.path.join(self.output_folder_path, "lanelet2_map.osm")
        )

        # Remove temp osm file
        os.remove(self.input_lanelet2_map_path)

        Debug.log("Autoware Lanelet2 Divider Node has been finished.", DebugMessageType.SUCCESS)
        exit(0)

    def declare_params(self) -> None:
        self.declare_parameter("input_lanelet2_map_path", "")
        self.declare_parameter("output_folder_path", "")
        self.declare_parameter("mgrs_grid", "")
        self.declare_parameter("grid_edge_size", 100)


def main(args=None):
    rclpy.init(args=args)
    autoware_lanelet2_divider = AutowareLanelet2Divider()
    rclpy.spin(autoware_lanelet2_divider)
    autoware_lanelet2_divider.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
