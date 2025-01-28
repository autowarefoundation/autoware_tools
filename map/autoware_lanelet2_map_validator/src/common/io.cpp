// Copyright 2024 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "lanelet2_map_validator/io.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pugixml.hpp>

#include <filesystem>
#include <iostream>
#include <string>

namespace lanelet::autoware::validation
{
std::string get_validator_version()
{
  std::string package_share_directory =
    ament_index_cpp::get_package_share_directory("autoware_lanelet2_map_validator");
  std::filesystem::path package_xml =
    std::filesystem::path(package_share_directory) / "package.xml";

  if (!std::filesystem::exists(package_xml)) {
    throw std::runtime_error("package.xml not found in " + package_share_directory);
  }

  pugi::xml_document doc;
  if (!doc.load_file(package_xml.c_str())) {
    throw std::runtime_error("Failed to parse package.xml!");
  }

  pugi::xml_node version_node = doc.child("package").child("version");
  if (!version_node) {
    throw std::runtime_error("No <version> tag found in package.xml!");
  }

  return version_node.text().as_string();
}

void insert_validator_info_to_map(
  std::string osm_file, std::string requirements, std::string requirements_version)
{
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(osm_file.c_str());

  if (!result) {
    throw std::invalid_argument("Failed to load osm file!");
  }

  pugi::xml_node osm_node = doc.child("osm");
  if (!osm_node) {
    throw std::invalid_argument("No <osm> tag found in the osm file!");
  }

  pugi::xml_node validation_node = osm_node.child("validation");

  if (!validation_node) {
    validation_node = osm_node.prepend_child("validation");
  }

  // Helper function to set or append an attribute
  auto set_or_append_attribute = [](pugi::xml_node & node, const char * name, const char * value) {
    pugi::xml_attribute attr = node.attribute(name);
    if (attr) {
      attr.set_value(value);
    } else {
      node.append_attribute(name) = value;
    }
  };

  // Set or append the required attributes
  set_or_append_attribute(validation_node, "name", "autoware_lanelet2_map_validator");
  set_or_append_attribute(validation_node, "validator_version", get_validator_version().c_str());
  set_or_append_attribute(validation_node, "requirements", requirements.c_str());
  set_or_append_attribute(validation_node, "requirements_version", requirements_version.c_str());

  if (!doc.save_file(osm_file.c_str())) {
    throw std::runtime_error("Failed to save the validator info to osm file");
  }

  std::cout << "Modified validator information in the osm file." << std::endl;
}
}  // namespace lanelet::autoware::validation
