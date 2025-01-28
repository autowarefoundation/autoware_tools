// Copyright 2025 Autoware Foundation
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

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <regex>
#include <set>
#include <string>

namespace lanelet::autoware::validation
{

class VersionControlTest : public ::testing::Test
{
protected:
  void SetUp() override { version_format_regex_ = std::regex(R"(\d+\.\d+\.\d+)"); }
  bool has_duplicate_attributes(const pugi::xml_node & node)
  {
    std::set<std::string> attribute_names;
    for (pugi::xml_attribute attr = node.first_attribute(); attr; attr = attr.next_attribute()) {
      if (!attribute_names.insert(attr.name()).second) {
        return true;
      }
    }
    return false;
  }

  std::regex version_format_regex_;
};

TEST_F(VersionControlTest, GetValidVersion)  // NOLINT for gtest
{
  std::string version;

  ASSERT_NO_THROW(version = get_validator_version(););

  ASSERT_TRUE(std::regex_match(version, version_format_regex_))
    << "The version \"" << version << "\" does not match the format X.Y.Z";
}

TEST_F(VersionControlTest, AddValidationInfo)  // NOLINT for gtest
{
  const std::string package_share_directory =
    ament_index_cpp::get_package_share_directory("autoware_lanelet2_map_validator");

  const std::string empty_osm_file_name = package_share_directory + "/data/temp_empty.osm";

  std::ofstream empty_osm_file(empty_osm_file_name);
  empty_osm_file << R"(
    <osm>
    </osm>
  )";
  empty_osm_file.close();

  ASSERT_NO_THROW({
    insert_validator_info_to_map(empty_osm_file_name, "autoware_requirement_set.json", "1.2.3");
  });

  pugi::xml_document doc;
  ASSERT_TRUE(doc.load_file(empty_osm_file_name.c_str()));

  pugi::xml_node validation_node = doc.child("osm").child("validation");
  ASSERT_TRUE(validation_node);

  ASSERT_FALSE(has_duplicate_attributes(validation_node));

  EXPECT_STREQ(validation_node.attribute("name").value(), "autoware_lanelet2_map_validator");
  EXPECT_TRUE(std::regex_match(
    validation_node.attribute("validator_version").value(), version_format_regex_));
  EXPECT_STREQ(validation_node.attribute("requirements").value(), "autoware_requirement_set.json");
  EXPECT_STREQ(validation_node.attribute("requirements_version").value(), "1.2.3");

  EXPECT_TRUE(std::filesystem::remove(empty_osm_file_name))
    << "Failed to remove temporary file " << empty_osm_file_name;
}

TEST_F(VersionControlTest, ModifyValidationInfo)  // NOLINT for gtest
{
  const std::string package_share_directory =
    ament_index_cpp::get_package_share_directory("autoware_lanelet2_map_validator");

  const std::string info_osm_file_name = package_share_directory + "/data/temp_info.osm";

  std::ofstream info_osm_file(info_osm_file_name);
  info_osm_file << R"(
    <osm>
      <validation name="autoware_lanelet2_map_validator" validator_version="0.0.0" requirements="autoware_requirement_set.json" requirements_version="1.1.1" />
    </osm>
  )";
  info_osm_file.close();

  ASSERT_NO_THROW({
    insert_validator_info_to_map(info_osm_file_name, "original_requirement_set.json", "1.2.3");
  });

  pugi::xml_document doc;
  ASSERT_TRUE(doc.load_file(info_osm_file_name.c_str()));

  pugi::xml_node validation_node = doc.child("osm").child("validation");
  ASSERT_TRUE(validation_node);

  ASSERT_FALSE(has_duplicate_attributes(validation_node));

  EXPECT_STREQ(validation_node.attribute("name").value(), "autoware_lanelet2_map_validator");
  EXPECT_TRUE(std::regex_match(
    validation_node.attribute("validator_version").value(), version_format_regex_));
  EXPECT_STRNE(validation_node.attribute("validator_version").value(), "0.0.0");
  EXPECT_STREQ(validation_node.attribute("requirements").value(), "original_requirement_set.json");
  EXPECT_STREQ(validation_node.attribute("requirements_version").value(), "1.2.3");

  EXPECT_TRUE(std::filesystem::remove(info_osm_file_name))
    << "Failed to remove temporary file " << info_osm_file_name;
}

}  // namespace lanelet::autoware::validation
