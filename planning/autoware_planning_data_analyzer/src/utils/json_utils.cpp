// Copyright 2025 TIER IV, Inc.
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

#include "json_utils.hpp"

#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::utils
{

bool save_json_to_file(
  const nlohmann::json & json, const std::string & file_path, const rclcpp::Logger & logger)
{
  std::ofstream output_file(file_path);
  if (!output_file.is_open()) {
    RCLCPP_ERROR(logger, "Failed to open output file: %s", file_path.c_str());
    return false;
  }

  output_file << std::setw(2) << json << std::endl;
  output_file.close();

  RCLCPP_INFO(logger, "Saved JSON to: %s", file_path.c_str());
  return true;
}

nlohmann::json load_json_from_file(const std::string & file_path, const rclcpp::Logger & logger)
{
  nlohmann::json json;

  std::ifstream input_file(file_path);
  if (!input_file.is_open()) {
    RCLCPP_ERROR(logger, "Failed to open JSON file: %s", file_path.c_str());
    return json;
  }

  try {
    input_file >> json;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to parse JSON file '%s': %s", file_path.c_str(), e.what());
    return nlohmann::json();
  }

  input_file.close();

  RCLCPP_INFO(logger, "Loaded JSON from: %s", file_path.c_str());
  return json;
}

}  // namespace autoware::planning_data_analyzer::utils
