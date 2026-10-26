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

#ifndef UTILS__JSON_UTILS_HPP_
#define UTILS__JSON_UTILS_HPP_

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::planning_data_analyzer::utils
{

/**
 * @brief Save JSON object to file
 * @param json JSON object to save
 * @param file_path Output file path
 * @param logger ROS logger for error messages
 * @return true if successful, false otherwise
 */
bool save_json_to_file(
  const nlohmann::json & json, const std::string & file_path, const rclcpp::Logger & logger);

/**
 * @brief Load JSON object from file
 * @param file_path Input file path
 * @param logger ROS logger for error messages
 * @return JSON object (empty if failed)
 */
nlohmann::json load_json_from_file(const std::string & file_path, const rclcpp::Logger & logger);

}  // namespace autoware::planning_data_analyzer::utils

#endif  // UTILS__JSON_UTILS_HPP_
