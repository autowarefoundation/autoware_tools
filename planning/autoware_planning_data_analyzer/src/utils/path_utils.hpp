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

#ifndef UTILS__PATH_UTILS_HPP_
#define UTILS__PATH_UTILS_HPP_

#include <rclcpp/logger.hpp>

#include <filesystem>
#include <string>

namespace autoware::planning_data_analyzer::utils
{

/**
 * @brief Expand tilde (~) to home directory path
 * @param path Path that may contain ~ at the beginning
 * @return Expanded path with home directory, or original path if ~ is not at the beginning or HOME
 * is not set
 */
std::string expand_home_directory(const std::string & path);

/**
 * @brief Ensure directory exists and is writable
 * @param dir Directory path to validate
 * @param logger ROS logger for logging messages
 * @throws std::runtime_error if directory cannot be created or is not writable
 */
void ensure_directory_writable(const std::filesystem::path & dir, rclcpp::Logger logger);

}  // namespace autoware::planning_data_analyzer::utils

#endif  // UTILS__PATH_UTILS_HPP_
