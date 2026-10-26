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

#include "path_utils.hpp"

#include <rclcpp/logging.hpp>

#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>

namespace autoware::planning_data_analyzer::utils
{

std::string expand_home_directory(const std::string & path)
{
  if (path.empty() || path[0] != '~') {
    return path;
  }

  const char * home = std::getenv("HOME");
  if (home == nullptr) {
    return path;
  }

  return std::string(home) + path.substr(1);
}

void ensure_directory_writable(const std::filesystem::path & dir, rclcpp::Logger logger)
{
  if (dir.empty()) {
    return;
  }

  // Create directory if it doesn't exist
  if (!std::filesystem::exists(dir)) {
    std::filesystem::create_directories(dir);
    RCLCPP_INFO(logger, "Created directory: %s", dir.string().c_str());
  }

  // Validate write permissions
  const auto test_file = dir / ".write_test";
  std::ofstream test_stream(test_file);
  if (!test_stream.is_open()) {
    throw std::runtime_error("No write permission to directory: " + dir.string());
  }
  test_stream.close();
  std::filesystem::remove(test_file);
}

}  // namespace autoware::planning_data_analyzer::utils
