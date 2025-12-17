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

#include "bag_utils.hpp"

#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::utils
{

bool open_bag(
  rosbag2_cpp::Reader & reader, const std::string & bag_path, const rclcpp::Logger & logger)
{
  try {
    reader.open(bag_path);
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to open bag '%s': %s", bag_path.c_str(), e.what());
    return false;
  }
}

void set_topic_filter(rosbag2_cpp::Reader & reader, const std::vector<std::string> & topics)
{
  if (topics.empty()) {
    return;
  }

  rosbag2_storage::StorageFilter filter;
  filter.topics = topics;
  reader.set_filter(filter);
}

}  // namespace autoware::planning_data_analyzer::utils
