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

#ifndef AUTOWARE__POINTCLOUD_DIVIDER__UTILITY_HPP_
#define AUTOWARE__POINTCLOUD_DIVIDER__UTILITY_HPP_

#include <pcl/point_types.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::pointcloud_divider::util
{
// Create a directory
inline bool make_dir(const std::string & path)
{
  std::string cmd = "mkdir -p " + path;
  std::string err_msg = "Error: Failed to execute command " + cmd;

  if (std::system(cmd.c_str())) {
    perror(err_msg.c_str());
    exit(EXIT_FAILURE);
  }

  return true;
}

// Remove a file/directory at @path
inline bool remove(const std::string & path)
{
  if (!std::filesystem::remove_all(path)) {
    std::cerr << "Error: File at " << path << " does not exist." << std::endl;
    return false;
  }

  return true;
}

// Parse the name of the PCD file, and return the number of points in the file
// Can only be used for file names formated as "***_<point_number>.pcd"
inline size_t point_num(const std::string & pcd_path)
{
  // Get the file name only
  auto last_underbar_pos = pcd_path.rfind("_");
  auto last_dot_pos = pcd_path.rfind(".");
  auto point_num_str = pcd_path.substr(last_underbar_pos + 1, last_dot_pos - last_underbar_pos - 1);

  return std::stoi(point_num_str);
}

/**
 * Split a line into multiple strings using the input
 * delimiter. Output strings are stored into a vector
 * of strings.
 *
 * Params:
 * 	@line[in]	: the input line to be splitted
 * 	@del[in]	: the delimiter
 * 	@vals[out]	: the output vector of strings
 *
 */
inline int split(const std::string & line, const std::string & del, std::vector<std::string> & vals)
{
  vals.clear();
  size_t start = 0, end = 0;

  do {
    end = line.find(del, start);

    if (end != std::string::npos) {
      vals.push_back(line.substr(start, end - start));

      start = end + del.size();
    }
  } while (end != std::string::npos);

  vals.push_back(line.substr(start));

  return vals.size();
}

template <typename PointT>
inline void zero_point(PointT & p);

template <>
inline void zero_point(pcl::PointXYZ & p)
{
  p.x = p.y = p.z = 0;
}

template <>
inline void zero_point(pcl::PointXYZI & p)
{
  p.x = p.y = p.z = p.intensity = 0;
}

// Remove trailing whitespace, newline, and carriage return characters from a string
inline std::string trim(const std::string & input)
{
  std::string output;

  if (!input.empty()) {
    output.reserve(input.size());

    for (auto & c : input) {
      if (c != ' ' && c != '\n' && c != '\r') {
        output.push_back(c);
      }
    }
  }

  return output;
}

}  // namespace autoware::pointcloud_divider::util

#endif  // AUTOWARE__POINTCLOUD_DIVIDER__UTILITY_HPP_
