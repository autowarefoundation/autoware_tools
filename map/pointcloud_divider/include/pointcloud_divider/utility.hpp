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

#ifndef POINTCLOUD_DIVIDER__UTILITY_HPP_
#define POINTCLOUD_DIVIDER__UTILITY_HPP_

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace util
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
inline size_t point_num(const std::string & pcd_path)
{
  // Get the file name only
  auto last_ubar_pos = pcd_path.rfind("_");
  auto last_dot_pos = pcd_path.rfind(".");
  auto pnum_str = pcd_path.substr(last_ubar_pos + 1, last_dot_pos - last_ubar_pos - 1);

  return std::stoi(pnum_str);
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

}  // namespace util

#endif  // POINTCLOUD_DIVIDER__UTILITY_HPP_
