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

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef AUTOWARE__POINTCLOUD_DIVIDER__SCD_IO_READER_HPP_
#define AUTOWARE__POINTCLOUD_DIVIDER__SCD_IO_READER_HPP_

#include "utility.hpp"
#include "grid_info.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace autoware::pointcloud_divider
{

#ifndef INVALID_LOC_
#define INVALID_LOC_ (0xFFFF)
#endif

template <typename PointT>
class SCDReader
{
  typedef pcl::PointCloud<PointT> PclCloudType;

public:
  SCDReader()
  {
    buffer_ = nullptr;
    clear();
  }

  // Set a file to reading
  void setInput(const std::string & pcd_path);
  // Read a block of points from the input stream
  size_t readABlock(PclCloudType & output);
  // Sequentially read a segment from the list of segments
  size_t readASegment(PclCloudType & output);
  // Read a segment specified by a 2D key from the list of segments
  size_t readASegment(PclCloudType & output, const std::string & segment_idx);
  size_t readASegment(PclCloudType & output, int x, int y);

  // Get path to the current opening PCD
  const std::string & get_path() const { return pcd_path_; }

  void setBlockSize(size_t block_size) { block_size_ = block_size; }

  bool good() { return file_.good(); }

  size_t point_num()
  {
    if (file_.is_open()) {
      return point_num_;
    }

    return 0;
  }

  ~SCDReader() { clear(); }

private:
  void readHeader(std::ifstream & input);
  size_t readABlock(std::ifstream & input, PclCloudType & output);

  size_t readABlockBinary(std::ifstream & input, PclCloudType & output);
  size_t readABlockASCII(std::ifstream & input, PclCloudType & output);

  size_t readASegment(std::ifstream & input, PclCloudType & output);
  size_t readASegmentBinary(std::ifstream & input, PclCloudType & output);
  size_t readASegmentASCII(std::ifstream & input, PclCloudType & output);

  size_t readASegment(std::ifstream & input, PclCloudType & output, const GridInfo<2>& key);
  size_t readASegmentBinary(std::ifstream & input, PclCloudType & output, const GridInfo<2>& key);
  size_t readASegmentASCII(std::ifstream & input, PclCloudType & output, const GridInfo<2>& key);

  void clear()
  {
    version_.clear();
    width_ = height_ = 0;
    field_names_.clear();
    field_sizes_.clear();
    field_types_.clear();
    field_counts_.clear();
    origin_x_ = origin_y_ = origin_z_ = 0.0;
    orientation_w_ = orientation_x_ = orientation_y_ = orientation_z_ = 0.0;
    point_num_ = 0;
    loaded_point_num_ = 0;
    binary_ = true;

    if (file_.is_open()) {
      file_.close();
    }

    file_.clear();

    point_size_ = read_size_ = 0;
    if (buffer_) {
      delete[] buffer_;
    }

    buffer_ = nullptr;
    block_size_ = 30000000;
  }

  // Metadata
  std::string version_;
  size_t width_, height_;
  std::vector<std::string> field_names_;
  std::vector<size_t> field_sizes_;
  std::vector<std::string> field_types_;
  std::vector<size_t> field_counts_;
  // A map of segment index and the reading location
  std::unordered_map<GridInfo<2>, std::pair<int64_t, size_t>> segment_map_;
  // Viewpoint
  float origin_x_, origin_y_, origin_z_;
  float orientation_w_, orientation_x_, orientation_y_, orientation_z_;
  // Number of points in the PCD file
  size_t point_num_;
  // Number of points loaded so far, reset every time setInput is called
  size_t loaded_point_num_;
  bool binary_;                   // Data: true: binary, false: ascii
  std::ifstream file_;            // Input stream of the PCD file
  size_t block_size_ = 30000000;  // Number of points to read in each readABlock
  size_t point_size_, read_size_;
  char * buffer_;
  std::string pcd_path_;            // Path to the current opening PCD
  std::vector<size_t> read_loc_;    // Locations to read fields of a point
  std::vector<size_t> read_sizes_;  // Sizes of fields of a point
};

template <typename PointT>
void SCDReader<PointT>::setInput(const std::string & pcd_path)
{
  clear();

  file_.open(pcd_path);

  if (!file_.is_open()) {
    fprintf(
      stderr, "[%s, %d] %s::Error: Failed to open a file at %s\n", __FILE__, __LINE__, __func__,
      pcd_path.c_str());
    exit(EXIT_FAILURE);
  }

  pcd_path_ = pcd_path;
  readHeader(file_);
}

template <typename PointT>
size_t SCDReader<PointT>::readABlock(PclCloudType & output)
{
  return readABlock(file_, output);
}

// Build the location vectors for reading binary points
template <typename PointT>
inline void buildReadMetadata(
  std::vector<std::string> & field_names, std::vector<size_t> & field_sizes,
  std::vector<size_t> & field_counts, std::vector<size_t> & read_loc,
  std::vector<size_t> & read_sizes);

// Build the location vector for reading ascii points
template <typename PointT>
inline void buildReadMetadataASCII(
  std::vector<std::string> & field_names, std::vector<size_t> & read_loc);

template <typename PointT>
void SCDReader<PointT>::readHeader(std::ifstream & input)
{
  std::string line;
  std::vector<std::string> vals;  // Each line is splitted into multiple values by a delimiter

  try {
    while (input) {
      std::getline(input, line);

      if (line.empty()) {
        continue;
      }

      util::split(line, " ", vals);

      // Read comments
      if (vals[0] == "#") {
        continue;
      }

      if (vals[0] == "VERSION") {
        version_ = vals[1];

        continue;
      }

      if (vals[0] == "FIELDS") {
        for (size_t i = 1; i < vals.size(); ++i) {
          field_names_.push_back(util::trim(vals[i]));
        }

        continue;
      }

      if (vals[0] == "SIZE") {
        if (vals.size() - 1 != field_names_.size()) {
          fprintf(
            stderr,
            "[%s, %d] %s::Error: Invalid PCD file format: Number of sizes is different from number "
            "of fields. "
            "File %s\n",
            __FILE__, __LINE__, __func__, pcd_path_.c_str());
          exit(EXIT_FAILURE);
        }

        for (size_t i = 1; i < vals.size(); ++i) {
          field_sizes_.push_back(std::stoi(vals[i]));
        }

        continue;
      }

      if (vals[0] == "TYPE") {
        if (vals.size() - 1 != field_names_.size()) {
          fprintf(
            stderr,
            "[%s, %d] %s::Error: Invalid PCD file format: Number of types is different from number "
            "of fields. "
            "File %s\n",
            __FILE__, __LINE__, __func__, pcd_path_.c_str());
          exit(EXIT_FAILURE);
        }

        for (size_t i = 1; i < vals.size(); ++i) {
          field_types_.push_back(util::trim(vals[i]));
        }

        continue;
      }

      if (vals[0] == "COUNT") {
        if (vals.size() - 1 != field_names_.size()) {
          fprintf(
            stderr,
            "[%s, %d] %s::Error: Invalid PCD file format: Number of counts is different from "
            "number of fields. "
            "File %s\n",
            __FILE__, __LINE__, __func__, pcd_path_.c_str());
          exit(EXIT_FAILURE);
        }

        for (size_t i = 1; i < vals.size(); ++i) {
          field_counts_.push_back(std::stoi(vals[i]));
        }

        continue;
      }

      if (vals[0] == "WIDTH") {
        width_ = std::stoi(vals[1]);

        continue;
      }

      if (vals[0] == "HEIGHT") {
        height_ = std::stoi(vals[1]);

        continue;
      }

      if (vals[0] == "VIEWPOINT") {
        if (vals.size() != 8) {
          fprintf(
            stderr,
            "[%s, %d] %s::Error: Invalid PCD file format: Viewpoint must contain 7 fields. Number "
            "of actual "
            "fields: %lu. File %s\n",
            __FILE__, __LINE__, __func__, vals.size() - 1, pcd_path_.c_str());
          exit(EXIT_FAILURE);
        }

        origin_x_ = std::stof(vals[1]);
        origin_y_ = std::stof(vals[2]);
        origin_z_ = std::stof(vals[3]);

        orientation_w_ = std::stof(vals[4]);
        orientation_x_ = std::stof(vals[5]);
        orientation_y_ = std::stof(vals[6]);
        orientation_z_ = std::stof(vals[7]);

        continue;
      }

      if (vals[0] == "POINTS") {
        point_num_ = std::stoi(vals[1]);

        continue;
      }

      if (vals[0] == "SEGMENTS") {
        // A list of 4 numbers (segment index x, segment index y, offset in the file, and number of points in the segment)
        for (size_t i = 1; i < vals.size(); i += 4) {
          GridInfo<2> key(std::stoi(vals[i], std::stoi(vals[i + 1])));

          segment_map_[key] = std::pair<int64_t, size_t>(std::stol(vals[i + 2]), std::stoll(vals[i + 3])); 
        }
      }

      if (vals[0] == "DATA") {
        binary_ = vals[1].find("binary") != std::string::npos;

        break;
      }

      // After finish parsing fields, break
      // TODO(AnhNguyen): Should I roll back to the previous position?
      break;
    }
  } catch (...) {
    fprintf(
      stderr, "[%s, %d] %s::Error: Failed to read the PCD header from file %s\n", __FILE__,
      __LINE__, __func__, pcd_path_.c_str());
    exit(EXIT_FAILURE);
  }

  // Compute the number of bytes per reading
  point_size_ = read_size_ = 0;

  for (size_t i = 0; i < field_sizes_.size(); ++i) {
    point_size_ += field_sizes_[i] * field_counts_[i];
  }

  read_size_ = point_size_ * block_size_;

  if (buffer_) {
    delete[] buffer_;
  }

  buffer_ = new char[read_size_];

  if (field_sizes_.size() > 0) {
    // Construct read loc and read size, used to read data from files to points
    if (binary_) {
      buildReadMetadata<PointT>(field_names_, field_sizes_, field_counts_, read_loc_, read_sizes_);
    } else {
      buildReadMetadataASCII<PointT>(field_names_, read_loc_);
    }
  }
}

inline void setFieldReadMetadata(
  const std::string & field_tag, std::vector<std::string> & field_names,
  std::vector<size_t> & field_sizes, std::vector<size_t> & tmp_read_loc, size_t & read_loc,
  size_t & read_size)
{
  for (size_t i = 0; i < field_names.size(); ++i) {
    if (field_names[i] == field_tag) {
      read_loc = tmp_read_loc[i];
      read_size = field_sizes[i];

      return;
    }
  }

  // If the field tag was not found, set both read_loc and read_size to 0
  std::cerr << "Field " << field_tag << " is not available, and will be set to 0." << std::endl;
  read_loc = 0;
  read_size = 0;
}

inline void setFieldReadMetadata(
  const std::string & field_tag, std::vector<std::string> & field_names, size_t & read_loc)
{
  for (size_t i = 0; i < field_names.size(); ++i) {
    if (field_names[i] == field_tag) {
      read_loc = i;

      return;
    }
  }

  // If the field tag was not found, set read_loc to -1
  std::cerr << "Field " << field_tag << " is not available, and will be set to 0." << std::endl;
  read_loc = INVALID_LOC_;
}

template <>
inline void buildReadMetadata<pcl::PointXYZ>(
  std::vector<std::string> & field_names, std::vector<size_t> & field_sizes,
  std::vector<size_t> & field_counts, std::vector<size_t> & read_loc,
  std::vector<size_t> & read_sizes)
{
  size_t field_num = field_names.size();

  read_loc.resize(3);
  read_sizes.resize(3);

  std::vector<size_t> tmp_read_loc(field_num);

  tmp_read_loc[0] = 0;

  for (size_t i = 0; i < field_num - 1; ++i) {
    tmp_read_loc[i + 1] = field_sizes[i] * field_counts[i] + tmp_read_loc[i];
  }

  // Find field x, y, z
  setFieldReadMetadata("x", field_names, field_sizes, tmp_read_loc, read_loc[0], read_sizes[0]);
  setFieldReadMetadata("y", field_names, field_sizes, tmp_read_loc, read_loc[1], read_sizes[1]);
  setFieldReadMetadata("z", field_names, field_sizes, tmp_read_loc, read_loc[2], read_sizes[2]);
}

template <>
inline void buildReadMetadata<pcl::PointXYZI>(
  std::vector<std::string> & field_names, std::vector<size_t> & field_sizes,
  std::vector<size_t> & field_counts, std::vector<size_t> & read_loc,
  std::vector<size_t> & read_sizes)
{
  size_t field_num = field_names.size();

  read_loc.resize(4);
  read_sizes.resize(4);

  std::vector<size_t> tmp_read_loc(field_num);

  tmp_read_loc[0] = 0;

  for (size_t i = 0; i < field_num - 1; ++i) {
    tmp_read_loc[i + 1] = field_sizes[i] * field_counts[i] + tmp_read_loc[i];
  }

  // Find field x, y, z, intensity
  setFieldReadMetadata("x", field_names, field_sizes, tmp_read_loc, read_loc[0], read_sizes[0]);
  setFieldReadMetadata("y", field_names, field_sizes, tmp_read_loc, read_loc[1], read_sizes[1]);
  setFieldReadMetadata("z", field_names, field_sizes, tmp_read_loc, read_loc[2], read_sizes[2]);
  setFieldReadMetadata(
    "intensity", field_names, field_sizes, tmp_read_loc, read_loc[3], read_sizes[3]);
}

template <>
inline void buildReadMetadataASCII<pcl::PointXYZ>(
  std::vector<std::string> & field_names, std::vector<size_t> & read_loc)
{
  read_loc.resize(3);

  setFieldReadMetadata("x", field_names, read_loc[0]);
  setFieldReadMetadata("y", field_names, read_loc[1]);
  setFieldReadMetadata("z", field_names, read_loc[2]);
}

template <>
inline void buildReadMetadataASCII<pcl::PointXYZI>(
  std::vector<std::string> & field_names, std::vector<size_t> & read_loc)
{
  read_loc.resize(4);

  setFieldReadMetadata("x", field_names, read_loc[0]);
  setFieldReadMetadata("y", field_names, read_loc[1]);
  setFieldReadMetadata("z", field_names, read_loc[2]);
  setFieldReadMetadata("intensity", field_names, read_loc[3]);
}

template <typename PointT>
inline void parsePoint(
  const char * input, std::vector<size_t> & rsize, std::vector<size_t> & loc, PointT & output);

template <>
inline void parsePoint(
  const char * input, std::vector<size_t> & rsize, std::vector<size_t> & loc,
  pcl::PointXYZ & output)
{
  memcpy(&(output.x), input + loc[0], rsize[0]);
  memcpy(&(output.y), input + loc[1], rsize[1]);
  memcpy(&(output.z), input + loc[2], rsize[2]);
}

template <>
inline void parsePoint(
  const char * input, std::vector<size_t> & rsize, std::vector<size_t> & loc,
  pcl::PointXYZI & output)
{
  memcpy(&(output.x), input + loc[0], rsize[0]);
  memcpy(&(output.y), input + loc[1], rsize[1]);
  memcpy(&(output.z), input + loc[2], rsize[2]);
  memcpy(&(output.intensity), input + loc[3], rsize[3]);
}

template <typename PointT>
size_t SCDReader<PointT>::readABlockBinary(std::ifstream & input, PclCloudType & output)
{
  output.clear();
  output.reserve(block_size_);

  PointT p;

  if (input) {
    input.read(buffer_, read_size_);

    // Parse the buffer and convert to point
    for (int i = 0; i < input.gcount() && loaded_point_num_ < point_num_;
         i += point_size_, ++loaded_point_num_) {
      parsePoint(buffer_ + i, read_sizes_, read_loc_, p);
      output.push_back(p);
    }

    if (loaded_point_num_ == point_num_) {
      input.setstate(std::ios_base::eofbit);
    }

    return input.gcount();
  }

  return 0;
}

template <typename PointT>
void parsePoint(const std::string & point_line, const std::vector<size_t> & loc, PointT & output);

template <>
inline void parsePoint(
  const std::string & point_line, const std::vector<size_t> & loc, pcl::PointXYZ & output)
{
  std::vector<std::string> vals;

  util::split(point_line, " ", vals);
  output.x = (loc[0] != INVALID_LOC_) ? std::stof(vals[loc[0]]) : 0;
  output.y = (loc[1] != INVALID_LOC_) ? std::stof(vals[loc[1]]) : 0;
  output.z = (loc[2] != INVALID_LOC_) ? std::stof(vals[loc[2]]) : 0;
}

template <>
inline void parsePoint(
  const std::string & point_line, const std::vector<size_t> & loc, pcl::PointXYZI & output)
{
  std::vector<std::string> vals;

  util::split(point_line, " ", vals);
  output.x = (loc[0] != INVALID_LOC_) ? std::stof(vals[loc[0]]) : 0;
  output.y = (loc[1] != INVALID_LOC_) ? std::stof(vals[loc[1]]) : 0;
  output.z = (loc[2] != INVALID_LOC_) ? std::stof(vals[loc[2]]) : 0;
  output.intensity = (loc[3] != INVALID_LOC_) ? std::stof(vals[loc[3]]) : 0;
}

template <typename PointT>
size_t SCDReader<PointT>::readABlockASCII(std::ifstream & input, PclCloudType & output)
{
  output.clear();
  output.reserve(block_size_);

  std::string point_line;
  size_t read_byte_num = 0;

  PointT p;

  while (input && loaded_point_num_ < point_num_) {
    std::getline(input, point_line);

    if (!input.fail()) {
      // Parse the buffer and convert to point
      parsePoint(point_line, read_loc_, p);
      output.push_back(p);

      read_byte_num += input.gcount();
      ++loaded_point_num_;
    } else {
      fprintf(
        stderr, "[%s, %d] %s::Error: Failed to read a block of points from file. File %s\n",
        __FILE__, __LINE__, __func__, pcd_path_.c_str());
      exit(EXIT_FAILURE);
    }
  }

  if (loaded_point_num_ == point_num_) {
    input.setstate(std::ios_base::eofbit);
  }

  return read_byte_num;
}

template <typename PointT>
size_t SCDReader<PointT>::readABlock(std::ifstream & input, PclCloudType & output)
{
  if (binary_) {
    return readABlockBinary(input, output);
  }

  return readABlockASCII(input, output);
}

template <typename PointT>
size_t SCDReader<PointT>::readASegment(std::ifstream & input, PclCloudType & output)
{
  if (binary_) {
    return readASegmentBinary(input, output);
  }

  return readASegmentASCII(input, output);
}

template <typename PointT>
size_t SCDReader<PointT>::readASegment(std::ifstream & input, PclCloudType & output, const GridInfo<2> & key)
{
  if (binary_) {
    return readASegmentBinary(input, output, key);
  }

  return readASegmentASCII(input, output, key);
}

}  // namespace autoware::pointcloud_divider

#endif  // AUTOWARE__POINTCLOUD_DIVIDER__PCD_IO_READER_HPP_
