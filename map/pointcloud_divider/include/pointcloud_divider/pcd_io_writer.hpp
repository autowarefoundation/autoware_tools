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

#ifndef POINTCLOUD_DIVIDER__PCD_IO_WRITER_HPP_
#define POINTCLOUD_DIVIDER__PCD_IO_WRITER_HPP_

#include "utility.hpp"

#include <pcl/PCLPointField.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

template <typename PointT>
class CustomPCDWriter
{
  typedef pcl::PointCloud<PointT> PclCloudType;

public:
  CustomPCDWriter()
  {
    buffer_ = nullptr;
    point_num_ = 0;
    written_point_num_ = 0;
    binary_ = true;
    point_size_ = 0;
    write_size_ = 0;

    init();
  }

  // Set a file for writing
  void setOutput(const std::string & pcd_path);
  // Write metadata to the output
  void writeMetadata(size_t point_num, bool binary_mode);
  // Write a block of points to the output stream
  void write(const PclCloudType & input);

  // Get path to the current opening PCD
  const std::string & get_path() const { return pcd_path_; }

  void setBlockSize(size_t block_size) { block_size_ = block_size; }

  bool good() { return file_.good(); }

  ~CustomPCDWriter() { clear(); }

private:
  void writeABlockBinary(const PclCloudType & input, size_t loc, size_t proc_size);
  void writeABlockASCII(const PclCloudType & input, size_t loc, size_t proc_size);

  // If the number of points written to file is less than the number of points
  // in the header, fill the remaining points with 0. This often indicates
  // that there are something wrong with the processing, and filling by 0 helps
  // analyze the problem easier.
  void padding();

  void clear()
  {
    padding();

    fields_.clear();
    field_sizes_.clear();
    point_num_ = 0;
    written_point_num_ = 0;
    binary_ = true;

    if (file_.is_open()) {
      file_.close();
    }

    file_.clear();

    point_size_ = write_size_ = 0;
    if (buffer_) {
      delete[] buffer_;
    }

    buffer_ = nullptr;
    block_size_ = 30000000;
  }

  void init()
  {
    // Remember field sizes
    fields_ = pcl::getFields<PointT>();
    size_t i = 0;

    for (const auto & field : fields_) {
      // Remove unidentified fields
      if (field.name == "_") {
        continue;
      }

      size_t fs = field.count * pcl::getFieldSize(field.datatype);

      field_sizes_.push_back(fs);
      fields_[i++] = field;
      point_size_ += fs;
    }

    fields_.resize(i);

    // Reserve a buffer for writing
    write_size_ = point_size_ * block_size_;
    buffer_ = new char[write_size_];
  }

  // Metadata
  std::vector<pcl::PCLPointField> fields_;
  std::vector<size_t> field_sizes_;

  // Number of points in the PCD file
  size_t point_num_;
  bool binary_;                   // Data: true: binary, false: ascii
  std::ofstream file_;            // Input stream of the PCD file
  size_t block_size_ = 30000000;  // Maximum number of points to write in each writeABlock
  size_t point_size_, write_size_;
  char * buffer_;
  std::string pcd_path_;  // Path to the current opening PCD
  size_t written_point_num_;   // To track the number of points written to the file
};

template <typename PointT>
void CustomPCDWriter<PointT>::setOutput(const std::string & pcd_path)
{
  clear();
  init();

  file_.open(pcd_path);

  if (!file_.is_open()) {
    fprintf(
      stderr, "[%s, %d] %s::Error: Failed to open a file at %s\n", __FILE__, __LINE__, __func__,
      pcd_path.c_str());
    exit(EXIT_FAILURE);
  }

  pcd_path_ = pcd_path;
}

template <typename PointT>
void CustomPCDWriter<PointT>::writeMetadata(size_t point_num, bool binary_mode)
{
  if (!file_.is_open()) {
    fprintf(
      stderr, "[%s, %d] %s::Error: File is not opening at %s!\n", __FILE__, __LINE__, __func__,
      pcd_path_.c_str());
    exit(EXIT_FAILURE);
  }

  pcl::PCDWriter pcd_writer;
  std::string pcd_metadata = pcd_writer.generateHeader(PclCloudType(), point_num);

  file_ << pcd_metadata;

  binary_ = binary_mode;
  point_num_ = point_num;

  if (binary_) {
    file_ << "DATA binary" << std::endl;
  } else {
    file_ << "DATA ascii" << std::endl;
  }
}

template <typename PointT>
void CustomPCDWriter<PointT>::write(const PclCloudType & input)
{
  for (size_t read_loc = 0; read_loc < input.size(); read_loc += block_size_) {
    size_t proc_size =
      (read_loc + block_size_ < input.size()) ? block_size_ : input.size() - read_loc;

    if (binary_) {
      writeABlockBinary(input, read_loc, proc_size);
    } else {
      writeABlockASCII(input, read_loc, proc_size);
    }
  }

  written_point_num_ += input.size();
}

template <typename PointT>
void CustomPCDWriter<PointT>::writeABlockBinary(
  const PclCloudType & input, size_t loc, size_t proc_size)
{
  // TODO (anh.nguyen.2@tier4.jp): check the number of points written to the output file
  // Read points to the write buffer
  for (size_t i = loc, write_loc = 0; i < proc_size; ++i) {
    const char * p = reinterpret_cast<const char *>(&input[i]);

    for (size_t fid = 0; fid < fields_.size(); ++fid) {
      memcpy(buffer_ + write_loc, p + fields_[fid].offset, field_sizes_[fid]);
      write_loc += field_sizes_[fid];
    }
  }

  // Write the buffer to the file
  file_.write(buffer_, proc_size * point_size_);
}

template <typename PointT>
void CustomPCDWriter<PointT>::writeABlockASCII(
  const PclCloudType & cloud, size_t loc, size_t proc_size)
{
  // To reduce I/O cost, write several points together
  std::ostringstream stream;
  // When the number of points in the stream reach this number, write the stream to file
  const size_t max_point_num = 10;
  size_t packed_point_num = 0;

  for (size_t i = loc; i < proc_size; ++i) {
    auto & point = cloud[i];

    // Copied from the PCL library
    for (size_t d = 0; d < fields_.size(); ++d) {
      // Ignore invalid padded dimensions that are inherited from binary data
      if (fields_[d].name == "_") continue;

      int count = fields_[d].count;
      if (count == 0)
        count = 1;  // we simply cannot tolerate 0 counts (coming from older converter code)

      for (int c = 0; c < count; ++c) {
        switch (fields_[d].datatype) {
          // TODO(anh.nguyen.2@tier4.jp): PCL 1.12 has not supported BOOL, INT64, and UINT64 yet
          // case pcl::PCLPointField::BOOL:
          // {
          //   bool value;
          //   memcpy (&value, reinterpret_cast<const char*>(&point) + fields_[d].offset + c *
          //   sizeof(bool), sizeof(bool)); stream << boost::numeric_cast<std::int32_t>(value);
          //   break;
          // }
          case pcl::PCLPointField::INT8: {
            std::int8_t value;
            memcpy(
              &value,
              reinterpret_cast<const char *>(&point) + fields_[d].offset + c * sizeof(std::int8_t),
              sizeof(std::int8_t));
            stream << boost::numeric_cast<std::int32_t>(value);
            break;
          }
          case pcl::PCLPointField::UINT8: {
            std::uint8_t value;
            memcpy(
              &value,
              reinterpret_cast<const char *>(&point) + fields_[d].offset + c * sizeof(std::uint8_t),
              sizeof(std::uint8_t));
            stream << boost::numeric_cast<std::uint32_t>(value);
            break;
          }
          case pcl::PCLPointField::INT16: {
            std::int16_t value;
            memcpy(
              &value,
              reinterpret_cast<const char *>(&point) + fields_[d].offset + c * sizeof(std::int16_t),
              sizeof(std::int16_t));
            stream << boost::numeric_cast<std::int16_t>(value);
            break;
          }
          case pcl::PCLPointField::UINT16: {
            std::uint16_t value;
            memcpy(
              &value,
              reinterpret_cast<const char *>(&point) + fields_[d].offset +
                c * sizeof(std::uint16_t),
              sizeof(std::uint16_t));
            stream << boost::numeric_cast<std::uint16_t>(value);
            break;
          }
          case pcl::PCLPointField::INT32: {
            std::int32_t value;
            memcpy(
              &value,
              reinterpret_cast<const char *>(&point) + fields_[d].offset + c * sizeof(std::int32_t),
              sizeof(std::int32_t));
            stream << boost::numeric_cast<std::int32_t>(value);
            break;
          }
          case pcl::PCLPointField::UINT32: {
            std::uint32_t value;
            memcpy(
              &value,
              reinterpret_cast<const char *>(&point) + fields_[d].offset +
                c * sizeof(std::uint32_t),
              sizeof(std::uint32_t));
            stream << boost::numeric_cast<std::uint32_t>(value);
            break;
          }
          // case pcl::PCLPointField::INT64:
          // {
          //   std::int64_t value;
          //   memcpy (&value, reinterpret_cast<const char*>(&point) + fields_[d].offset + c *
          //   sizeof(std::int64_t), sizeof(std::int64_t)); stream <<
          //   boost::numeric_cast<std::int64_t>(value); break;
          // }
          // case pcl::PCLPointField::UINT64:
          // {
          //   std::uint64_t value;
          //   memcpy (&value, reinterpret_cast<const char*>(&point) + fields_[d].offset + c *
          //   sizeof(std::uint64_t), sizeof(std::uint64_t)); stream <<
          //   boost::numeric_cast<std::uint64_t>(value); break;
          // }
          case pcl::PCLPointField::FLOAT32: {
            /*
             * Despite the float type, store the rgb field as uint32
             * because several fully opaque color values are mapped to
             * nan.
             */
            if ("rgb" == fields_[d].name) {
              std::uint32_t value;
              memcpy(
                &value,
                reinterpret_cast<const char *>(&point) + fields_[d].offset + c * sizeof(float),
                sizeof(float));
              stream << boost::numeric_cast<std::uint32_t>(value);
              break;
            }
            float value;
            memcpy(
              &value,
              reinterpret_cast<const char *>(&point) + fields_[d].offset + c * sizeof(float),
              sizeof(float));
            if (std::isnan(value))
              stream << "nan";
            else
              stream << boost::numeric_cast<float>(value);
            break;
          }
          case pcl::PCLPointField::FLOAT64: {
            double value;
            memcpy(
              &value,
              reinterpret_cast<const char *>(&point) + fields_[d].offset + c * sizeof(double),
              sizeof(double));
            if (std::isnan(value))
              stream << "nan";
            else
              stream << boost::numeric_cast<double>(value);
            break;
          }
          default:
            fprintf(
              stderr, "[%s, %d] %s::Incorrect field data type specified (%d)!\n", __FILE__,
              __LINE__, __func__, fields_[d].datatype);
            break;
        }

        if (d < fields_.size() - 1 || c < static_cast<int>(fields_[d].count - 1)) {
          stream << " ";
        }
      }
    }

    stream << "\n";

    ++packed_point_num;

    if (packed_point_num == max_point_num) {
      // Copy the stream, trim it, and write it to disk
      std::string result = stream.str();
      boost::trim(result);
      stream.str("");
      file_ << result << "\n";
      packed_point_num = 0;
    }
  }

  // Write the rest of stream to file
  if (packed_point_num > 0) {
    // Copy the stream, trim it, and write it to disk
    std::string result = stream.str();
    boost::trim(result);
    stream.str("");
    file_ << result << "\n";
    packed_point_num = 0;
  }
}

template <typename PointT>
void CustomPCDWriter<PointT>::padding()
{
  if (written_point_num_ < point_num_) {
    PclCloudType all_zeros;

    all_zeros.resize(point_num_ - written_point_num_);

    PointT zp;

    memset(&zp, 0x00, sizeof(PointT));

    std::fill(all_zeros.begin(), all_zeros.end(), zp);

    write(all_zeros);
  }
}

#endif  // POINTCLOUD_DIVIDER__PCD_IO_WRITER_HPP_
