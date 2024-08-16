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

// BSD 3-Clause License
//
// Copyright (c) 2023, MAP IV
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "include/pointcloud_merger_node.hpp"

#include <autoware/pointcloud_divider/pcd_divider.hpp>
#include <autoware/pointcloud_divider/utility.hpp>
#include <autoware/pointcloud_merger/pcd_merger.hpp>

#include <pcl/console/print.h>

#include <filesystem>

namespace fs = std::filesystem;

namespace autoware::pointcloud_merger
{

template <class PointT>
std::vector<std::string> PCDMerger<PointT>::discoverPCDs(const std::string & input)
{
  fs::path input_path(input);

  if (!fs::is_directory(input_path)) {
    RCLCPP_ERROR(logger_, "Error: Invalid input directory %s", input.c_str());
    rclcpp::shutdown();
  }

  std::vector<std::string> pcd_list;

  for (auto & entry : fs::directory_iterator(input_path)) {
    if (fs::is_regular_file(entry.symlink_status())) {
      auto file_name = entry.path().string();
      auto extension = entry.path().extension().string();

      if (extension == ".pcd" || extension == ".PCD") {
        pcd_list.push_back(file_name);
      }
    }
  }

  RCLCPP_INFO(logger_, "Found %lu PCD files", pcd_list.size());

  return pcd_list;
}

template <class PointT>
void PCDMerger<PointT>::run()
{
  auto pcd_list = discoverPCDs(input_dir_);

  run(pcd_list);
}

template <class PointT>
void PCDMerger<PointT>::run(const std::vector<std::string> & pcd_names)
{
  // Just in case the downsampling option is on
  if (leaf_size_ > 0) {
    tmp_dir_ = "./pointcloud_merger_tmp/";

    if (fs::exists(tmp_dir_)) {
      fs::remove_all(tmp_dir_);
    }

    autoware::pointcloud_divider::util::make_dir(tmp_dir_);
  }

  if (fs::exists(output_pcd_)) {
    fs::remove_all(output_pcd_);
  }

  if (leaf_size_ > 0) {
    mergeWithDownsample(pcd_names);
    autoware::pointcloud_divider::util::remove(tmp_dir_);
  } else {
    mergeWithoutDownsample(pcd_names);
  }
}

template <class PointT>
void PCDMerger<PointT>::mergeWithDownsample(const std::vector<std::string> & input_pcds)
{
  RCLCPP_INFO(logger_, "Downsampling by PointCloudDivider");
  // Divide the input clouds to smaller segments
  autoware::pointcloud_divider::PCDDivider<PointT> pcd_divider(logger_);

  pcd_divider.setOutputDir(tmp_dir_);
  pcd_divider.setGridSize(leaf_size_ * 100, leaf_size_ * 100);
  pcd_divider.setPrefix("tmp_segment");
  pcd_divider.setLeafSize(leaf_size_);
  pcd_divider.setDebugMode(false);
  pcd_divider.run(input_pcds);

  // Now go get path of the segmentation pcd
  std::vector<std::string> seg_names;
  fs::path tmp_path(tmp_dir_);

  for (auto & entry : fs::directory_iterator(tmp_path)) {
    if (fs::is_regular_file(entry.symlink_status())) {
      auto fname = entry.path().string();
      auto ext = fname.substr(fname.size() - 4);

      if (ext == ".pcd") {
        seg_names.push_back(fname);
      }
    }
  }

  // Now merge the downsampled segments
  mergeWithoutDownsample(seg_names);
}

template <class PointT>
void PCDMerger<PointT>::mergeWithoutDownsample(const std::vector<std::string> & input_pcds)
{
  if (input_pcds.size() == 0) {
    RCLCPP_INFO(logger_, "No input PCDs. Return!");

    return;
  }

  // Check the number of points of the merger
  size_t total_point_num = 0;
  autoware::pointcloud_divider::CustomPCDReader<PointT> reader;

  for (const auto & pcd_name : input_pcds) {
    reader.setInput(pcd_name);

    total_point_num += reader.point_num();
  }

  writer_.setOutput(output_pcd_);
  writer_.writeMetadata(total_point_num, true);

  size_t file_counter = 0;

  for (const auto & pcd_name : input_pcds) {
    if (!rclcpp::ok()) {
      return;
    }

    RCLCPP_INFO(
      logger_, "Processing file [%lu/%lu] %s", file_counter, input_pcds.size(), pcd_name.c_str());
    ++file_counter;

    reader.setInput(pcd_name);

    do {
      PclCloudType new_cloud;

      reader.readABlock(new_cloud);
      writer_.write(new_cloud);
    } while (reader.good() && rclcpp::ok());
  }
}

template <class PointT>
void PCDMerger<PointT>::paramInitialize()
{
  try {
    YAML::Node conf = YAML::LoadFile(config_file_);
    auto params = conf["/**"]["ros__parameters"];

    leaf_size_ = params["leaf_size"].as<double>();

  } catch (YAML::Exception & e) {
    RCLCPP_ERROR(logger_, "YAML Error: %s", e.what());
    rclcpp::shutdown();
    exit(EXIT_FAILURE);
  }
}

template class PCDMerger<pcl::PointXYZ>;
template class PCDMerger<pcl::PointXYZI>;
// template class PCDMerger<pcl::PointXYZINormal>;
// template class PCDMerger<pcl::PointXYZRGB>;
// template class PCDMerger<pcl::PointNormal>;

}  // namespace autoware::pointcloud_merger
