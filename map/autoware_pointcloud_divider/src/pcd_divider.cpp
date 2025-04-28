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

#include <autoware/pointcloud_divider/pcd_divider.hpp>
#include <autoware/pointcloud_divider/utility.hpp>
#include <autoware/pointcloud_divider/voxel_grid_filter.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <filesystem>
#include <list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace autoware::pointcloud_divider
{

template <class PointT>
std::vector<std::string> PCDDivider<PointT>::discoverPCDs(const std::string & input)
{
  // Discover PCD files in the input_dir
  std::vector<std::string> pcd_list;
  fs::path input_path(input);

  if (fs::is_directory(input_path)) {
    RCLCPP_INFO(logger_, "Input PCD directory: %s", input.c_str());

    for (auto & entry : fs::directory_iterator(input_path)) {
      if (fs::is_regular_file(entry.symlink_status())) {
        auto file_name = entry.path().string();
        auto extension = entry.path().extension().string();

        if (extension == ".pcd" || extension == ".PCD") {
          pcd_list.push_back(file_name);
        }
      }
    }
  } else if (fs::is_regular_file(input_path)) {
    auto file_name = input_path.string();
    auto extension = input_path.extension().string();

    if (extension == ".pcd" || extension == ".PCD") {
      RCLCPP_INFO(logger_, "Input PCD file: %s", input.c_str());

      pcd_list.push_back(file_name);
    } else {
      RCLCPP_ERROR(logger_, "Error: The input file is not PCD format %s", input.c_str());
      exit(EXIT_FAILURE);
    }
  } else {
    RCLCPP_ERROR(logger_, "Error: Invalid input %s", input.c_str());
    exit(EXIT_FAILURE);
  }

  return pcd_list;
}

template <class PointT>
void PCDDivider<PointT>::run(bool meta_gen)
{
  // Discover PCD files
  auto pcd_list = discoverPCDs(input_pcd_or_dir_);

  // Process pcd files
  if (meta_gen) {
    // Only generate a metadata file
    meta_generator(pcd_list);
  } else {
    // Do the segmentation
    run(pcd_list);
  }
}

template <class PointT>
void PCDDivider<PointT>::run(const std::vector<std::string> & pcd_names)
{
  checkOutputDirectoryValidity();

  grid_set_.clear();

  for (const std::string & pcd_name : pcd_names) {
    if (!rclcpp::ok()) {
      return;
    }

    if (debug_mode_) {
      RCLCPP_INFO(logger_, "Dividing file %s", pcd_name.c_str());
    }

    do {
      auto cloud_ptr = loadPCD(pcd_name);

      dividePointCloud(cloud_ptr);
    } while (reader_.good() && rclcpp::ok());
  }

  saveTheRest();

  RCLCPP_INFO(logger_, "Merge and downsampling... ");

  // Now merge and downsample
  mergeAndDownsample();

  std::string yaml_file_path = output_dir_ + "/pointcloud_map_metadata.yaml";
  saveGridInfoToYAML(yaml_file_path);

  RCLCPP_INFO(logger_, "Done!");
}

template <class PointT>
void PCDDivider<PointT>::checkOutputDirectoryValidity()
{
  tmp_dir_ = output_dir_ + "/tmp/";

  if (fs::exists(tmp_dir_)) {
    fs::remove_all(tmp_dir_);
  }

  if (fs::exists(output_dir_)) {
    fs::remove_all(output_dir_);
  }

  util::make_dir(output_dir_ + "/pointcloud_map.pcd/");
  util::make_dir(tmp_dir_);
}

template <class PointT>
typename pcl::PointCloud<PointT>::Ptr PCDDivider<PointT>::loadPCD(
  const std::string & pcd_name, bool load_all)
{
  if (pcd_name != reader_.get_path()) {
    reader_.setInput(pcd_name);
  }

  PclCloudPtr cloud_ptr(new PclCloudType);

  if (load_all) {
    size_t total_point_num = reader_.point_num();
    size_t copy_loc = 0;

    PclCloudType tmp_cloud;

    cloud_ptr->resize(total_point_num);

    int seg_count = 0;

    while (reader_.good() && rclcpp::ok()) {
      RCLCPP_INFO(logger_, "Copy segment %d", seg_count++);
      size_t read_size = reader_.readABlock(tmp_cloud);

      RCLCPP_INFO(
        logger_,
        "Read size (bytes/points) = %lu/%lu, tmp size = %lu, cloud size = %lu, total pnum = %lu",
        read_size, read_size / sizeof(PointT), tmp_cloud.size(), cloud_ptr->size(),
        total_point_num);

      if (read_size > 0) {
        if (!std::memcpy(
              cloud_ptr->data() + copy_loc, tmp_cloud.data(), tmp_cloud.size() * sizeof(PointT))) {
          RCLCPP_ERROR(logger_, "Error: Failed to copy points from tmp buffer to cloud");
          exit(EXIT_FAILURE);
        }
      }

      copy_loc += tmp_cloud.size();
    }
  } else {
    reader_.readABlock(*cloud_ptr);
  }

  return cloud_ptr;
}

template <class PointT>
void PCDDivider<PointT>::savePCD(const std::string & path, const pcl::PointCloud<PointT> & cloud)
{
  if (pcl::io::savePCDFileBinary(path, cloud) == -1) {
    RCLCPP_ERROR(logger_, "Error: Cannot save PCD: %s", path.c_str());
    rclcpp::shutdown();
    exit(EXIT_FAILURE);
  }
}

template <class PointT>
void PCDDivider<PointT>::dividePointCloud(const PclCloudPtr & cloud_ptr)
{
  if (!cloud_ptr || cloud_ptr->size() <= 0) {
    return;
  }

  for (const PointT p : *cloud_ptr) {
    if (!rclcpp::ok()) {
      rclcpp::shutdown();
      exit(EXIT_SUCCESS);
    }

    auto grid_key = pointToGrid2(p, grid_size_x_, grid_size_y_);
    auto it = grid_to_cloud_.find(grid_key);

    // If the grid has not existed yet, create a new one
    if (it == grid_to_cloud_.end()) {
      auto & new_grid = grid_to_cloud_[grid_key];

      std::get<0>(new_grid).reserve(max_block_size_);

      std::get<0>(new_grid).push_back(p);  // Push the first point to the cloud
      std::get<1>(new_grid) = 0;           // Counter set to 0
      std::get<2>(new_grid) = 0;           // Prev size is 0
    } else {
      auto & cloud = std::get<0>(it->second);
      auto & prev_size = std::get<2>(it->second);

      cloud.push_back(p);

      ++resident_point_num_;

      // If the number of points in the segment reach maximum, save the segment to file
      if (cloud.size() == max_block_size_) {
        saveGridPCD(it);
      } else {
        // Otherwise, update the seg_by_size_ if the change of size is significant
        if (cloud.size() - prev_size >= 10000) {
          prev_size = cloud.size();
          auto seg_to_size_it = seg_to_size_itr_map_.find(grid_key);

          if (seg_to_size_it == seg_to_size_itr_map_.end()) {
            auto size_it = seg_by_size_.insert(std::make_pair(prev_size, it));
            seg_to_size_itr_map_[grid_key] = size_it;
          } else {
            seg_by_size_.erase(seg_to_size_it->second);
            auto new_size_it = seg_by_size_.insert(std::make_pair(prev_size, it));
            seg_to_size_it->second = new_size_it;
          }
        }
      }

      // If the number of resident points reach maximum, save the biggest resident segment to SSD
      if (resident_point_num_ >= max_resident_point_num_) {
        auto max_size_seg_it = seg_by_size_.rbegin();

        saveGridPCD(max_size_seg_it->second);
      }
    }
  }
}

template <class PointT>
void PCDDivider<PointT>::saveGridPCD(GridMapItr & grid_it)
{
  auto & cloud = std::get<0>(grid_it->second);
  auto & counter = std::get<1>(grid_it->second);
  auto & prev_size = std::get<2>(grid_it->second);

  // Try to create a new directory to contain the cloud
  std::ostringstream seg_path, file_path;

  seg_path << tmp_dir_ << "/" << grid_it->first << "/";
  file_path << seg_path.str() << counter << "_" << cloud.size() << ".pcd";

  util::make_dir(seg_path.str());

  if (pcl::io::savePCDFileBinary(file_path.str(), cloud)) {
    RCLCPP_ERROR(logger_, "Error: Cannot save a PCD file at %s", file_path.str().c_str());
    rclcpp::shutdown();
    exit(EXIT_FAILURE);
  }

  resident_point_num_ -= cloud.size();

  // Clear the content of the segment cloud and reserve space for further points
  cloud.clear();
  cloud.reserve(max_block_size_);
  ++counter;  // Increase the counter so the next segment save will not overwrite the previously
              // saved one
  prev_size = 0;

  // Update the seg_by_size_ and seg_to_size_itr_map_
  auto it = seg_to_size_itr_map_.find(grid_it->first);

  if (it != seg_to_size_itr_map_.end()) {
    seg_by_size_.erase(it->second);
    seg_to_size_itr_map_.erase(it);
  }
}

template <class PointT>
void PCDDivider<PointT>::saveTheRest()
{
  for (auto it = grid_to_cloud_.begin(); it != grid_to_cloud_.end(); ++it) {
    auto & cloud = std::get<0>(it->second);

    if (cloud.size() > 0) {
      saveGridPCD(it);
    }
  }
}

template <class PointT>
void PCDDivider<PointT>::mergeAndDownsample()
{
  // Scan the tmp directory and find the segment folders
  fs::path tmp_path(tmp_dir_);

  // Count the number of sub folders, to estimate progress
  int seg_num = 0;

  for (auto & entry : fs::directory_iterator(tmp_path)) {
    if (fs::is_directory(entry.symlink_status())) {
      ++seg_num;
    }
  }

  for (auto & tmp_dir_entry : fs::directory_iterator(tmp_path)) {
    if (fs::is_directory(tmp_dir_entry.symlink_status())) {
      if (debug_mode_) {
        RCLCPP_INFO(logger_, "Saving segment %s", tmp_dir_entry.path().string().c_str());
      }
      std::list<std::string> pcd_list;
      size_t total_point_num = 0;

      for (auto & seg_entry : fs::directory_iterator(tmp_dir_entry.path())) {
        if (fs::is_regular_file(seg_entry.symlink_status())) {
          auto fname = seg_entry.path().string();
          auto ext = fname.substr(fname.size() - 4);

          if (ext == ".pcd") {
            pcd_list.push_back(fname);
            total_point_num += util::point_num(fname);
          }
        }
      }

      // Fuse all PCDs and downsample if necessary
      mergeAndDownsample(tmp_dir_entry.path().string(), pcd_list, total_point_num);
    }
  }

  // Remove tmp dir
  util::remove(tmp_dir_);
}

template <class PointT>
void PCDDivider<PointT>::mergeAndDownsample(
  const std::string & dir_path, std::list<std::string> & pcd_list, size_t total_point_num)
{
  PclCloudPtr new_cloud(new PclCloudType);

  new_cloud->reserve(total_point_num);

  // Merge all PCDs that belong to the specified segment to a single segment point cloud
  for (auto & fname : pcd_list) {
    PclCloudType seg_cloud;

    if (pcl::io::loadPCDFile(fname, seg_cloud)) {
      RCLCPP_ERROR(logger_, "Error: Failed to open a PCD file at %s", fname.c_str());
      rclcpp::shutdown();
      exit(EXIT_FAILURE);
    }

    for (auto & p : seg_cloud) {
      new_cloud->push_back(p);
    }
  }

  // Downsample if needed
  if (leaf_size_ > 0) {
    VoxelGridFilter<PointT> vgf;
    PclCloudPtr filtered_cloud(new PclCloudType);

    vgf.setResolution(leaf_size_);
    vgf.filter(*new_cloud, *filtered_cloud);

    new_cloud = filtered_cloud;
  }

  // Save the new_cloud
  // Extract segment name only (format gx_gy)
  int end_name, start_name;

  for (end_name = dir_path.size() - 1; end_name >= 0 && dir_path[end_name] == '/'; --end_name) {
  }
  for (start_name = end_name; start_name >= 0 && dir_path[start_name] != '/'; --start_name) {
  }

  std::string seg_name_only = dir_path.substr(start_name + 1, end_name - start_name);

  // Parse the @seg_path to get the indices of the segment
  auto underbar_pos = seg_name_only.rfind("_");
  int gx = std::stoi(seg_name_only.substr(0, underbar_pos));
  int gy = std::stoi(seg_name_only.substr(underbar_pos + 1));

  grid_set_.insert(GridInfo<2>(gx, gy));

  // Construct the path to save the new_cloud
  std::string save_path;

  // If save large pcd was turned on, create a folder to contain segment pcds
  if (use_large_grid_) {
    int large_gx = static_cast<int>(std::floor(static_cast<float>(gx) / g_grid_size_x_));
    int large_gy = static_cast<int>(std::floor(static_cast<float>(gy) / g_grid_size_y_));
    std::string large_folder = output_dir_ + "/pointcloud_map.pcd/" + std::to_string(large_gx) +
                               "_" + std::to_string(large_gy) + "/";

    // Create a new folder for the large grid
    util::make_dir(large_folder);

    save_path = large_folder + file_prefix_ + "_" + seg_name_only + ".pcd";
  } else {
    save_path = output_dir_ + "/pointcloud_map.pcd/" + file_prefix_ + "_" + seg_name_only + ".pcd";
  }

  // Save the merged (filtered) cloud
  if (pcl::io::savePCDFileBinary(save_path, *new_cloud)) {
    RCLCPP_ERROR(logger_, "Error: Failed to save a point cloud at %s", save_path.c_str());
    rclcpp::shutdown();
    exit(EXIT_FAILURE);
  }

  // Delete the folder containing the segments
  util::remove(dir_path);
}

template <class PointT>
std::string PCDDivider<PointT>::makeFileName(const GridInfo<2> & grid) const
{
  return file_prefix_ + "_" + std::to_string(grid.ix) + "_" + std::to_string(grid.iy) + ".pcd";
}

template <class PointT>
void PCDDivider<PointT>::paramInitialize()
{
  try {
    YAML::Node conf = YAML::LoadFile(config_file_);
    auto params = conf["/**"]["ros__parameters"];
    use_large_grid_ = params["use_large_grid"].as<bool>();
    leaf_size_ = params["leaf_size"].as<double>();
    grid_size_x_ = params["grid_size_x"].as<double>();
    grid_size_y_ = params["grid_size_y"].as<double>();
  } catch (YAML::Exception & e) {
    RCLCPP_ERROR(logger_, "YAML Error: %s", e.what());
    rclcpp::shutdown();
    exit(EXIT_FAILURE);
  }

  g_grid_size_x_ = grid_size_x_ * 10;
  g_grid_size_y_ = grid_size_y_ * 10;
}

template <class PointT>
void PCDDivider<PointT>::saveGridInfoToYAML(const std::string & yaml_file_path)
{
  std::ofstream yaml_file(yaml_file_path);

  if (!yaml_file.is_open()) {
    RCLCPP_ERROR(logger_, "Error: Cannot open YAML file: %s", yaml_file_path.c_str());
    rclcpp::shutdown();
    exit(EXIT_FAILURE);
  }

  yaml_file << "x_resolution: " << grid_size_x_ << std::endl;
  yaml_file << "y_resolution: " << grid_size_y_ << std::endl;

  for (const auto & grid : grid_set_) {
    std::string file_name = makeFileName(grid);
    fs::path p(file_name);
    yaml_file << p.filename().string() << ": [" << grid.ix << ", " << grid.iy << "]" << std::endl;
  }

  yaml_file.close();
}

template <class PointT>
void PCDDivider<PointT>::meta_generator(const std::vector<std::string> & pcd_list)
{
  std::string yaml_file_path = output_dir_ + "/pointcloud_map_metadata.yaml";
  std::unordered_set<GridInfo<2>> segment_set;

  for (auto & pcd_name : pcd_list) {
    RCLCPP_ERROR(logger_, "pcd_name = %s", pcd_name.c_str());
    do {
      auto cloud_ptr = loadPCD(pcd_name);

      for (auto & p : *cloud_ptr) {
        if (!rclcpp::ok()) {
          exit(0);
        }

        auto seg_key = pointToGrid2(p, grid_size_x_, grid_size_y_);

        if (segment_set.find(seg_key) == segment_set.end()) {
          segment_set.insert(seg_key);
        }
      }
    } while (reader_.good() && rclcpp::ok());
  }

  std::ofstream output_metadata(yaml_file_path);

  output_metadata << "x_resolution: " << grid_size_x_ << std::endl;
  output_metadata << "y_resolution: " << grid_size_y_ << std::endl;

  for (auto & it : segment_set) {
    output_metadata << file_prefix_ << "_" << it.ix << "_" << it.iy << ".pcd: [" << it.ix << ", "
                    << it.iy << "]" << std::endl;
  }
}

template class PCDDivider<pcl::PointXYZ>;
template class PCDDivider<pcl::PointXYZI>;

}  // namespace autoware::pointcloud_divider
