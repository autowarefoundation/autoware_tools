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

#ifndef POINTCLOUD_DIVIDER__POINTCLOUD_DIVIDER_HPP_
#define POINTCLOUD_DIVIDER__POINTCLOUD_DIVIDER_HPP_

#include <yaml-cpp/yaml.h>

#include <list>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#define PCL_NO_PRECOMPILE
#include "grid_info.hpp"
#include "pcd_io.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

template <class PointT>
class PointCloudDivider
{
  typedef pcl::PointCloud<PointT> PclCloudType;
  typedef typename PclCloudType::Ptr PclCloudPtr;
  typedef std::unordered_map<GridInfo<2>, std::tuple<PclCloudType, int, size_t>> GridMapType;
  typedef typename GridMapType::iterator GridMapItr;
  typedef std::multimap<size_t, GridMapItr> GridMapSizeType;
  typedef typename GridMapSizeType::iterator GridMapSizeItr;

public:
  PointCloudDivider() {}
  ~PointCloudDivider() {}

  std::pair<double, double> getGridSize() const
  {
    return std::pair<double, double>(grid_size_x_, grid_size_y_);
  }

  void run(
    std::vector<std::string> & pcd_names, const std::string & output_dir,
    const std::string & file_prefix, const std::string & config);

  void run(
    const PclCloudPtr & cloud, const std::string & output_dir, const std::string & file_prefix,
    const std::string & config);

  void run(const std::vector<std::string> & pcd_names);

  void setPrefix(const std::string & prefix) { file_prefix_ = prefix; }

  void setOutputDir(const std::string & output_dir)
  {
    output_dir_ = output_dir;
    tmp_dir_ = output_dir + "/tmp/";
  }

  void setGridSize(float res_x, float res_y)
  {
    grid_size_x_ = res_x;
    grid_size_y_ = res_y;
    g_grid_size_x_ = grid_size_x_ * 10;
    g_grid_size_y_ = grid_size_y_ * 10;
  }

  void setLargeGridMode(bool use_large_grid) { use_large_grid_ = use_large_grid; }

  void setLeafSize(float leaf_size) { leaf_size_ = leaf_size; }

  void setDebugMode(bool mode) { debug_mode_ = mode; }

  std::string makeFileName(const GridInfo<2> & grid) const;

  void setUseLargeGrid(const bool use_large_grid) { use_large_grid_ = use_large_grid; }

private:
  std::string output_dir_, file_prefix_, config_file_;

  std::unordered_set<GridInfo<2>> grid_set_;

  // Params from yaml
  bool use_large_grid_ = false;
  double leaf_size_ = 0.1;
  double grid_size_x_ = 100;
  double grid_size_y_ = 100;
  double g_grid_size_x_ = grid_size_x_ * 10;
  double g_grid_size_y_ = grid_size_y_ * 10;

  // Maximum number of points per PCD block
  const size_t max_block_size_ = 500000;

  // Map of points distributed to grids
  GridMapType grid_to_cloud_;

  // Segments but sorted by size
  GridMapSizeType seg_by_size_;
  // Map segments to size iterator
  std::unordered_map<GridInfo<2>, GridMapSizeItr> seg_to_size_itr_map_;
  // Only 100 million points are allowed to reside in the main memory at max
  const size_t max_resident_point_num_ = 100000000;
  size_t resident_point_num_ = 0;
  std::string tmp_dir_;
  CustomPCDReader<PointT> reader_;
  bool debug_mode_ = true;  // Print debug messages or not

  PclCloudPtr loadPCD(const std::string & pcd_name);
  void savePCD(const std::string & pcd_name, const pcl::PointCloud<PointT> & cloud);
  void dividePointCloud(const PclCloudPtr & cloud_ptr);
  void paramInitialize();
  void saveGridInfoToYAML(const std::string & yaml_file_path);
  void checkOutputDirectoryValidity() const;

  void saveGridPCD(GridMapItr & grid_it);
  void saveTheRest();
  void mergeAndDownsample();
  void mergeAndDownsample(
    const std::string & dir_path, std::list<std::string> & pcd_list, size_t total_point_num);
};

#endif  // POINTCLOUD_DIVIDER__POINTCLOUD_DIVIDER_HPP_
