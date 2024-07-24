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

#ifndef POINTCLOUD_DIVIDER__VOXEL_GRID_FILTER_HPP_
#define POINTCLOUD_DIVIDER__VOXEL_GRID_FILTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>

template <typename PointT>
class VoxelGridFilter
{
  typedef pcl::PointCloud<PointT> PclCloudType;
  typedef typename PclCloudType::Ptr PclCloudPtr;

public:
  VoxelGridFilter() { resolution_ = 0; }

  void setResolution(float res)
  {
    if (res > 0) {
      resolution_ = res;
    }
  }

  void filter(const PclCloudType & input, PclCloudType & output);

private:
  float resolution_;
};

template class VoxelGridFilter<pcl::PointXYZ>;
template class VoxelGridFilter<pcl::PointXYZI>;

#endif  // POINTCLOUD_DIVIDER__VOXEL_GRID_FILTER_HPP_
