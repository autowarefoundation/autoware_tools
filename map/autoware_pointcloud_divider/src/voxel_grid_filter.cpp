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

#include <autoware/pointcloud_divider/centroid.hpp>
#include <autoware/pointcloud_divider/grid_info.hpp>
#include <autoware/pointcloud_divider/voxel_grid_filter.hpp>

#include <unordered_map>

template <typename PointT>
void VoxelGridFilter<PointT>::filter(const PclCloudType & input, PclCloudType & output)
{
  if (resolution_ <= 0) {
    return;
  }

  std::unordered_map<GridInfo<3>, Centroid<PointT>> grid_map;

  for (auto & p : input) {
    auto gkey = pointToGrid3(p, resolution_, resolution_, resolution_);

    grid_map[gkey].add(p);
  }

  // Extract centroids
  output.reserve(grid_map.size());

  for (auto & it : grid_map) {
    output.push_back(it.second.get());
  }
}
