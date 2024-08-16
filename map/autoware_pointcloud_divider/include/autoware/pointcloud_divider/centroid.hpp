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

#ifndef AUTOWARE__POINTCLOUD_DIVIDER__CENTROID_HPP_
#define AUTOWARE__POINTCLOUD_DIVIDER__CENTROID_HPP_

#include "utility.hpp"

#include <pcl/point_types.h>

#include <iostream>

namespace autoware::pointcloud_divider
{

template <typename PointT>
void accumulate(const PointT & p, const PointT & first_p, PointT & acc_diff);
template <typename PointT>
void compute_centroid(
  const PointT & acc_diff, const PointT & first_p, size_t point_num, PointT & centroid);

template <>
void accumulate(const pcl::PointXYZ & p, const pcl::PointXYZ & first_p, pcl::PointXYZ & acc_diff)
{
  acc_diff.x += p.x - first_p.x;
  acc_diff.y += p.y - first_p.y;
  acc_diff.z += p.z - first_p.z;
}

template <>
void accumulate(const pcl::PointXYZI & p, const pcl::PointXYZI & first_p, pcl::PointXYZI & acc_diff)
{
  acc_diff.x += p.x - first_p.x;
  acc_diff.y += p.y - first_p.y;
  acc_diff.z += p.z - first_p.z;
  acc_diff.intensity += p.intensity - first_p.intensity;
}

template <>
void compute_centroid(
  const pcl::PointXYZ & acc_diff, const pcl::PointXYZ & first_p, size_t point_num,
  pcl::PointXYZ & centroid)
{
  double double_point_num = static_cast<double>(point_num);

  centroid.x = acc_diff.x / double_point_num + first_p.x;
  centroid.y = acc_diff.y / double_point_num + first_p.y;
  centroid.z = acc_diff.z / double_point_num + first_p.z;
}

template <>
void compute_centroid(
  const pcl::PointXYZI & acc_diff, const pcl::PointXYZI & first_p, size_t point_num,
  pcl::PointXYZI & centroid)
{
  double double_point_num = static_cast<double>(point_num);

  centroid.x = acc_diff.x / double_point_num + first_p.x;
  centroid.y = acc_diff.y / double_point_num + first_p.y;
  centroid.z = acc_diff.z / double_point_num + first_p.z;
  centroid.intensity = acc_diff.intensity / double_point_num + first_p.intensity;
}

template <typename PointT>
struct Centroid
{
  Centroid()
  {
    point_num_ = 0;
    util::zero_point(acc_diff_);
    util::zero_point(first_point_);
  }

  void add(const PointT & p)
  {
    if (point_num_ == 0) {
      first_point_ = p;
    } else {
      accumulate(p, first_point_, acc_diff_);
    }

    ++point_num_;
  }

  PointT get()
  {
    PointT centroid;

    if (point_num_ > 0) {
      compute_centroid(acc_diff_, first_point_, point_num_, centroid);
    } else {
      std::cerr << "Error: there is no point in the centroid group!" << std::endl;
      exit(EXIT_FAILURE);
    }

    return centroid;
  }

  PointT acc_diff_, first_point_;
  size_t point_num_;
};

}  // Namespace autoware::pointcloud_divider

#endif  // AUTOWARE__POINTCLOUD_DIVIDER__CENTROID_HPP_
