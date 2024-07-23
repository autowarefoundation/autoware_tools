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

#ifndef POINTCLOUD_MERGER__POINTCLOUD_MERGER_HPP_
#define POINTCLOUD_MERGER__POINTCLOUD_MERGER_HPP_

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

#define PCL_NO_PRECOMPILE
#include <pointcloud_divider/pcd_io.hpp>

template <class PointT>
class PointCloudMerger
{
  typedef pcl::PointCloud<PointT> PclCloudType;
  typedef typename PclCloudType::Ptr PclCloudPtr;

public:
  PointCloudMerger() {}
  ~PointCloudMerger() {}

  void run(
    std::vector<std::string> & pcd_names, const std::string & output_pcd,
    const std::string & config);

private:
  std::string output_pcd_, config_file_;

  // Params from yaml
  double leaf_size_ = 0.1;

  // Maximum number of points per PCD block
  const size_t max_block_size_ = 500000;

  std::string tmp_dir_;
  CustomPCDWriter<PointT> writer_;

  void paramInitialize();
  void mergeWithoutDownsample(std::vector<std::string> & input_pcds);
  void mergeWithDownsample(std::vector<std::string> & input_pcds);
};

#endif  // POINTCLOUD_MERGER__POINTCLOUD_MERGER_HPP_
