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

#include <autoware/pointcloud_divider/pointcloud_divider_node.hpp>
#include <autoware/pointcloud_merger/pointcloud_merger_node.hpp>
#include <autoware/pointcloud_divider/utility.hpp>
#include <autoware/pointcloud_merger/pointcloud_merger_node.hpp>

#include <filesystem>

#include <pcl/console/print.h>


namespace fs = std::filesystem;

namespace autoware::pointcloud_merger
{

template <class PointT>
void PointCloudMerger<PointT>::run(
  std::vector<std::string> & pcd_names, const std::string & output_pcd, const std::string & config)
{
  output_pcd_ = output_pcd;
  config_file_ = config;

  paramInitialize();

  // Just in case the downsampling option is on
  if (leaf_size_ > 0) {
    tmp_dir_ = "./pointcloud_merger_tmp/";

    if (fs::exists(tmp_dir_)) {
      fs::remove_all(tmp_dir_);
    }

    util::make_dir(tmp_dir_);
  }

  if (fs::exists(output_pcd_)) {
    fs::remove_all(output_pcd_);
  }

  if (leaf_size_ > 0) {
    mergeWithDownsample(pcd_names);
    util::remove(tmp_dir_);
  } else {
    mergeWithoutDownsample(pcd_names);
  }
}

template <class PointT>
void PointCloudMerger<PointT>::mergeWithDownsample(std::vector<std::string> & input_pcds)
{
  std::cout << "Downsampling by PointCloudDivider" << std::endl;
  // Divide the input clouds to smaller segments
  autoware::pointcloud_divider::PointCloudDivider<PointT> pcd_divider(this->get_node_options());

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
void PointCloudMerger<PointT>::mergeWithoutDownsample(std::vector<std::string> & input_pcds)
{
  // Check the number of points of the merger
  size_t total_point_num = 0;
  CustomPCDReader<PointT> reader;

  for (const auto & pcd_name : input_pcds) {
    std::cout << "Checking file " << pcd_name << "..." << std::endl;

    reader.setInput(pcd_name);

    total_point_num += reader.point_num();
  }

  writer_.setOutput(output_pcd_);
  writer_.writeMetadata(total_point_num, true);

  int progress = 0;

  std::cout << std::endl;

  for (const auto & pcd_name : input_pcds) {
    std::cout << "Merging... "
              << static_cast<int>(
                   static_cast<float>(progress) / static_cast<float>(input_pcds.size()) * 100)
              << "%\r" << std::flush;
    ++progress;
    reader.setInput(pcd_name);

    do {
      PclCloudType new_cloud;

      reader.readABlock(new_cloud);
      writer_.write(new_cloud);
    } while (reader.good());
  }
}

template <class PointT>
void PointCloudMerger<PointT>::paramInitialize()
{
  try {
    YAML::Node conf = YAML::LoadFile(config_file_)["pointcloud_merger"];
    leaf_size_ = conf["leaf_size"].as<double>();
  } catch (YAML::Exception & e) {
    std::cerr << "YAML Error: " << e.what() << std::endl;
    exit(1);
  }
}

// template class PointCloudMerger<pcl::PointXYZ>;
template class PointCloudMerger<pcl::PointXYZI>;
// template class PointCloudMerger<pcl::PointXYZINormal>;
// template class PointCloudMerger<pcl::PointXYZRGB>;
// template class PointCloudMerger<pcl::PointNormal>;

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_merger::PointCloudMerger<pcl::PointXYZ>)
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_merger::PointCloudMerger<pcl::PointXYZI>)


void printErrorAndExit(const std::string & message)
{
  std::cerr << "Error: " << message << std::endl;
  exit(1);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  // Change the default PCL's log level to suppress the following message:
  // `Failed to find match for field 'intensity'.`
  pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_ERROR);

  if (argc <= 1) {
    printErrorAndExit("There should be at least 6 runtime arguments.");
  }

  const int n_pcd = std::stoi(argv[1]);

  if (argc != 4 + n_pcd) {
    printErrorAndExit(
      "There should be " + std::to_string(4 + n_pcd) +
      " runtime arguments. input: " + std::to_string(argc));
  }

  std::vector<std::string> pcd_name;
  for (int pcd_id = 0; pcd_id < n_pcd; pcd_id++) {
    pcd_name.push_back(argv[2 + pcd_id]);
  }
  const std::string output_pcd = argv[n_pcd + 2];
  const std::string config = argv[n_pcd + 3];

  // Currently, any PCD will be loaded as pcl::PointXYZI.
  autoware::pointcloud_merger::PointCloudMerger<pcl::PointXYZI> merger(node_options);
  merger.run(pcd_name, output_pcd, config);

  std::cout << "pointcloud_merger has finished successfully" << std::endl;
  return 0;
}
