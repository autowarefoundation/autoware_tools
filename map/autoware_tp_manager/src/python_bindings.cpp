// Copyright 2025 Autoware Foundation
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

#include <autoware/pointcloud_divider/pcd_divider.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pybind11/pybind11.h>

#include <string>

using PCDDividerWrapper = autoware::pointcloud_divider::PCDDivider<pcl::PointXYZI>;

PYBIND11_MODULE(pcd_divider_wrapper, m)
{
  pybind11::class_<PCDDividerWrapper>(m, "PCDDivider")
    .def(pybind11::init<const std::string &>())
    .def("setInput", &PCDDividerWrapper::setInput)
    .def("setOutputDir", &PCDDividerWrapper::setOutputDir)
    .def("setPrefix", &PCDDividerWrapper::setPrefix)
    .def("setLeafSize", &PCDDividerWrapper::setLeafSize)
    .def("setGridSize", &PCDDividerWrapper::setGridSize)
    .def("divide_pcd", static_cast<void (PCDDividerWrapper::*)()>(&PCDDividerWrapper::run))
    .def("meta_generator", &PCDDividerWrapper::meta_generator);

  m.def("init_ros", []() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  });

  m.def("shutdown_ros", []() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  });
}
