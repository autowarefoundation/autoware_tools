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

#ifndef MAP_VALIDATION_TESTER_HPP_
#define MAP_VALIDATION_TESTER_HPP_

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>

#include <gtest/gtest.h>
#include <lanelet2_io/Io.h>

#include <memory>
#include <string>
#include <vector>

class MapValidationTester : public ::testing::Test
{
protected:
  void load_target_map(std::string file_name)
  {
    const auto projector = std::make_unique<lanelet::projection::MGRSProjector>();

    std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("autoware_lanelet2_map_validator");

    map_ = lanelet::load(
      package_share_directory + "/data/map/" + file_name, *projector, &loading_errors_);

    EXPECT_NE(map_, nullptr);
  }

  lanelet::LaneletMapPtr map_{nullptr};
  std::vector<std::string> loading_errors_;
};

#endif  // MAP_VALIDATION_TESTER_HPP_
