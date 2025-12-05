// Copyright 2024 TIER IV, Inc.
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

#include "../src/bag_handler.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>
#include <gtest/gtest.h>

#include <chrono>
#include <memory>

using namespace autoware::planning_data_analyzer;
using namespace std::chrono;

class BagHandlerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    timestamp_ = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
  }

  rcutils_time_point_value_t timestamp_;
};

TEST_F(BagHandlerTest, BagDataConstruction)
{
  auto bag_data = std::make_shared<BagData>(timestamp_);

  EXPECT_EQ(bag_data->timestamp, timestamp_);
  EXPECT_EQ(bag_data->buffers.size(), 6u);
  // Check default topic names
  EXPECT_TRUE(bag_data->buffers.count("/tf"));
  EXPECT_TRUE(bag_data->buffers.count("/localization/kinematic_state"));
  EXPECT_TRUE(bag_data->buffers.count("/localization/acceleration"));
  EXPECT_TRUE(bag_data->buffers.count("/planning/scenario_planning/trajectory"));
  EXPECT_TRUE(bag_data->buffers.count("/perception/object_recognition/objects"));
  EXPECT_TRUE(bag_data->buffers.count("/vehicle/status/steering_status"));
}
