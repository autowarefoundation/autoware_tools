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

#include "bag_handler.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

using autoware::planning_data_analyzer::BagData;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;
using std::chrono::system_clock;

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
  EXPECT_EQ(bag_data->buffers.size(), 8u);
  // Check default topic names
  EXPECT_TRUE(bag_data->buffers.count("/tf"));
  EXPECT_TRUE(bag_data->buffers.count("/localization/kinematic_state"));
  EXPECT_TRUE(bag_data->buffers.count("/localization/acceleration"));
  EXPECT_TRUE(bag_data->buffers.count("/planning/trajectory"));
  EXPECT_TRUE(bag_data->buffers.count("/diffusion_planner/output/trajectories"));
  EXPECT_TRUE(bag_data->buffers.count("/perception/object_recognition/objects"));
  EXPECT_TRUE(bag_data->buffers.count("/perception/traffic_light_recognition/traffic_signals"));
  EXPECT_TRUE(bag_data->buffers.count("/vehicle/status/steering_status"));
}

TEST_F(BagHandlerTest, GetLatestBeforeOrEqual)
{
  autoware::planning_data_analyzer::Buffer<autoware::planning_data_analyzer::Trajectory> buffer;

  autoware::planning_data_analyzer::Trajectory t1;
  t1.header.stamp.sec = 1;
  t1.header.stamp.nanosec = 0;
  buffer.msgs.push_back(t1);

  autoware::planning_data_analyzer::Trajectory t2;
  t2.header.stamp.sec = 2;
  t2.header.stamp.nanosec = 0;
  buffer.msgs.push_back(t2);

  autoware::planning_data_analyzer::Trajectory t3;
  t3.header.stamp.sec = 3;
  t3.header.stamp.nanosec = 0;
  buffer.msgs.push_back(t3);

  EXPECT_EQ(buffer.get_latest_before_or_equal(rclcpp::Time(0, 0).nanoseconds()), nullptr);
  EXPECT_NE(buffer.get_latest_before_or_equal(rclcpp::Time(1, 0).nanoseconds()), nullptr);
  EXPECT_EQ(
    buffer.get_latest_before_or_equal(rclcpp::Time(1, 500000000).nanoseconds())->header.stamp.sec,
    1);
  EXPECT_EQ(
    buffer.get_latest_before_or_equal(rclcpp::Time(2, 100000000).nanoseconds())->header.stamp.sec,
    2);
  EXPECT_EQ(
    buffer.get_latest_before_or_equal(rclcpp::Time(10, 0).nanoseconds())->header.stamp.sec, 3);
}
