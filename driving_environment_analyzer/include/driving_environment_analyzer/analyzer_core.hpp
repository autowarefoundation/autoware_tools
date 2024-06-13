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

#ifndef DRIVING_ENVIRONMENT_ANALYZER__ANALYZER_CORE_HPP_
#define DRIVING_ENVIRONMENT_ANALYZER__ANALYZER_CORE_HPP_

#include "driving_environment_analyzer/type_alias.hpp"
#include "rosbag2_cpp/reader.hpp"

#include <autoware_route_handler/route_handler.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace driving_environment_analyzer::analyzer_core
{

struct ODDRawData
{
  rcutils_time_point_value_t timestamp;
  Odometry odometry;
  PredictedObjects objects;
  TFMessage tf;
  TFMessage tf_static;
  CooperateStatusArray rtc_status;
};

class AnalyzerCore
{
public:
  explicit AnalyzerCore(rclcpp::Node & node);
  ~AnalyzerCore();

  bool isDataReadyForStaticODDAnalysis() const;
  bool isDataReadyForDynamicODDAnalysis() const { return odd_raw_data_.has_value(); }

  void analyzeStaticODDFactor() const;
  void analyzeDynamicODDFactor(std::ofstream & ofs_csv_file) const;

  void addHeader(std::ofstream & ofs_csv_file) const;

  void setBagFile(const std::string & file_name);

  void setTimeStamp(const rcutils_time_point_value_t & timestamp)
  {
    odd_raw_data_ = getRawData(timestamp);
  }

  void setMap(const LaneletMapBin & msg) { route_handler_.setMap(msg); }

  void clearData() { odd_raw_data_ = std::nullopt; }

  std::pair<rcutils_time_point_value_t, rcutils_time_point_value_t> getBagStartEndTime()
  {
    const auto metadata = reader_.get_metadata();
    const auto start_time =
      duration_cast<seconds>(metadata.starting_time.time_since_epoch()).count();
    const auto duration_time = duration_cast<seconds>(metadata.duration).count();
    return {start_time, start_time + duration_time};
  }

  Odometry getOdometry() const { return odd_raw_data_.value().odometry; }
  PredictedObjects getObjects() const { return odd_raw_data_.value().objects; }
  TFMessage getTF() const { return odd_raw_data_.value().tf; }
  TFMessage getTFStatic() const { return odd_raw_data_.value().tf_static; }

private:
  Pose getEgoPose() const { return odd_raw_data_.value().odometry.pose.pose; }

  double getEgoSpeed() const { return odd_raw_data_.value().odometry.twist.twist.linear.x; }

  template <class T>
  std::optional<T> getLastTopic(const std::string & topic_name);
  template <class T>
  std::optional<T> seekTopic(
    const std::string & topic_name, const rcutils_time_point_value_t & timestamp);
  std::optional<ODDRawData> getRawData(const rcutils_time_point_value_t & timestamp);

  std::optional<ODDRawData> odd_raw_data_{std::nullopt};

  autoware::route_handler::RouteHandler route_handler_;

  rosbag2_cpp::Reader reader_;

  rclcpp::Logger logger_;
};
}  // namespace driving_environment_analyzer::analyzer_core

#endif  // DRIVING_ENVIRONMENT_ANALYZER__ANALYZER_CORE_HPP_
