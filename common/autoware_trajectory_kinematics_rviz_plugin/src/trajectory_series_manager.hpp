// Copyright 2026 TIER IV, Inc.
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

#ifndef TRAJECTORY_SERIES_MANAGER_HPP_
#define TRAJECTORY_SERIES_MANAGER_HPP_

#include "kinematics_types.hpp"

#include <QObject>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/scored_candidate_trajectories.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace autoware::visualization::trajectory_kinematics_rviz_plugin
{

/// @brief ROS message type used when subscribing to a topic name.
enum class TopicMessageKind {
  Trajectory = 0,
  ScoredCandidateTrajectories = 1,
};

/// @brief Subscribes to trajectory-related topics, converts messages to TrajectorySeriesData, and
/// emits dataUpdated.
///
/// Callbacks use QPointer for safe delivery if the manager is destroyed. Per-topic FNV-1a
/// fingerprints suppress duplicate `dataUpdated` emissions when the decoded series content is
/// identical to the last message (e.g. repeated static transforms on the same bag frame).
class TrajectorySeriesManager : public QObject
{
  Q_OBJECT

public:
  /// @brief Stores the ROS node used to create subscriptions (typically the RViz display node).
  explicit TrajectorySeriesManager(rclcpp::Node::SharedPtr node, QObject * parent = nullptr);

  /// @brief Replaces all subscriptions to match `configs` (empty topic strings are skipped); clears
  /// cached series for removed topics.
  void setTopicConfigs(const std::vector<std::pair<std::string, TopicMessageKind>> & configs);

  /// @brief Returns a concatenation of the latest `TrajectorySeriesData` vectors for every
  /// subscribed topic (order not guaranteed across topics).
  std::vector<TrajectorySeriesData> allSeries() const;

  /// @brief Stable key for UI and persistence: `topic + "|" + key` (key distinguishes candidates on
  /// one topic).
  static std::string seriesId(const TrajectorySeriesData & s);

Q_SIGNALS:
  /// @brief Emitted after a topic's cached series change to a new fingerprint (may arrive at high
  /// rate).
  void dataUpdated();

private:
  /// @brief Converts Trajectory to one series and updates cache; emits only if the content
  /// fingerprint changed.
  void onTrajectory(
    const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg, const std::string & topic);

  /// @brief Converts ScoredCandidateTrajectories to multiple series and updates cache; emits only
  /// if the fingerprint changed.
  void onScoredCandidates(
    const autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories::ConstSharedPtr msg,
    const std::string & topic);

  rclcpp::Node::SharedPtr node_;
  mutable std::mutex mutex_;
  std::map<std::string, std::shared_ptr<rclcpp::SubscriptionBase>> subscriptions_;
  std::map<std::string, std::vector<TrajectorySeriesData>> last_series_by_topic_;
  /// @brief FNV-1a hash of decoded series per topic; identical replays skip `dataUpdated`.
  std::map<std::string, std::uint64_t> last_series_fingerprint_by_topic_;
};

}  // namespace autoware::visualization::trajectory_kinematics_rviz_plugin

#endif  // TRAJECTORY_SERIES_MANAGER_HPP_
