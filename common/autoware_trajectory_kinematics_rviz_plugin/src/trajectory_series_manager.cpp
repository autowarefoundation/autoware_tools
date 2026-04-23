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

#include "trajectory_series_manager.hpp"

#include "trajectory_message_adapter.hpp"

#include <QPointer>

#include <cstring>
#include <string>
#include <utility>
#include <vector>

namespace
{
constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

void fnv1aByte(std::uint64_t * h, unsigned char b)
{
  *h ^= static_cast<std::uint64_t>(b);
  *h *= kFnvPrime;
}

void fnv1aBytes(std::uint64_t * h, const void * data, std::size_t len)
{
  const auto * p = static_cast<const unsigned char *>(data);
  for (std::size_t i = 0; i < len; ++i) {
    fnv1aByte(h, p[i]);
  }
}

void fnv1aDouble(std::uint64_t * h, double v)
{
  static_assert(
    sizeof(double) == sizeof(std::uint64_t),
    "FNV fingerprint: std::memcpy double to uint64_t for hashing");
  std::uint64_t bits = 0;
  std::memcpy(&bits, &v, sizeof(bits));
  fnv1aBytes(h, &bits, sizeof(bits));
}

using autoware::visualization::trajectory_kinematics_rviz_plugin::TrajectorySeriesData;

/// Content fingerprint for deduplicating identical replays on the same topic.
std::uint64_t fingerprintSeriesVector(const std::vector<TrajectorySeriesData> & vec)
{
  std::uint64_t h = kFnvOffset;
  for (const auto & s : vec) {
    for (unsigned char c : s.topic) {
      fnv1aByte(&h, c);
    }
    fnv1aByte(&h, 0);
    for (unsigned char c : s.key) {
      fnv1aByte(&h, c);
    }
    fnv1aByte(&h, 0);
    for (unsigned char c : s.label) {
      fnv1aByte(&h, c);
    }
    fnv1aByte(&h, 0);
    const std::size_t number_points = s.points.size();
    fnv1aBytes(&h, &number_points, sizeof(number_points));
    for (const auto & pt : s.points) {
      fnv1aDouble(&h, pt.time_from_start_sec);
      fnv1aDouble(&h, pt.arc_length_m);
      fnv1aDouble(&h, pt.longitudinal_velocity_mps);
      fnv1aDouble(&h, pt.acceleration_mps2);
      fnv1aDouble(&h, pt.curvature_1pm);
      fnv1aDouble(&h, pt.lateral_acceleration_mps2);
    }
  }
  return h;
}

}  // namespace

namespace autoware::visualization::trajectory_kinematics_rviz_plugin
{

TrajectorySeriesManager::TrajectorySeriesManager(rclcpp::Node::SharedPtr node, QObject * parent)
: QObject(parent), node_(std::move(node))
{
}

void TrajectorySeriesManager::setTopicConfigs(
  const std::vector<std::pair<std::string, TopicMessageKind>> & configs)
{
  subscriptions_.clear();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_series_by_topic_.clear();
    last_series_fingerprint_by_topic_.clear();
  }

  if (!node_) {
    return;
  }

  const auto qos = rclcpp::QoS(10);

  for (const auto & entry : configs) {
    const std::string & topic = entry.first;
    if (topic.empty()) {
      continue;
    }
    if (entry.second == TopicMessageKind::Trajectory) {
      // QPointer: if this QObject is destroyed while the executor still holds the subscription,
      // skip the callback.
      QPointer<TrajectorySeriesManager> weak_this(this);
      auto sub = node_->create_subscription<autoware_planning_msgs::msg::Trajectory>(
        topic, qos,
        [weak_this, topic](const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg) {
          if (!weak_this) {
            return;
          }
          weak_this->onTrajectory(msg, topic);
        });
      subscriptions_[topic] = sub;
    } else {
      // Same QObject lifetime guard as Trajectory subscriptions.
      QPointer<TrajectorySeriesManager> weak_this(this);
      auto sub = node_->create_subscription<
        autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories>(
        topic, qos,
        [weak_this, topic](
          const autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories::ConstSharedPtr
            msg) {
          if (!weak_this) {
            return;
          }
          weak_this->onScoredCandidates(msg, topic);
        });
      subscriptions_[topic] = sub;
    }
  }
}

std::vector<TrajectorySeriesData> TrajectorySeriesManager::allSeries() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<TrajectorySeriesData> out;
  for (const auto & kv : last_series_by_topic_) {
    for (const auto & s : kv.second) {
      out.push_back(s);
    }
  }
  return out;
}

std::string TrajectorySeriesManager::seriesId(const TrajectorySeriesData & s)
{
  return s.topic + "|" + s.key;
}

void TrajectorySeriesManager::onTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg, const std::string & topic)
{
  auto series_vec = trajectoryMsgToSeries(*msg, topic);
  const std::uint64_t fp = fingerprintSeriesVector(series_vec);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Suppress duplicate GUI refreshes when the decoded trajectory is bitwise unchanged (e.g.
    // static bag loop).
    const auto it = last_series_fingerprint_by_topic_.find(topic);
    if (it != last_series_fingerprint_by_topic_.end() && it->second == fp) {
      return;
    }
    last_series_fingerprint_by_topic_[topic] = fp;
    last_series_by_topic_[topic] = std::move(series_vec);
  }
  Q_EMIT dataUpdated();
}

void TrajectorySeriesManager::onScoredCandidates(
  const autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories::ConstSharedPtr msg,
  const std::string & topic)
{
  auto series_vec = scoredCandidateTrajectoriesToSeries(*msg, topic);
  const std::uint64_t fp = fingerprintSeriesVector(series_vec);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Same deduplication as onTrajectory: skip emit when candidate set and scores are unchanged.
    const auto it = last_series_fingerprint_by_topic_.find(topic);
    if (it != last_series_fingerprint_by_topic_.end() && it->second == fp) {
      return;
    }
    last_series_fingerprint_by_topic_[topic] = fp;
    last_series_by_topic_[topic] = std::move(series_vec);
  }
  Q_EMIT dataUpdated();
}

}  // namespace autoware::visualization::trajectory_kinematics_rviz_plugin
