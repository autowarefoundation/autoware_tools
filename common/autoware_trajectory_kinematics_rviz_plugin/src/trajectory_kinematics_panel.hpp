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

#ifndef TRAJECTORY_KINEMATICS_PANEL_HPP_
#define TRAJECTORY_KINEMATICS_PANEL_HPP_

#include "kinematics_types.hpp"
#include "trajectory_kinematics_plot_widget.hpp"
#include "trajectory_series_manager.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QTimer>
#include <rviz_common/panel.hpp>

class QHideEvent;
class QShowEvent;

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::visualization::trajectory_kinematics_rviz_plugin
{

/// @brief Qt item roles for metadata stored on each row of `series_list_` (series identity and
/// source topic).
enum class SeriesListItemRole : int {
  kSeriesId = Qt::UserRole,
  kAllSeriesIndex = Qt::UserRole + 1,
  kTopicName = Qt::UserRole + 2,
};

/// @brief RViz panel that subscribes to trajectory topics, lists selectable series, and plots
/// kinematics.
///
/// Subscriptions and cached samples are owned by TrajectorySeriesManager. The panel wires UI state
/// (axes, fixed ranges, which series are checked) to TrajectoryKinematicsPlotWidget and persists
/// configuration via rviz_common::Config.
class TrajectoryKinematicsPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  /// @brief Builds the widget hierarchy, timers, and signal connections (ROS node is set in
  /// onInitialize()).
  explicit TrajectoryKinematicsPanel(QWidget * parent = nullptr);

  /// @brief Acquires the display ROS node, constructs TrajectorySeriesManager, and applies saved or
  /// default topics.
  void onInitialize() override;

  /// @brief Restores topics, axes, checked series, and plot range locks from RViz layout storage.
  void load(const rviz_common::Config & config) override;

  /// @brief Persists topic configuration, axis selection, checked series, and plot range settings.
  void save(rviz_common::Config config) const override;

protected:
  // Note: Manual refresh only - no automatic polling
  void showEvent(QShowEvent * event) override;
  void hideEvent(QHideEvent * event) override;

private Q_SLOTS:
  /// @brief Appends the chosen (or manually entered) topic and message kind to the subscription
  /// list.
  void onAddTopic();

  /// @brief Removes subscriptions for ROS topic names taken from the current list selection.
  void onRemoveTopic();

  /// @brief Restarts the coalesce timer so high-rate `dataUpdated` bursts collapse to one refresh.
  void onDataUpdated();

  /// @brief Merges new series into `series_checked_`, rebuilds the list if its signature changed,
  /// and redraws the plot.
  void processDataUpdated();

  /// @brief Persists checkbox state per series id and refreshes the plot.
  void onSeriesItemChanged();

  /// @brief Replots when X/Y axis combo selection changes.
  void onAxisChanged();

  /// @brief Refills the topic dropdown from `get_topic_names_and_types()` filtered by the selected
  /// message type.
  void refreshTopicList();

  /// @brief Pushes checked series and current axes/ranges to the plot widget and updates end-point
  /// summary labels.
  void refreshPlot();

private:
  /// @brief Calls TrajectorySeriesManager::setTopicConfigs() with `configured_topics_` and triggers
  /// a full data pass.
  void applyTopicConfigsToManager();

  /// @brief Trims whitespace and ensures a leading `/` for ROS topic names.
  static std::string normalizeTopicString(std::string s);

  /// @brief Rebuilds `series_list_` from TrajectorySeriesManager::allSeries() when the (id, label)
  /// signature changes.
  void rebuildSeriesListWidget();

  /// @brief Shows end-of-path velocity, acceleration, and time from the first plotted series (if
  /// any).
  void updateSummary(const std::vector<TrajectorySeriesData> & plotted);

  /// @brief Fills X/Y axis combos from xAxisDefinitions() / yAxisDefinitions().
  void populateAxisCombos();

  /// @brief Applies Material Design–style colors and the panel UI font to child widgets.
  void applyPanelStyle();

  /// @brief Encodes topic configuration as `topic|kind` segments joined by `;` for RViz
  /// persistence.
  static std::string serializeTopics(
    const std::vector<std::pair<std::string, TopicMessageKind>> & v);

  /// @brief Parses serializeTopics() output into `out` (best-effort; invalid segments are skipped).
  static bool deserializeTopics(
    const std::string & s, std::vector<std::pair<std::string, TopicMessageKind>> * out);

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<TrajectorySeriesManager> series_manager_;

  QWidget * panel_root_{nullptr};
  QComboBox * topic_combo_{nullptr};
  QLineEdit * manual_topic_edit_{nullptr};
  QComboBox * kind_combo_{nullptr};
  QPushButton * refresh_topics_button_{nullptr};
  QPushButton * add_topic_button_{nullptr};
  QPushButton * remove_topic_button_{nullptr};

  QListWidget * series_list_{nullptr};
  QComboBox * x_axis_combo_{nullptr};
  QComboBox * y_axis_combo_{nullptr};

  QLabel * metric_velocity_{nullptr};
  QLabel * metric_accel_{nullptr};
  QLabel * metric_duration_{nullptr};

  TrajectoryKinematicsPlotWidget * plot_widget_{nullptr};

  /// Coalesces rapid `dataUpdated` signals before rebuilding list + plot (GUI thread).
  QTimer * plot_refresh_coalesce_timer_{nullptr};

  std::vector<std::pair<std::string, TopicMessageKind>> configured_topics_;
  std::unordered_map<std::string, bool> series_checked_;
  /// When unchanged, skip rebuilding the list so selection is stable during message updates.
  /// Tuples are (series id, display label, has_received_data) so data-state changes refresh the
  /// list.
  std::vector<std::tuple<std::string, std::string, bool>> last_series_list_signature_;
};

}  // namespace autoware::visualization::trajectory_kinematics_rviz_plugin

#endif  // TRAJECTORY_KINEMATICS_PANEL_HPP_
