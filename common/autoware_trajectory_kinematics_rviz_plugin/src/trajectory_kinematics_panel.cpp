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

#include "trajectory_kinematics_panel.hpp"

#include "material_colors.hpp"
#include "ui_font.hpp"

#include <QAbstractItemView>
#include <QBrush>
#include <QColor>
#include <QHBoxLayout>
#include <QHideEvent>
#include <QIcon>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPixmap>
#include <QShowEvent>
#include <QStringList>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

#include <algorithm>
#include <exception>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::visualization::trajectory_kinematics_rviz_plugin
{
namespace
{

constexpr char kTrajectoryType[] = "autoware_planning_msgs/msg/Trajectory";
constexpr char kScoredType[] = "autoware_internal_planning_msgs/msg/ScoredCandidateTrajectories";

/// Shown when the graph has no matching topics (not a valid subscription target).
const QString kEmptyTopicPlaceholder = QStringLiteral("(No matching topics — press Refresh)");

/// Debounce window for merging high-rate `dataUpdated` → panel refresh (ms).
constexpr int kPlotRefreshCoalesceMs = 50;

/// Accent colors for series (aligned with material info / success / warning / tertiary).
const QColor kColorCycle[] = {
  style::hexToQColor(style::default_colors.info),
  style::hexToQColor(style::default_colors.success),
  style::hexToQColor(style::default_colors.warning),
  style::hexToQColor(style::default_colors.tertiary_fixed),
  style::hexToQColor(style::default_colors.secondary_fixed_dim),
  style::hexToQColor(style::default_colors.primary_fixed_dim),
};

constexpr int kColorCycleCount = static_cast<int>(sizeof(kColorCycle) / sizeof(kColorCycle[0]));

QIcon makeTrajectoryColorIcon(const QColor & c)
{
  constexpr int d = 20;
  QPixmap pm(d, d);
  pm.fill(Qt::transparent);
  QPainter painter(&pm);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setBrush(c);
  painter.setPen(QColor(255, 255, 255, 100));
  painter.drawRoundedRect(2, 2, d - 4, d - 4, 4, 4);
  painter.end();
  return QIcon(pm);
}

QIcon makeRefreshIcon()
{
  constexpr int d = 20;
  QPixmap pm(d, d);
  pm.fill(Qt::transparent);
  QPainter painter(&pm);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QPen(Qt::white, 2.0, Qt::SolidLine, Qt::RoundCap));

  const QPointF center(d / 2.0, d / 2.0);
  constexpr double radius = 6.5;

  // Arc: from ~1 o'clock counterclockwise around to ~12 o'clock
  QPainterPath arc_path;
  arc_path.moveTo(center + QPointF(0, -radius));
  arc_path.arcTo(
    QRectF(center.x() - radius, center.y() - radius, radius * 2, radius * 2), 90.0, -300.0);
  painter.drawPath(arc_path);

  // Arrowhead at the top of the arc
  painter.drawLine(QPointF(0, -radius + 1.5), QPointF(2.5, -radius - 2.0));
  painter.drawLine(QPointF(0, -radius + 1.5), QPointF(-2.5, -radius - 2.0));

  painter.end();
  return QIcon(pm);
}

QColor colorForSeriesListIndex(int index_in_all_series)
{
  return kColorCycle[index_in_all_series % kColorCycleCount];
}

QString panelStyleSheet(const style::MaterialColors & c)
{
  return QStringLiteral(
           "QWidget#TkmPanelRoot {"
           "  background-color: %1; color: %2;"
           "  font-family: \"Segoe UI\", \"Ubuntu\", \"Noto Sans\", \"Liberation Sans\", "
           "sans-serif;"
           "  font-size: 13px; font-weight: 600;"
           "}"
           "QWidget#TkmPanelRoot QLabel { color: %2; font-weight: 600; }"
           "QWidget#TkmPanelRoot QComboBox {"
           "  background-color: %3; color: %2; border: 1px solid %4; border-radius: 6px;"
           "  padding: 4px 8px; min-height: 24px; font-weight: 600;"
           "}"
           "QWidget#TkmPanelRoot QComboBox:hover { border-color: %5; }"
           "QWidget#TkmPanelRoot QComboBox::drop-down { border: none; width: 20px; }"
           "QWidget#TkmPanelRoot QComboBox QAbstractItemView {"
           "  background-color: %3; color: %2; selection-background-color: %6; font-weight: 600;"
           "}"
           "QWidget#TkmPanelRoot QPushButton {"
           "  background-color: %7; color: %8; border: none; border-radius: 6px;"
           "  padding: 6px 14px; font-weight: 700;"
           "}"
           "QWidget#TkmPanelRoot QPushButton:hover { background-color: %9; }"
           "QWidget#TkmPanelRoot QPushButton:pressed { background-color: %10; }"
           "QWidget#TkmPanelRoot QListWidget {"
           "  background-color: %3; color: %2; border: 1px solid %4; border-radius: 6px;"
           "  font-weight: 600;"
           "}"
           "QWidget#TkmPanelRoot QLineEdit#TkmManualTopic {"
           "  background-color: %3; color: %2; border: 1px solid %4; border-radius: 6px;"
           "  padding: 5px 10px; font-weight: 600;"
           "}"
           "QWidget#TkmPanelRoot QCheckBox { color: %2; font-weight: 600; spacing: 6px; }")
    .arg(QString::fromStdString(c.background))
    .arg(QString::fromStdString(c.on_surface))
    .arg(QString::fromStdString(c.surface_container))
    .arg(QString::fromStdString(c.outline_variant))
    .arg(QString::fromStdString(c.outline))
    .arg(QString::fromStdString(c.primary_container))
    .arg(QString::fromStdString(c.enabled_button_bg))
    .arg(QString::fromStdString(c.on_primary))
    .arg(QString::fromStdString(c.hover_button_bg))
    .arg(QString::fromStdString(c.pressed_button_bg));
}

}  // namespace

TrajectoryKinematicsPanel::TrajectoryKinematicsPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  panel_root_ = new QWidget(this);
  panel_root_->setObjectName(QStringLiteral("TkmPanelRoot"));
  panel_root_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  auto * root = new QVBoxLayout(panel_root_);
  root->setSpacing(6);
  root->setContentsMargins(6, 6, 6, 6);

  {
    auto * row = new QHBoxLayout();
    row->addWidget(new QLabel(QStringLiteral("Topic:")));
    topic_combo_ = new QComboBox();
    topic_combo_->setEditable(false);
    topic_combo_->setMinimumWidth(125);
    topic_combo_->setMaxVisibleItems(18);
    topic_combo_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    topic_combo_->setMinimumContentsLength(28);
    topic_combo_->setToolTip(QStringLiteral(
      "Open the list and click a topic. Use \"Other topic\" below to type a name not "
      "listed."));
    row->addWidget(topic_combo_, 1);
    kind_combo_ = new QComboBox();
    kind_combo_->addItem(
      QStringLiteral("Trajectory"), static_cast<int>(TopicMessageKind::Trajectory));
    kind_combo_->addItem(
      QStringLiteral("CandidateTrajectories"),
      static_cast<int>(TopicMessageKind::ScoredCandidateTrajectories));
    row->addWidget(kind_combo_);
    refresh_topics_button_ = new QPushButton(makeRefreshIcon(), QString());
    refresh_topics_button_->setMaximumWidth(36);
    refresh_topics_button_->setToolTip(
      QStringLiteral("Reload topic list from the ROS graph (filtered by message type)"));
    row->addWidget(refresh_topics_button_);
    add_topic_button_ = new QPushButton(QStringLiteral("Add"));
    remove_topic_button_ = new QPushButton(QStringLiteral("Remove"));
    remove_topic_button_->setToolTip(
      QStringLiteral("Select one or more trajectories below, then remove their ROS topic "
                     "subscription(s)."));
    row->addWidget(add_topic_button_);
    row->addWidget(remove_topic_button_);
    root->addLayout(row);
  }

  {
    auto * row = new QHBoxLayout();
    row->addWidget(new QLabel(QStringLiteral("Other topic:")));
    manual_topic_edit_ = new QLineEdit();
    manual_topic_edit_->setObjectName(QStringLiteral("TkmManualTopic"));
    manual_topic_edit_->setPlaceholderText(
      QStringLiteral("Optional: type a full topic name if it is not in the list above"));
    manual_topic_edit_->setToolTip(
      QStringLiteral("If set, Add uses this text instead of the dropdown (for topics not yet on "
                     "the graph)."));
    row->addWidget(manual_topic_edit_, 1);
    root->addLayout(row);
  }

  {
    auto * series_label = new QLabel(QStringLiteral("Trajectories (chart color)"));
    series_label->setStyleSheet(QStringLiteral("font-weight: 700;"));
    root->addWidget(series_label);
    series_list_ = new QListWidget();
    series_list_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    series_list_->setToolTip(
      QStringLiteral("Swatch = plot line color. Check rows to plot. Select row(s) and use Remove "
                     "to drop that topic subscription."));
    series_list_->setIconSize(QSize(20, 20));
    series_list_->setMinimumHeight(60);
    series_list_->setMaximumHeight(90);
    root->addWidget(series_list_);
  }

  {
    auto * row = new QHBoxLayout();
    row->addWidget(new QLabel(QStringLiteral("X axis:")));
    x_axis_combo_ = new QComboBox();
    row->addWidget(x_axis_combo_);
    row->addWidget(new QLabel(QStringLiteral("Y axis:")));
    y_axis_combo_ = new QComboBox();
    row->addWidget(y_axis_combo_);
    root->addLayout(row);
  }

  {
    auto * row = new QHBoxLayout();
    row->addWidget(new QLabel(QStringLiteral("End v:")));
    metric_velocity_ = new QLabel(QStringLiteral("-"));
    row->addWidget(metric_velocity_);
    row->addWidget(new QLabel(QStringLiteral("End a:")));
    metric_accel_ = new QLabel(QStringLiteral("-"));
    row->addWidget(metric_accel_);
    row->addWidget(new QLabel(QStringLiteral("End t:")));
    metric_duration_ = new QLabel(QStringLiteral("-"));
    row->addWidget(metric_duration_);
    root->addLayout(row);
  }

  plot_widget_ = new TrajectoryKinematicsPlotWidget();
  root->addWidget(plot_widget_, 1);

  auto * outer = new QVBoxLayout(this);
  outer->setContentsMargins(2, 2, 2, 2);
  outer->addWidget(panel_root_);
  setLayout(outer);

  plot_refresh_coalesce_timer_ = new QTimer(this);
  plot_refresh_coalesce_timer_->setSingleShot(true);
  plot_refresh_coalesce_timer_->setInterval(kPlotRefreshCoalesceMs);

  populateAxisCombos();
  applyPanelStyle();

  connect(add_topic_button_, &QPushButton::clicked, this, &TrajectoryKinematicsPanel::onAddTopic);
  connect(
    remove_topic_button_, &QPushButton::clicked, this, &TrajectoryKinematicsPanel::onRemoveTopic);
  connect(
    refresh_topics_button_, &QPushButton::clicked, this,
    &TrajectoryKinematicsPanel::refreshTopicList);
  connect(
    kind_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
    &TrajectoryKinematicsPanel::refreshTopicList);
  connect(
    plot_refresh_coalesce_timer_, &QTimer::timeout, this,
    &TrajectoryKinematicsPanel::processDataUpdated);
  connect(
    series_list_, &QListWidget::itemChanged, this, &TrajectoryKinematicsPanel::onSeriesItemChanged);
  connect(
    x_axis_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
    &TrajectoryKinematicsPanel::onAxisChanged);
  connect(
    y_axis_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
    &TrajectoryKinematicsPanel::onAxisChanged);
}

void TrajectoryKinematicsPanel::applyPanelStyle()
{
  const QFont ui_font = makePanelUiFont();
  panel_root_->setFont(ui_font);
  plot_widget_->setFont(ui_font);
  panel_root_->setStyleSheet(panelStyleSheet(style::default_colors));
}

PlotAxisRangeOptions readPlotRangeOptionsDefault()
{
  // Always use dynamic range; lock flags are off and min/max values are ignored.
  return PlotAxisRangeOptions{};
}

void TrajectoryKinematicsPanel::populateAxisCombos()
{
  x_axis_combo_->clear();
  for (const auto & d : xAxisDefinitions()) {
    x_axis_combo_->addItem(QString::fromUtf8(d.label), static_cast<int>(d.id));
  }
  y_axis_combo_->clear();
  for (const auto & d : yAxisDefinitions()) {
    y_axis_combo_->addItem(QString::fromUtf8(d.label), static_cast<int>(d.id));
  }
  x_axis_combo_->setCurrentIndex(0);
  y_axis_combo_->setCurrentIndex(0);
}

void TrajectoryKinematicsPanel::onInitialize()
{
  auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    RCLCPP_ERROR(
      rclcpp::get_logger("TrajectoryKinematicsPanel"),
      "Failed to acquire ROS node abstraction; panel cannot subscribe.");
    return;
  }
  node_ = node_abstraction->get_raw_node();
  series_manager_ = std::make_unique<TrajectorySeriesManager>(node_, this);
  // QueuedConnection: subscription callbacks may not run on the GUI thread; slot runs in the
  // panel's thread.
  connect(
    series_manager_.get(), &TrajectorySeriesManager::dataUpdated, this,
    &TrajectoryKinematicsPanel::onDataUpdated, Qt::QueuedConnection);
  refreshTopicList();
  // Topic polling starts in showEvent when the panel is visible. `load()` runs after onInitialize
  // when a layout is restored and calls applyTopicConfigsToManager() again; this call covers the
  // fresh-panel case (no saved config yet).
  applyTopicConfigsToManager();
}

void TrajectoryKinematicsPanel::refreshTopicList()
{
  if (!node_) {
    return;
  }

  const auto kind = static_cast<TopicMessageKind>(kind_combo_->currentData().toInt());
  const char * want_type = (kind == TopicMessageKind::Trajectory) ? kTrajectoryType : kScoredType;

  std::map<std::string, std::vector<std::string>> topic_map;
  try {
    topic_map = node_->get_topic_names_and_types();
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      rclcpp::get_logger("TrajectoryKinematicsPanel"), "get_topic_names_and_types failed: %s",
      e.what());
    return;
  }

  std::vector<std::string> names;
  for (const auto & kv : topic_map) {
    for (const auto & typ : kv.second) {
      if (typ == want_type) {
        names.push_back(kv.first);
        break;
      }
    }
  }
  std::sort(names.begin(), names.end());

  const QString previous =
    (topic_combo_->currentIndex() >= 0) ? topic_combo_->currentText() : QString();

  topic_combo_->blockSignals(true);
  topic_combo_->clear();
  if (names.empty()) {
    topic_combo_->addItem(kEmptyTopicPlaceholder);
    topic_combo_->setCurrentIndex(0);
  } else {
    for (const auto & n : names) {
      topic_combo_->addItem(QString::fromStdString(n));
    }
    const int idx = topic_combo_->findText(previous, Qt::MatchExactly);
    topic_combo_->setCurrentIndex(idx >= 0 ? idx : 0);
  }
  topic_combo_->blockSignals(false);
}

std::string TrajectoryKinematicsPanel::normalizeTopicString(std::string s)
{
  const auto first = s.find_first_not_of(" \t");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = s.find_last_not_of(" \t");
  s = s.substr(first, last - first + 1);
  if (!s.empty() && s.front() != '/') {
    s.insert(s.begin(), '/');
  }
  return s;
}

void TrajectoryKinematicsPanel::onAddTopic()
{
  std::string topic;
  const QString manual = manual_topic_edit_->text().trimmed();
  if (!manual.isEmpty()) {
    topic = normalizeTopicString(manual.toStdString());
  } else {
    if (topic_combo_->count() == 0 || topic_combo_->currentIndex() < 0) {
      return;
    }
    const QString picked = topic_combo_->currentText();
    if (picked == kEmptyTopicPlaceholder) {
      return;
    }
    topic = normalizeTopicString(picked.toStdString());
  }
  if (topic.empty()) {
    return;
  }
  const auto kind = static_cast<TopicMessageKind>(kind_combo_->currentData().toInt());
  for (const auto & p : configured_topics_) {
    if (p.first == topic && p.second == kind) {
      return;
    }
  }
  configured_topics_.emplace_back(topic, kind);
  manual_topic_edit_->clear();
  applyTopicConfigsToManager();
}

void TrajectoryKinematicsPanel::onRemoveTopic()
{
  const QList<QListWidgetItem *> selected = series_list_->selectedItems();
  if (selected.isEmpty()) {
    return;
  }
  std::set<std::string> topics_to_remove;
  for (const auto * item : selected) {
    const QString t = item->data(static_cast<int>(SeriesListItemRole::kTopicName)).toString();
    if (!t.isEmpty()) {
      topics_to_remove.insert(t.toStdString());
    }
  }
  if (topics_to_remove.empty()) {
    return;
  }
  configured_topics_.erase(
    std::remove_if(
      configured_topics_.begin(), configured_topics_.end(),
      [&topics_to_remove](const std::pair<std::string, TopicMessageKind> & p) {
        return topics_to_remove.count(p.first) > 0;
      }),
    configured_topics_.end());
  manual_topic_edit_->clear();
  applyTopicConfigsToManager();
}

void TrajectoryKinematicsPanel::applyTopicConfigsToManager()
{
  if (!series_manager_) {
    return;
  }
  series_manager_->setTopicConfigs(configured_topics_);
  processDataUpdated();
}

void TrajectoryKinematicsPanel::onDataUpdated()
{
  // Single-shot timer merges bursts of identical-topic replays into one list rebuild + plot update.
  plot_refresh_coalesce_timer_->start();
}

void TrajectoryKinematicsPanel::processDataUpdated()
{
  if (!series_manager_) {
    return;
  }
  const auto all = series_manager_->allSeries();
  std::unordered_map<std::string, bool> present;
  for (const auto & s : all) {
    const std::string id = TrajectorySeriesManager::seriesId(s);
    present[id] = true;
    // New series default to checked so first receipt shows a line without extra clicks.
    if (series_checked_.find(id) == series_checked_.end()) {
      series_checked_[id] = true;
    }
  }
  // Drop checkbox state for ids that disappeared (topic removed or candidate list shrunk).
  for (auto it = series_checked_.begin(); it != series_checked_.end();) {
    if (present.find(it->first) == present.end()) {
      it = series_checked_.erase(it);
    } else {
      ++it;
    }
  }
  rebuildSeriesListWidget();
  refreshPlot();
}

void TrajectoryKinematicsPanel::rebuildSeriesListWidget()
{
  if (!series_manager_) {
    return;
  }
  const auto all = series_manager_->allSeries();
  std::vector<std::tuple<std::string, std::string, bool>> current_sig;
  current_sig.reserve(all.size());
  for (const auto & s : all) {
    current_sig.emplace_back(TrajectorySeriesManager::seriesId(s), s.label, s.has_received_data);
  }
  // Skip QListWidget rebuild when only point values changed but ids/labels/data-state are unchanged
  // (keeps selection/focus stable).
  if (current_sig == last_series_list_signature_) {
    return;
  }
  last_series_list_signature_ = std::move(current_sig);

  series_list_->blockSignals(true);
  series_list_->clear();
  for (size_t ai = 0; ai < all.size(); ++ai) {
    const auto & s = all[ai];
    const std::string id = TrajectorySeriesManager::seriesId(s);
    QString display_label = QString::fromStdString(s.label);
    if (!s.has_received_data) {
      display_label += QStringLiteral(" (waiting...)");
    }
    auto * item = new QListWidgetItem(display_label);
    if (!s.has_received_data) {
      item->setForeground(QBrush(Qt::gray));
    }
    item->setData(static_cast<int>(SeriesListItemRole::kSeriesId), QString::fromStdString(id));
    item->setData(static_cast<int>(SeriesListItemRole::kAllSeriesIndex), static_cast<int>(ai));
    item->setData(
      static_cast<int>(SeriesListItemRole::kTopicName), QString::fromStdString(s.topic));
    item->setIcon(makeTrajectoryColorIcon(colorForSeriesListIndex(static_cast<int>(ai))));
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    const bool checked = series_checked_[id];
    item->setCheckState(checked ? Qt::Checked : Qt::Unchecked);
    series_list_->addItem(item);
  }
  series_list_->blockSignals(false);
}

void TrajectoryKinematicsPanel::onSeriesItemChanged()
{
  for (int i = 0; i < series_list_->count(); ++i) {
    auto * item = series_list_->item(i);
    const std::string id =
      item->data(static_cast<int>(SeriesListItemRole::kSeriesId)).toString().toStdString();
    series_checked_[id] = (item->checkState() == Qt::Checked);
  }
  refreshPlot();
}

void TrajectoryKinematicsPanel::onAxisChanged()
{
  refreshPlot();
}

void TrajectoryKinematicsPanel::refreshPlot()
{
  if (!series_manager_ || !plot_widget_) {
    return;
  }
  const AxisId x_axis = static_cast<AxisId>(x_axis_combo_->currentData().toInt());
  const AxisId y_axis = static_cast<AxisId>(y_axis_combo_->currentData().toInt());
  const auto all = series_manager_->allSeries();
  std::vector<TrajectorySeriesData> selected;
  std::vector<QColor> colors;
  for (int i = 0; i < series_list_->count(); ++i) {
    auto * item = series_list_->item(i);
    if (item->checkState() != Qt::Checked) {
      continue;
    }
    const int ai = item->data(static_cast<int>(SeriesListItemRole::kAllSeriesIndex)).toInt();
    if (ai < 0 || static_cast<size_t>(ai) >= all.size()) {
      continue;
    }
    const std::string want_id =
      item->data(static_cast<int>(SeriesListItemRole::kSeriesId)).toString().toStdString();
    // Stale list row: `all` order can change while the widget still holds an old index — skip
    // mismatched ids.
    if (TrajectorySeriesManager::seriesId(all[static_cast<size_t>(ai)]) != want_id) {
      continue;
    }
    selected.push_back(all[static_cast<size_t>(ai)]);
    colors.push_back(colorForSeriesListIndex(ai));
  }
  plot_widget_->updatePlot(selected, colors, x_axis, y_axis, readPlotRangeOptionsDefault());
  updateSummary(selected);
}

void TrajectoryKinematicsPanel::updateSummary(const std::vector<TrajectorySeriesData> & plotted)
{
  if (plotted.empty() || plotted.front().points.empty()) {
    metric_velocity_->setText(QStringLiteral("-"));
    metric_accel_->setText(QStringLiteral("-"));
    metric_duration_->setText(QStringLiteral("-"));
    return;
  }
  const auto & back = plotted.front().points.back();
  metric_velocity_->setText(QString::number(back.longitudinal_velocity_mps, 'f', 3) + " m/s");
  metric_accel_->setText(QString::number(back.acceleration_mps2, 'f', 3) + " m/s²");
  metric_duration_->setText(QString::number(back.time_from_start_sec, 'f', 3) + " s");
}

std::string TrajectoryKinematicsPanel::serializeTopics(
  const std::vector<std::pair<std::string, TopicMessageKind>> & v)
{
  std::ostringstream oss;
  for (size_t i = 0; i < v.size(); ++i) {
    if (i > 0) {
      oss << ";";
    }
    oss << v[i].first << "|" << static_cast<int>(v[i].second);
  }
  return oss.str();
}

bool TrajectoryKinematicsPanel::deserializeTopics(
  const std::string & s, std::vector<std::pair<std::string, TopicMessageKind>> * out)
{
  out->clear();
  if (s.empty()) {
    return true;
  }
  std::istringstream iss(s);
  std::string segment;
  while (std::getline(iss, segment, ';')) {
    const auto pos = segment.find('|');
    if (pos == std::string::npos) {
      continue;
    }
    std::string topic = segment.substr(0, pos);
    int kind_i = 0;
    try {
      kind_i = std::stoi(segment.substr(pos + 1));
    } catch (const std::exception &) {
      continue;
    }
    topic = normalizeTopicString(std::move(topic));
    if (topic.empty()) {
      continue;
    }
    const auto kind = static_cast<TopicMessageKind>(kind_i);
    out->emplace_back(topic, kind);
  }
  return true;
}

void TrajectoryKinematicsPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("topics_config", QString::fromStdString(serializeTopics(configured_topics_)));
  config.mapSetValue("x_axis", static_cast<int>(x_axis_combo_->currentData().toInt()));
  config.mapSetValue("y_axis", static_cast<int>(y_axis_combo_->currentData().toInt()));
  QStringList checked_ids;
  for (const auto & kv : series_checked_) {
    if (kv.second) {
      checked_ids.push_back(QString::fromStdString(kv.first));
    }
  }
  checked_ids.sort();
  config.mapSetValue("checked_series", checked_ids.join(QStringLiteral(",")));
}

void TrajectoryKinematicsPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString topics_q;
  if (config.mapGetString("topics_config", &topics_q)) {
    deserializeTopics(topics_q.toStdString(), &configured_topics_);
  }
  int x_axis = static_cast<int>(AxisId::TIME_FROM_START);
  int y_axis = static_cast<int>(AxisId::LONGITUDINAL_VELOCITY);
  config.mapGetInt("x_axis", &x_axis);
  config.mapGetInt("y_axis", &y_axis);
  for (int i = 0; i < x_axis_combo_->count(); ++i) {
    if (x_axis_combo_->itemData(i).toInt() == x_axis) {
      x_axis_combo_->setCurrentIndex(i);
      break;
    }
  }
  for (int i = 0; i < y_axis_combo_->count(); ++i) {
    if (y_axis_combo_->itemData(i).toInt() == y_axis) {
      y_axis_combo_->setCurrentIndex(i);
      break;
    }
  }
  QString checked_q;
  series_checked_.clear();
  if (config.mapGetString("checked_series", &checked_q)) {
    for (const auto & p : checked_q.split(',')) {
      if (p.isEmpty()) {
        continue;
      }
      series_checked_[p.toStdString()] = true;
    }
  }

  applyTopicConfigsToManager();
}

void TrajectoryKinematicsPanel::showEvent(QShowEvent * event)
{
  rviz_common::Panel::showEvent(event);
}

void TrajectoryKinematicsPanel::hideEvent(QHideEvent * event)
{
  rviz_common::Panel::hideEvent(event);
}

}  // namespace autoware::visualization::trajectory_kinematics_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::visualization::trajectory_kinematics_rviz_plugin::TrajectoryKinematicsPanel,
  rviz_common::Panel)
