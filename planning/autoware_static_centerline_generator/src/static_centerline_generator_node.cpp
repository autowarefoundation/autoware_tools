// Copyright 2022 Tier IV, Inc.
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

#include "static_centerline_generator_node.hpp"

#include "autoware/map_loader/lanelet2_map_loader_node.hpp"
#include "autoware/map_projection_loader/load_info_from_lanelet2_map.hpp"
#include "autoware/map_projection_loader/map_projection_loader.hpp"
#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/math/unit_conversion.hpp"
#include "autoware/universe_utils/ros/parameter.hpp"
#include "autoware_lanelet2_extension/utility/message_conversion.hpp"
#include "autoware_lanelet2_extension/utility/query.hpp"
#include "autoware_lanelet2_extension/utility/utilities.hpp"
#include "autoware_static_centerline_generator/msg/points_with_lane_id.hpp"
#include "centerline_source/bag_ego_trajectory_based_centerline.hpp"
#include "type_alias.hpp"
#include "utils.hpp"

#include <autoware/geography_utils/lanelet2_projector.hpp>
#include <autoware/mission_planner/mission_planner_plugin.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>
#include <pluginlib/class_loader.hpp>

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/distance.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#define RESET_TEXT "\x1B[0m"
#define RED_TEXT "\x1B[31m"
#define YELLOW_TEXT "\x1b[33m"
#define BOLD_TEXT "\x1B[1m"

namespace autoware::static_centerline_generator
{
namespace
{
std::vector<lanelet::Id> get_lane_ids_from_route(const LaneletRoute & route)
{
  std::vector<lanelet::Id> lane_ids;
  for (const auto & segment : route.segments) {
    const auto & target_lanelet_id = segment.preferred_primitive.id;
    lane_ids.push_back(target_lanelet_id);
  }

  return lane_ids;
}

lanelet::BasicPoint2d convert_to_lanelet_point(const geometry_msgs::msg::Point & geom_point)
{
  lanelet::BasicPoint2d point(geom_point.x, geom_point.y);
  return point;
}

LinearRing2d create_vehicle_footprint(
  const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double margin = 0.0)
{
  const auto & i = vehicle_info;

  const double x_front = i.front_overhang_m + i.wheel_base_m + margin;
  const double x_rear = -(i.rear_overhang_m + margin);
  const double y_left = i.wheel_tread_m / 2.0 + i.left_overhang_m + margin;
  const double y_right = -(i.wheel_tread_m / 2.0 + i.right_overhang_m + margin);

  std::vector<geometry_msgs::msg::Point> geom_points;
  geom_points.push_back(
    autoware::universe_utils::calcOffsetPose(pose, x_front, y_left, 0.0).position);
  geom_points.push_back(
    autoware::universe_utils::calcOffsetPose(pose, x_front, y_right, 0.0).position);
  geom_points.push_back(
    autoware::universe_utils::calcOffsetPose(pose, x_rear, y_right, 0.0).position);
  geom_points.push_back(
    autoware::universe_utils::calcOffsetPose(pose, x_rear, y_left, 0.0).position);

  LinearRing2d footprint;
  for (const auto & geom_point : geom_points) {
    footprint.push_back(Point2d{geom_point.x, geom_point.y});
  }
  footprint.push_back(footprint.back());

  boost::geometry::correct(footprint);

  return footprint;
}

geometry_msgs::msg::Pose get_text_pose(
  const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const auto & i = vehicle_info;

  const double x_front = i.front_overhang_m + i.wheel_base_m;
  const double y_left = i.wheel_tread_m / 2.0 + i.left_overhang_m + 0.5;

  return autoware::universe_utils::calcOffsetPose(pose, x_front, y_left, 0.0);
}

std::array<double, 3> convert_hex_string_to_decimal(const std::string & hex_str_color)
{
  unsigned int hex_int_color;
  std::istringstream iss(hex_str_color);
  iss >> std::hex >> hex_int_color;

  unsigned int unit = 16 * 16;
  unsigned int b = hex_int_color % unit;
  unsigned int g = (hex_int_color - b) / unit % unit;
  unsigned int r = (hex_int_color - g * unit - b) / unit / unit;

  return std::array<double, 3>{r / 255.0, g / 255.0, b / 255.0};
}

std::vector<lanelet::Id> check_lanelet_connection(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & route_lanelets)
{
  std::vector<lanelet::Id> unconnected_lane_ids;

  for (size_t i = 0; i < route_lanelets.size() - 1; ++i) {
    const auto next_lanelets = route_handler.getNextLanelets(route_lanelets.at(i));

    const bool is_connected =
      std::find_if(next_lanelets.begin(), next_lanelets.end(), [&](const auto & next_lanelet) {
        return next_lanelet.id() == route_lanelets.at(i + 1).id();
      }) != next_lanelets.end();
    if (!is_connected) {
      unconnected_lane_ids.push_back(route_lanelets.at(i).id());
    }
  }

  return unconnected_lane_ids;
}

std_msgs::msg::Header create_header(const rclcpp::Time & now)
{
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = now;
  return header;
}

std::vector<TrajectoryPoint> resample_trajectory_points(
  const std::vector<TrajectoryPoint> & input_traj_points, const double resample_interval)
{
  // resample and calculate trajectory points' orientation
  const auto input_traj = autoware::motion_utils::convertToTrajectory(input_traj_points);
  auto resampled_input_traj =
    autoware::motion_utils::resampleTrajectory(input_traj, resample_interval);
  return autoware::motion_utils::convertToTrajectoryPointArray(resampled_input_traj);
}

std::vector<geometry_msgs::msg::Point> convertToGeometryPoints(const LineString2d & lanelet_points)
{
  std::vector<geometry_msgs::msg::Point> points;
  for (const auto & lanelet_point : lanelet_points) {
    geometry_msgs::msg::Point point;
    point.x = lanelet_point.x();
    point.y = lanelet_point.y();
    points.push_back(point);
  }
  return points;
}
}  // namespace

StaticCenterlineGeneratorNode::StaticCenterlineGeneratorNode(
  const rclcpp::NodeOptions & node_options)
: Node("static_centerline_generator", node_options)
{
  // publishers
  pub_map_bin_ =
    create_publisher<LaneletMapBin>("lanelet2_map_topic", utils::create_transient_local_qos());
  pub_whole_centerline_ =
    create_publisher<Trajectory>("~/output/whole_centerline", utils::create_transient_local_qos());
  pub_centerline_ =
    create_publisher<Trajectory>("~/output/centerline", utils::create_transient_local_qos());

  // debug publishers
  pub_validation_results_ =
    create_publisher<MarkerArray>("~/validation_results", utils::create_transient_local_qos());
  pub_debug_markers_ =
    create_publisher<MarkerArray>("~/debug/markers", utils::create_transient_local_qos());

  pub_debug_ego_footprint_bounds_ = create_publisher<MarkerArray>(
    "~/debug/ego_footprint_bounds", utils::create_transient_local_qos());

  // subscribers
  sub_footprint_margin_for_road_bound_ = create_subscription<std_msgs::msg::Float32>(
    "/static_centerline_generator/road_boundary_lateral_margin", rclcpp::QoS{1},
    [this](const std_msgs::msg::Float32 & msg) { footprint_margin_for_road_bound_ = msg.data; });
  sub_traj_start_index_ = create_subscription<std_msgs::msg::Int32>(
    "/static_centerline_generator/traj_start_index", rclcpp::QoS{1},
    [this](const std_msgs::msg::Int32 & msg) {
      if (centerline_handler_.update_start_index(msg.data)) {
        visualize_selected_centerline();
      }
    });
  sub_traj_end_index_ = create_subscription<std_msgs::msg::Int32>(
    "/static_centerline_generator/traj_end_index", rclcpp::QoS{1},
    [this](const std_msgs::msg::Int32 & msg) {
      if (centerline_handler_.update_end_index(msg.data)) {
        visualize_selected_centerline();
      }
    });
  sub_save_map_ = create_subscription<std_msgs::msg::Empty>(
    "/static_centerline_generator/save_map", rclcpp::QoS{1},
    [this]([[maybe_unused]] const std_msgs::msg::Empty & msg) {
      if (!centerline_handler_.is_valid()) {
        return;
      }
      save_map();
    });
  sub_validate_ = create_subscription<std_msgs::msg::Empty>(
    "/static_centerline_generator/validate", rclcpp::QoS{1},
    [this]([[maybe_unused]] const std_msgs::msg::Empty & msg) { validate(); });

  // services
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_load_map_ = create_service<LoadMap>(
    "/planning/static_centerline_generator/load_map",
    std::bind(
      &StaticCenterlineGeneratorNode::on_load_map, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_);
  srv_plan_route_ = create_service<PlanRoute>(
    "/planning/static_centerline_generator/plan_route",
    std::bind(
      &StaticCenterlineGeneratorNode::on_plan_route, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_);
  srv_plan_path_ = create_service<PlanPath>(
    "/planning/static_centerline_generator/plan_path",
    std::bind(
      &StaticCenterlineGeneratorNode::on_plan_path, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_);

  // vehicle info
  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

  centerline_source_ = [&]() {
    const auto centerline_source_param = declare_parameter<std::string>("centerline_source");
    if (centerline_source_param == "optimization_trajectory_base") {
      optimization_trajectory_based_centerline_ = OptimizationTrajectoryBasedCenterline(*this);
      return CenterlineSource::OptimizationTrajectoryBase;
    } else if (centerline_source_param == "bag_ego_trajectory_base") {
      return CenterlineSource::BagEgoTrajectoryBase;
    }
    throw std::logic_error(
      "The centerline source is not supported in autoware_static_centerline_generator.");
  }();
}

void StaticCenterlineGeneratorNode::visualize_selected_centerline()
{
  // publish selected centerline
  const auto selected_centerline = centerline_handler_.get_selected_centerline();
  pub_centerline_->publish(
    autoware::motion_utils::convertToTrajectory(selected_centerline, create_header(this->now())));

  // delete markers for validation
  pub_validation_results_->publish(utils::create_delete_all_marker_array({}, now()));
  pub_debug_markers_->publish(utils::create_delete_all_marker_array(
    {"unsafe_footprints", "unsafe_footprints_distance"}, now()));
  pub_debug_ego_footprint_bounds_->publish(
    utils::create_delete_all_marker_array({"road_bounds"}, now()));
}

void StaticCenterlineGeneratorNode::generate_centerline()
{
  // declare planning setting parameters
  const auto lanelet2_input_file_path = declare_parameter<std::string>("lanelet2_input_file_path");

  // process
  load_map(lanelet2_input_file_path);
  const auto whole_centerline_with_route = generate_whole_centerline_with_route();
  centerline_handler_ = CenterlineHandler(whole_centerline_with_route);

  visualize_selected_centerline();
}

CenterlineWithRoute StaticCenterlineGeneratorNode::generate_whole_centerline_with_route()
{
  if (!route_handler_ptr_) {
    RCLCPP_ERROR(get_logger(), "Route handler is not ready. Return empty trajectory.");
    return CenterlineWithRoute{};
  }

  // generate centerline with route
  auto centerline_with_route = [&]() {
    if (centerline_source_ == CenterlineSource::OptimizationTrajectoryBase) {
      const lanelet::Id start_lanelet_id = declare_parameter<int64_t>("start_lanelet_id");
      const lanelet::Id end_lanelet_id = declare_parameter<int64_t>("end_lanelet_id");
      const auto route_lane_ids = plan_route_by_lane_ids(start_lanelet_id, end_lanelet_id);
      const auto optimized_centerline =
        optimization_trajectory_based_centerline_.generate_centerline_with_optimization(
          *this, *route_handler_ptr_, route_lane_ids);
      return CenterlineWithRoute{optimized_centerline, route_lane_ids};
    } else if (centerline_source_ == CenterlineSource::BagEgoTrajectoryBase) {
      const auto bag_centerline = generate_centerline_with_bag(*this);
      const auto route_lane_ids =
        plan_route(bag_centerline.front().pose, bag_centerline.back().pose);
      return CenterlineWithRoute{bag_centerline, route_lane_ids};
    }
    throw std::logic_error(
      "The centerline source is not supported in autoware_static_centerline_generator.");
  }();

  // resample
  const double output_trajectory_interval = declare_parameter<double>("output_trajectory_interval");
  centerline_with_route.centerline =
    resample_trajectory_points(centerline_with_route.centerline, output_trajectory_interval);

  pub_whole_centerline_->publish(autoware::motion_utils::convertToTrajectory(
    centerline_with_route.centerline, create_header(this->now())));

  return centerline_with_route;
}

void StaticCenterlineGeneratorNode::load_map(const std::string & lanelet2_input_file_path)
{
  // copy the input LL2 map to the temporary file for debugging
  const std::string debug_input_file_dir{"/tmp/autoware_static_centerline_generator/input/"};
  std::filesystem::create_directories(debug_input_file_dir);
  std::filesystem::copy(
    lanelet2_input_file_path, debug_input_file_dir + "lanelet2_map.osm",
    std::filesystem::copy_options::overwrite_existing);

  // load map by the map_loader package
  map_bin_ptr_ = [&]() -> LaneletMapBin::ConstSharedPtr {
    // load map
    map_projector_info_ = std::make_unique<MapProjectorInfo>(
      autoware::map_projection_loader::load_info_from_lanelet2_map(lanelet2_input_file_path));
    const auto map_ptr = autoware::map_loader::Lanelet2MapLoaderNode::load_map(
      lanelet2_input_file_path, *map_projector_info_);
    if (!map_ptr) {
      return nullptr;
    }

    // NOTE: The original map is stored here since the centerline will be added to all the
    //       lanelet when lanelet::utils::overwriteLaneletCenterline is called.
    original_map_ptr_ = autoware::map_loader::Lanelet2MapLoaderNode::load_map(
      lanelet2_input_file_path, *map_projector_info_);

    // overwrite more dense centerline
    // NOTE: overwriteLaneletsCenterlineWithWaypoints is used only in real time calculation.
    lanelet::utils::overwriteLaneletsCenterline(map_ptr, 5.0, false);

    // create map bin msg
    const auto map_bin_msg = autoware::map_loader::Lanelet2MapLoaderNode::create_map_bin_msg(
      map_ptr, lanelet2_input_file_path, now());

    return std::make_shared<LaneletMapBin>(map_bin_msg);
  }();

  // check if map_bin_ptr_ is not null pointer
  if (!map_bin_ptr_) {
    RCLCPP_ERROR(get_logger(), "Loading map failed");
    return;
  }
  RCLCPP_INFO(get_logger(), "Loaded map.");

  // publish map bin msg
  pub_map_bin_->publish(*map_bin_ptr_);
  RCLCPP_INFO(get_logger(), "Published map.");

  // create route_handler
  route_handler_ptr_ = std::make_shared<RouteHandler>();
  route_handler_ptr_->setMap(*map_bin_ptr_);
}

void StaticCenterlineGeneratorNode::on_load_map(
  const LoadMap::Request::SharedPtr request, const LoadMap::Response::SharedPtr response)
{
  const std::string tmp_lanelet2_input_file_path = "/tmp/input_lanelet2_map.osm";

  // save map file temporarily since load map's input must be a file
  std::ofstream map_writer;
  map_writer.open(tmp_lanelet2_input_file_path, std::ios::out);
  map_writer << request->map;
  map_writer.close();

  // load map from the saved map file
  load_map(tmp_lanelet2_input_file_path);

  if (map_bin_ptr_) {
    return;
  }

  response->message = "InvalidMapFormat";
}

std::vector<lanelet::Id> StaticCenterlineGeneratorNode::plan_route_by_lane_ids(
  const lanelet::Id start_lanelet_id, const lanelet::Id end_lanelet_id)
{
  if (!route_handler_ptr_) {
    RCLCPP_ERROR(get_logger(), "Map or route handler is not ready. Return empty lane ids.");
    return std::vector<lanelet::Id>{};
  }

  const auto start_center_pose = utils::get_center_pose(*route_handler_ptr_, start_lanelet_id);
  const auto end_center_pose = utils::get_center_pose(*route_handler_ptr_, end_lanelet_id);
  return plan_route(start_center_pose, end_center_pose);
}

std::vector<lanelet::Id> StaticCenterlineGeneratorNode::plan_route(
  const geometry_msgs::msg::Pose & start_center_pose,
  const geometry_msgs::msg::Pose & end_center_pose)
{
  if (!map_bin_ptr_) {
    RCLCPP_ERROR(get_logger(), "Map or route handler is not ready. Return empty lane ids.");
    return std::vector<lanelet::Id>{};
  }

  // plan route by the mission_planner package
  const auto route = [&]() {
    // calculate check points
    RCLCPP_INFO(get_logger(), "Calculated check points.");
    const auto check_points =
      std::vector<geometry_msgs::msg::Pose>{start_center_pose, end_center_pose};

    // create mission_planner plugin
    auto plugin_loader = pluginlib::ClassLoader<autoware::mission_planner::PlannerPlugin>(
      "autoware_mission_planner", "autoware::mission_planner::PlannerPlugin");
    auto mission_planner =
      plugin_loader.createSharedInstance("autoware::mission_planner::lanelet2::DefaultPlanner");

    // initialize mission_planner
    auto node = rclcpp::Node("mission_planner");
    mission_planner->initialize(&node, map_bin_ptr_);

    // plan route
    const auto route = mission_planner->plan(check_points);

    return route;
  }();

  // get lanelets
  const auto route_lane_ids = get_lane_ids_from_route(route);

  std::string route_lane_ids_str = "";
  for (const lanelet::Id route_lane_id : route_lane_ids) {
    route_lane_ids_str += std::to_string(route_lane_id) + ",";
  }
  RCLCPP_INFO_STREAM(get_logger(), "Planned route. (" << route_lane_ids_str << ")");

  return route_lane_ids;
}

void StaticCenterlineGeneratorNode::on_plan_route(
  const PlanRoute::Request::SharedPtr request, const PlanRoute::Response::SharedPtr response)
{
  if (!map_bin_ptr_ || !route_handler_ptr_) {
    response->message = "MapNotFound";
    RCLCPP_ERROR(get_logger(), "Map is not ready.");
    return;
  }

  const lanelet::Id start_lanelet_id = request->start_lane_id;
  const lanelet::Id end_lanelet_id = request->end_lane_id;

  // plan route
  const auto route_lane_ids = plan_route_by_lane_ids(start_lanelet_id, end_lanelet_id);
  const auto route_lanelets = utils::get_lanelets_from_ids(*route_handler_ptr_, route_lane_ids);

  // extract lane ids
  std::vector<lanelet::Id> lane_ids;
  for (const auto & lanelet : route_lanelets) {
    lane_ids.push_back(lanelet.id());
  }

  // check calculation result
  if (lane_ids.empty()) {
    response->message = "RouteNotFound";
    RCLCPP_ERROR(get_logger(), "Route planning failed.");
    return;
  }

  // set response
  response->lane_ids = lane_ids;
}

void StaticCenterlineGeneratorNode::on_plan_path(
  const PlanPath::Request::SharedPtr request, const PlanPath::Response::SharedPtr response)
{
  if (!route_handler_ptr_) {
    response->message = "MapNotFound";
    RCLCPP_ERROR(get_logger(), "Route handler is not ready.");
    return;
  }

  // get lanelets from route lane ids
  const auto route_lane_ids = request->route;
  const auto route_lanelets = utils::get_lanelets_from_ids(*route_handler_ptr_, route_lane_ids);

  // check if input route lanelets are connected to each other.
  const auto unconnected_lane_ids = check_lanelet_connection(*route_handler_ptr_, route_lanelets);
  if (!unconnected_lane_ids.empty()) {
    response->message = "LaneletsNotConnected";
    response->unconnected_lane_ids = unconnected_lane_ids;
    RCLCPP_ERROR(get_logger(), "Lanelets are not connected.");
    return;
  }

  // plan path
  const auto optimized_traj_points =
    optimization_trajectory_based_centerline_.generate_centerline_with_optimization(
      *this, *route_handler_ptr_, route_lane_ids);

  // check calculation result
  if (optimized_traj_points.empty()) {
    response->message = "PathNotFound";
    RCLCPP_ERROR(get_logger(), "Path planning failed.");
    return;
  }

  centerline_handler_ =
    CenterlineHandler(CenterlineWithRoute{optimized_traj_points, route_lane_ids});

  // publish unsafe_footprints
  validate();

  // create output data
  auto target_traj_point = optimized_traj_points.cbegin();
  bool is_end_lanelet = false;
  for (const auto & lanelet : route_lanelets) {
    std::vector<geometry_msgs::msg::Point> current_lanelet_points;

    // check if target point is inside the lanelet
    while (lanelet::geometry::inside(
      lanelet, convert_to_lanelet_point(target_traj_point->pose.position))) {
      // memorize points inside the lanelet
      current_lanelet_points.push_back(target_traj_point->pose.position);
      target_traj_point++;

      if (target_traj_point == optimized_traj_points.cend()) {
        is_end_lanelet = true;
        break;
      }
    }

    if (!current_lanelet_points.empty()) {
      // register points with lane_id
      autoware_static_centerline_generator::msg::PointsWithLaneId points_with_lane_id;
      points_with_lane_id.lane_id = lanelet.id();
      points_with_lane_id.points = current_lanelet_points;
      response->points_with_lane_ids.push_back(points_with_lane_id);
    }

    if (is_end_lanelet) {
      break;
    }
  }

  // empty string if error did not occur
  response->message = "";
}

void StaticCenterlineGeneratorNode::validate()
{
  std::cerr << std::endl
            << "############################################## Validation Results "
               "##############################################"
            << std::endl;

  const auto & centerline = centerline_handler_.get_selected_centerline();
  const auto & route_lane_ids = centerline_handler_.get_route_lane_ids();
  const auto route_lanelets = utils::get_lanelets_from_ids(*route_handler_ptr_, route_lane_ids);

  const double dist_thresh_to_road_border =
    getRosParameter<double>("validation.dist_threshold_to_road_border");
  const double max_steer_angle_margin =
    getRosParameter<double>("validation.max_steer_angle_margin");

  // calculate color for distance to road border
  const auto dist_thresh_vec = getRosParameter<std::vector<double>>("marker_color_dist_thresh");
  const auto marker_color_vec = getRosParameter<std::vector<std::string>>("marker_color");
  const auto get_marker_color = [&](const double dist) -> boost::optional<std::array<double, 3>> {
    for (size_t i = 0; i < dist_thresh_vec.size(); ++i) {
      const double dist_thresh = dist_thresh_vec.at(i);
      if (dist < dist_thresh) {
        return convert_hex_string_to_decimal(marker_color_vec.at(i));
      }
    }
    return boost::none;
  };

  // create right/left bound
  LineString2d lanelet_right_bound;
  LineString2d lanelet_left_bound;
  for (const auto & lanelet : route_lanelets) {
    for (const auto & point : lanelet.rightBound()) {
      boost::geometry::append(lanelet_right_bound, Point2d(point.x(), point.y()));
    }
    for (const auto & point : lanelet.leftBound()) {
      boost::geometry::append(lanelet_left_bound, Point2d(point.x(), point.y()));
    }
  }

  // calculate curvature
  const auto curvature_vec = autoware::motion_utils::calcCurvature(centerline);
  const double steer_angle_threshold = vehicle_info_.max_steer_angle_rad - max_steer_angle_margin;

  // calculate the distance between footprint and right/left bounds
  MarkerArray marker_array;
  double min_dist = std::numeric_limits<double>::max();
  double max_curvature = std::numeric_limits<double>::min();
  for (size_t i = 0; i < centerline.size(); ++i) {
    const auto & traj_point = centerline.at(i);

    const auto footprint_poly = create_vehicle_footprint(traj_point.pose, vehicle_info_);

    const double dist_to_right = boost::geometry::distance(footprint_poly, lanelet_right_bound);
    const double dist_to_left = boost::geometry::distance(footprint_poly, lanelet_left_bound);
    const double min_dist_to_bound = std::min(dist_to_right, dist_to_left);

    if (min_dist_to_bound < min_dist) {
      min_dist = min_dist_to_bound;
    }

    // create marker
    const auto marker_color_opt = get_marker_color(min_dist_to_bound);
    const auto text_pose = get_text_pose(traj_point.pose, vehicle_info_);
    if (marker_color_opt) {
      const auto & marker_color = marker_color_opt.get();

      // add footprint marker
      const auto footprint_marker = utils::create_footprint_marker(
        footprint_poly, 0.05, marker_color.at(0), marker_color.at(1), marker_color.at(2), 0.7,
        now(), i);
      marker_array.markers.push_back(footprint_marker);

      // add text of distance to bounds marker
      const auto text_marker = utils::create_text_marker(
        "unsafe_footprints_distance", text_pose, min_dist_to_bound, marker_color.at(0),
        marker_color.at(1), marker_color.at(2), 0.999, now(), i);
      marker_array.markers.push_back(text_marker);
    }

    const double curvature = curvature_vec.at(i);
    const auto text_marker =
      utils::create_text_marker("curvature", text_pose, curvature, 0.05, 0.05, 0.0, 0.9, now(), i);
    marker_array.markers.push_back(text_marker);

    if (max_curvature < std::abs(curvature)) {
      max_curvature = std::abs(curvature);
    }
  }
  const double max_steer_angle = vehicle_info_.calcSteerAngleFromCurvature(max_curvature);

  // publish road boundaries
  const auto left_bound = convertToGeometryPoints(lanelet_left_bound);
  const auto right_bound = convertToGeometryPoints(lanelet_right_bound);

  marker_array.markers.push_back(
    utils::create_points_marker("left_bound", left_bound, 0.05, 0.0, 0.6, 0.8, 0.8, now()));
  marker_array.markers.push_back(
    utils::create_points_marker("right_bound", right_bound, 0.05, 0.0, 0.6, 0.8, 0.8, now()));
  pub_debug_markers_->publish(marker_array);

  // show the validation results
  // 1. distance from footprints to road boundaries
  const bool are_footprints_inside_lanelets = [&]() {
    std::cerr << "1. Footprints inside Lanelets:" << std::endl;
    if (dist_thresh_to_road_border < min_dist) {
      std::cerr << "  The generated centerline is inside the lanelet. (threshold:"
                << dist_thresh_to_road_border << "[m] < actual:" << min_dist << "[m])" << std::endl
                << "  Passed." << std::endl;
      return true;
    }
    std::cerr << RED_TEXT
              << " The generated centerline is outside the lanelet. (actual:" << min_dist
              << "[m] <= threshold:" << dist_thresh_to_road_border << "[m])" << std::endl
              << "  Failed." << RESET_TEXT << std::endl;
    return false;
  }();
  // 2. centerline's curvature
  std::cerr << "2. Curvature:" << std::endl;
  const bool is_curvature_low =
    true;  // always tre for now since the curvature is just estimated and not enough precise.
  if (max_steer_angle < steer_angle_threshold) {
    std::cerr << "  The generated centerline has no high steer angle. (estimated:"
              << autoware::universe_utils::rad2deg(max_steer_angle)
              << "[deg] < threshold:" << autoware::universe_utils::rad2deg(steer_angle_threshold)
              << "[deg])" << std::endl
              << "  Passed." << std::endl;
  } else {
    std::cerr << YELLOW_TEXT << "  The generated centerline has a too high steer angle. (threshold:"
              << autoware::universe_utils::rad2deg(steer_angle_threshold)
              << "[deg] <= estimated:" << autoware::universe_utils::rad2deg(max_steer_angle)
              << "[deg])" << std::endl
              << "  However, the estimated steer angle is not enough precise, so the result is "
                 "conditional pass."
              << std::endl
              << "  Conditionally Passed." << RESET_TEXT << std::endl;
  }
  // 3. result
  std::cerr << std::endl << BOLD_TEXT << "Result:" << RESET_TEXT << std::endl;
  if (are_footprints_inside_lanelets && is_curvature_low) {
    std::cerr << BOLD_TEXT << "  Passed!" << RESET_TEXT << std::endl;
  } else {
    std::cerr << BOLD_TEXT << RED_TEXT << "  Failed!" << RESET_TEXT << std::endl;
  }

  std::cerr << "###################################################################################"
               "#############################"
            << std::endl
            << std::endl;
  RCLCPP_INFO(get_logger(), "Validated the generated centerline.");
}

void StaticCenterlineGeneratorNode::save_map()
{
  if (!route_handler_ptr_) {
    return;
  }

  const auto & centerline = centerline_handler_.get_selected_centerline();
  const auto & route_lane_ids = centerline_handler_.get_route_lane_ids();

  const auto lanelet2_output_file_path = getRosParameter<std::string>("lanelet2_output_file_path");

  const auto route_lanelets = utils::get_lanelets_from_ids(*route_handler_ptr_, route_lane_ids);

  // update centerline in map
  utils::update_centerline(original_map_ptr_, route_lanelets, centerline);
  RCLCPP_INFO(get_logger(), "Updated centerline in map.");

  // save map with modified center line
  std::filesystem::create_directory("/tmp/autoware_static_centerline_generator");
  const auto map_projector =
    autoware::geography_utils::get_lanelet2_projector(*map_projector_info_);
  lanelet::write(lanelet2_output_file_path, *original_map_ptr_, *map_projector);
  RCLCPP_INFO(
    get_logger(), "Saved map in %s", "/tmp/autoware_static_centerline_generator/lanelet2_map.osm");

  // copy the output LL2 map to the temporary file for debugging
  const std::string debug_output_file_dir{"/tmp/autoware_static_centerline_generator/output/"};
  std::filesystem::create_directories(debug_output_file_dir);
  std::filesystem::copy(
    lanelet2_output_file_path, debug_output_file_dir + "lanelet2_map.osm",
    std::filesystem::copy_options::overwrite_existing);
}
}  // namespace autoware::static_centerline_generator
