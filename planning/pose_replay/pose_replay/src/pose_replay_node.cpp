#include "pose_replay_interfaces/msg/uuid_route.hpp"
#include "pose_replay_interfaces/msg/uuid_routes.hpp"
#include "pose_replay_interfaces/srv/delete_route.hpp"
#include "pose_replay_interfaces/srv/get_uuid_routes.hpp"
#include "pose_replay_interfaces/srv/set_name.hpp"
#include "pose_replay_interfaces/srv/set_route.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_adapi_v1_msgs/msg/route.hpp"
#include "autoware_adapi_v1_msgs/srv/clear_route.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include <uuid/uuid.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using autoware_adapi_v1_msgs::msg::to_yaml;

const std::string save_filepath = "output.yaml";
using adapi_route = autoware_adapi_v1_msgs::msg::Route;

struct NamedRoute
{
  std::string name;
  adapi_route route;
};

// add name in? struct
// typedef std::unordered_map<std::string, autoware_adapi_v1_msgs::msg::Route> uuid_route_map;
using uuid_route_map = std::unordered_map<std::string, NamedRoute>;

class PoseReplayNode : public rclcpp::Node
{
public:
  PoseReplayNode(const rclcpp::NodeOptions & options) : Node("pose_replay_node", options)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "pose_replay_node running...");

    route_set_subscription_ = this->create_subscription<adapi_route>(
      "/api/routing/route", 10, [this](const adapi_route & msg) { route_set_callback(msg); });

    initial_pose_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    goal_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/planning/mission_planning/goal", 10);
    sync_notif_publisher_ =
      this->create_publisher<std_msgs::msg::String>("/pose_replay/update", 10);

    get_routes_service_ = this->create_service<pose_replay_interfaces::srv::GetUuidRoutes>(
      "/pose_replay/get_routes_service",
      [this](
        const std::shared_ptr<pose_replay_interfaces::srv::GetUuidRoutes::Request> & request,
        const std::shared_ptr<pose_replay_interfaces::srv::GetUuidRoutes::Response> & response) {
        get_routes_service(request, response);
      });
    set_route_service_ = this->create_service<pose_replay_interfaces::srv::SetRoute>(
      "/pose_replay/set_route_service",
      [this](
        const std::shared_ptr<pose_replay_interfaces::srv::SetRoute::Request> & request,
        const std::shared_ptr<pose_replay_interfaces::srv::SetRoute::Response> & response) {
        set_route_service(request, response);
      });

    delete_route_service_ = this->create_service<pose_replay_interfaces::srv::DeleteRoute>(
      "/pose_replay/delete_route_service",
      [this](
        const std::shared_ptr<pose_replay_interfaces::srv::DeleteRoute::Request> & request,
        const std::shared_ptr<pose_replay_interfaces::srv::DeleteRoute::Response> & response) {
        delete_route_service(request, response);
      });

    set_name_service_ = this->create_service<pose_replay_interfaces::srv::SetName>(
      "/pose_replay/set_name_service",
      [this](
        const std::shared_ptr<pose_replay_interfaces::srv::SetName::Request> & request,
        const std::shared_ptr<pose_replay_interfaces::srv::SetName::Response> & response) {
        set_name_service(request, response);
      });

    read_routes(save_filepath);
  };

  void get_routes_service(
    const std::shared_ptr<pose_replay_interfaces::srv::GetUuidRoutes::Request> & request,
    const std::shared_ptr<pose_replay_interfaces::srv::GetUuidRoutes::Response> & response)
  {
    pose_replay_interfaces::msg::UuidRoutes response_routes;
    auto requested_route_uuids = request->uuids;
    if (requested_route_uuids.size() == 0) {
      for (auto & [key, value] : routes) {
        pose_replay_interfaces::msg::UuidRoute uuid_route;
        uuid_route.uuid = key;
        // uuid_route.start = value.route.data[0].start;
        // uuid_route.goal = value.data[0].goal;
        uuid_route.name = value.name;
        response_routes.routes.push_back(uuid_route);
      }
    } else {
      for (auto & uuid : requested_route_uuids) {
        if (routes.count(uuid)) {
          auto route = routes[uuid];
          pose_replay_interfaces::msg::UuidRoute uuid_route;
          uuid_route.uuid = uuid;
          // uuid_route.start = route.data[0].start;
          // uuid_route.goal = route.data[0].goal;
          uuid_route.name = route.name;
          response_routes.routes.push_back(uuid_route);
        } else {
          RCLCPP_INFO(
            this->get_logger(), "No route found in save file with uuid %s.", uuid.c_str());
        }
      }
    }
    response->routes = response_routes;
  }

  void set_route_service(
    const std::shared_ptr<pose_replay_interfaces::srv::SetRoute::Request> & request,
    const std::shared_ptr<pose_replay_interfaces::srv::SetRoute::Response> & response)
  {
    std::string set_route_uuid = request->uuid;
    if (set_route_uuid.size() == 0) {
      response->uuid = "";
      return;
    }
    set_route(set_route_uuid);
    response->uuid = set_route_uuid;
  }

  void delete_route_service(
    const std::shared_ptr<pose_replay_interfaces::srv::DeleteRoute::Request> & request,
    const std::shared_ptr<pose_replay_interfaces::srv::DeleteRoute::Response> & response)
  {
    std::string delete_route_uuid = request->uuid;
    if (delete_route_uuid.size() == 0) {
      response->uuid = "";
      return;
    }
    delete_route(delete_route_uuid);
    response->uuid = delete_route_uuid;
  }

  void set_name_service(
    const std::shared_ptr<pose_replay_interfaces::srv::SetName::Request> & request,
    const std::shared_ptr<pose_replay_interfaces::srv::SetName::Response> & response)
  {
    std::string set_name_route_uuid = request->uuid;
    std::string new_name = request->name;
    if (set_name_route_uuid.size() == 0 || new_name.size() == 0) {
      response->uuid = "";
      response->name = "";
      return;
    }
    set_name(set_name_route_uuid, new_name);
    response->uuid = set_name_route_uuid;
    response->name = new_name;
  }

  void set_name(const std::string & uuid, const std::string & new_name)
  {
    std::ifstream yaml_file(save_filepath);

    // Read and save to function scope
    std::vector<YAML::Node> docs = YAML::LoadAll(yaml_file);
    yaml_file.close();

    // Delete from function scope
    for (auto p = docs.begin(); p != docs.end();) {
      if (!*p || !(*p)["uuid"]) {
        ++p;
        continue;
      }
      if ((*p)["uuid"].as<std::string>() == uuid) {
        (*p)["name"] = new_name;
        RCLCPP_INFO(
          this->get_logger(), "Name changed to \"%s\" for route with uuid: %s.", new_name.c_str(),
          uuid.c_str());
        p = docs.end();
      } else {
        ++p;
      }
    }

    // Wipe file and re-write to save
    clear_file(save_filepath);
    for (auto & d : docs) {
      write_route(save_filepath, d, true);
    }

    // Update local node to sync
    read_routes(save_filepath);

    std_msgs::msg::String stdmsg;
    stdmsg.data = "update";
    sync_notif_publisher_->publish(stdmsg);
  }

  void delete_route(const std::string & uuid)
  {
    std::ifstream yaml_file(save_filepath);

    // Read and save to function scope
    std::vector<YAML::Node> docs = YAML::LoadAll(yaml_file);
    yaml_file.close();

    // Delete from function scope
    for (auto p = docs.begin(); p != docs.end();) {
      if (!*p || !(*p)["uuid"]) {
        ++p;
        continue;
      }
      if ((*p)["uuid"].as<std::string>() == uuid) {
        p = docs.erase(p);
        RCLCPP_INFO(this->get_logger(), "Deleted route with uuid: %s.", uuid.c_str());
      } else {
        ++p;
      }
    }

    // Wipe file and re-write to save
    clear_file(save_filepath);
    for (auto & d : docs) {
      write_route(save_filepath, d, true);
    }

    // Update local node to sync
    read_routes(save_filepath);

    std_msgs::msg::String stdmsg;
    stdmsg.data = "update";
    sync_notif_publisher_->publish(stdmsg);
  }

  void clear_file(const std::string & filepath)
  {
    std::ofstream o;
    o.open(filepath, std::ios::trunc);
  }

  void clear_routes() { routes = {}; }

  template <typename T>
  void write_route(const std::string & filepath, T & value, bool append = true)
  {
    std::ofstream o;
    if (append) {
      o.open(filepath, std::ios::app);
    } else {
      o.open(filepath, std::ios::trunc);
    }

    if (!o.is_open()) throw std::runtime_error("Cannot open file: " + filepath);
    o << "---\n" << value << "\n";
    o.close();
  }

  auto read_yaml_to_node_vector(const std::string & filepath) -> std::vector<YAML::Node>
  {
    std::ifstream yaml_file(filepath);
    std::vector<YAML::Node> docs = YAML::LoadAll(yaml_file);
    if (docs.empty()) return {};
    return docs;
  }

  void read_routes(const std::string & filepath)
  {
    std::vector<YAML::Node> docs = read_yaml_to_node_vector(filepath);
    clear_routes();
    for (auto & doc : docs) {
      append_route(doc);
    }
  }

  void append_route(const YAML::Node & node)
  {
    uuid_route_map new_map = yaml_to_map(node);
    routes.insert(new_map.begin(), new_map.end());
  }

  auto route_yaml_to_hash(const std::string & yaml_str) -> std::string
  {
    YAML::Node node = YAML::Load(yaml_str);

    // Timestamps, Z-values, and orientation tend to differ between manually setting the route via
    // GUI and publishing to topics. Removed to not influence hash
    node.remove("header");
    node["data"][0]["start"]["position"].remove("z");
    node["data"][0]["goal"]["position"].remove("z");
    node["data"][0]["start"].remove("orientation");
    node["data"][0]["goal"].remove("orientation");
    std::string yaml_str_to_hash = YAML::Dump(node);
    std::string hash_uuid = std::to_string(std::hash<std::string>{}(yaml_str_to_hash));
    return hash_uuid;
  }

  auto prepend_uuid_name(const std::string & yaml_str) -> std::string
  {
    std::string hash_uuid = route_yaml_to_hash(yaml_str);
    std::ostringstream oss;
    oss << "name: \"New route"
        << "\"\n";
    oss << "uuid: \"" << hash_uuid << "\"\n";
    oss << yaml_str;
    std::string new_str = oss.str();
    return new_str;
  }

  // TODO: Fix route duplication when setting route from save file using rviz panel.
  void route_set_callback(const adapi_route & msg)
  {
    if (msg.data.empty()) return;

    std::string yaml_str = autoware_adapi_v1_msgs::msg::to_yaml(msg);

    // Removes duplicate routes by comparing hash signatures
    std::string hash_uuid_str = route_yaml_to_hash(yaml_str);
    if (routes.count(hash_uuid_str)) return;

    auto uuid_yaml_str = prepend_uuid_name(yaml_str);
    write_route(save_filepath, uuid_yaml_str, true);
    RCLCPP_INFO(this->get_logger(), "Route written to %s.", save_filepath.c_str());
    read_routes(save_filepath);

    std_msgs::msg::String stdmsg;
    stdmsg.data = "update";
    sync_notif_publisher_->publish(stdmsg);
  }

  void set_route(const std::string & uuid)
  {
    try {
      geometry_msgs::msg::PoseWithCovarianceStamped initialmsg;

      initialmsg.header.frame_id = routes.at(uuid).route.header.frame_id;

      initialmsg.pose.pose.position.x = routes.at(uuid).route.data[0].start.position.x;
      initialmsg.pose.pose.position.y = routes.at(uuid).route.data[0].start.position.y;
      initialmsg.pose.pose.position.z = routes.at(uuid).route.data[0].start.position.z;

      initialmsg.pose.pose.orientation.x = routes.at(uuid).route.data[0].start.orientation.x;
      initialmsg.pose.pose.orientation.y = routes.at(uuid).route.data[0].start.orientation.y;
      initialmsg.pose.pose.orientation.z = routes.at(uuid).route.data[0].start.orientation.z;
      initialmsg.pose.pose.orientation.w = routes.at(uuid).route.data[0].start.orientation.w;

      geometry_msgs::msg::PoseStamped goalmsg;

      goalmsg.header.frame_id = routes.at(uuid).route.header.frame_id;

      goalmsg.pose.position.x = routes.at(uuid).route.data[0].goal.position.x;
      goalmsg.pose.position.y = routes.at(uuid).route.data[0].goal.position.y;
      goalmsg.pose.position.z = routes.at(uuid).route.data[0].goal.position.z;

      goalmsg.pose.orientation.x = routes.at(uuid).route.data[0].goal.orientation.x;
      goalmsg.pose.orientation.y = routes.at(uuid).route.data[0].goal.orientation.y;
      goalmsg.pose.orientation.z = routes.at(uuid).route.data[0].goal.orientation.z;
      goalmsg.pose.orientation.w = routes.at(uuid).route.data[0].goal.orientation.w;

      initial_pose_publisher_->publish(initialmsg);
      goal_pose_publisher_->publish(goalmsg);

    } catch (const YAML::BadFile & e) {
      RCLCPP_INFO(this->get_logger(), "Save file not found at path: %s.", save_filepath.c_str());
    } catch (const YAML::BadConversion & e) {
      RCLCPP_INFO(this->get_logger(), "Bad conversion: %s", e.what());
    }
  }

  auto yaml_to_map(YAML::Node root) -> uuid_route_map
  {
    autoware_adapi_v1_msgs::msg::Route msg;

    // Parse header
    auto header_node = root["header"];
    msg.header.stamp.sec = header_node["stamp"]["sec"].as<int32_t>();
    msg.header.stamp.nanosec = header_node["stamp"]["nanosec"].as<uint32_t>();
    msg.header.frame_id = header_node["frame_id"].as<std::string>();

    // Parse data (array, max 1)
    auto data_node = root["data"];
    if (data_node && data_node.IsSequence() && !data_node.IsNull()) {
      for (const auto & route_node : data_node) {
        autoware_adapi_v1_msgs::msg::RouteData route_data;

        // Start pose
        route_data.start.position.x = route_node["start"]["position"]["x"].as<double>();
        route_data.start.position.y = route_node["start"]["position"]["y"].as<double>();
        route_data.start.position.z = route_node["start"]["position"]["z"].as<double>();

        route_data.start.orientation.x = route_node["start"]["orientation"]["x"].as<double>();
        route_data.start.orientation.y = route_node["start"]["orientation"]["y"].as<double>();
        route_data.start.orientation.z = route_node["start"]["orientation"]["z"].as<double>();
        route_data.start.orientation.w = route_node["start"]["orientation"]["w"].as<double>();

        // Goal pose
        route_data.goal.position.x = route_node["goal"]["position"]["x"].as<double>();
        route_data.goal.position.y = route_node["goal"]["position"]["y"].as<double>();
        route_data.goal.position.z = route_node["goal"]["position"]["z"].as<double>();

        route_data.goal.orientation.x = route_node["goal"]["orientation"]["x"].as<double>();
        route_data.goal.orientation.y = route_node["goal"]["orientation"]["y"].as<double>();
        route_data.goal.orientation.z = route_node["goal"]["orientation"]["z"].as<double>();
        route_data.goal.orientation.w = route_node["goal"]["orientation"]["w"].as<double>();

        // Segments
        auto segments_node = route_node["segments"];
        if (segments_node && segments_node.IsSequence()) {
          for (const auto & seg_node : segments_node) {
            autoware_adapi_v1_msgs::msg::RouteSegment seg;

            // Preferred
            seg.preferred.id = seg_node["preferred"]["id"].as<int64_t>();
            seg.preferred.type = seg_node["preferred"]["type"].as<std::string>();

            // Alternatives
            auto alt_node = seg_node["alternatives"];
            if (alt_node && alt_node.IsSequence()) {
              for (const auto & alt : alt_node) {
                autoware_adapi_v1_msgs::msg::RoutePrimitive alt_prim;
                alt_prim.id = alt["id"].as<int64_t>();
                alt_prim.type = alt["type"].as<std::string>();
                seg.alternatives.push_back(alt_prim);
              }
            }

            route_data.segments.push_back(seg);
          }
        }

        msg.data.push_back(route_data);
      }
    }

    // std::unordered_map<std::string, autoware_adapi_v1_msgs::msg::Route> map_obj =
    // {{root["uuid"].as<std::string>(), msg}};

    uuid_route_map map_obj = {
      {root["uuid"].as<std::string>(), {root["name"].as<std::string>(), msg}}

    };
    return map_obj;
  }

  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::Route>::SharedPtr route_set_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sync_notif_publisher_;
  rclcpp::Service<pose_replay_interfaces::srv::GetUuidRoutes>::SharedPtr get_routes_service_;
  rclcpp::Service<pose_replay_interfaces::srv::SetRoute>::SharedPtr set_route_service_;
  rclcpp::Service<pose_replay_interfaces::srv::DeleteRoute>::SharedPtr delete_route_service_;
  rclcpp::Service<pose_replay_interfaces::srv::SetName>::SharedPtr set_name_service_;

  uuid_route_map routes;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PoseReplayNode)
