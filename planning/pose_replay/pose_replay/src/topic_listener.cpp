#include <memory>
#include <functional>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp" 
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "autoware_adapi_v1_msgs/msg/route.hpp"
#include "autoware_adapi_v1_msgs/srv/clear_route.hpp"

#include "pose_replay_interfaces/msg/uuid_route.hpp"
#include "pose_replay_interfaces/msg/uuid_routes.hpp"
#include "pose_replay_interfaces/srv/get_uuid_route.hpp"
#include "pose_replay_interfaces/srv/get_uuid_routes.hpp"
#include "pose_replay_interfaces/srv/set_route.hpp"
#include "pose_replay_interfaces/srv/delete_route.hpp"

#include <chrono>

#include <yaml-cpp/yaml.h>
#include <uuid/uuid.h>

using namespace std::literals::chrono_literals;
using autoware_adapi_v1_msgs::msg::to_yaml;

const std::string save_filepath = "output.yaml";

typedef autoware_adapi_v1_msgs::msg::Route adapi_route;
typedef std::unordered_map<std::string, autoware_adapi_v1_msgs::msg::Route> uuid_route_map;

class TopicListener : public rclcpp::Node {

	public:
	TopicListener() : Node("topic_listener") {		
		RCLCPP_INFO_STREAM(this->get_logger(), "Listener online."); 
		
		route_set_subscription_ = this->create_subscription<adapi_route>(
			"/api/routing/route", 10, std::bind(&TopicListener::route_set_callback, this, std::placeholders::_1));
		
		initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
		goal_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal", 10);				
	
	////////	
		service_ = this->create_service<pose_replay_interfaces::srv::GetUuidRoute>(
			"get_route_service", std::bind(&TopicListener::get_route_service, this, std::placeholders::_1, std::placeholders::_2));
	////////
		all_routes_service_ = this->create_service<pose_replay_interfaces::srv::GetUuidRoutes>(
			"get_routes_service", std::bind(&TopicListener::get_routes_service, this, std::placeholders::_1, std::placeholders::_2));
	////////
		 set_route_service_ = this->create_service<pose_replay_interfaces::srv::SetRoute>(
			"set_route_service", std::bind(&TopicListener::set_route_service, this, std::placeholders::_1, std::placeholders::_2));
	////////
		delete_route_service_ = this->create_service<pose_replay_interfaces::srv::DeleteRoute>(
			"delete_route_service", std::bind(&TopicListener::delete_route_service, this, std::placeholders::_1, std::placeholders::_2));

	//	timer_ = this->create_wall_timer(5s, std::bind(&TopicListener::timer_callback, this));
	//	client_ = this->create_client<autoware_adapi_v1_msgs::srv::ClearRoute>("/api/routing/clear_route");	
		read_routes(save_filepath);
	//	test_write_to_history();
	//	delete_route("ba91685c-71bd-4254-a2e1-8d560a011d72");

	};


	void timer_callback()
	{
		set_route("1c51ec25-f14c-4ce2-bd41-a411fa781917");
	}
	
	void delete_route_service(const std::shared_ptr<pose_replay_interfaces::srv::DeleteRoute::Request> request,
			std::shared_ptr<pose_replay_interfaces::srv::DeleteRoute::Response> response){
		std::string requestuuid = request->uuid;
		if(requestuuid.size() == 0){
				return;
		}
		delete_route(requestuuid);	
		response->uuid = requestuuid;
	}

	void set_route_service(const std::shared_ptr<pose_replay_interfaces::srv::SetRoute::Request> request,
			std::shared_ptr<pose_replay_interfaces::srv::SetRoute::Response> response){

		std::string requestuuid = request->uuid;
		set_route(requestuuid);
		// reply with uuid if successful, empty if not
		response->uuid = requestuuid;	
	}
	
	void get_routes_service(const std::shared_ptr<pose_replay_interfaces::srv::GetUuidRoutes::Request> request,
			std::shared_ptr<pose_replay_interfaces::srv::GetUuidRoutes::Response> response){
		
		pose_replay_interfaces::msg::UuidRoutes responseroutes;
	
		auto uuids = request->uuids;
		if(uuids.size() == 0){
			for(auto [key, value] : routes){
				pose_replay_interfaces::msg::UuidRoute ur;
				ur.uuid = key;
				ur.start = value.data[0].start;
				ur.end = value.data[0].goal;
				responseroutes.routes.push_back(ur);
			}
		}

		response->routes = responseroutes;

	}
	
	void get_route_service(const std::shared_ptr<pose_replay_interfaces::srv::GetUuidRoute::Request> request,
			std::shared_ptr<pose_replay_interfaces::srv::GetUuidRoute::Response> response){
		
	//	if((request->uuid).size() == 0 || routes.count(response->uuid) == 0){
	//		return;
	//	}
		
		response->uuid = request->uuid;

		// needs checks and end -> goal rename
		response->start = routes.at(request->uuid).data[0].start;
		response->end = routes.at(request->uuid).data[0].goal;	
	}	
		
	void delete_route(std::string uuid)
	{
		std::ifstream yaml_file(save_filepath);
		
		// Read and save to function scope
		std::vector<YAML::Node> docs = YAML::LoadAll(yaml_file);
		
		// Delete from function scope
		for(std::vector<YAML::Node>::iterator p = docs.begin(); p != docs.end(); ){
			if(!*p || !(*p)["uuid"]){
				++p;
				continue;
			}
			if((*p)["uuid"].as<std::string>() == uuid){
 				RCLCPP_INFO(this->get_logger(), "Delete request received and object found.");
				p = docs.erase(p);
			} else {
				++p;
			}
		}
		
		// Write to save
		if(docs.size() == 0) clear_file("output.yaml");
		for(auto &d : docs){
			write_route("output.yaml", d, false);			
		}

		// Sync and update node
		read_routes("output.yaml");	
	}

	void clear_file(const std::string& filepath){
		std::ofstream o;
		o.open(filepath, std::ios::trunc);
	}

	template <typename T> void write_route(const std::string& filepath, T& value, bool append = true){
		std::ofstream o;
		if (append) {
			o.open(filepath, std::ios::app);
		} else {
			o.open(filepath, std::ios::trunc);			
		}

		if(!o.is_open()) throw std::runtime_error("Cannot open file: " + filepath);
		o << "---\n" << value << "\n";	
	}

	std::vector<YAML::Node> read_yaml_to_node_vector(std::string filepath)
	{	
		std::ifstream yaml_file(filepath);
		std::vector<YAML::Node> docs = YAML::LoadAll(yaml_file);
		if (docs.empty()) return {};
		return docs; 
	}
	
	void read_routes(std::string filepath)
	{	
		std::vector<YAML::Node> docs = read_yaml_to_node_vector(filepath);
		if(docs.empty()){
			routes = {};
			return;
		}	
		for (size_t i = 0; i < docs.size(); ++i) {
			append_route(docs[i]);
		} 
	}
	
	void append_route(YAML::Node node){
		uuid_route_map new_map = yaml_to_map(node);
		routes.insert(new_map.begin(), new_map.end());	
	}

	std::string prepend_uuid(const std::string yaml_str){	
		uuid_t binuuid;
		uuid_generate(binuuid);
		char uuid_str[37];
		uuid_unparse_lower(binuuid, uuid_str);
		
		std::ostringstream oss;
		oss << "uuid: \"" << uuid_str << "\"\n";
		oss << yaml_str;
		std::string new_str = oss.str();
		return new_str;
	}

	std::string route_to_yaml_string(std::string uuid, adapi_route route){
		std::ostringstream oss;
		oss << "uuid: \"" << uuid << "\"\n";
		oss << autoware_adapi_v1_msgs::msg::to_yaml(route);
		std::string new_str = oss.str();
		return new_str;
	};

	void route_set_callback(const adapi_route& msg)
	{	
		if(msg.data.empty()) return;
		std::string yaml_str = autoware_adapi_v1_msgs::msg::to_yaml(msg);
		auto uuid_yaml_str = prepend_uuid(yaml_str);
		write_route(save_filepath, uuid_yaml_str, true);
		RCLCPP_INFO(this->get_logger(), "Route written to output.yaml.");
	}
	
	void test_write_to_history(){
		std::vector<std::string> yaml_strs = {};
		for(auto &[uuid, route] : routes){
			std::string s = route_to_yaml_string(uuid, route);
			yaml_strs.push_back(s);
		}
		
		for(auto &ys : yaml_strs){
			write_route("new_output.yaml", ys, true);
		}	
	} 


	void set_route(std::string uuid)
	{	
		try {
			geometry_msgs::msg::PoseWithCovarianceStamped msg;
			
			msg.header.frame_id = routes.at(uuid).header.frame_id;

			msg.pose.pose.position.x = routes.at(uuid).data[0].start.position.x;
			msg.pose.pose.position.y = routes.at(uuid).data[0].start.position.y;
			msg.pose.pose.position.z = routes.at(uuid).data[0].start.position.z;

			msg.pose.pose.orientation.x = routes.at(uuid).data[0].start.orientation.x;
			msg.pose.pose.orientation.y = routes.at(uuid).data[0].start.orientation.y;
			msg.pose.pose.orientation.z = routes.at(uuid).data[0].start.orientation.z;
			msg.pose.pose.orientation.w = routes.at(uuid).data[0].start.orientation.w;
/*
			if (routes.at(uuid).covariance && routes.at(uuid).covariance.size() == 36) {
			    for (size_t i = 0; i < 36; i++) {
				msg.pose.covariance[i] = routes.at(uuid).covariance[i];
			    }
			} else {
			    std::fill(msg.pose.covariance.begin(), msg.pose.covariance.end(), 0.0);
			}
*/
			std::fill(msg.pose.covariance.begin(), msg.pose.covariance.end(), 0.0);

			geometry_msgs::msg::PoseStamped goalmsg;

			goalmsg.header.frame_id = routes.at(uuid).header.frame_id;

			goalmsg.pose.position.x = routes.at(uuid).data[0].goal.position.x;
			goalmsg.pose.position.y = routes.at(uuid).data[0].goal.position.y;
			goalmsg.pose.position.z = routes.at(uuid).data[0].goal.position.z;

			goalmsg.pose.orientation.x = routes.at(uuid).data[0].goal.orientation.x;
			goalmsg.pose.orientation.y = routes.at(uuid).data[0].goal.orientation.y;
			goalmsg.pose.orientation.z = routes.at(uuid).data[0].goal.orientation.z;
			goalmsg.pose.orientation.w = routes.at(uuid).data[0].goal.orientation.w;

			initial_pose_publisher_->publish(msg);
			goal_pose_publisher_->publish(goalmsg);			

		} catch(const YAML::BadFile& e) {
			RCLCPP_INFO(this->get_logger(), "Output.yaml not found.");
		} catch(const YAML::BadConversion& e) {
			RCLCPP_INFO(this->get_logger(), "Bad conversion: %s", e.what());
		}

		
	}
	
	std::unordered_map<std::string, autoware_adapi_v1_msgs::msg::Route> yaml_to_map(YAML::Node root)
	{
		
		autoware_adapi_v1_msgs::msg::Route msg;

		// Parse header
		auto header_node = root["header"];

		msg.header.stamp.sec = header_node["stamp"]["sec"].as<int32_t>();
		msg.header.stamp.nanosec = header_node["stamp"]["nanosec"].as<uint32_t>();
		msg.header.frame_id = header_node["frame_id"].as<std::string>();

		// Parse data (array, max 1)
		auto data_node = root["data"];
		if (data_node && data_node.IsSequence() && !data_node.IsNull())
		{
		    for (const auto &route_node : data_node)
		    {
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
			if (segments_node && segments_node.IsSequence())
			{
			    for (const auto &seg_node : segments_node)
			    {
				autoware_adapi_v1_msgs::msg::RouteSegment seg;

				// Preferred
				seg.preferred.id = seg_node["preferred"]["id"].as<int64_t>();
				seg.preferred.type = seg_node["preferred"]["type"].as<std::string>();

				// Alternatives
				auto alt_node = seg_node["alternatives"];
				if (alt_node && alt_node.IsSequence())
				{
				    for (const auto &alt : alt_node)
				    {
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
		
		std::unordered_map<std::string, autoware_adapi_v1_msgs::msg::Route> m = {{root["uuid"].as<std::string>(), msg}};	
		return m;
	}

	rclcpp::Subscription<autoware_adapi_v1_msgs::msg::Route>::SharedPtr route_set_subscription_;	
//	rclcpp::Client<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedPtr client_;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;	
	rclcpp::Service<pose_replay_interfaces::srv::GetUuidRoute>::SharedPtr service_;		
	
	rclcpp::Service<pose_replay_interfaces::srv::GetUuidRoutes>::SharedPtr all_routes_service_;
	rclcpp::Service<pose_replay_interfaces::srv::SetRoute>::SharedPtr set_route_service_;	
	rclcpp::Service<pose_replay_interfaces::srv::DeleteRoute>::SharedPtr delete_route_service_;	

	uuid_route_map routes; 
};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TopicListener>());
	rclcpp::shutdown();
}
