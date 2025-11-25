#include <memory>
#include <functional>
#include <fstream>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp" 
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "autoware_planning_msgs/msg/lanelet_route.hpp"

#include "autoware_adapi_v1_msgs/msg/route.hpp"
#include "autoware_adapi_v1_msgs/srv/set_route_points.hpp"
#include "autoware_adapi_v1_msgs/srv/clear_route.hpp"
#include <chrono>

using namespace std::literals::chrono_literals;

class TopicListener : public rclcpp::Node {

	public:
	TopicListener() : Node("topic_listener") {		
		RCLCPP_INFO_STREAM(this->get_logger(), "Listener online."); 
		
		route_set_subscription_ = this->create_subscription<autoware_adapi_v1_msgs::msg::Route>(
			"/api/routing/route", 10, std::bind(&TopicListener::test_callback, this, std::placeholders::_1));
		
	//	client_ = this->create_client<autoware_adapi_v1_msgs::srv::ClearRoute>("/api/routing/clear_route");
	};

	void test_callback(const autoware_adapi_v1_msgs::msg::Route& msg)
	{	
		if(msg.data.empty()) return;
		
		auto data = msg.data[0];
		
		auto start = data.start;
		auto goal = data.goal;

		nlohmann::json ji = pose_to_json(start);
		nlohmann::json jg = pose_to_json(goal);	
		
		fwrite_json(ji);
		fwrite_json(jg);
	
		RCLCPP_INFO(this->get_logger(), "Route log start: %f", msg.data[0].start.position.x);
		RCLCPP_INFO(this->get_logger(), "Route log end: %f", msg.data[0].goal.position.x);
	}
	
	nlohmann::json pose_to_json(geometry_msgs::msg::Pose p) 
	{	
		nlohmann::json j;
		
		j["uuid"] = 1;
		j["type"] = "initial / goal";
		j["pose"]["position"]["x"] = p.position.x;
		j["pose"]["position"]["y"] = p.position.y;
		j["pose"]["position"]["z"] = p.position.z;
		j["pose"]["orientation"]["x"] = p.orientation.x;
		j["pose"]["orientation"]["y"] = p.orientation.y;
		j["pose"]["orientation"]["z"] = p.orientation.z;
		j["pose"]["orientation"]["w"] = p.orientation.w;

		return j;
	}

	void fwrite_json(nlohmann::json j) 
	{
		std::ofstream o;
		o.open("output.json", std::ios::app);
		if(o.is_open()) o << j.dump(4);	
		o.close();
	};

	rclcpp::Subscription<autoware_adapi_v1_msgs::msg::Route>::SharedPtr route_set_subscription_;	
//	rclcpp::Client<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedPtr client_;

};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TopicListener>());
	rclcpp::shutdown();
}
