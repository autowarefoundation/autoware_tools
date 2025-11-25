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
using autoware_adapi_v1_msgs::msg::to_yaml;

class TopicListener : public rclcpp::Node {

	public:
	TopicListener() : Node("topic_listener") {		
		RCLCPP_INFO_STREAM(this->get_logger(), "Listener online."); 
		
		route_set_subscription_ = this->create_subscription<autoware_adapi_v1_msgs::msg::Route>(
			"/api/routing/route", 10, std::bind(&TopicListener::route_set_callback, this, std::placeholders::_1));
		
	//	client_ = this->create_client<autoware_adapi_v1_msgs::srv::ClearRoute>("/api/routing/clear_route");
	};

	void route_set_callback(const autoware_adapi_v1_msgs::msg::Route& msg)
	{	
		if(msg.data.empty()) return;
		route_to_yaml(msg);
		RCLCPP_INFO(this->get_logger(), "Route written to output.yaml.");
	}

	void route_to_yaml(const autoware_adapi_v1_msgs::msg::Route& msg){
		std::string yaml = autoware_adapi_v1_msgs::msg::to_yaml(msg);
		std::ofstream o;
		o.open("output.yaml", std::ios::app);
		if(o.is_open()) o << yaml;	
		o.close();
	}	
	

	rclcpp::Subscription<autoware_adapi_v1_msgs::msg::Route>::SharedPtr route_set_subscription_;	
//	rclcpp::Client<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedPtr client_;

};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TopicListener>());
	rclcpp::shutdown();
}
