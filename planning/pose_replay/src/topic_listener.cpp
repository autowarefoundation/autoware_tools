#include <memory>
#include <functional>
#include <fstream>
#include <string>

#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp" 
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class TopicListener : public rclcpp::Node {

	public:
	TopicListener() : Node("topic_listener") {
		RCLCPP_INFO_STREAM(this->get_logger(), "Listener online."); 

		// /initialpose for initial pose
		initial_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			"initialpose", 10, std::bind(&TopicListener::initial_pose_topic_callback, this, std::placeholders::_1));

		// /planning/mission_planning/goal for goal - currently this includes checkpoints, need to distinguish
		goal_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"planning/mission_planning/goal", 10, std::bind(&TopicListener::goal_pose_topic_callback, this, std::placeholders::_1));
	};	

	private:
	void initial_pose_topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) const 
	{
		double x = msg.pose.pose.position.x;
		write_to_file_callback("initial", x);
		RCLCPP_INFO(this->get_logger(), "I heard: '%f' and recorded this initial pose", x);	
	}
	
	void goal_pose_topic_callback(const geometry_msgs::msg::PoseStamped& msg) const 
	{
		double x = msg.pose.position.x;
		write_to_file_callback("checkpoint/goal", x);
		RCLCPP_INFO(this->get_logger(), "I heard: '%f' and recorded this checkpoint or goal pose", x);
	}
	
	void write_to_file_callback(std::string label, double x) const
	{
		std::ofstream outFile("output.txt");
		if(outFile){
			RCLCPP_INFO(this->get_logger(), "In here");
			std::cout << label << ": " << x << std::endl;
		}
		outFile.close(); 
	}
	
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscription_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscription_;

};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TopicListener>());
	rclcpp::shutdown();
}
