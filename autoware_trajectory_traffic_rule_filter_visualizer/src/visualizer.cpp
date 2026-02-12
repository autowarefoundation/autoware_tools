#include <autoware/trajectory_traffic_rule_filter/filters/traffic_light_filter.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <string>
#include <vector>

struct AppState
{
  int max_accel_scaled = 20;            // 2.0 m/s^2
  int max_jerk_scaled = 15;             // 1.5 m/s^3
  int delay_response_time_scaled = 5;   // 0.5 s
  int yellow_lamp_period_scaled = 30;   // 3.0 s
  int yellow_light_stop_velocity_scaled = 2; // 0.2 m/s
  int current_acceleration_scaled = 0;  // 0.0 m/s^2
  int enable_pass_judge = 1;
  int resolution = 5;                   // 5 pixels per unit
};

void update_plot(int, void * data)
{
  auto * state = static_cast<AppState *>(data);
  
  traffic_rule_filter::Params params;
  params.traffic_light_filter.max_accel = state->max_accel_scaled / 10.0;
  params.traffic_light_filter.max_jerk = state->max_jerk_scaled / 10.0;
  params.traffic_light_filter.delay_response_time = state->delay_response_time_scaled / 10.0;
  params.traffic_light_filter.yellow_lamp_period = state->yellow_lamp_period_scaled / 10.0;
  params.traffic_light_filter.yellow_light_stop_velocity = state->yellow_light_stop_velocity_scaled / 10.0;
  params.traffic_light_filter.enable_pass_judge = state->enable_pass_judge != 0;
  
  autoware::trajectory_traffic_rule_filter::plugin::TrafficLightFilter filter;
  filter.set_parameters(params);
  filter.set_logger(rclcpp::get_logger("amber_light_visualizer"));
  
  double current_acceleration = state->current_acceleration_scaled / 10.0;
  int res = std::max(1, state->resolution);

  const int width = 800;
  const int height = 600;
  cv::Mat plot = cv::Mat::zeros(height, width, CV_8UC3);

  // X axis: Velocity (0 to 40 m/s) -> 800 / 20 = 40
  // Y axis: Distance (0 to 60 m) -> 600 / 10 = 60
  
  const double max_v = 40.0;
  const double max_d = 60.0;

  for (int y = 0; y < height; y += res) {
    for (int x = 0; x < width; x += res) {
      double v = (static_cast<double>(x) / width) * max_v;
      double d = (1.0 - static_cast<double>(y) / height) * max_d;
      
      bool can_pass = filter.can_pass_amber_light(d, v, current_acceleration);
      cv::Scalar color = can_pass ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
      
      cv::rectangle(plot, cv::Point(x, y), cv::Point(x + res, y + res), color, cv::FILLED);
    }
  }

  // Draw axes and labels
  cv::line(plot, cv::Point(0, height - 1), cv::Point(width, height - 1), cv::Scalar(255, 255, 255), 2);
  cv::line(plot, cv::Point(0, 0), cv::Point(0, height), cv::Scalar(255, 255, 255), 2);
  
  cv::putText(plot, "Velocity (m/s)", cv::Point(width - 150, height - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
  cv::putText(plot, "Distance (m)", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

  cv::imshow("Amber Light Pass Logic", plot);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  AppState state;
  cv::namedWindow("Amber Light Pass Logic", cv::WINDOW_AUTOSIZE);

  cv::createTrackbar("Max Accel (x0.1)", "Amber Light Pass Logic", &state.max_accel_scaled, 100, update_plot, &state);
  cv::createTrackbar("Max Jerk (x0.1)", "Amber Light Pass Logic", &state.max_jerk_scaled, 100, update_plot, &state);
  cv::createTrackbar("Delay (x0.1s)", "Amber Light Pass Logic", &state.delay_response_time_scaled, 50, update_plot, &state);
  cv::createTrackbar("Yellow (x0.1s)", "Amber Light Pass Logic", &state.yellow_lamp_period_scaled, 100, update_plot, &state);
  cv::createTrackbar("Stop V (x0.1)", "Amber Light Pass Logic", &state.yellow_light_stop_velocity_scaled, 50, update_plot, &state);
  cv::createTrackbar("Current Acc (x0.1)", "Amber Light Pass Logic", &state.current_acceleration_scaled, 50, update_plot, &state);
  cv::createTrackbar("Enable Pass Judge", "Amber Light Pass Logic", &state.enable_pass_judge, 1, update_plot, &state);
  cv::createTrackbar("Resolution", "Amber Light Pass Logic", &state.resolution, 20, update_plot, &state);

  update_plot(0, &state);
  cv::waitKey(0);

  rclcpp::shutdown();
  return 0;
}
