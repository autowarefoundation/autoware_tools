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
  AppState * state = static_cast<AppState *>(data);
  
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

  const int margin_left = 60;
  const int margin_bottom = 60;
  const int margin_right = 160; // Extra space for legend
  const int margin_top = 40;
  
  const int plot_width = 800;
  const int plot_height = 600;
  
  const int total_width = plot_width + margin_left + margin_right;
  const int total_height = plot_height + margin_top + margin_bottom;
  
  cv::Mat plot = cv::Mat::zeros(total_height, total_width, CV_8UC3);

  const double max_v = 40.0;
  const double max_d = 60.0;

  // Draw Heatmap
  for (int y = 0; y < plot_height; y += res) {
    for (int x = 0; x < plot_width; x += res) {
      double v = (static_cast<double>(x) / plot_width) * max_v;
      double d = (1.0 - static_cast<double>(y) / plot_height) * max_d;
      
      bool can_pass = filter.can_pass_amber_light(d, v, current_acceleration);
      cv::Scalar color = can_pass ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
      
      cv::rectangle(plot, 
                    cv::Point(x + margin_left, y + margin_top), 
                    cv::Point(x + margin_left + res, y + margin_top + res), 
                    color, cv::FILLED);
    }
  }

  // Draw axes
  cv::Point origin(margin_left, total_height - margin_bottom);
  cv::Point x_end(margin_left + plot_width, total_height - margin_bottom);
  cv::Point y_end(margin_left, margin_top);
  
  cv::line(plot, origin, x_end, cv::Scalar(255, 255, 255), 2);
  cv::line(plot, origin, y_end, cv::Scalar(255, 255, 255), 2);

  // X-axis Ticks and Scale (Velocity)
  for (int v_int = 0; v_int <= static_cast<int>(max_v); v_int += 5) {
    int x = margin_left + static_cast<int>((static_cast<double>(v_int) / max_v) * plot_width);
    cv::line(plot, cv::Point(x, origin.y), cv::Point(x, origin.y + 5), cv::Scalar(255, 255, 255), 1);
    cv::putText(plot, std::to_string(v_int), cv::Point(x - 10, origin.y + 20), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
  }
  cv::putText(plot, "Velocity (m/s)", cv::Point(margin_left + plot_width / 2 - 50, origin.y + 45), 
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

  // Y-axis Ticks and Scale (Distance)
  for (int d_int = 0; d_int <= static_cast<int>(max_d); d_int += 10) {
    int y = origin.y - static_cast<int>((static_cast<double>(d_int) / max_d) * plot_height);
    cv::line(plot, cv::Point(origin.x - 5, y), cv::Point(origin.x, y), cv::Scalar(255, 255, 255), 1);
    cv::putText(plot, std::to_string(d_int), cv::Point(origin.x - 30, y + 5), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
  }
  cv::putText(plot, "Distance (m)", cv::Point(5, margin_top - 10), 
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

  // Legend
  int legend_x = margin_left + plot_width + 20;
  int legend_y = margin_top + 20;
  
  cv::rectangle(plot, cv::Point(legend_x, legend_y), cv::Point(legend_x + 20, legend_y + 20), cv::Scalar(0, 255, 0), cv::FILLED);
  cv::putText(plot, "Can Pass", cv::Point(legend_x + 30, legend_y + 15), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
  
  cv::rectangle(plot, cv::Point(legend_x, legend_y + 30), cv::Point(legend_x + 20, legend_y + 50), cv::Scalar(0, 0, 255), cv::FILLED);
  cv::putText(plot, "Must Stop", cv::Point(legend_x + 30, legend_y + 45), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

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
