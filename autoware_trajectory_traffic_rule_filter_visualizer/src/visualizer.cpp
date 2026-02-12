#include <autoware/boundary_departure_checker/utils.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <string>
#include <vector>

struct AmberLightParams
{
  double max_accel;
  double max_jerk;
  double delay_response_time;
  double yellow_lamp_period;
  double yellow_light_stop_velocity;
  bool enable_pass_judge;
};

// Reusing the logic from TrafficLightFilter::can_pass_amber_light
bool can_pass_amber_light(
  const double distance_to_stop_line, const double current_velocity,
  const double current_acceleration, const AmberLightParams & params)
{
  const double reachable_distance = current_velocity * params.yellow_lamp_period;

  const double pass_judge_line_distance =
    autoware::boundary_departure_checker::utils::calc_judge_line_dist_with_jerk_limit(
      current_velocity, current_acceleration, params.max_accel, params.max_jerk,
      params.delay_response_time);

  const bool distance_stoppable = pass_judge_line_distance < distance_to_stop_line;
  const bool slow_velocity = current_velocity < params.yellow_light_stop_velocity;
  const bool stoppable = distance_stoppable || slow_velocity;
  const bool reachable = distance_to_stop_line < reachable_distance;

  if (params.enable_pass_judge && !stoppable) {
    if (!reachable) {
      return false;
    }
  } else {
    return false;
  }
  return true;
}

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
  
  AmberLightParams params;
  params.max_accel = state->max_accel_scaled / 10.0;
  params.max_jerk = state->max_jerk_scaled / 10.0;
  params.delay_response_time = state->delay_response_time_scaled / 10.0;
  params.yellow_lamp_period = state->yellow_lamp_period_scaled / 10.0;
  params.yellow_light_stop_velocity = state->yellow_light_stop_velocity_scaled / 10.0;
  params.enable_pass_judge = state->enable_pass_judge != 0;
  
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
      
      bool can_pass = can_pass_amber_light(d, v, current_acceleration, params);
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
  (void)argc;
  (void)argv;

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

  return 0;
}
