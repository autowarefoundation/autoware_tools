#include <autoware/trajectory_traffic_rule_filter/filters/traffic_light_filter.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

const std::string WINDOW_NAME = "Amber Light Pass Logic";

struct AppState
{
  bool needs_redraw = true;
};

void on_trackbar(int, void * data)
{
  AppState * state = static_cast<AppState *>(data);
  state->needs_redraw = true;
}

void draw_plot(AppState &)
{
  cv::Rect rect = cv::getWindowImageRect(WINDOW_NAME);
  int total_width = std::max(rect.width, 400);
  int total_height = std::max(rect.height, 300);

  const int margin_left = 60;
  const int margin_bottom = 60;
  const int margin_right = 160; 
  const int margin_top = 40;
  
  int plot_width = std::max(50, total_width - margin_left - margin_right);
  int plot_height = std::max(50, total_height - margin_top - margin_bottom);
  
  cv::Mat plot = cv::Mat::zeros(total_height, total_width, CV_8UC3);

  traffic_rule_filter::Params params;
  params.traffic_light_filter.max_accel = cv::getTrackbarPos("Max Accel (x0.1)", WINDOW_NAME) / 10.0;
  params.traffic_light_filter.max_jerk = cv::getTrackbarPos("Max Jerk (x0.1)", WINDOW_NAME) / 10.0;
  params.traffic_light_filter.delay_response_time = cv::getTrackbarPos("Delay (x0.1s)", WINDOW_NAME) / 10.0;
  params.traffic_light_filter.yellow_lamp_period = cv::getTrackbarPos("Yellow (x0.1s)", WINDOW_NAME) / 10.0;
  params.traffic_light_filter.yellow_light_stop_velocity = cv::getTrackbarPos("Stop V (x0.1)", WINDOW_NAME) / 10.0;
  params.traffic_light_filter.enable_pass_judge = true;
  
  autoware::trajectory_traffic_rule_filter::plugin::TrafficLightFilter filter;
  filter.set_parameters(params);
  filter.set_logger(rclcpp::get_logger("amber_light_visualizer"));
  
  double current_acceleration = cv::getTrackbarPos("Current Acc (x0.1)", WINDOW_NAME) / 10.0;
  int res = std::max(1, cv::getTrackbarPos("Resolution", WINDOW_NAME));

  const double max_v = std::max(1.0, static_cast<double>(cv::getTrackbarPos("Plot Max V (m/s)", WINDOW_NAME)));
  const double max_d = std::max(1.0, static_cast<double>(cv::getTrackbarPos("Plot Max D (m)", WINDOW_NAME)));

  // Draw Heatmap
  for (int y = 0; y < plot_height; y += res) {
    for (int x = 0; x < plot_width; x += res) {
      double v = (static_cast<double>(x) / plot_width) * max_v;
      double d = (1.0 - static_cast<double>(y) / plot_height) * max_d;
      
      bool can_pass = filter.can_pass_amber_light(d, v, current_acceleration);
      cv::Scalar color = can_pass ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
      
      int block_w = std::min(res, plot_width - x);
      int block_h = std::min(res, plot_height - y);

      cv::rectangle(plot, 
                    cv::Point(x + margin_left, y + margin_top), 
                    cv::Point(x + margin_left + block_w, y + margin_top + block_h), 
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
  int x_tick_step = (max_v > 50) ? 10 : 5;
  for (int v_int = 0; v_int <= static_cast<int>(max_v); v_int += x_tick_step) {
    int x = margin_left + static_cast<int>((static_cast<double>(v_int) / max_v) * plot_width);
    cv::line(plot, cv::Point(x, origin.y), cv::Point(x, origin.y + 5), cv::Scalar(255, 255, 255), 1);
    cv::putText(plot, std::to_string(v_int), cv::Point(x - 10, origin.y + 20), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
  }
  cv::putText(plot, "Velocity (m/s)", cv::Point(margin_left + plot_width / 2 - 50, origin.y + 45), 
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

  // Y-axis Ticks and Scale (Distance)
  int y_tick_step = (max_d > 100) ? 20 : 10;
  for (int d_int = 0; d_int <= static_cast<int>(max_d); d_int += y_tick_step) {
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
  
  if (legend_x + 100 < total_width) {
    cv::rectangle(plot, cv::Point(legend_x, legend_y), cv::Point(legend_x + 20, legend_y + 20), cv::Scalar(0, 255, 0), cv::FILLED);
    cv::putText(plot, "Can Pass", cv::Point(legend_x + 30, legend_y + 15), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
    
    cv::rectangle(plot, cv::Point(legend_x, legend_y + 30), cv::Point(legend_x + 20, legend_y + 50), cv::Scalar(0, 0, 255), cv::FILLED);
    cv::putText(plot, "Must Stop", cv::Point(legend_x + 30, legend_y + 45), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
  }

  cv::imshow(WINDOW_NAME, plot);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  AppState state;
  cv::namedWindow(WINDOW_NAME, cv::WINDOW_NORMAL);
  cv::resizeWindow(WINDOW_NAME, 800, 600);

  cv::createTrackbar("Max Accel (x0.1)", WINDOW_NAME, nullptr, 100, on_trackbar, &state);
  cv::createTrackbar("Max Jerk (x0.1)", WINDOW_NAME, nullptr, 100, on_trackbar, &state);
  cv::createTrackbar("Delay (x0.1s)", WINDOW_NAME, nullptr, 50, on_trackbar, &state);
  cv::createTrackbar("Yellow (x0.1s)", WINDOW_NAME, nullptr, 100, on_trackbar, &state);
  cv::createTrackbar("Stop V (x0.1)", WINDOW_NAME, nullptr, 50, on_trackbar, &state);
  cv::createTrackbar("Current Acc (x0.1)", WINDOW_NAME, nullptr, 50, on_trackbar, &state);
  cv::createTrackbar("Enable Pass Judge", WINDOW_NAME, nullptr, 1, on_trackbar, &state);
  cv::createTrackbar("Resolution", WINDOW_NAME, nullptr, 20, on_trackbar, &state);
  cv::createTrackbar("Plot Max V (m/s)", WINDOW_NAME, nullptr, 100, on_trackbar, &state);
  cv::createTrackbar("Plot Max D (m)", WINDOW_NAME, nullptr, 200, on_trackbar, &state);

  // Set initial values
  cv::setTrackbarPos("Max Accel (x0.1)", WINDOW_NAME, 20);
  cv::setTrackbarPos("Max Jerk (x0.1)", WINDOW_NAME, 15);
  cv::setTrackbarPos("Delay (x0.1s)", WINDOW_NAME, 5);
  cv::setTrackbarPos("Yellow (x0.1s)", WINDOW_NAME, 30);
  cv::setTrackbarPos("Stop V (x0.1)", WINDOW_NAME, 2);
  cv::setTrackbarPos("Current Acc (x0.1)", WINDOW_NAME, 0);
  cv::setTrackbarPos("Resolution", WINDOW_NAME, 5);
  cv::setTrackbarPos("Plot Max V (m/s)", WINDOW_NAME, 20);
  cv::setTrackbarPos("Plot Max D (m)", WINDOW_NAME, 40);

  cv::waitKey(100); // Give time for window manager
  cv::Rect prev_rect = cv::getWindowImageRect(WINDOW_NAME);

  while (rclcpp::ok()) {
    // Check if window exists
    if (cv::getWindowProperty(WINDOW_NAME, cv::WND_PROP_AUTOSIZE) < 0) break;

    cv::Rect curr_rect = cv::getWindowImageRect(WINDOW_NAME);
    if (curr_rect.width != prev_rect.width || curr_rect.height != prev_rect.height) {
      state.needs_redraw = true;
      prev_rect = curr_rect;
    }

    if (state.needs_redraw) {
      draw_plot(state);
      state.needs_redraw = false;
    }

    int key = cv::waitKey(30);
    if (key == 27) break; // ESC
  }

  rclcpp::shutdown();
  return 0;
}
