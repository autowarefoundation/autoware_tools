#include <autoware/trajectory_traffic_rule_filter/filters/traffic_light_filter.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

const std::string WINDOW_NAME = "Amber Light Pass Logic";

struct Control
{
  std::string name;
  double min_val;
  double max_val;
  double value;
  bool is_bool = false;
  bool is_int = false;

  // UI position
  cv::Rect rect;

  std::string format_value() const
  {
    if (is_bool) return value > 0.5 ? "ON" : "OFF";
    std::stringstream ss;
    if (is_int)
      ss << static_cast<int>(value);
    else
      ss << std::fixed << std::setprecision(1) << value;
    return ss.str();
  }
};

struct AppState
{
  bool needs_redraw = true;
  std::vector<Control> controls;
  int active_control = -1;
};

void init_controls(AppState & state)
{
  state.controls = {
    {"Max Accel", 0.0, 10.0, 2.0},
    {"Max Jerk", 0.0, 10.0, 1.5},
    {"Delay", 0.0, 5.0, 0.5},
    {"Yellow Period", 0.0, 10.0, 3.0},
    {"Stop Velocity", 0.0, 5.0, 0.2},
    {"Current Accel", -5.0, 5.0, 0.0},
    {"Pass Judge", 0.0, 1.0, 1.0, true},
    {"Resolution", 1.0, 20.0, 5.0, false, true},
    {"Plot Max V", 5.0, 100.0, 40.0, false, true},
    {"Plot Max D", 10.0, 200.0, 60.0, false, true}};
}

void on_mouse(int event, int x, int y, int /*flags*/, void * userdata)
{
  AppState * state = static_cast<AppState *>(userdata);

  if (event == cv::EVENT_LBUTTONDOWN) {
    for (size_t i = 0; i < state->controls.size(); ++i) {
      if (state->controls[i].rect.contains(cv::Point(x, y))) {
        if (state->controls[i].is_bool) {
          state->controls[i].value = (state->controls[i].value > 0.5) ? 0.0 : 1.0;
          state->needs_redraw = true;
        } else {
          state->active_control = static_cast<int>(i);
        }
        break;
      }
    }
  } else if (event == cv::EVENT_LBUTTONUP) {
    state->active_control = -1;
  }

  if (
    state->active_control != -1 &&
    (event == cv::EVENT_MOUSEMOVE || event == cv::EVENT_LBUTTONDOWN)) {
    Control & c = state->controls[state->active_control];
    double ratio = static_cast<double>(x - c.rect.x) / c.rect.width;
    ratio = std::max(0.0, std::min(1.0, ratio));
    c.value = c.min_val + ratio * (c.max_val - c.min_val);
    if (c.is_int) c.value = std::round(c.value);
    state->needs_redraw = true;
  }
}

void draw_plot(AppState & state)
{
  cv::Rect win_rect = cv::getWindowImageRect(WINDOW_NAME);
  int total_width = std::max(win_rect.width, 600);
  int total_height = std::max(win_rect.height, 400);

  const int margin_controls = 200;
  const int margin_left = 60;
  const int margin_bottom = 60;
  const int margin_right = 120;
  const int margin_top = 40;

  int plot_area_start_x = margin_controls + margin_left;
  int plot_width = std::max(50, total_width - plot_area_start_x - margin_right);
  int plot_height = std::max(50, total_height - margin_top - margin_bottom);

  cv::Mat plot = cv::Mat::zeros(total_height, total_width, CV_8UC3);

  // Update control rectangles and draw them
  int cy = 40;
  for (auto & c : state.controls) {
    c.rect = cv::Rect(20, cy, 160, 15);

    cv::putText(
      plot, c.name + ": " + c.format_value(), cv::Point(c.rect.x, c.rect.y - 5),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);

    if (c.is_bool) {
      cv::rectangle(plot, cv::Rect(c.rect.x, c.rect.y, 20, 20), cv::Scalar(100, 100, 100), 1);
      if (c.value > 0.5) {
        cv::rectangle(
          plot, cv::Rect(c.rect.x + 3, c.rect.y + 3, 14, 14), cv::Scalar(0, 255, 0), cv::FILLED);
      }
      cy += 45;
    } else {
      cv::rectangle(plot, c.rect, cv::Scalar(60, 60, 60), cv::FILLED);
      int handle_x =
        c.rect.x + static_cast<int>((c.value - c.min_val) / (c.max_val - c.min_val) * c.rect.width);
      cv::rectangle(
        plot, cv::Rect(handle_x - 3, c.rect.y - 2, 6, c.rect.height + 4), cv::Scalar(180, 180, 180),
        cv::FILLED);
      cy += 40;
    }
  }

  // Parameters for the filter
  traffic_rule_filter::Params params;
  params.traffic_light_filter.max_accel = -state.controls[0].value;
  params.traffic_light_filter.max_jerk = -state.controls[1].value;
  params.traffic_light_filter.delay_response_time = state.controls[2].value;
  params.traffic_light_filter.amber_lamp_period = state.controls[3].value;
  params.traffic_light_filter.amber_light_stop_velocity = state.controls[4].value;
  params.traffic_light_filter.enable_pass_judge = state.controls[6].value > 0.5;

  autoware::trajectory_traffic_rule_filter::plugin::TrafficLightFilter filter;
  filter.set_parameters(params);
  filter.set_logger(rclcpp::get_logger("amber_light_visualizer"));

  double current_acceleration = state.controls[5].value;
  int res = std::max(1, static_cast<int>(state.controls[7].value));
  const double max_v = state.controls[8].value;
  const double max_d = state.controls[9].value;

  // Draw Heatmap
  for (int y = 0; y < plot_height; y += res) {
    for (int x = 0; x < plot_width; x += res) {
      double v = (static_cast<double>(x) / plot_width) * max_v;
      double d = (1.0 - static_cast<double>(y) / plot_height) * max_d;

      bool can_pass = filter.can_pass_amber_light(d, v, current_acceleration);
      cv::Scalar color = can_pass ? cv::Scalar(0, 200, 0) : cv::Scalar(0, 0, 200);

      int block_w = std::min(res, plot_width - x);
      int block_h = std::min(res, plot_height - y);

      cv::rectangle(
        plot, cv::Point(x + plot_area_start_x, y + margin_top),
        cv::Point(x + plot_area_start_x + block_w, y + margin_top + block_h), color, cv::FILLED);
    }
  }

  // Draw axes
  cv::Point origin(plot_area_start_x, total_height - margin_bottom);
  cv::Point x_end(plot_area_start_x + plot_width, total_height - margin_bottom);
  cv::Point y_end(plot_area_start_x, margin_top);

  cv::line(plot, origin, x_end, cv::Scalar(255, 255, 255), 1);
  cv::line(plot, origin, y_end, cv::Scalar(255, 255, 255), 1);

  // X-axis Ticks
  for (double v = 0; v <= max_v; v += (max_v > 50 ? 10 : 5)) {
    int x = plot_area_start_x + static_cast<int>((v / max_v) * plot_width);
    cv::line(
      plot, cv::Point(x, origin.y), cv::Point(x, origin.y + 5), cv::Scalar(255, 255, 255), 1);
    cv::putText(
      plot, std::to_string(static_cast<int>(v)), cv::Point(x - 10, origin.y + 20),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
  }
  cv::putText(
    plot, "Velocity (m/s)", cv::Point(plot_area_start_x + plot_width / 2 - 50, origin.y + 45),
    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

  // Y-axis Ticks
  for (double d = 0; d <= max_d; d += (max_d > 100 ? 20 : 10)) {
    int y = origin.y - static_cast<int>((d / max_d) * plot_height);
    cv::line(
      plot, cv::Point(origin.x - 5, y), cv::Point(origin.x, y), cv::Scalar(255, 255, 255), 1);
    cv::putText(
      plot, std::to_string(static_cast<int>(d)), cv::Point(origin.x - 30, y + 5),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
  }
  cv::putText(
    plot, "Distance (m)", cv::Point(plot_area_start_x - 40, margin_top - 15),
    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

  // Legend
  int legend_x = plot_area_start_x + plot_width + 20;
  int legend_y = margin_top + 20;
  if (legend_x + 80 < total_width) {
    cv::rectangle(plot, cv::Rect(legend_x, legend_y, 15, 15), cv::Scalar(0, 200, 0), cv::FILLED);
    cv::putText(
      plot, "Pass", cv::Point(legend_x + 20, legend_y + 12), cv::FONT_HERSHEY_SIMPLEX, 0.4,
      cv::Scalar(255, 255, 255), 1);
    cv::rectangle(
      plot, cv::Rect(legend_x, legend_y + 25, 15, 15), cv::Scalar(0, 0, 200), cv::FILLED);
    cv::putText(
      plot, "Stop", cv::Point(legend_x + 20, legend_y + 37), cv::FONT_HERSHEY_SIMPLEX, 0.4,
      cv::Scalar(255, 255, 255), 1);
  }

  cv::imshow(WINDOW_NAME, plot);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  AppState state;
  init_controls(state);

  cv::namedWindow(WINDOW_NAME, cv::WINDOW_NORMAL);
  cv::resizeWindow(WINDOW_NAME, 1000, 600);
  cv::setMouseCallback(WINDOW_NAME, on_mouse, &state);

  cv::Rect prev_rect = cv::getWindowImageRect(WINDOW_NAME);

  while (rclcpp::ok()) {
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

    int key = cv::waitKey(20);
    if (key == 27) break;
  }

  rclcpp::shutdown();
  return 0;
}
