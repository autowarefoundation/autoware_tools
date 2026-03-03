// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <string>

class ImageUndistortion : public rclcpp::Node
{
public:
  ImageUndistortion() : Node("image_undistortion")
  {
    this->declare_parameter("interpolation", 1);

    std::string input_topic = this->declare_parameter("input_topic", "input/image");
    std::string transport_type = this->declare_parameter("transport", "raw");

    RCLCPP_INFO(
      this->get_logger(), "Subscribing to: %s (Type: %s)", input_topic.c_str(),
      transport_type.c_str());

    sub_camera_ = image_transport::create_camera_subscription(
      this, input_topic,
      std::bind(
        &ImageUndistortion::imageCallback, this, std::placeholders::_1, std::placeholders::_2),
      transport_type, rmw_qos_profile_sensor_data);

    pub_rect_ = image_transport::create_camera_publisher(this, "output/image_rect");

    RCLCPP_INFO(this->get_logger(), "Rectify Node has been started.");
  }

private:
  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
  {
    model_.fromCameraInfo(info_msg);

    cv::Mat raw_image;
    try {
      raw_image = cv_bridge::toCvShare(image_msg, "bgr8")->image;
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat rect_image;
    int interpolation_method = this->get_parameter("interpolation").as_int();
    model_.rectifyImage(raw_image, rect_image, interpolation_method);

    sensor_msgs::msg::Image::SharedPtr out_img_msg =
      cv_bridge::CvImage(image_msg->header, "bgr8", rect_image).toImageMsg();

    auto out_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(*info_msg);
    out_info_msg->d.assign(out_info_msg->d.size(), 0.0);

    cv::Rect roi = model_.rectifiedRoi();
    if (roi.width > 0 && roi.height > 0) {
      out_info_msg->roi.x_offset = roi.x;
      out_info_msg->roi.y_offset = roi.y;
      out_info_msg->roi.height = roi.height;
      out_info_msg->roi.width = roi.width;
    }
    out_info_msg->roi.do_rectify = false; 

    pub_rect_.publish(out_img_msg, out_info_msg);
  }

  image_transport::CameraSubscriber sub_camera_;
  image_transport::CameraPublisher pub_rect_;
  image_geometry::PinholeCameraModel model_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageUndistortion>());
  rclcpp::shutdown();
  return 0;
}
