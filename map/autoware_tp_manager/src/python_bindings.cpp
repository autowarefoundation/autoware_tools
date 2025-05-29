#include <autoware/pointcloud_divider/pcd_divider.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pybind11/pybind11.h>

using PCDDividerWrapper = autoware::pointcloud_divider::PCDDivider<pcl::PointXYZI>;

PYBIND11_MODULE(pcd_divider_wrapper, m)
{
  pybind11::class_<PCDDividerWrapper>(m, "PCDDivider")
    .def(pybind11::init<const std::string &>())
    .def("setInput", &PCDDividerWrapper::setInput)
    .def("setOutputDir", &PCDDividerWrapper::setOutputDir)
    .def("setPrefix", &PCDDividerWrapper::setPrefix)
    .def("setLeafSize", &PCDDividerWrapper::setLeafSize)
    .def("setGridSize", &PCDDividerWrapper::setGridSize)
    .def("divide_pcd", static_cast<void (PCDDividerWrapper::*)()>(&PCDDividerWrapper::run))
    .def("meta_generator", &PCDDividerWrapper::meta_generator);

  m.def("init_ros", []() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  });

  m.def("shutdown_ros", []() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  });
}
