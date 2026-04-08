# autoware_trajectory_kinematics_rviz_plugin

RViz2 **panel plugin** that plots kinematic quantities along planned paths and compares **multiple trajectories** on one Qt Charts graph.

## Features

- Subscribe to one or more topics publishing:
  - `autoware_planning_msgs/msg/Trajectory`, or
  - `autoware_internal_planning_msgs/msg/ScoredCandidateTrajectories`
- Choose **X** (time from start or arc length) and **Y** (velocity, acceleration, curvature, lateral acceleration).
- Checkbox which series to plot; optional **fixed X/Y axis ranges**; hover tooltip with crosshair.
- Panel layout and topic list are stored in the RViz config (`save` / `load`).

## Usage

1. Build the workspace with this package in the overlay (standard `colcon build`).
2. Start RViz2, **Add Panel** → select **`autoware_trajectory_kinematics_rviz_plugin/TrajectoryKinematicsPanel`**.
3. Choose message kind, add topics (dropdown or “Other topic”), tick trajectories to plot, set axes.

## ROS interfaces

- **Subscribes** (user-configured): topics you add, as `Trajectory` or `ScoredCandidateTrajectories` depending on the kind selector.
- **Does not publish** topics.

## Dependencies

See `package.xml`. Requires Qt5 **Charts** (rosdep key `libqt5-charts-dev`, installs `libqt5charts5-dev` on Ubuntu), plus `rviz_common`, `rclcpp`, and planning message packages. Qt5 Core/Gui/Widgets are pulled in via `rviz_common`.

## Package layout

In this workspace the package is under `src/tools/common/autoware_trajectory_kinematics_rviz_plugin/`. In other Autoware checkouts the same package may live under a different path; the ROS package name is always `autoware_trajectory_kinematics_rviz_plugin`.
