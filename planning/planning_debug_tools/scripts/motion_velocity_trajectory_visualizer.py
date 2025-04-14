#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2025 TIER IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from autoware_planning_msgs.msg import Trajectory
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node


def calcArcLength(traj, start=0, end=None):
    if end is None:
        end = len(traj)
    s_arr = []
    ds = 0.0
    s_sum = 0.0

    if len(traj) > 0:
        s_arr.append(s_sum)

    for i in range(start + 1, end):
        p0 = traj[i - 1]
        p1 = traj[i]
        dx = p1.pose.position.x - p0.pose.position.x
        dy = p1.pose.position.y - p0.pose.position.y
        ds = np.sqrt(dx**2 + dy**2)
        s_sum += ds
        s_arr.append(s_sum)
    return s_arr


def calcTrajectoryInterval(traj, start, end):
    ds_arr = []

    for i in range(start + 1, end):
        p0 = traj[i - 1]
        p1 = traj[i]
        dx = p1.pose.position.x - p0.pose.position.x
        dy = p1.pose.position.y - p0.pose.position.y
        ds = np.sqrt(dx**2 + dy**2)
        ds_arr.append(ds)
    return ds_arr


def calcTime(traj, start, end):
    t_arr = []
    t_sum = 0.0
    ds_arr = calcTrajectoryInterval(traj, start, end)

    if len(traj) > 0:
        t_arr.append(t_sum)

    for i in range(start, end - 1):
        v = traj[i].longitudinal_velocity_mps
        ds = ds_arr[i - start]
        dt = ds / max(v, 0.1)
        t_sum += dt
        t_arr.append(t_sum)
    return t_arr


def calcClosestIndex(traj, ego_pose):
    closest = -1
    min_dist_squared = 1.0e10
    for i in range(0, len(traj.points)):
        dist_sq = calcSquaredDist2d(ego_pose, traj.points[i].pose)
        if dist_sq < min_dist_squared:
            min_dist_squared = dist_sq
            closest = i
    return closest


def calcSquaredDist2d(p1, p2):
    dx = p1.position.x - p2.position.x
    dy = p1.position.y - p2.position.y
    return dx * dx + dy * dy


class TrajectoryToPlot:
    def __init__(self, label, topic, color, ls, draw_on_update=False):
        self.label = label
        self.topic = topic
        self.color = color
        self.ls = ls
        self.msg = Trajectory()
        self.subscriber = None
        self.plot = None
        self.draw_on_update = draw_on_update
        self.max_s = 0.0
        self.max_vel = 0.0

    def updatePlot(self, ego_pose):
        closest_idx = calcClosestIndex(self.msg, ego_pose)
        x = calcArcLength(self.msg.points)
        x = [a - x[closest_idx] for a in x]  # shift relative to the ego pose arc length
        self.max_s = max(x)
        y = [p.longitudinal_velocity_mps for p in self.msg.points]
        self.max_vel = max(y)
        self.plot.set_data(x, y)


def createTrajectoryCallback(trajectory_to_plot, node):
    def cb(msg):
        trajectory_to_plot.msg = msg
        trajectory_to_plot.updatePlot(node.ego_pose)
        if trajectory_to_plot.draw_on_update:
            node.redraw()

    return cb


class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__("trajectory_visualizer")

        self.fig = plt.figure()
        # trajectory topics to plot
        topic_header = "/planning/scenario_planning/lane_driving/motion_planning/"
        self.trajectories_to_plot = [
            TrajectoryToPlot(
                "Input trajectory (path optimizer)",
                topic_header + "path_optimizer/trajectory",
                "black",
                "-",
            ),
            TrajectoryToPlot(
                "Planning trajectory",
                "/planning/scenario_planning/trajectory",
                "orange",
                "-",
                draw_on_update=True,
            ),
            TrajectoryToPlot(
                "Output trajectory (MVP)",
                topic_header + "motion_velocity_planner/trajectory",
                "red",
                ":",
            ),
            TrajectoryToPlot(
                "Run Out (MVP)",
                topic_header + "motion_velocity_planner/debug/run_out/trajectory",
                "purple",
                ":",
            ),
            # TrajectoryToPlot("Out of Lane (MVP)", topic_header + "motion_velocity_planner/debug/out_of_lane/trajectory", "purple", ":"),
            # TrajectoryToPlot("Stop (MVP)", topic_header + "motion_velocity_planner/debug/obstacle_cruise/trajectory", "purple", ":"),
            # TrajectoryToPlot("Slowdown (MVP)", topic_header + "motion_velocity_planner/debug/obstacle_slowdown/trajectory", "purple", ":"),
            # TrajectoryToPlot("Cruise (MVP)", topic_header + "motion_velocity_planner/debug/obstacle_cruise/trajectory", "purple", ":"),
        ]

        self.ego_pose = Pose()
        self.localization_twist = Twist()

        self.setPlotTrajectoryVelocity()

        self.sub_localization_twist = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.callbackLocalizationOdom, 1
        )
        for trajectory_to_plot in self.trajectories_to_plot:
            trajectory_to_plot.subscriber = self.create_subscription(
                Trajectory,
                trajectory_to_plot.topic,
                createTrajectoryCallback(trajectory_to_plot, self),
                1,
            )

        plt.show(block=False)

    def callbackLocalizationOdom(self, cmd):
        self.ego_pose = cmd.pose.pose
        self.localization_twist = cmd.twist.twist
        if self.current_velocity_plot:
            self.current_velocity_plot.set_data([0], [self.localization_twist.linear.x])

    def redraw(self):
        max_s = 0.0
        max_vel = 0.0
        for trajectory in self.trajectories_to_plot:
            max_s = max(max_s, trajectory.max_s)
            max_vel = max(max_vel, trajectory.max_vel)
        self.ax1.set_xlim(-1.0, max_s + 1.0)
        self.ax1.set_ylim(-1.0, max_vel + 1.0)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def setPlotTrajectoryVelocity(self):
        self.ax1 = plt.subplot(1, 1, 1)
        for trajectory_to_plot in self.trajectories_to_plot:
            (trajectory_to_plot.plot,) = self.ax1.plot(
                [],
                [],
                label=trajectory_to_plot.label,
                ls=trajectory_to_plot.ls,
                color=trajectory_to_plot.color,
            )
        (self.current_velocity_plot,) = self.ax1.plot(
            [], [], label="Current velocity", marker="x", color="red"
        )
        self.ax1.set_title("Motion Velocity Planning - Velocity Profiles")
        self.ax1.legend()
        self.ax1.set_ylabel("vel [m/s]")
        self.ax1.set_xlabel("arc length [m]")
        plt.autoscale(True)
        self.ax1.set_autoscale_on(True)
        self.ax1.margins(1.0, 1.0)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = TrajectoryVisualizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
