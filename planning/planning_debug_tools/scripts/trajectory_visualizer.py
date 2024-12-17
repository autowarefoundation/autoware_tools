#!/usr/bin/env python3

# Copyright 2020 Tier IV, Inc.
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

import argparse

from autoware_planning_msgs.msg import Path
from autoware_planning_msgs.msg import PathPoint
from autoware_planning_msgs.msg import Trajectory
from autoware_planning_msgs.msg import TrajectoryPoint
from autoware_vehicle_msgs.msg import VelocityReport
from geometry_msgs.msg import Pose
from matplotlib import animation
import matplotlib.pyplot as plt
import message_filters
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tier4_planning_msgs.msg import PathPointWithLaneId
from tier4_planning_msgs.msg import PathWithLaneId
from tier4_planning_msgs.msg import VelocityLimit

parser = argparse.ArgumentParser()
parser.add_argument("-l", "--length", help="max arclength in plot")
parser.add_argument(
    "-t",
    "--type",
    help="Options  VEL(default): show velocity only, VEL_ACC_JERK: show vel & acc & jerk",
)

parser.add_argument(
    "-v",
    "--max-velocity",
    type=int,
    help="maximum plotting velocity in Matplotlib",
)

args = parser.parse_args()

PLOT_MIN_ARCLENGTH = -5

if args.length is None:
    PLOT_MAX_ARCLENGTH = 200
else:
    PLOT_MAX_ARCLENGTH = int(args.length)
print("max arclength = " + str(PLOT_MAX_ARCLENGTH))

if args.type is None:
    PLOT_TYPE = "VEL"
elif args.type == "VEL":
    PLOT_TYPE = "VEL"
elif args.type == "VEL_ACC_JERK":
    PLOT_TYPE = "VEL_ACC_JERK"
else:
    print("invalid type. set default VEL.")
    PLOT_TYPE = "VEL"
print("plot type = " + PLOT_TYPE)

if args.max_velocity is None:
    MAX_VELOCITY = 20
else:
    MAX_VELOCITY = args.max_velocity


class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__("trajectory_visualizer")

        self.fig = plt.figure()

        self.max_vel = 0.0
        self.min_vel = 0.0
        self.min_accel = 0.0
        self.max_accel = 0.0
        self.min_jerk = 0.0
        self.max_jerk = 0.0

        # update flag
        self.update_ex_vel_lim = False
        self.update_lat_acc_fil = False
        self.update_steer_rate_fil = False
        self.update_raw_traj = False
        self.update_resample_traj = False
        self.update_final_traj = False
        self.update_behavior_path_planner_path = False
        self.update_behavior_velocity_planner_path = False
        self.update_path_smoother_path = False
        self.update_path_optimizer_traj = False
        self.update_motion_velocity_planner_traj = False
        self.update_obstacle_stop_planner_traj = False

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.self_pose = Pose()
        self.self_pose_received = False
        self.localization_vx = 0.0
        self.vehicle_vx = 0.0
        self.velocity_limit = None

        self.trajectory_external_velocity_limited = Trajectory()
        self.trajectory_lateral_acc_filtered = Trajectory()
        self.trajectory_steer_rate_filtered = Trajectory()
        self.trajectory_raw = Trajectory()
        self.trajectory_time_resampled = Trajectory()
        self.trajectory_final = Trajectory()

        self.behavior_path_planner_path = PathWithLaneId()
        self.behavior_velocity_planner_path = Path()
        self.path_smoother_path = Path()
        self.path_optimizer_traj = Trajectory()
        self.motion_velocity_planner_traj = Trajectory()
        self.obstacle_stop_planner_traj = Trajectory()

        self.plotted = [False] * 9
        self.sub_localization_kinematics = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.CallbackLocalizationTwist, 1
        )
        self.sub_vehicle_twist = self.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status", self.CallbackVehicleTwist, 1
        )

        self.sub_external_velocity_limit = self.create_subscription(
            VelocityLimit, "/planning/scenario_planning/max_velocity", self.CallbackVelocityLimit, 1
        )

        # BUFFER_SIZE = 65536*100
        optimizer_debug = "/planning/scenario_planning/velocity_smoother/debug/"
        self.sub_velocity_smoother_external_velocity_limited = message_filters.Subscriber(
            self, Trajectory, optimizer_debug + "trajectory_external_velocity_limited"
        )
        self.sub_velocity_smoother_lateral_acc_filtered = message_filters.Subscriber(
            self, Trajectory, optimizer_debug + "trajectory_lateral_acc_filtered"
        )
        self.sub_velocity_smoother_traj_raw = message_filters.Subscriber(
            self, Trajectory, optimizer_debug + "trajectory_raw"
        )
        self.sub_velocity_smoother_resample = message_filters.Subscriber(
            self, Trajectory, optimizer_debug + "trajectory_time_resampled"
        )
        self.sub_velocity_smoother_steering_rate = message_filters.Subscriber(
            self, Trajectory, optimizer_debug + "trajectory_steering_rate_limited"
        )
        self.sub_final_traj = message_filters.Subscriber(
            self, Trajectory, "/planning/scenario_planning/trajectory"
        )

        lane_driving = "/planning/scenario_planning/lane_driving"
        self.sub_behavior_path_planner = message_filters.Subscriber(
            self, PathWithLaneId, lane_driving + "/behavior_planning/path_with_lane_id"
        )
        self.sub_behavior_velocity_planner = message_filters.Subscriber(
            self, Path, lane_driving + "/behavior_planning/path"
        )
        self.sub_path_smoother = message_filters.Subscriber(
            self,
            Path,
            lane_driving + "/motion_planning/path_smoother/path",
        )
        self.sub_path_optimizer = message_filters.Subscriber(
            self,
            Trajectory,
            lane_driving + "/motion_planning/path_optimizer/trajectory",
        )
        self.sub_motion_velocity_planner = message_filters.Subscriber(
            self, Trajectory, lane_driving + "/motion_planning/motion_velocity_planner/trajectory"
        )
        self.sub_obstacle_stop_planner = message_filters.Subscriber(
            self, Trajectory, lane_driving + "/trajectory"
        )

        self.ts1 = message_filters.ApproximateTimeSynchronizer(
            [
                self.sub_velocity_smoother_external_velocity_limited,
                self.sub_velocity_smoother_lateral_acc_filtered,
                self.sub_velocity_smoother_traj_raw,
                self.sub_velocity_smoother_resample,
                self.sub_velocity_smoother_steering_rate,
            ],
            30,
            0.5,
        )
        self.ts1.registerCallback(self.CallbackMotionVelOptTraj)

        self.ts2 = message_filters.ApproximateTimeSynchronizer(
            [
                self.sub_final_traj,
                self.sub_behavior_path_planner,
                self.sub_behavior_velocity_planner,
                self.sub_path_smoother,
                self.sub_path_optimizer,
                self.sub_motion_velocity_planner,
                self.sub_obstacle_stop_planner,
            ],
            30,
            1,
            0,
        )
        self.ts2.registerCallback(self.CallbackLaneDrivingTraj)

        # main process
        if PLOT_TYPE == "VEL_ACC_JERK":
            self.setPlotTrajectory()
            self.ani = animation.FuncAnimation(
                self.fig, self.plotTrajectory, interval=100, blit=True
            )
        else:
            self.setPlotTrajectoryVelocity()
            self.ani = animation.FuncAnimation(
                self.fig, self.plotTrajectoryVelocity, interval=100, blit=True
            )

        plt.show()

        return

    def CallbackLocalizationTwist(self, cmd):
        self.localization_vx = cmd.twist.twist.linear.x
        self.self_pose = cmd.pose.pose
        self.self_pose_received = True

    def CallbackVehicleTwist(self, cmd):
        self.vehicle_vx = cmd.longitudinal_velocity

    def CallbackVelocityLimit(self, cmd):
        self.velocity_limit = cmd.max_velocity

    def CallbackMotionVelOptTraj(self, cmd1, cmd2, cmd3, cmd4, cmd5):
        self.get_logger().info("CallbackMotionVelOptTraj called")
        self.CallbackTrajExVelLim(cmd1)
        self.CallbackTrajLatAccFiltered(cmd2)
        self.CallbackTrajRaw(cmd3)
        self.CallbackTrajTimeResampled(cmd4)
        self.CallbackTrajSteerRateFiltered(cmd5)

    def CallbackTrajExVelLim(self, cmd):
        self.trajectory_external_velocity_limited = cmd
        self.update_ex_vel_lim = True

    def CallbackTrajLatAccFiltered(self, cmd):
        self.trajectory_lateral_acc_filtered = cmd
        self.update_lat_acc_fil = True

    def CallbackTrajSteerRateFiltered(self, cmd):
        self.trajectory_steer_rate_filtered = cmd
        self.update_steer_rate_fil = True

    def CallbackTrajRaw(self, cmd):
        self.trajectory_raw = cmd
        self.update_raw_traj = True

    def CallbackTrajTimeResampled(self, cmd):
        self.trajectory_time_resampled = cmd
        self.update_resample_traj = True

    def CallbackTrajFinal(self, cmd):
        self.trajectory_final = cmd
        self.update_final_traj = True

    def CallbackLaneDrivingTraj(self, cmd5, cmd6, cmd7, cmd8, cmd9, cmd10, cmd11):
        self.get_logger().info("CallbackLaneDrivingTraj called")
        self.CallbackTrajFinal(cmd5)
        self.CallbackLBehaviorPathPlannerPath(cmd6)
        self.CallbackBehaviorVelocityPlannerPath(cmd7)
        self.CallbackPathSmootherPath(cmd8)
        self.CallbackPathOptimizerTraj(cmd9)
        self.CallbackObstacleStopPlannerTraj(cmd10)
        self.CallbackMotionVelocityPlannerTraj(cmd11)

    def CallbackLBehaviorPathPlannerPath(self, cmd):
        self.behavior_path_planner_path = cmd
        self.update_behavior_path_planner_path = True

    def CallbackBehaviorVelocityPlannerPath(self, cmd):
        self.behavior_velocity_planner_path = cmd
        self.update_behavior_velocity_planner_path = True

    def CallbackPathSmootherPath(self, cmd):
        self.path_smoother_path = cmd
        self.update_path_smoother_path = True

    def CallbackPathOptimizerTraj(self, cmd):
        self.path_optimizer_traj = cmd
        self.update_path_optimizer_traj = True

    def CallbackMotionVelocityPlannerTraj(self, cmd):
        self.motion_velocity_planner_traj = cmd
        self.update_motion_velocity_planner_traj = True

    def CallbackObstacleStopPlannerTraj(self, cmd):
        self.obstacle_stop_planner_traj = cmd
        self.update_obstacle_stop_planner_traj = True

    def setPlotTrajectoryVelocity(self):
        self.ax1 = plt.subplot(1, 1, 1)  # row, col, index(<raw*col)
        (self.im_behavior_path_planner,) = self.ax1.plot(
            [], [], label="0: behavior_path_planner_path", marker=""
        )
        (self.im_behavior_velocity_planner,) = self.ax1.plot(
            [], [], label="1: behavior_velocity_planner_path", marker="", ls="--"
        )
        (self.im_path_smoother,) = self.ax1.plot(
            [], [], label="2: path_smoother_path", marker="", ls="-."
        )
        (self.im_path_optimizer,) = self.ax1.plot(
            [], [], label="3: path_optimizer_traj", marker="", ls="-."
        )
        (self.im_motion_velocity_planner,) = self.ax1.plot(
            [], [], label="4: motion_velocity_planner_traj", marker="", ls="--"
        )
        (self.im_obstacle_stop_planner,) = self.ax1.plot(
            [], [], label="5: obstacle_stop_planner_traj", marker="", ls="--"
        )
        (self.im_raw) = self.ax1.plot([], [], label="6-1: opt input", marker="", ls="--")
        (self.im_external_velocity_limited,) = self.ax1.plot(
            [], [], label="6-2: opt external_velocity_limited", marker="", ls="--"
        )
        (self.im_lat_acc_fil,) = self.ax1.plot(
            [], [], label="6-3: opt lat_acc_filtered", marker=".", ls="--"
        )
        (self.im_steer_rate_fil,) = self.ax1.plot(
            [], [], label="6-4: opt steer_rate_filtered", marker="", ls="-."
        )
        (self.im_resample,) = self.ax1.plot(
            [], [], label="6-5: opt time_resampled", marker="*", ls="--"
        )
        (self.im_final,) = self.ax1.plot([], [], label="6-6: opt final", marker="", ls="-")
        (self.im_localization,) = self.ax1.plot(
            [], [], label="localization twist vx", color="r", marker="*", ls=":", markersize=10
        )
        (self.im_vehicle,) = self.ax1.plot(
            [], [], label="vehicle twist vx", color="k", marker="+", ls=":", markersize=10
        )
        (self.im_external_velocity_limit,) = self.ax1.plot(
            [], [], label="external velocity limit", color="k", marker=""
        )

        self.ax1.set_title("trajectory's velocity")
        self.ax1.legend()
        self.ax1.set_xlim([PLOT_MIN_ARCLENGTH, PLOT_MAX_ARCLENGTH])
        self.ax1.set_ylim([0, MAX_VELOCITY])
        self.ax1.set_ylabel("vel [m/s]")

        return (
            self.im_behavior_path_planner,
            self.im_behavior_velocity_planner,
            self.im_path_smoother,
            self.im_path_optimizer,
            self.im_motion_velocity_planner,
            self.im_obstacle_stop_planner,
            self.im_raw,
            self.im_external_velocity_limited,
            self.im_lat_acc_fil,
            self.im_steer_rate_fil,
            self.im_resample,
            self.im_final,
            self.im_localization,
            self.im_vehicle,
            self.im_external_velocity_limit,
        )

    def plotTrajectoryVelocity(self, data):
        if self.self_pose_received is False:
            self.get_logger().info("plot start but self pose is not received")
            return (
                self.im_behavior_path_planner,
                self.im_behavior_velocity_planner,
                self.im_path_smoother,
                self.im_path_optimizer,
                self.im_motion_velocity_planner,
                self.im_obstacle_stop_planner,
                self.im_raw,
                self.im_external_velocity_limited,
                self.im_lat_acc_fil,
                self.im_steer_rate_fil,
                self.im_resample,
                self.im_final,
                self.im_localization,
                self.im_vehicle,
                self.im_external_velocity_limit,
            )
        self.get_logger().info("plot start")

        if self.update_final_traj:
            self.im_localization.set_data([0], [self.localization_vx])
            self.im_vehicle.set_data([0], [self.vehicle_vx])

            if self.velocity_limit is not None:
                x = [PLOT_MIN_ARCLENGTH, PLOT_MAX_ARCLENGTH]
                y = [self.velocity_limit, self.velocity_limit]
                self.im_external_velocity_limit.set_data(x, y)

                if len(y) != 0:
                    self.min_vel = np.min(y)

        trajectory_data = [
            (
                self.behavior_path_planner_path,
                self.update_behavior_path_planner_path,
                self.im_behavior_path_planner,
            ),
            (
                self.behavior_velocity_planner_path,
                self.update_behavior_velocity_planner_path,
                self.im_behavior_velocity_planner,
            ),
            (self.path_smoother_path, self.update_path_smoother_path, self.im_path_smoother),
            (self.path_optimizer_traj, self.update_path_optimizer_traj, self.im_path_optimizer),
            (
                self.motion_velocity_planner_traj,
                self.update_motion_velocity_planner_traj,
                self.im_motion_velocity_planner,
            ),
            (
                self.obstacle_stop_planner_traj,
                self.update_obstacle_stop_planner_traj,
                self.im_obstacle_stop_planner,
            ),
            (self.trajectory_raw, self.update_raw_traj, self.im_raw),
            (
                self.trajectory_external_velocity_limited,
                self.update_ex_vel_lim,
                self.im_external_velocity_limit,
            ),
            (self.trajectory_lateral_acc_filtered, self.update_lat_acc_fil, self.im_lat_acc_fil),
            (
                self.trajectory_steer_rate_filtered,
                self.update_steer_rate_fil,
                self.im_steer_rate_fil,
            ),
            (self.trajectory_time_resampled, self.update_resample_traj, self.im_resample),
            (self.trajectory_final, self.update_final_traj, self.im_final),
        ]

        # update all trajectory plots
        for traj, update_flag, image in trajectory_data:
            if update_flag:
                x = self.CalcArcLength(traj)
                y = self.ToVelList(traj)
                image.set_data(x, y)
                update_flag = False

        return (
            self.im_behavior_path_planner,
            self.im_behavior_velocity_planner,
            self.im_path_smoother,
            self.im_path_optimizer,
            self.im_motion_velocity_planner,
            self.im_obstacle_stop_planner,
            self.im_raw,
            self.im_external_velocity_limited,
            self.im_lat_acc_fil,
            self.im_steer_rate_fil,
            self.im_resample,
            self.im_final,
            self.im_localization,
            self.im_vehicle,
            self.im_external_velocity_limit,
        )

    def getPoint(self, path_point):
        if isinstance(path_point, (TrajectoryPoint, PathPoint)):
            return path_point
        elif isinstance(path_point, PathPointWithLaneId):
            return path_point.point
        else:
            raise TypeError("invalid path_point argument type")

    def CalcDistance(self, point0, point1):
        p0 = self.getPoint(point0).pose.position
        p1 = self.getPoint(point1).pose.position
        dx = p1.x - p0.x
        dy = p1.y - p0.y
        return np.sqrt(dx**2 + dy**2)

    def CalcArcLength(self, traj):
        if len(traj.points) == 0:
            return

        s_arr = []
        s_sum = 0.0

        closest_id = self.calcClosestIndex(traj)
        for i in range(1, closest_id):
            s_sum -= self.CalcDistance(
                self.getPoint(traj.points[i - 1]), self.getPoint(traj.points[i])
            )
        s_arr.append(s_sum)
        for i in range(1, len(traj.points)):
            s_sum += self.CalcDistance(
                self.getPoint(traj.points[i - 1]), self.getPoint(traj.points[i])
            )
            s_arr.append(s_sum)
        return s_arr

    def ToVelList(self, traj):
        return [self.getPoint(p).longitudinal_velocity_mps for p in traj.points]

    def CalcAcceleration(self, traj):
        a_arr = []
        for i in range(1, len(traj.points) - 1):
            p0 = traj.points[i - 1]
            p1 = traj.points[i]
            v0 = p0.longitudinal_velocity_mps
            v1 = p1.longitudinal_velocity_mps
            v = 0.5 * (v1 + v0)
            dx = p1.pose.position.x - p0.pose.position.x
            dy = p1.pose.position.y - p0.pose.position.y
            ds = np.sqrt(dx**2 + dy**2)
            dt = ds / max(abs(v), 0.001)
            a = (v1 - v0) / dt
            a_arr.append(a)
        if len(traj.points) > 0:
            a_arr.append(0)
            a_arr.append(0)
        return a_arr

    def CalcJerk(self, traj):
        j_arr = []
        for i in range(1, len(traj.points) - 2):
            p0 = traj.points[i - 1]
            p1 = traj.points[i]
            p2 = traj.points[i + 1]
            v0 = p0.longitudinal_velocity_mps
            v1 = p1.longitudinal_velocity_mps
            v2 = p2.longitudinal_velocity_mps

            dx0 = p1.pose.position.x - p0.pose.position.x
            dy0 = p1.pose.position.y - p0.pose.position.y
            ds0 = np.sqrt(dx0**2 + dy0**2)

            dx1 = p2.pose.position.x - p1.pose.position.x
            dy1 = p2.pose.position.y - p1.pose.position.y
            ds1 = np.sqrt(dx1**2 + dy1**2)

            dt0 = ds0 / max(abs(0.5 * (v1 + v0)), 0.001)
            dt1 = ds1 / max(abs(0.5 * (v2 + v1)), 0.001)

            a0 = (v1 - v0) / max(dt0, 0.001)
            a1 = (v2 - v1) / max(dt1, 0.001)
            j = (a1 - a0) / max(dt1, 0.001)
            j_arr.append(j)
        if len(traj.points) > 0:
            j_arr.append(0)
            j_arr.append(0)
            j_arr.append(0)
        return j_arr

    def setPlotTrajectory(self):
        self.ax1 = plt.subplot(3, 1, 1)  # row, col, index(<raw*col)
        (self.im0,) = self.ax1.plot([], [], label="0: raw", marker="")
        (self.im_behavior_path_planner,) = self.ax1.plot(
            [], [], label="3: time_resampled", marker=""
        )
        (self.im_behavior_velocity_planner,) = self.ax1.plot(
            [], [], label="4: final velocity", marker=""
        )
        self.ax1.set_title("trajectory's velocity")
        self.ax1.legend()
        self.ax1.set_xlim([PLOT_MIN_ARCLENGTH, PLOT_MAX_ARCLENGTH])
        self.ax1.set_ylim([0, MAX_VELOCITY])
        self.ax1.set_ylabel("vel [m/s]")

        self.ax2 = plt.subplot(3, 1, 2)
        self.ax2.set_xlim([PLOT_MIN_ARCLENGTH, PLOT_MAX_ARCLENGTH])
        self.ax2.set_ylim([-1, 1])
        self.ax2.set_ylabel("acc [m/ss]")
        (self.im_path_optimizer,) = self.ax2.plot([], [], label="final accel")

        self.ax3 = plt.subplot(3, 1, 3)
        self.ax3.set_xlim([PLOT_MIN_ARCLENGTH, PLOT_MAX_ARCLENGTH])
        self.ax3.set_ylim([-2, 2])
        self.ax3.set_xlabel("arclength [m]")
        self.ax3.set_ylabel("jerk [m/sss]")
        (self.im_obstacle_stop_planner,) = self.ax3.plot([], [], label="final jerk")

        return (
            self.im0,
            self.im_behavior_path_planner,
            self.im_behavior_velocity_planner,
            self.im_path_optimizer,
            self.im_obstacle_stop_planner,
        )

    def plotTrajectory(self, data):
        self.get_logger().info("plot called")

        # copy
        trajectory_raw = self.trajectory_raw
        # trajectory_external_velocity_limited = self.trajectory_external_velocity_limited
        # trajectory_lateral_acc_filtered = self.trajectory_lateral_acc_filtered
        trajectory_time_resampled = self.trajectory_time_resampled
        trajectory_final = self.trajectory_final

        # ax1
        if self.update_raw_traj:
            x = self.CalcArcLength(trajectory_raw)
            y = self.ToVelList(trajectory_raw)
            self.im0.set_data(x, y)
            self.update_raw_traj = False
            if len(y) != 0:
                self.max_vel = max(10.0, np.max(y))
                self.min_vel = np.min(y)
                # change y-range
                self.ax1.set_ylim([self.min_vel - 1.0, self.max_vel + 1.0])

        if self.update_resample_traj:
            x = self.CalcArcLength(trajectory_time_resampled)
            y = self.ToVelList(trajectory_time_resampled)
            self.im_behavior_path_planner.set_data(x, y)
            self.update_resample_traj = False

        if self.update_final_traj:
            x = self.CalcArcLength(trajectory_final)
            y = self.ToVelList(trajectory_final)
            self.im_behavior_velocity_planner.set_data(x, y)
            self.update_final_traj = False

            # ax2
            y = self.CalcAcceleration(trajectory_final)
            if len(y) != 0:
                self.max_accel = max(0.0, np.max(y))
                self.min_accel = min(0.0, np.min(y))
                # change y-range
                self.ax2.set_ylim([self.min_accel - 1.0, self.max_accel + 1.0])
                if len(x) == len(y):
                    self.im_path_optimizer.set_data(x, y)

            # ax3
            y = self.CalcJerk(trajectory_final)
            if len(y) != 0:
                self.max_jerk = max(0.0, np.max(y))
                self.min_jerk = min(0.0, np.min(y))
                # change y-range
                # self.ax3.set_ylim([self.min_jerk - 1.0, self.max_jerk + 1.0])
                self.ax3.set_ylim([-2.0, 2.0])  # fixed range
                if len(x) == len(y):
                    self.im_obstacle_stop_planner.set_data(x, y)

        return (
            self.im0,
            self.im_behavior_path_planner,
            self.im_behavior_velocity_planner,
            self.im_path_optimizer,
            self.im_obstacle_stop_planner,
        )

    def calcClosestIndex(self, path):
        points = np.array(
            [
                [self.getPoint(p).pose.position.x, self.getPoint(p).pose.position.y]
                for p in path.points
            ]
        )
        self_pose = np.array([self.self_pose.position.x, self.self_pose.position.y])
        dists_squared = np.sum((points - self_pose) ** 2, axis=1)
        return np.argmin(dists_squared)

    def closeFigure(self):
        plt.close(self.fig)


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
