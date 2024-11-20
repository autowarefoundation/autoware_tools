#!/usr/bin/env python3

# Copyright 2024 Proxima Technology Inc, TIER IV
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

import lanelet2
from lanelet2.core import BasicPoint2d
from lanelet2.core import BoundingBox2d
from lanelet2.io import Origin
from lanelet2.io import load
from lanelet2.routing import RoutingGraph
from lanelet2_extension_python.projection import MGRSProjector
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d


class LaneletUtils:
    @staticmethod
    def search_nearest_lanelet(point2d, handler, search_radius=5.0):
        # Set a search radius to create a bounding box around the point
        radius = BasicPoint2d(search_radius, search_radius)
        bb = BoundingBox2d(point2d - radius, point2d + radius)
        # Search for lanelets within the bounding box
        lanelets_at_p = handler.lanelet_map.laneletLayer.search(bb)
        # Find the nearest lanelet to the specified point
        nearest_lanelet = None
        min_distance = float("inf")
        for lanelet_at_p in lanelets_at_p:
            closest_idx = LaneletUtils.closest_segment(lanelet_at_p.centerline, point2d)
            center_point = lanelet_at_p.centerline[closest_idx]
            distance = np.linalg.norm([point2d.x - center_point.x, point2d.y - center_point.y])
            if distance < min_distance:
                min_distance = distance
                nearest_lanelet = lanelet_at_p

        return nearest_lanelet

    @staticmethod
    def closest_segment(lineString, pointToProject2d):
        # Find the closest segment in the lineString to the specified point
        min_dist = float("inf")
        min_idx = None
        for idx in range(len(lineString) - 1):
            point = lineString[idx]
            point_suc = lineString[idx + 1]
            # Calculate the perpendicular distance from the point to the line segment
            dist = LaneletUtils.approximate_distance(
                point, point_suc, pointToProject2d
            )  # In trajectory generation, approximate_distance performs better than perpendicular_distance.
            # dist = LaneletUtils.perpendicular_distance(point, point_suc, pointToProject2d)
            if dist < min_dist:
                min_dist = dist
                min_idx = idx
        return min_idx

    @staticmethod
    def approximate_distance(point1, point2, point):
        # Calculate the perpendicular distance from a point to a line segment
        a = np.linalg.norm([point1.x - point.x, point1.y - point.y])
        b = np.linalg.norm([point2.x - point.x, point2.y - point.y])
        return np.min([a, b])

    """
    @staticmethod
    def perpendicular_distance(point1, point2, point):
        # Calculate the perpendicular distance from a point to a line segment
        a = np.array([point1.x - point2.x, point1.y - point2.y])
        b = np.array([point1.x - point.x, point1.y - point.y])
        return np.linalg.norm(np.cross(a, b) / np.linalg.norm(a))
    """

    @staticmethod
    def interpolate_two_lines(oneLine, anotherLine):
        # Create interpolation parameters based on the length of each line
        t_one = np.linspace(0.0, 1.0, len(oneLine))
        t_another = np.linspace(0.0, 1.0, len(anotherLine))

        # Set up linear interpolation functions for both lines
        linear_interp_one = interp1d(
            t_one, oneLine, axis=0, kind="linear", fill_value="extrapolate"
        )
        linear_interp_another = interp1d(
            t_another, anotherLine, axis=0, kind="linear", fill_value="extrapolate"
        )

        # Define a common interpolation parameter
        t_interp = np.linspace(0.0, 1.0, max(len(oneLine), len(anotherLine)))

        # Perform interpolation between the two lines
        interpolated_points = np.array([(1.0 - t_interp).tolist()]).T * linear_interp_one(
            t_interp
        ) + np.array([t_interp.tolist()]).T * linear_interp_another(t_interp)

        return interpolated_points


class LaneletMapHandler:
    def __init__(self, map_path):
        # Initialize the map projector and load the lanelet map
        longitude = 139.6503
        latitude = 35.6762
        projector = MGRSProjector(Origin(latitude, longitude))

        # to do : error handling when loading map if necessary
        self.lanelet_map = load(map_path, projector)

        # Initialize traffic rules and routing graph
        self.traffic_rules = lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.Vehicle,
        )
        self.routing_graph = RoutingGraph(self.lanelet_map, self.traffic_rules)

        # Store the segmented shortest route
        self.shortest_segmented_route = []

    def get_shortest_path(self, ego_point, goal_point):
        # ego_point = goal_point
        # goal_point = goal_point

        # Find the nearest lanelet and index to start and goal points
        ego_lanelet, idx_ego_lanelet = self.find_nearest_lanelet_with_index(ego_point)
        goal_lanelet, idx_goal_lanelet = self.find_nearest_lanelet_with_index(goal_point)

        if ego_lanelet is None or goal_lanelet is None:
            return None, None

        # Check if start and goal are in the same lanelet
        if ego_lanelet == goal_lanelet:
            ego_lanelet = self.get_following_lanelet(ego_lanelet)
            if ego_lanelet is None:
                return None, None  # Return None if no path can be found

        # Get the shortest path between the two lanelets
        shortest_path = self.routing_graph.getRoute(ego_lanelet, goal_lanelet).shortestPath()

        if shortest_path is None:
            return None, None

        # Segment the path and store the result
        self.shortest_segmented_route = self.segment_path(
            shortest_path, idx_ego_lanelet, idx_goal_lanelet
        )

        x = []
        y = []
        # Store the x and y coordinates
        for segment in self.shortest_segmented_route:
            x += [point[0] for point in segment]
            y += [point[1] for point in segment]

        return np.array(x), np.array(y)

    def segment_path(self, shortest_path, start_idx, end_idx):
        # Segment the shortest path based on start and goal indexes
        lanelet_center_cache = {}
        segmented_route = []

        i = 0
        while i < len(shortest_path):
            lanelet = shortest_path[i]
            # Cache centerline for efficiency
            center_line = lanelet_center_cache.setdefault(
                lanelet.id, self.get_lanelet_centerline(lanelet)
            )

            # Check for lane changes and interpolate if necessary
            if i < len(shortest_path) - 1:
                center_line, plus_lane_idx = self.check_lane_change(
                    lanelet, shortest_path[i + 1], center_line
                )
                i += plus_lane_idx

            # Adjust segments based on path position
            if i == 0:
                segment = center_line[start_idx:]
            elif i == len(shortest_path) - 1:
                segment = center_line[:end_idx]
            else:
                segment = center_line
            segmented_route.append(segment)

            i += 1
        return segmented_route

    def find_nearest_lanelet_with_index(self, point):
        # Find the nearest lanelet and the closest segment index
        point_2D = BasicPoint2d(point[0], point[1])
        lanelet = LaneletUtils.search_nearest_lanelet(point_2D, self)

        if lanelet is None:
            return None, None
        idx = LaneletUtils.closest_segment(lanelet.centerline, point_2D)
        return lanelet, idx

    def get_following_lanelet(self, lanelet):
        # Retrieve the next lanelet if it exists
        following_relations = self.routing_graph.followingRelations(lanelet)
        return following_relations[0].lanelet if following_relations else None

    def get_lanelet_centerline(self, lanelet):
        # Return the centerline points of a lanelet
        return [[point.x, point.y] for point in lanelet.centerline]

    def check_lane_change(self, current_lanelet, next_lanelet, center_line):
        # Check if a lane change to the next lanelet is possible
        for adjacent in self.routing_graph.rightRelations(
            current_lanelet
        ) + self.routing_graph.leftRelations(current_lanelet):
            if adjacent.lanelet == next_lanelet:
                interpolated_line = LaneletUtils.interpolate_two_lines(
                    center_line, self.get_lanelet_centerline(next_lanelet)
                )
                return interpolated_line, 1
        return center_line, 0

    def plot_trajectory_on_map(self, trajectory=None, trajectory_labels=None):
        plt.close()

        # Use interactive mode for plotting
        plt.ion()

        # Set up the plot for displaying the lanelet map
        plt.figure(figsize=(10, 10))
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.title("Lanelet2 Map Boundary Lines")

        # Plot boundaries of each lanelet
        for lanelet in self.lanelet_map.laneletLayer:
            plt.plot(
                [p.x for p in lanelet.leftBound], [p.y for p in lanelet.leftBound], color="gray"
            )
            plt.plot(
                [p.x for p in lanelet.rightBound], [p.y for p in lanelet.rightBound], color="gray"
            )

        # Plot the trajectory if provided
        if trajectory is not None and trajectory_labels is None:
            trajectory_len = len(trajectory)
            plt.plot(
                [trajectory[i, 0] for i in range(trajectory_len)],
                [trajectory[i, 1] for i in range(trajectory_len)],
                linestyle="--",
                linewidth=2,
            )

        elif trajectory is not None and trajectory_labels is not None:
            trajectory_len = len(trajectory)
            previous_label = trajectory_labels[0]

            trajectory_x = []
            trajectory_y = []

            for i in range(trajectory_len):
                if trajectory_labels[i] is not previous_label or i == trajectory_len - 1:
                    plt.plot(
                        trajectory_x,
                        trajectory_y,
                        linestyle="--",
                        linewidth=2,
                        label=previous_label,
                    )
                    trajectory_x = []
                    trajectory_y = []

                trajectory_x.append(trajectory[i, 0])
                trajectory_y.append(trajectory[i, 1])
                previous_label = trajectory_labels[i]

        plt.legend()
        plt.show()
        plt.pause(1.0)
