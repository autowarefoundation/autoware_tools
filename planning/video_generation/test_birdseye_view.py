"""Checks for ego-up geometry and parked-vehicle filtering."""

import math
from types import SimpleNamespace as S
import unittest

from birdseye_view import ego_geometry
from birdseye_view import latest
from birdseye_view import parked_polygons
from birdseye_view import project_points
from birdseye_view import trajectory_view_range
import numpy as np


class BirdseyeViewTest(unittest.TestCase):
    def test_view_center_uses_rotated_front_edge_midpoint(self):
        pose = S(
            position=S(x=10.0, y=20.0),
            orientation=S(x=0.0, y=0.0, z=math.sin(math.pi / 4), w=math.cos(math.pi / 4)),
        )
        footprint = [(3.0, 2.0), (3.0, -1.0), (-1.0, -1.0), (-1.0, 2.0)]
        center, yaw, outline = ego_geometry(footprint, pose)
        np.testing.assert_allclose(center, [9.5, 23.0])
        np.testing.assert_allclose(outline.mean(axis=0), [9.5, 21.0])
        np.testing.assert_allclose(project_points(center, center, yaw), [400.0, 258.0])
        self.assertTrue(np.all(project_points(outline[2:], center, yaw)[:, 1] > 258))

    def test_ego_forward_projects_up_for_any_heading(self):
        center = np.array([100.0, 200.0])
        points = np.array([[100.0, 200.0], [100.0, 201.0]])
        pixels = project_points(points, center, math.pi / 2)
        self.assertAlmostEqual(pixels[0, 0], pixels[1, 0])
        self.assertAlmostEqual(pixels[0, 1] - pixels[1, 1], 64.5)

    def test_square_sidebar_view_keeps_metric_scale(self):
        center = np.array([0.0, 0.0])
        pixels = project_points(
            np.array([[0.0, 0.0], [1.0, 0.0]]),
            center,
            0.0,
            size=(476, 450),
        )
        np.testing.assert_allclose(pixels, [[238.0, 225.0], [238.0, 168.75]])

    def test_view_fits_current_trajectory_and_caps_at_ten_metres(self):
        center = np.array([0.0, 0.0])
        size = (476, 450)
        self.assertEqual(trajectory_view_range({}, center, 0.0, 4.0, 10.0, size), 4.0)
        near = {"ML planner": ("map", np.array([[0.0, 0.0], [7.5, 0.0]]), [])}
        self.assertEqual(trajectory_view_range(near, center, 0.0, 4.0, 10.0, size), 8.0)
        near["ML planner"] = (
            "map",
            np.array([[0.0, 0.0], [7.5, 0.0]]),
            [np.array([[8.3, 0.0], [8.3, 1.0]])],
        )
        self.assertEqual(trajectory_view_range(near, center, 0.0, 4.0, 10.0, size), 9.0)
        far = {"MPPI": ("map", np.array([[0.0, 0.0], [25.0, 0.0]]), [])}
        self.assertEqual(trajectory_view_range(far, center, 0.0, 4.0, 10.0, size), 10.0)
        ego_outline = np.array([[0.0, 1.0], [0.0, -1.0], [-5.0, -1.0], [-5.0, 1.0]])
        self.assertEqual(trajectory_view_range({}, center, 0.0, 4.0, 10.0, size, ego_outline), 8.0)

    def test_stale_samples_disappear(self):
        series = [(100, "scene")]
        self.assertEqual(latest(series, [100], 150, 50), "scene")
        self.assertIsNone(latest(series, [100], 151, 50))

    def test_only_slow_vehicle_objects_are_parked(self):
        def obj(label, speed):
            return S(
                classification=[S(label=label, probability=1.0)],
                kinematics=S(
                    twist_with_covariance=S(twist=S(linear=S(x=speed, y=0.0))),
                    pose_with_covariance=S(
                        pose=S(
                            position=S(x=0.0, y=0.0),
                            orientation=S(x=0.0, y=0.0, z=0.0, w=1.0),
                        )
                    ),
                ),
                shape=S(type=0, dimensions=S(x=4.0, y=2.0)),
            )

        polygons = parked_polygons(S(objects=[obj(1, 0.1), obj(1, 1.0), obj(9, 0.0)]))
        self.assertEqual(len(polygons), 1)
        self.assertEqual(polygons[0].shape, (5, 2))


if __name__ == "__main__":
    unittest.main()
