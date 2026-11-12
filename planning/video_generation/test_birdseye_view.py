"""Checks for ego-up geometry and parked-vehicle filtering."""

import math
from types import SimpleNamespace as S
import unittest

from birdseye_view import ego_geometry
from birdseye_view import latest
from birdseye_view import parked_polygons
from birdseye_view import project_points
import numpy as np


class BirdseyeViewTest(unittest.TestCase):
    def test_view_center_uses_rotated_footprint_center(self):
        pose = S(
            position=S(x=10.0, y=20.0),
            orientation=S(x=0.0, y=0.0, z=math.sin(math.pi / 4), w=math.cos(math.pi / 4)),
        )
        footprint = [(3.0, 1.0), (3.0, -1.0), (-1.0, -1.0), (-1.0, 1.0)]
        center, yaw, outline = ego_geometry(footprint, pose)
        np.testing.assert_allclose(center, [10.0, 21.0])
        np.testing.assert_allclose(outline.mean(axis=0), center)
        np.testing.assert_allclose(project_points(center, center, yaw), [400.0, 258.0])

    def test_ego_forward_projects_up_for_any_heading(self):
        center = np.array([100.0, 200.0])
        points = np.array([[100.0, 200.0], [100.0, 201.0]])
        pixels = project_points(points, center, math.pi / 2)
        self.assertAlmostEqual(pixels[0, 0], pixels[1, 0])
        self.assertAlmostEqual(pixels[0, 1] - pixels[1, 1], 64.5)

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
