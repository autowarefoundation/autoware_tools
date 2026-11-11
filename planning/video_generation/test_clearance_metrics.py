"""Geometry and selector checks for the offline clearance plot."""

import math
from types import SimpleNamespace as S
import unittest

from clearance_metrics import _at_pose
from clearance_metrics import _object_polygon
from clearance_metrics import _selector_ids
from clearance_metrics import road_border_lines
from clearance_metrics import visible_ml_road_border
import numpy as np
from shapely.geometry import Polygon


class ClearanceMetricsTest(unittest.TestCase):
    def test_live_ml_clearance_fills_event_margins_until_reference_is_available(self):
        reference = (np.array([10, 20, 30, 40]), np.array([2.0, 2.0, 2.0, 2.0]))
        live = (
            np.array([10, 17, 20, 30, 35, 40, 50]),
            np.array([1.4, 1.4, 0.1, 1.0, 1.3, 1.5, 1.49]),
        )
        shown_reference, shown_live = visible_ml_road_border(
            reference, live, [S(start=15, end=40)], 1.5
        )
        np.testing.assert_allclose(shown_reference[1], [np.nan, 2.0, 2.0, np.nan])
        np.testing.assert_allclose(shown_live[1], [1.4, 1.4, np.nan, np.nan, 1.3, np.nan, 1.49])

    def test_live_ml_clearance_without_events(self):
        reference = (np.array([10]), np.array([2.0]))
        live = (np.array([10, 20]), np.array([1.4, 1.5]))
        shown_reference, shown_live = visible_ml_road_border(reference, live, [], 1.5)
        np.testing.assert_allclose(shown_reference[1], [np.nan])
        np.testing.assert_allclose(shown_live[1], [1.4, np.nan])

    def test_road_border_strip_recovers_uninflated_centerline(self):
        def point(x, y):
            return S(x=x, y=y)

        marker = S(
            ns="road_borders",
            action=0,
            type=11,
            header=S(frame_id="map"),
            points=[
                point(0, -0.1),
                point(0, 0.1),
                point(10, -0.1),
                point(10, 0.1),
                point(10, -0.1),
                point(0, 0.1),
            ],
        )
        frame, tree, count = road_border_lines(S(markers=[marker]))
        footprint = Polygon([(4, 1), (6, 1), (6, 2), (4, 2)])
        self.assertEqual((frame, count), ("map", 1))
        self.assertAlmostEqual(footprint.distance(tree.nearest(footprint)), 1.0)

    def test_rotated_ego_footprint_distance(self):
        angle = math.pi / 2
        pose = S(
            position=S(x=10, y=20),
            orientation=S(x=0, y=0, z=math.sin(angle / 2), w=math.cos(angle / 2)),
        )
        polygon = Polygon(_at_pose([(2, 1), (2, -1), (-1, -1), (-1, 1)], pose))
        self.assertAlmostEqual(polygon.bounds[0], 9)
        self.assertAlmostEqual(polygon.bounds[2], 11)
        self.assertAlmostEqual(polygon.bounds[3], 22)

    def test_only_parked_avoidance_decisions_are_selected(self):
        values = {
            "selector/status": "ready",
            "selector/objects/abc/reason": "parked_off_lane_center",
            "selector/objects/abc/disposition": "avoidance",
            "selector/objects/def/reason": "traffic_lane_or_recent_motion",
            "selector/objects/def/disposition": "yield",
            "selector/objects/ghi/reason": "parked_off_lane_center",
            "selector/objects/ghi/disposition": "yield",
        }
        status = S(
            name="trajectory_modifier: mppi_cost_breakdown",
            values=[S(key=key, value=value) for key, value in values.items()],
        )
        self.assertEqual(_selector_ids(S(status=[status])), {"abc"})

    def test_selected_box_uses_object_shape_and_pose(self):
        obj = S(
            kinematics=S(
                pose_with_covariance=S(
                    pose=S(
                        position=S(x=5, y=0),
                        orientation=S(x=0, y=0, z=0, w=1),
                    )
                )
            ),
            shape=S(type=0, dimensions=S(x=4, y=2)),
        )
        obstacle = _object_polygon(obj)
        ego = Polygon([(0, -1), (2, -1), (2, 1), (0, 1)])
        self.assertAlmostEqual(ego.distance(obstacle), 1.0)


if __name__ == "__main__":
    unittest.main()
