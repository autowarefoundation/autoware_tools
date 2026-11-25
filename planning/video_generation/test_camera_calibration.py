"""Checks reconstruction of missing camera static TF from vehicle calibration."""

from pathlib import Path
import tempfile
import unittest

from generate_bag_video import calibrated_camera_transforms


class CameraCalibrationTest(unittest.TestCase):
    def test_camera_chain_and_optical_rotation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "sensors_calibration.yaml").write_text(
                "base_link:\n"
                "  sensor_kit_base_link: {x: 4, y: 1, z: 3, roll: 0, pitch: 0, yaw: 0}\n"
            )
            (root / "sensor_kit_calibration.yaml").write_text(
                "sensor_kit_base_link:\n"
                "  front/camera_link: {x: 1, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0}\n"
            )
            transforms = calibrated_camera_transforms(root, {"front/camera_optical_link"})

        self.assertEqual(
            [(item.header.frame_id, item.child_frame_id) for item in transforms],
            [
                ("base_link", "sensor_kit_base_link"),
                ("sensor_kit_base_link", "front/camera_link"),
                ("front/camera_link", "front/camera_optical_link"),
            ],
        )
        self.assertEqual(transforms[0].transform.translation.x, 4)
        self.assertEqual(transforms[1].transform.translation.x, 1)
        rotation = transforms[2].transform.rotation
        self.assertAlmostEqual(abs(rotation.x), 0.5)
        self.assertAlmostEqual(abs(rotation.y), 0.5)
        self.assertAlmostEqual(abs(rotation.z), 0.5)
        self.assertAlmostEqual(abs(rotation.w), 0.5)
        self.assertAlmostEqual(rotation.x**2 + rotation.y**2 + rotation.z**2 + rotation.w**2, 1)

    def test_nonfinite_calibration_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "sensors_calibration.yaml").write_text(
                "base_link:\n"
                "  sensor_kit_base_link: {x: .nan, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0}\n"
            )
            (root / "sensor_kit_calibration.yaml").write_text(
                "sensor_kit_base_link:\n"
                "  front/camera_link: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0}\n"
            )
            with self.assertRaisesRegex(ValueError, "Nonfinite calibration"):
                calibrated_camera_transforms(root, {"front/camera_optical_link"})


if __name__ == "__main__":
    unittest.main()
