"""Tests for camera intrinsic model and loaders.

REQ-CAM-001: CameraModel stores fx, fy, cx, cy and exposes 3×3 K matrix
REQ-CAM-002: dist_coeffs follows OpenCV [k1,k2,p1,p2,k3] convention
REQ-CAM-003: from_exif resolves sensor size from DJI model database
REQ-CAM-004: from_exif accepts explicit sensor_width_mm/sensor_height_mm
REQ-CAM-005: from_exif raises ValueError for unknown model without explicit sensor
REQ-CAM-006: pixel_to_ray returns unit vectors; centre pixel → [0,0,1] optical axis
REQ-CAM-007: Agisoft cx/cy are offsets from image centre, not from top-left corner

Non-DJI additions (Epic 01):
REQ-CAM-005 (new): from_sensor_spec builds CameraModel from physical dimensions
REQ-CAM-006 (new): Phase One iXM sensors in aerial sensor database — SCENARIO-CAM-001/002/003
"""

import numpy as np
import pytest

from direct_georef.camera import CameraModel, from_exif, from_sensor_spec


class TestCameraModel:
    def test_K_matrix_shape(self):
        """REQ-CAM-001"""
        cam = CameraModel(fx=1000, fy=1000, cx=500, cy=400)
        K = cam.K
        assert K.shape == (3, 3)
        assert K[0, 0] == 1000
        assert K[1, 1] == 1000
        assert K[0, 2] == 500
        assert K[1, 2] == 400
        assert K[2, 2] == 1.0

    def test_dist_coeffs_order(self):
        """dist_coeffs follows OpenCV [k1,k2,p1,p2,k3] convention"""
        cam = CameraModel(fx=1000, fy=1000, cx=500, cy=400,
                          k1=0.1, k2=0.2, p1=0.3, p2=0.4, k3=0.5)
        d = cam.dist_coeffs
        assert d[0] == 0.1  # k1
        assert d[1] == 0.2  # k2
        assert d[2] == 0.3  # p1
        assert d[3] == 0.4  # p2
        assert d[4] == 0.5  # k3

    def test_has_distortion_false(self):
        cam = CameraModel(fx=1000, fy=1000, cx=500, cy=400)
        assert not cam.has_distortion

    def test_has_distortion_true(self):
        cam = CameraModel(fx=1000, fy=1000, cx=500, cy=400, k1=0.01)
        assert cam.has_distortion

    def test_pixel_to_ray_centre(self):
        """REQ-CAM-005: centre pixel gives optical axis direction [0,0,1]."""
        cam = CameraModel(fx=1000, fy=1000, cx=500, cy=400)
        ray = cam.pixel_to_ray(500, 400)
        np.testing.assert_allclose(ray, [0, 0, 1], atol=1e-10)

    def test_pixel_to_ray_unit(self):
        """REQ-CAM-005: returned ray is always a unit vector."""
        cam = CameraModel(fx=1000, fy=1000, cx=500, cy=400)
        for (u, v) in [(0, 0), (1000, 800), (250, 200), (750, 600)]:
            ray = cam.pixel_to_ray(u, v)
            assert abs(np.linalg.norm(ray) - 1.0) < 1e-10


class TestFromExif:
    def test_fc300c_sensor_lookup(self):
        """REQ-CAM-002: FC300C sensor size resolved from database."""
        cam = from_exif(3.61, 4000, 3000, model="FC300C")
        # Sensor 6.17 × 4.55 mm → px = 6.17/4000 = 0.0015425
        expected_fx = 3.61 / (6.17 / 4000)
        assert abs(cam.fx - expected_fx) < 0.1

    def test_fc300c_null_padded_model(self):
        """REQ-CAM-002: model string with null-byte padding is handled."""
        cam = from_exif(3.61, 4000, 3000, model="FC300C\x00\x00\x00")
        assert cam.fx > 0

    def test_explicit_sensor(self):
        """REQ-CAM-003: explicit sensor dimensions override database."""
        cam = from_exif(4.0, 4000, 3000,
                        sensor_width_mm=6.0, sensor_height_mm=4.5)
        expected_fx = 4.0 / (6.0 / 4000)
        assert abs(cam.fx - expected_fx) < 0.1

    def test_unknown_model_raises(self):
        """REQ-CAM-004: unknown model without explicit sensor raises ValueError."""
        with pytest.raises(ValueError, match="sensor size"):
            from_exif(3.61, 4000, 3000, model="UNKNOWN_CAM")

    def test_principal_point_image_centre(self):
        """Principal point defaults to image centre."""
        cam = from_exif(3.61, 4000, 3000, model="FC300C")
        assert cam.cx == 2000.0
        assert cam.cy == 1500.0

    def test_label_contains_model(self):
        cam = from_exif(3.61, 4000, 3000, model="FC300C")
        assert "FC300C" in cam.label

    def test_phase_one_ixm100_lookup(self):
        """REQ-CAM-006 (new): Phase One IXM-100 resolved from aerial sensor database.
        SCENARIO-CAM-002
        """
        cam = from_exif(35.0, 11664, 8750, model="IXM-100")
        expected_fx = 35.0 / (43.9 / 11664)
        expected_fy = 35.0 / (32.9 / 8750)
        assert abs(cam.fx - expected_fx) < 1.0, f"fx {cam.fx:.1f} != {expected_fx:.1f}"
        assert abs(cam.fy - expected_fy) < 1.0, f"fy {cam.fy:.1f} != {expected_fy:.1f}"
        assert cam.cx == 11664 / 2
        assert cam.cy == 8750 / 2


class TestFromSensorSpec:
    """REQ-CAM-005 (new): from_sensor_spec builds CameraModel from physical dimensions."""

    def test_phase_one_ixm100_35mm(self):
        """SCENARIO-CAM-001: Phase One iXM-100 at 35 mm focal length."""
        cam = from_sensor_spec(
            focal_length_mm=35.0,
            sensor_width_mm=43.9,
            sensor_height_mm=32.9,
            image_width=11664,
            image_height=8750,
        )
        expected_fx = 35.0 / (43.9 / 11664)
        expected_fy = 35.0 / (32.9 / 8750)
        assert abs(cam.fx - expected_fx) < 1.0, f"fx {cam.fx:.1f} != {expected_fx:.1f}"
        assert abs(cam.fy - expected_fy) < 1.0, f"fy {cam.fy:.1f} != {expected_fy:.1f}"

    def test_principal_point_at_centre(self):
        """REQ-CAM-007: principal point is image centre."""
        cam = from_sensor_spec(35.0, 43.9, 32.9, 11664, 8750)
        assert cam.cx == 11664 / 2
        assert cam.cy == 8750 / 2

    def test_square_pixel_sensor(self):
        """Sensor with equal pixel pitch → fx == fy."""
        # 36 × 24 mm sensor, 6000 × 4000 px: pitch = 6 µm both axes
        cam = from_sensor_spec(50.0, 36.0, 24.0, 6000, 4000)
        assert abs(cam.fx - cam.fy) < 0.01

    def test_label_indicates_source(self):
        """SCENARIO-CAM-003: from_sensor_spec works for any sensor."""
        cam = from_sensor_spec(35.0, 43.9, 32.9, 11664, 8750)
        assert "spec" in cam.label.lower()

    def test_gsd_at_841m(self):
        """At 841 m flying height, GSD ≈ 9 cm for Phase One iXM-100 at 35 mm."""
        cam = from_sensor_spec(35.0, 43.9, 32.9, 11664, 8750)
        f_px = (cam.fx + cam.fy) / 2.0
        gsd = 841.0 / f_px
        assert 0.08 < gsd < 0.10, f"GSD {gsd:.4f} m/px out of expected range"
