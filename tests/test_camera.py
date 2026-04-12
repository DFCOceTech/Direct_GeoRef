"""Tests for camera intrinsic model and loaders.

REQ-CAM-001: CameraModel stores fx, fy, cx, cy and exposes 3×3 K matrix
REQ-CAM-002: from_exif resolves sensor size from DJI model database
REQ-CAM-003: from_exif accepts explicit sensor_width_mm/sensor_height_mm
REQ-CAM-004: from_exif raises ValueError for unknown model without explicit sensor
REQ-CAM-005: pixel_to_ray returns unit vectors; centre pixel → [0,0,1] optical axis
REQ-CAM-006: Agisoft cx/cy are offsets from image centre, not from top-left corner
"""

import numpy as np
import pytest

from direct_georef.camera import CameraModel, from_exif


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
