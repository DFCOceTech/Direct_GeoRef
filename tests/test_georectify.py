"""Tests for direct georectification.

REQ-GEO-001: Centre pixel maps to camera GPS position (within 1m at nadir)
REQ-GEO-002: Footprint edge lengths match expected GSD × image dimensions
REQ-GEO-003: GSD ≈ flying_height / focal_length_pixels
REQ-GEO-004: Rotation matrix third column is Down (0,0,1) for pure-nadir camera
REQ-GEO-005: Zero yaw/pitch=-90/roll=0 → corners are symmetric around camera
REQ-GEO-006: Flying height <= 0 raises ValueError
REQ-GEO-007: Yaw rotation shifts footprint heading, not position of centre pixel
"""

import numpy as np
import pytest

from direct_georef.camera import CameraModel
from direct_georef.georectify import georectify, _rotation_camera_to_ned
from direct_georef.metadata import ImageMetadata


def _make_meta(lat=64.0, lon=-50.0, alt_rel=100.0,
               yaw=0.0, pitch=-90.0, roll=0.0) -> ImageMetadata:
    return ImageMetadata(
        path="synthetic.jpg",
        latitude=lat, longitude=lon, altitude_abs_m=alt_rel,
        altitude_rel_m=alt_rel,
        gimbal_yaw=yaw, gimbal_pitch=pitch, gimbal_roll=roll,
        image_width=4000, image_height=3000,
    )


def _make_cam(fx=2340.0, fy=2340.0, w=4000, h=3000) -> CameraModel:
    return CameraModel(fx=fx, fy=fy, cx=w/2, cy=h/2, width=w, height=h)


R_EARTH = 6_371_000.0


def _dist_m(p1, p2):
    lat1, lon1 = np.radians(p1[0]), np.radians(p1[1])
    lat2, lon2 = np.radians(p2[0]), np.radians(p2[1])
    dlat = (lat2 - lat1) * R_EARTH
    dlon = (lon2 - lon1) * R_EARTH * np.cos((lat1 + lat2) / 2)
    return np.sqrt(dlat**2 + dlon**2)


class TestRotationMatrix:
    def test_nadir_optical_axis_points_down(self):
        """REQ-GEO-004: for pitch=-90, optical axis (camera z=[0,0,1]) → NED Down."""
        R = _rotation_camera_to_ned(yaw_deg=0, pitch_deg=-90, roll_deg=0)
        z_c = np.array([0, 0, 1.0])
        z_ned = R @ z_c
        np.testing.assert_allclose(z_ned, [0, 0, 1], atol=1e-10)

    def test_level_optical_axis_points_north(self):
        """For pitch=0, yaw=0, optical axis points North (+N in NED)."""
        R = _rotation_camera_to_ned(yaw_deg=0, pitch_deg=0, roll_deg=0)
        z_c = np.array([0, 0, 1.0])
        z_ned = R @ z_c
        np.testing.assert_allclose(z_ned, [1, 0, 0], atol=1e-10)

    def test_yaw_90_optical_axis_points_east(self):
        """Yaw=90° means heading East; at pitch=0 optical axis points East."""
        R = _rotation_camera_to_ned(yaw_deg=90, pitch_deg=0, roll_deg=0)
        z_c = np.array([0, 0, 1.0])
        z_ned = R @ z_c
        np.testing.assert_allclose(z_ned, [0, 1, 0], atol=1e-10)

    def test_rotation_is_orthogonal(self):
        """R must be a proper rotation matrix (R @ Rᵀ = I, det = +1)."""
        for yaw, pitch, roll in [(0, -90, 0), (-76.9, -90, 0), (45, -45, 10)]:
            R = _rotation_camera_to_ned(yaw, pitch, roll)
            np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-10)
            assert abs(np.linalg.det(R) - 1.0) < 1e-10


class TestGeoRectify:
    def test_centre_pixel_at_camera_position(self):
        """REQ-GEO-001: centre pixel maps to camera GPS within 1m."""
        meta = _make_meta(yaw=0.0)
        cam = _make_cam()
        result = georectify(meta, cam, full_grid=True)
        cx, cy = cam.width // 2, cam.height // 2
        lat_c = result.lat_grid[cy, cx]
        lon_c = result.lon_grid[cy, cx]
        d = _dist_m((lat_c, lon_c), (meta.latitude, meta.longitude))
        assert d < 1.0, f"Centre pixel {d:.2f} m from camera GPS"

    def test_footprint_edge_length(self):
        """REQ-GEO-002: TL→TR edge ≈ image_width * GSD within 1%."""
        meta = _make_meta(alt_rel=100.0, yaw=0.0)
        cam = _make_cam(fx=2340.0, fy=2340.0)
        result = georectify(meta, cam)
        width_m = _dist_m(result.corners_latlon['TL'], result.corners_latlon['TR'])
        expected = cam.width * result.gsd_m
        assert abs(width_m - expected) / expected < 0.01

    def test_gsd_formula(self):
        """REQ-GEO-003: GSD ≈ flying_height / focal_length_pixels."""
        h, fx = 120.0, 2340.0
        meta = _make_meta(alt_rel=h)
        cam = _make_cam(fx=fx, fy=fx)
        result = georectify(meta, cam)
        expected_gsd = h / fx
        assert abs(result.gsd_m - expected_gsd) / expected_gsd < 0.001

    def test_negative_height_raises(self):
        """REQ-GEO-006: flying height ≤ 0 raises ValueError."""
        meta = _make_meta(alt_rel=-10.0)
        cam = _make_cam()
        with pytest.raises(ValueError, match="Flying height"):
            georectify(meta, cam)

    def test_corners_symmetric_zero_yaw(self):
        """REQ-GEO-005: zero yaw, pure nadir → footprint symmetric about camera."""
        meta = _make_meta(lat=45.0, lon=10.0, alt_rel=100.0, yaw=0.0)
        cam = _make_cam()
        result = georectify(meta, cam)
        c = result.corners_latlon
        # TL and BL should have same longitude (aligned North-South)
        assert abs(c['TL'][1] - c['BL'][1]) < 1e-6
        # TL and TR should have same latitude
        assert abs(c['TL'][0] - c['TR'][0]) < 1e-6

    def test_yaw_rotates_footprint(self):
        """REQ-GEO-007: different yaw → rotated footprint, same centre."""
        meta0 = _make_meta(yaw=0.0)
        meta45 = _make_meta(yaw=45.0)
        cam = _make_cam()
        r0  = georectify(meta0,  cam)
        r45 = georectify(meta45, cam)
        cx0  = np.mean([c[1] for c in r0.corners_latlon.values()])
        cx45 = np.mean([c[1] for c in r45.corners_latlon.values()])
        # Centres should be in the same place
        assert abs(cx0 - cx45) < 1e-5
        # But corners should differ
        assert abs(r0.corners_latlon['TL'][1] - r45.corners_latlon['TL'][1]) > 1e-4

    def test_full_grid_shape(self):
        meta = _make_meta()
        cam = _make_cam()
        result = georectify(meta, cam, full_grid=True)
        assert result.lat_grid.shape == (cam.height, cam.width)
        assert result.lon_grid.shape == (cam.height, cam.width)

    def test_corners_only_no_grid(self):
        meta = _make_meta()
        cam = _make_cam()
        result = georectify(meta, cam, full_grid=False)
        assert result.lat_grid is None
        assert result.lon_grid is None
        assert len(result.corners_latlon) == 4
