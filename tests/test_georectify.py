"""Tests for direct georectification.

REQ-GEO-001: Centre pixel maps to camera GPS position (within 1m at nadir)
REQ-GEO-002: Footprint edge lengths match expected GSD × image dimensions
REQ-GEO-003: GSD ≈ flying_height / focal_length_pixels
REQ-GEO-004: Rotation matrix third column is Down (0,0,1) for pure-nadir camera
REQ-GEO-005: Zero yaw/pitch=-90/roll=0 → corners are symmetric around camera
REQ-GEO-006: Flying height <= 0 raises ValueError
REQ-GEO-007: Yaw rotation shifts footprint heading, not position of centre pixel

Non-DJI additions (Epic 01):
REQ-GEO-003 (new): surface_altitude_m applied to AMSL when altitude_rel_m is None — SCENARIO-GEO-002
REQ-GEO-004 (new): Phase One aerialgps yaw/pitch/roll maps correctly — SCENARIO-GEO-001
REQ-GEO-004 (new): yaw 0–360° range treated identically to ±180° range
"""

import numpy as np
import pytest

from direct_georef.camera import CameraModel, from_sensor_spec
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


def _make_meta_abs(lat=64.0, lon=-50.0, alt_abs=841.0,
                   yaw=0.0, pitch=-90.0, roll=0.0) -> ImageMetadata:
    """Meta with altitude_rel_m=None — simulates non-DJI image using AMSL."""
    return ImageMetadata(
        path="synthetic.jpg",
        latitude=lat, longitude=lon, altitude_abs_m=alt_abs,
        altitude_rel_m=None,
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


class TestSurfaceAltitude:
    """REQ-GEO-003 (new): surface_altitude_m applied when altitude_rel_m is None."""

    def test_surface_altitude_applied_for_amsl(self):
        """SCENARIO-GEO-002: h = alt_abs - surface_alt when altitude_rel_m is None."""
        # alt_abs=500, surface=100 → flying height should be 400
        meta = _make_meta_abs(alt_abs=500.0)
        cam = _make_cam()
        result = georectify(meta, cam, surface_altitude_m=100.0, full_grid=False)
        assert abs(result.flying_height_m - 400.0) < 0.01, \
            f"Expected 400 m flying height, got {result.flying_height_m}"

    def test_surface_altitude_zero_default(self):
        """surface_altitude_m=0 → flying_height = altitude_abs_m."""
        meta = _make_meta_abs(alt_abs=841.083)
        cam = _make_cam()
        result = georectify(meta, cam, surface_altitude_m=0.0, full_grid=False)
        assert abs(result.flying_height_m - 841.083) < 0.01

    def test_relative_altitude_takes_priority(self):
        """altitude_rel_m always wins over AMSL calculation, surface_alt ignored."""
        meta = _make_meta(alt_rel=180.0)  # altitude_rel_m=180, altitude_abs_m=180
        meta.altitude_abs_m = 841.0       # set abs to something very different
        cam = _make_cam()
        result = georectify(meta, cam, surface_altitude_m=0.0, full_grid=False)
        assert abs(result.flying_height_m - 180.0) < 0.01


class TestPhaseOneConventions:
    """REQ-GEO-004 (new): Phase One aerialgps: angle convention is DJI-compatible."""

    def test_nadir_pitch_minus_90_is_nadir(self):
        """aerialgps:GPSIMUPitch=-90.807° produces a near-nadir footprint.

        Corner-averaging a wide-angle camera at non-nadir gives a perspective-
        distorted centroid tens of meters from the nadir point.  We therefore
        use a narrow-FOV camera (tiny image) so perspective error is negligible,
        and verify the footprint centroid is within 5 m of the camera GPS.
        """
        meta = _make_meta(yaw=301.57, pitch=-90.807, roll=0.238)
        # Narrow-FOV camera: 10×10 px → each corner only ~0.2° off-axis
        cam_narrow = CameraModel(
            fx=2340.0, fy=2340.0, cx=5.0, cy=5.0, width=10, height=10
        )
        result = georectify(meta, cam_narrow, full_grid=False)
        c = result.corners_latlon
        centre_lat = (c['TL'][0] + c['TR'][0] + c['BL'][0] + c['BR'][0]) / 4
        centre_lon = (c['TL'][1] + c['TR'][1] + c['BL'][1] + c['BR'][1]) / 4
        d = _dist_m((centre_lat, centre_lon), (meta.latitude, meta.longitude))
        # 0.807° off-nadir at 100m → ~1.4m nadir shift; 5m gives generous margin
        assert d < 5.0, f"Footprint centre {d:.1f} m from camera GPS (expected <5 m)"

    def test_yaw_360_equivalent_to_yaw_0(self):
        """Yaw=360° is identical to yaw=0° (trig functions are 2π-periodic)."""
        meta0   = _make_meta(yaw=0.0)
        meta360 = _make_meta(yaw=360.0)
        cam = _make_cam()
        r0   = georectify(meta0,   cam, full_grid=False)
        r360 = georectify(meta360, cam, full_grid=False)
        for corner in ('TL', 'TR', 'BL', 'BR'):
            assert abs(r0.corners_latlon[corner][0] - r360.corners_latlon[corner][0]) < 1e-8
            assert abs(r0.corners_latlon[corner][1] - r360.corners_latlon[corner][1]) < 1e-8

    def test_phase_one_full_pipeline_gsd(self):
        """SCENARIO-GEO-001: Phase One iXM-100 at 841 m → GSD ≈ 9 cm."""
        meta = _make_meta_abs(
            lat=78.81890, lon=-0.67384,
            alt_abs=841.083,
            yaw=301.571, pitch=-90.807, roll=0.238,
        )
        cam = from_sensor_spec(35.0, 43.9, 32.9, 11664, 8750)
        result = georectify(meta, cam, surface_altitude_m=0.0, full_grid=False)
        assert 0.08 < result.gsd_m < 0.10, f"GSD {result.gsd_m:.4f} m/px out of range"
        assert abs(result.flying_height_m - 841.083) < 0.01
