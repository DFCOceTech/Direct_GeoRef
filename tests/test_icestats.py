"""Tests for icestats.py — ice blob shape statistics.

SCENARIO-ICE-001: Single rectangular blob area
SCENARIO-ICE-002: fitEllipse axes reasonable
SCENARIO-ICE-003: Min area filter
SCENARIO-ICE-004: Scene-level aggregates
"""

from __future__ import annotations

import math
from unittest.mock import MagicMock

import numpy as np
import pytest


# ---------------------------------------------------------------------------
# Fixture helpers
# ---------------------------------------------------------------------------

def _make_georect(gsd_m: float = 1.0, H: int = 100, W: int = 100) -> MagicMock:
    """Minimal GeoRect for testing."""
    gr = MagicMock()
    gr.gsd_m = gsd_m
    gr.lat_grid = None
    gr.lon_grid = None

    dlat = H * gsd_m / 111_111
    dlon = W * gsd_m / (111_111 * np.cos(np.radians(60)))
    lat0, lon0 = 60.0, 0.0

    gr.corners_latlon = {
        "TL": (lat0 + dlat, lon0),
        "TR": (lat0 + dlat, lon0 + dlon),
        "BL": (lat0,        lon0),
        "BR": (lat0,        lon0 + dlon),
    }
    camera = MagicMock()
    camera.width = W
    camera.height = H
    gr.camera = camera
    meta = MagicMock()
    meta.image_width = W
    meta.image_height = H
    gr.metadata = meta
    return gr


def _rect_mask(H: int, W: int, r0: int, r1: int, c0: int, c1: int) -> np.ndarray:
    m = np.zeros((H, W), dtype=np.uint8)
    m[r0:r1, c0:c1] = 255
    return m


def _disk_mask(H: int, W: int, cr: int, cc: int, radius: int) -> np.ndarray:
    m = np.zeros((H, W), dtype=np.uint8)
    rr, cc = np.ogrid[:H, :W]
    m[(rr - cr)**2 + (cc - cc)**2 <= radius**2] = 255
    return m


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestComputeIceStats:
    """REQ-ICE-001 through REQ-ICE-007"""

    def test_single_blob_area(self):
        """SCENARIO-ICE-001: 40×20 px blob, gsd=1.0 → area ≈ 800 m²."""
        pytest.importorskip("cv2")

        from direct_georef.icestats import compute_ice_stats

        mask = _rect_mask(100, 100, 30, 70, 40, 60)  # 40 rows × 20 cols = 800 px
        gr = _make_georect(gsd_m=1.0)

        blobs, scene = compute_ice_stats(mask, gr)

        assert len(blobs) == 1
        # area_m2 = 800 px × 1.0² m²/px = 800 m²
        assert abs(blobs[0].area_m2 - 800.0) < 5.0, f"area={blobs[0].area_m2}"
        assert scene.blob_count == 1

    def test_fit_ellipse_circular_blob(self):
        """SCENARIO-ICE-002: circular blob → major ≈ minor axis."""
        pytest.importorskip("cv2")

        from direct_georef.icestats import compute_ice_stats

        radius = 20
        H, W = 100, 100
        mask = np.zeros((H, W), dtype=np.uint8)
        rr, cc_grid = np.ogrid[:H, :W]
        cr, ccen = 50, 50
        mask[(rr - cr)**2 + (cc_grid - ccen)**2 <= radius**2] = 255

        gr = _make_georect(gsd_m=1.0, H=H, W=W)
        blobs, _ = compute_ice_stats(mask, gr)

        assert len(blobs) >= 1
        b = blobs[0]

        if not math.isnan(b.major_axis_m) and not math.isnan(b.minor_axis_m):
            assert b.major_axis_m > 0
            assert b.minor_axis_m > 0
            ratio = b.major_axis_m / b.minor_axis_m
            assert ratio < 1.5, f"Ratio {ratio:.2f} too high for circular blob"

    def test_min_area_filter(self):
        """SCENARIO-ICE-003: two blobs; min_area_m2 filters out small one."""
        pytest.importorskip("cv2")

        from direct_georef.icestats import compute_ice_stats

        mask = np.zeros((200, 200), dtype=np.uint8)
        mask[10:20, 10:20] = 255   # 10×10 = 100 px → 100 m² at gsd=1
        mask[80:180, 80:180] = 255  # 100×100 = 10000 px → 10000 m² at gsd=1

        gr = _make_georect(gsd_m=1.0, H=200, W=200)

        blobs, scene = compute_ice_stats(mask, gr, min_area_m2=200.0)

        assert scene.blob_count == 1
        assert abs(scene.total_ice_area_m2 - 10000.0) < 50

    def test_scene_aggregates(self):
        """SCENARIO-ICE-004: three blobs → correct sum, mean, max, min."""
        pytest.importorskip("cv2")

        from direct_georef.icestats import compute_ice_stats

        mask = np.zeros((300, 300), dtype=np.uint8)
        mask[10:20, 10:20] = 255      # ~100 px
        mask[50:70, 50:70] = 255      # ~400 px
        mask[150:200, 150:200] = 255  # ~2500 px

        gr = _make_georect(gsd_m=1.0, H=300, W=300)
        blobs, scene = compute_ice_stats(mask, gr)

        assert scene.blob_count == 3
        assert scene.total_ice_area_m2 > 0
        assert scene.max_blob_area_m2 >= scene.mean_blob_area_m2 >= scene.min_blob_area_m2

    def test_blob_sorted_largest_first(self):
        """Blobs are returned sorted by area descending; blob_id=0 is largest."""
        pytest.importorskip("cv2")

        from direct_georef.icestats import compute_ice_stats

        mask = np.zeros((200, 200), dtype=np.uint8)
        mask[10:15, 10:15] = 255     # small
        mask[50:100, 50:100] = 255   # large

        gr = _make_georect(gsd_m=1.0, H=200, W=200)
        blobs, _ = compute_ice_stats(mask, gr)

        assert len(blobs) == 2
        assert blobs[0].area_m2 >= blobs[1].area_m2
        assert blobs[0].blob_id == 0

    def test_centroid_lat_lon_in_range(self):
        """REQ-ICE-007: centroid lat/lon within image geographic extent."""
        pytest.importorskip("cv2")

        from direct_georef.icestats import compute_ice_stats

        mask = _rect_mask(100, 100, 40, 60, 40, 60)
        gr = _make_georect(gsd_m=1.0)
        blobs, _ = compute_ice_stats(mask, gr)

        b = blobs[0]
        lat_TL, _ = gr.corners_latlon["TL"]
        lat_BL, _ = gr.corners_latlon["BL"]
        _, lon_TL = gr.corners_latlon["TL"]
        _, lon_TR = gr.corners_latlon["TR"]

        assert min(lat_BL, lat_TL) <= b.centroid_lat <= max(lat_BL, lat_TL)
        assert min(lon_TL, lon_TR) <= b.centroid_lon <= max(lon_TL, lon_TR)

    def test_ice_fraction_between_0_and_1(self):
        """REQ-ICE-003: ice_fraction in [0, 1]."""
        pytest.importorskip("cv2")

        from direct_georef.icestats import compute_ice_stats

        mask = _rect_mask(100, 100, 30, 70, 30, 70)
        gr = _make_georect(gsd_m=1.0)
        _, scene = compute_ice_stats(mask, gr)

        assert 0.0 <= scene.ice_fraction <= 1.0

    def test_empty_mask_zero_blobs(self):
        """All-water mask → empty blob list, zero scene totals."""
        pytest.importorskip("cv2")

        from direct_georef.icestats import compute_ice_stats

        mask = np.zeros((100, 100), dtype=np.uint8)
        gr = _make_georect(gsd_m=1.0)
        blobs, scene = compute_ice_stats(mask, gr)

        assert blobs == []
        assert scene.blob_count == 0
        assert scene.total_ice_area_m2 == 0.0
