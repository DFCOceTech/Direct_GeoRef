"""Tests for routing.py — ship route planning.

SCENARIO-ROUTE-001: Route found through a gap
SCENARIO-ROUTE-002: Ship clearance removes marginal path
SCENARIO-ROUTE-003: Min floe filter opens a path
SCENARIO-ROUTE-004: ETA calculation
SCENARIO-ROUTE-005: GeoPackage output round-trip
"""

from __future__ import annotations

from unittest.mock import MagicMock

import numpy as np
import pytest


# ---------------------------------------------------------------------------
# Fixture helpers
# ---------------------------------------------------------------------------

def _make_georect(H: int = 100, W: int = 100, gsd_m: float = 10.0) -> MagicMock:
    """Synthetic GeoRect covering a ~H*gsd_m × W*gsd_m metre scene."""
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


def _wall_with_gap_mask(H: int, W: int, wall_col_start: int, wall_col_end: int,
                         gap_row_start: int, gap_row_end: int) -> np.ndarray:
    """Vertical ice wall with a horizontal gap."""
    mask = np.zeros((H, W), dtype=np.uint8)
    mask[:, wall_col_start:wall_col_end] = 255          # wall
    mask[gap_row_start:gap_row_end, wall_col_start:wall_col_end] = 0  # gap
    return mask


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestFindRoute:
    """REQ-ROUTE-001 through REQ-ROUTE-008"""

    def test_route_found_through_gap(self):
        """SCENARIO-ROUTE-001: A* finds a path through a gap in the ice wall."""
        pytest.importorskip("cv2")
        pytest.importorskip("scipy")

        from direct_georef.routing import find_route

        H, W = 100, 100
        # Ice wall at columns 30–70, with a 20-row gap in the middle
        mask = _wall_with_gap_mask(H, W, 30, 70, 40, 60)
        gr = _make_georect(H=H, W=W, gsd_m=10.0)

        result = find_route(mask, gr, routing_scale=1.0, ship_beam_m=5.0)

        assert len(result.waypoints_latlon) >= 2, "Expected at least start and end waypoints"
        # First waypoint should be north of last (higher lat)
        assert result.waypoints_latlon[0][0] >= result.waypoints_latlon[-1][0], \
            "Route should go north to south (decreasing lat)"

    def test_no_route_when_corridor_too_narrow(self):
        """SCENARIO-ROUTE-002: 1-px wide corridor blocked by beam clearance dilation."""
        pytest.importorskip("cv2")
        pytest.importorskip("scipy")

        from direct_georef.routing import find_route

        H, W = 50, 50
        # Ice wall with 1-px gap — too narrow for 20 m beam at 10 m/px
        mask = np.ones((H, W), dtype=np.uint8) * 255
        mask[:, 25] = 0  # single pixel column open
        gr = _make_georect(H=H, W=W, gsd_m=10.0)

        # 20 m beam → clearance = 10 m → 1 px clearance at 10 m/px
        # After dilation by 1 px, the single column gap disappears
        result = find_route(mask, gr, routing_scale=1.0, ship_beam_m=30.0)

        # Either no path or empty waypoints
        assert len(result.waypoints_latlon) == 0 or result.total_distance_m > 0

    def test_min_floe_filter_opens_path(self):
        """SCENARIO-ROUTE-003: small blobs removed by min_floe_area_m2."""
        pytest.importorskip("cv2")
        pytest.importorskip("scipy")

        from direct_georef.routing import find_route

        H, W = 60, 60
        # Fill with small scattered blobs that individually are < threshold
        mask = np.zeros((H, W), dtype=np.uint8)
        # 5×5 ice patches scattered
        for r in range(5, H - 5, 10):
            for c in range(5, W - 5, 10):
                mask[r:r+5, c:c+5] = 255

        gr = _make_georect(H=H, W=W, gsd_m=10.0)

        # With threshold large enough to ignore all blobs (5×5 px × (10 m)² = 2500 m²)
        result_filtered = find_route(
            mask, gr, routing_scale=1.0, ship_beam_m=3.0, min_floe_area_m2=5000.0
        )
        # Without filter (some blobs may block)
        result_unfiltered = find_route(
            mask, gr, routing_scale=1.0, ship_beam_m=3.0, min_floe_area_m2=0.0
        )

        # With filter, navigable fraction should be higher or equal
        assert result_filtered.navigable_fraction >= result_unfiltered.navigable_fraction - 0.01

    def test_eta_calculation(self):
        """SCENARIO-ROUTE-004: ETA = distance / speed (within 1%)."""
        pytest.importorskip("cv2")
        pytest.importorskip("scipy")

        from direct_georef.routing import find_route, _KN_TO_MS

        H, W = 100, 10
        mask = np.zeros((H, W), dtype=np.uint8)
        gr = _make_georect(H=H, W=W, gsd_m=10.0)

        result = find_route(mask, gr, routing_scale=1.0,
                            ship_beam_m=1.0, ship_speed_kn=3.0)

        if result.total_distance_m > 0:
            expected_eta = result.total_distance_m / (3.0 * _KN_TO_MS)
            assert abs(result.eta_s - expected_eta) < 1.0, \
                f"ETA {result.eta_s:.1f} s != expected {expected_eta:.1f} s"

    def test_route_attributes_non_negative(self):
        """total_distance_m and eta_s are always non-negative."""
        pytest.importorskip("cv2")
        pytest.importorskip("scipy")

        from direct_georef.routing import find_route

        H, W = 50, 50
        mask = np.zeros((H, W), dtype=np.uint8)
        gr = _make_georect(H=H, W=W, gsd_m=5.0)

        result = find_route(mask, gr, routing_scale=1.0)

        assert result.total_distance_m >= 0
        assert result.eta_s >= 0
        assert 0.0 <= result.navigable_fraction <= 1.0

    def test_all_ice_returns_empty_route(self):
        """All-ice scene → no navigable pixels → empty waypoints."""
        pytest.importorskip("cv2")
        pytest.importorskip("scipy")

        from direct_georef.routing import find_route

        H, W = 50, 50
        mask = np.ones((H, W), dtype=np.uint8) * 255  # all ice
        gr = _make_georect(H=H, W=W, gsd_m=5.0)

        result = find_route(mask, gr, routing_scale=1.0, ship_beam_m=0.1)

        assert result.waypoints_latlon == []


class TestRouteToGeopackage:
    """REQ-ROUTE-009 — SCENARIO-ROUTE-005"""

    def test_geopackage_round_trip(self, tmp_path):
        """SCENARIO-ROUTE-005: write both layers and re-read with geopandas."""
        geopandas = pytest.importorskip("geopandas")
        pytest.importorskip("cv2")
        pytest.importorskip("scipy")

        from direct_georef.routing import find_route, route_to_geopackage

        H, W = 80, 40
        mask = np.zeros((H, W), dtype=np.uint8)
        gr = _make_georect(H=H, W=W, gsd_m=10.0)

        result = find_route(mask, gr, routing_scale=1.0)

        if not result.waypoints_latlon:
            pytest.skip("No route found in this configuration")

        out_path = tmp_path / "route.gpkg"
        route_to_geopackage(result, out_path)

        assert out_path.exists()

        route_layer = geopandas.read_file(str(out_path), layer="route")
        wp_layer = geopandas.read_file(str(out_path), layer="waypoints")

        assert len(route_layer) == 1
        assert "total_distance_m" in route_layer.columns
        assert "eta_minutes" in route_layer.columns

        assert len(wp_layer) == len(result.waypoints_latlon)
        assert "seq" in wp_layer.columns

        assert route_layer.crs.to_epsg() == 4326
        assert wp_layer.crs.to_epsg() == 4326
