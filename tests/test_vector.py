"""Tests for vector.py — ice polygon export.

SCENARIO-VEC-001: Single ice polygon extracted
SCENARIO-VEC-002: Min area filter excludes small patches
SCENARIO-VEC-003: Sequence export merges images
SCENARIO-VEC-004: GeoPackage round-trip
"""

from __future__ import annotations

import os
import tempfile
from unittest.mock import MagicMock

import numpy as np
import pytest

# ---------------------------------------------------------------------------
# Synthetic GeoRect fixture
# ---------------------------------------------------------------------------

def _make_georect(gsd_m: float = 1.0) -> MagicMock:
    """Minimal GeoRect-like object for testing.

    Covers a ~100×100 m scene centred near 60°N, 0°E.
    Corners are set so bilinear interp maps pixel (0,0)→TL and (99,99)→BR.
    """
    gr = MagicMock()
    gr.gsd_m = gsd_m
    gr.lat_grid = None
    gr.lon_grid = None

    # ~100 m in lat ≈ 0.0009°; ~100 m in lon at 60°N ≈ 0.0018°
    dlat = 100 * gsd_m / 111_111
    dlon = 100 * gsd_m / (111_111 * np.cos(np.radians(60)))
    lat0, lon0 = 60.0, 0.0

    gr.corners_latlon = {
        "TL": (lat0 + dlat, lon0),
        "TR": (lat0 + dlat, lon0 + dlon),
        "BL": (lat0,        lon0),
        "BR": (lat0,        lon0 + dlon),
    }

    # Camera/metadata dims
    camera = MagicMock()
    camera.width = 100
    camera.height = 100
    gr.camera = camera
    meta = MagicMock()
    meta.image_width = 100
    meta.image_height = 100
    gr.metadata = meta

    return gr


def _make_mask_with_patches(size: int = 100) -> np.ndarray:
    """100×100 mask with a 20×20 patch and a 4×4 patch."""
    mask = np.zeros((size, size), dtype=np.uint8)
    # Large patch
    mask[40:60, 40:60] = 255   # 20×20 = 400 px
    # Small patch
    mask[5:9, 5:9] = 255       # 4×4 = 16 px
    return mask


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestIceMaskToPolygons:
    """REQ-VEC-001, REQ-VEC-002, REQ-VEC-006 — SCENARIO-VEC-001, VEC-002"""

    def test_single_polygon_extracted(self):
        """SCENARIO-VEC-001: one ice patch → one polygon with area > 0."""
        pytest.importorskip("geopandas")
        pytest.importorskip("cv2")

        from direct_georef.vector import ice_mask_to_polygons

        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[40:60, 40:60] = 255  # 20×20 patch
        gr = _make_georect(gsd_m=1.0)

        gdf = ice_mask_to_polygons(mask, gr)

        assert len(gdf) == 1, "Expected exactly one polygon"
        assert gdf.iloc[0]["area_m2"] > 0
        assert gdf.geometry.iloc[0].is_valid
        assert gdf.crs.to_epsg() == 4326

    def test_schema_columns(self):
        """REQ-VEC-006: required columns are present."""
        pytest.importorskip("geopandas")
        pytest.importorskip("cv2")

        from direct_georef.vector import ice_mask_to_polygons

        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[40:60, 40:60] = 255
        gr = _make_georect()

        gdf = ice_mask_to_polygons(mask, gr, image_idx=3)

        for col in ["image_idx", "blob_id", "area_m2", "geometry"]:
            assert col in gdf.columns, f"Missing column: {col}"
        assert gdf.iloc[0]["image_idx"] == 3

    def test_min_area_filter_excludes_small(self):
        """SCENARIO-VEC-002: small patch filtered out when min_area_m2 > small area."""
        pytest.importorskip("geopandas")
        pytest.importorskip("cv2")

        from direct_georef.vector import ice_mask_to_polygons

        mask = _make_mask_with_patches()
        gr = _make_georect(gsd_m=1.0)

        # Small patch area ≈ 16 m² (4×4 px × 1 m²/px)
        # Large patch area ≈ 400 m² (20×20 px × 1 m²/px)
        gdf = ice_mask_to_polygons(mask, gr, min_area_m2=50.0)

        assert len(gdf) == 1, "Only the large patch should survive"
        assert gdf.iloc[0]["area_m2"] > 100

    def test_empty_mask_returns_empty_gdf(self):
        """All-water mask → empty GeoDataFrame."""
        pytest.importorskip("geopandas")
        pytest.importorskip("cv2")

        from direct_georef.vector import ice_mask_to_polygons

        mask = np.zeros((100, 100), dtype=np.uint8)
        gr = _make_georect()

        gdf = ice_mask_to_polygons(mask, gr)

        assert len(gdf) == 0
        assert gdf.crs.to_epsg() == 4326

    def test_area_m2_approximately_correct(self):
        """Area reported ≈ area_px × gsd_m² (within 10% for cv2 contour approx)."""
        pytest.importorskip("geopandas")
        pytest.importorskip("cv2")

        from direct_georef.vector import ice_mask_to_polygons

        mask = np.zeros((200, 200), dtype=np.uint8)
        mask[80:120, 80:120] = 255   # 40×40 = 1600 px
        gr = _make_georect(gsd_m=2.0)  # 2 m/px → expected ~6400 m²
        gr.camera.width = 200
        gr.camera.height = 200
        gr.metadata.image_width = 200
        gr.metadata.image_height = 200

        gdf = ice_mask_to_polygons(mask, gr)
        area = gdf.iloc[0]["area_m2"]

        assert 4000 < area < 9000, f"area_m2={area} far from expected ~6400"


class TestMasksToPolygons:
    """REQ-VEC-005 — SCENARIO-VEC-003"""

    def test_sequence_merges_image_idx(self):
        """SCENARIO-VEC-003: two masks → image_idx 0 and 1 in merged GDF."""
        pytest.importorskip("geopandas")
        pytest.importorskip("cv2")

        from direct_georef.vector import masks_to_polygons

        mask1 = np.zeros((100, 100), dtype=np.uint8)
        mask1[40:60, 40:60] = 255

        mask2 = np.zeros((100, 100), dtype=np.uint8)
        mask2[10:30, 10:30] = 255

        gr = _make_georect()

        gdf = masks_to_polygons([mask1, mask2], [gr, gr])

        assert len(gdf) == 2
        assert set(gdf["image_idx"].tolist()) == {0, 1}

    def test_mismatched_lengths_raises(self):
        """Mismatched masks/georects → ValueError."""
        pytest.importorskip("geopandas")
        pytest.importorskip("cv2")

        from direct_georef.vector import masks_to_polygons

        with pytest.raises(ValueError):
            masks_to_polygons([np.zeros((10, 10), dtype=np.uint8)], [])


class TestToGeopackage:
    """REQ-VEC-003 — SCENARIO-VEC-004"""

    def test_geopackage_round_trip(self, tmp_path):
        """SCENARIO-VEC-004: write .gpkg and re-read with geopandas."""
        geopandas = pytest.importorskip("geopandas")
        pytest.importorskip("cv2")

        from direct_georef.vector import ice_mask_to_polygons, to_geopackage

        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[40:60, 40:60] = 255
        gr = _make_georect()

        gdf = ice_mask_to_polygons(mask, gr)
        out_path = tmp_path / "ice.gpkg"
        to_geopackage(gdf, out_path, layer="ice")

        assert out_path.exists()
        loaded = geopandas.read_file(str(out_path), layer="ice")
        assert len(loaded) == len(gdf)
        assert loaded.crs.to_epsg() == 4326
