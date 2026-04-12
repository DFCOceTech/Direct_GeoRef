"""Ice polygon vector export.

Converts binary ice detection masks to georeferenced polygon GeoDataFrames
and exports them as GeoPackage files.

REQ coverage: REQ-VEC-001 through REQ-VEC-006
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

try:
    import cv2  # type: ignore
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False

try:
    import geopandas as gpd  # type: ignore
    from shapely.geometry import Polygon  # type: ignore
    _GPD_AVAILABLE = True
except ImportError:
    _GPD_AVAILABLE = False

from .georectify import GeoRect


# ---------------------------------------------------------------------------
# Coordinate helper
# ---------------------------------------------------------------------------

def _pixel_to_latlon(
    u: np.ndarray,
    v: np.ndarray,
    georect: GeoRect,
    _wh: Optional[tuple[int, int]] = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Bilinear interpolation from pixel (col u, row v) to geographic lat/lon.

    Uses the four GeoRect corner GCPs (TL, TR, BL, BR).  For nadir imagery
    with a well-calibrated camera this is accurate to within a fraction of a GSD.
    If ``georect.lat_grid`` is not None, uses nearest-pixel lookup instead
    (more accurate for off-nadir imagery with large footprints).

    Parameters
    ----------
    u : column indices (float), shape arbitrary
    v : row indices (float), shape arbitrary
    _wh : (width, height) of the coordinate space for u/v.  If None, uses the
        GeoRect camera/metadata image dimensions.  Pass the mask shape when u/v
        come from a downscaled mask so bilinear parameters stay in [0, 1].

    Returns
    -------
    lat_deg, lon_deg : arrays of same shape as u/v
    """
    u = np.asarray(u, dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)

    if _wh is not None:
        W, H = _wh
    else:
        W = georect.camera.width or georect.metadata.image_width
        H = georect.camera.height or georect.metadata.image_height

    if georect.lat_grid is not None:
        # Nearest-pixel lookup in the full grid (always full-res dims)
        W_full = georect.lat_grid.shape[1]
        H_full = georect.lat_grid.shape[0]
        # Map u/v from _wh space → full-res space
        u_full = u * (W_full - 1) / max(W - 1, 1)
        v_full = v * (H_full - 1) / max(H - 1, 1)
        col = np.clip(np.round(u_full).astype(int), 0, W_full - 1)
        row = np.clip(np.round(v_full).astype(int), 0, H_full - 1)
        return georect.lat_grid[row, col], georect.lon_grid[row, col]

    # Bilinear interpolation from 4 corner GCPs
    t = u / max(W - 1, 1)   # horizontal parameter [0, 1]
    s = v / max(H - 1, 1)   # vertical   parameter [0, 1]

    lat_TL, lon_TL = georect.corners_latlon["TL"]
    lat_TR, lon_TR = georect.corners_latlon["TR"]
    lat_BL, lon_BL = georect.corners_latlon["BL"]
    lat_BR, lon_BR = georect.corners_latlon["BR"]

    lat = (1 - s) * (1 - t) * lat_TL + (1 - s) * t * lat_TR \
        +       s  * (1 - t) * lat_BL +       s  * t * lat_BR
    lon = (1 - s) * (1 - t) * lon_TL + (1 - s) * t * lon_TR \
        +       s  * (1 - t) * lon_BL +       s  * t * lon_BR

    return lat, lon


# ---------------------------------------------------------------------------
# Contour → Shapely polygon
# ---------------------------------------------------------------------------

def _contour_to_polygon(
    contour: np.ndarray,
    georect: GeoRect,
    _wh: Optional[tuple[int, int]] = None,
) -> Optional[Polygon]:
    """Convert an OpenCV contour (N×1×2 int32) to a shapely Polygon in WGS-84.

    Returns None if the contour has fewer than 3 points (degenerate polygon).
    """
    pts = contour.reshape(-1, 2)  # (N, 2) as [col, row]
    if len(pts) < 3:
        return None

    u = pts[:, 0].astype(np.float64)
    v = pts[:, 1].astype(np.float64)

    lat, lon = _pixel_to_latlon(u, v, georect, _wh=_wh)
    coords = list(zip(lon, lat))  # GeoJSON convention: (lon, lat)

    try:
        poly = Polygon(coords)
        if not poly.is_valid:
            poly = poly.buffer(0)  # attempt repair
        return poly if poly.is_valid and not poly.is_empty else None
    except Exception:
        return None


# ---------------------------------------------------------------------------
# Main public API
# ---------------------------------------------------------------------------

def ice_mask_to_polygons(
    mask: np.ndarray,
    georect: GeoRect,
    min_area_m2: float = 0.0,
    image_idx: int = 0,
) -> "gpd.GeoDataFrame":
    """Convert a binary ice mask to a GeoDataFrame of georeferenced polygons.

    Parameters
    ----------
    mask : (H, W) uint8 array; 255 = ice, 0 = water.
    georect : GeoRect from ``georectify()``.
    min_area_m2 : Polygons with georeferenced area below this threshold are dropped.
    image_idx : Source image index; written to the ``image_idx`` column.

    Returns
    -------
    GeoDataFrame with CRS EPSG:4326 and columns:
      image_idx, blob_id, area_m2, geometry
    """
    if not _CV2_AVAILABLE:
        raise ImportError("opencv-python is required for ice_mask_to_polygons()")
    if not _GPD_AVAILABLE:
        raise ImportError("geopandas and shapely are required for ice_mask_to_polygons()")

    # Ensure uint8 binary
    binary = (mask > 0).astype(np.uint8) * 255

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Use actual mask dimensions for pixel→geo transform so downscaled masks work correctly
    mask_wh = (binary.shape[1], binary.shape[0])  # (width, height)

    # Effective GSD at mask scale = gsd_full * (W_full / W_mask)
    W_full = georect.camera.width or georect.metadata.image_width
    scale = binary.shape[1] / W_full if W_full else 1.0
    gsd_eff = georect.gsd_m / scale  # GSD at the mask's resolution

    rows: list[dict] = []
    blob_id = 0

    for contour in contours:
        area_px = cv2.contourArea(contour)
        area_m2 = area_px * gsd_eff * gsd_eff

        if area_m2 < min_area_m2:
            continue

        poly = _contour_to_polygon(contour, georect, _wh=mask_wh)
        if poly is None:
            continue

        rows.append({
            "image_idx": image_idx,
            "blob_id": blob_id,
            "area_m2": round(area_m2, 2),
            "geometry": poly,
        })
        blob_id += 1

    if rows:
        gdf = gpd.GeoDataFrame(rows, crs="EPSG:4326")
    else:
        gdf = gpd.GeoDataFrame(
            columns=["image_idx", "blob_id", "area_m2", "geometry"],
            crs="EPSG:4326",
        )

    return gdf


def masks_to_polygons(
    masks: list[np.ndarray],
    georects: list[GeoRect],
    min_area_m2: float = 0.0,
) -> "gpd.GeoDataFrame":
    """Process a sequence of masks and GeoRects into a single GeoDataFrame.

    Parameters
    ----------
    masks : list of (H, W) uint8 binary ice masks.
    georects : list of GeoRect objects, same length as masks.
    min_area_m2 : Per-polygon area threshold.

    Returns
    -------
    Merged GeoDataFrame with image_idx indicating the source image.
    """
    if len(masks) != len(georects):
        raise ValueError("masks and georects must have the same length")

    frames: list[gpd.GeoDataFrame] = []
    for idx, (m, gr) in enumerate(zip(masks, georects)):
        gdf = ice_mask_to_polygons(m, gr, min_area_m2=min_area_m2, image_idx=idx)
        frames.append(gdf)

    if not frames:
        return gpd.GeoDataFrame(
            columns=["image_idx", "blob_id", "area_m2", "geometry"],
            crs="EPSG:4326",
        )

    return gpd.GeoDataFrame(
        pd_concat(frames, ignore_index=True),
        crs="EPSG:4326",
    )


def to_geopackage(
    gdf: "gpd.GeoDataFrame",
    path: str | Path,
    layer: str = "ice_polygons",
) -> None:
    """Write a GeoDataFrame to a GeoPackage file.

    Parameters
    ----------
    gdf : GeoDataFrame to write.
    path : Output file path (.gpkg extension recommended).
    layer : Layer name within the GeoPackage.
    """
    if not _GPD_AVAILABLE:
        raise ImportError("geopandas is required for to_geopackage()")
    gdf.to_file(str(path), layer=layer, driver="GPKG")


# ---------------------------------------------------------------------------
# pandas concat import (deferred to avoid top-level pandas import if unused)
# ---------------------------------------------------------------------------

def pd_concat(frames, **kwargs):
    import pandas as pd  # type: ignore
    return pd.concat(frames, **kwargs)
