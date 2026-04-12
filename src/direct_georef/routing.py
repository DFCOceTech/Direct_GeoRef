"""Ship route planning through sea ice.

Uses A* pathfinding on a downscaled occupancy grid to find the optimal
north-to-south route through ice, accounting for ship beam clearance and
ice proximity penalties.

REQ coverage: REQ-ROUTE-001 through REQ-ROUTE-009
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

try:
    import cv2  # type: ignore
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False

try:
    from scipy.ndimage import distance_transform_edt  # type: ignore
    _SCIPY_AVAILABLE = True
except ImportError:
    _SCIPY_AVAILABLE = False

try:
    import geopandas as gpd  # type: ignore
    from shapely.geometry import LineString, MultiPoint, Point  # type: ignore
    _GPD_AVAILABLE = True
except ImportError:
    _GPD_AVAILABLE = False

from .georectify import GeoRect
from .vector import _pixel_to_latlon

# 1 knot in m/s
_KN_TO_MS = 0.5144


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------

@dataclass
class RouteResult:
    """Result from find_route().

    Attributes
    ----------
    waypoints_latlon : Simplified waypoints as (lat, lon) tuples.
        Empty list if no navigable path was found.
    total_distance_m : Total route length in metres.
    eta_s : Estimated travel time in seconds at ship_speed_kn.
    navigable_fraction : Fraction of the routing grid that is navigable.
    """
    waypoints_latlon: list[tuple[float, float]]
    total_distance_m: float
    eta_s: float
    navigable_fraction: float


# ---------------------------------------------------------------------------
# A* implementation
# ---------------------------------------------------------------------------

def _astar(
    navigable: np.ndarray,
    cost_map: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[tuple[int, int]]:
    """A* shortest path on a 2D 8-connected grid.

    Parameters
    ----------
    navigable : (H, W) bool — True where the ship may travel.
    cost_map  : (H, W) float — additional cost per pixel (proximity penalty).
    start, goal : (row, col) grid coordinates.

    Returns
    -------
    List of (row, col) from start to goal (inclusive), or [] if no path.
    """
    H, W = navigable.shape

    # Octile distance heuristic (admissible for 8-connected grid)
    def heuristic(r: int, c: int) -> float:
        dr = abs(r - goal[0])
        dc = abs(c - goal[1])
        return (dr + dc) + (math.sqrt(2) - 2) * min(dr, dc)

    # 8-connected offsets and their base step costs
    _DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1),
             (-1, -1), (-1, 1), (1, -1), (1, 1)]
    _STEP = [1.0, 1.0, 1.0, 1.0,
             math.sqrt(2), math.sqrt(2), math.sqrt(2), math.sqrt(2)]

    g_score: dict[tuple[int, int], float] = {start: 0.0}
    parent: dict[tuple[int, int], Optional[tuple[int, int]]] = {start: None}
    open_heap: list[tuple[float, float, int, int]] = [
        (heuristic(*start), 0.0, start[0], start[1])
    ]
    closed: set[tuple[int, int]] = set()

    while open_heap:
        _f, g, r, c = heapq.heappop(open_heap)
        node = (r, c)

        if node in closed:
            continue
        closed.add(node)

        if node == goal:
            # Reconstruct path
            path: list[tuple[int, int]] = []
            cur: Optional[tuple[int, int]] = goal
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            return path[::-1]

        for (dr, dc), base_step in zip(_DIRS, _STEP):
            nr, nc = r + dr, c + dc
            if not (0 <= nr < H and 0 <= nc < W):
                continue
            nb = (nr, nc)
            if not navigable[nr, nc] or nb in closed:
                continue

            new_g = g + base_step + float(cost_map[nr, nc])
            if nb not in g_score or new_g < g_score[nb]:
                g_score[nb] = new_g
                parent[nb] = node
                heapq.heappush(open_heap, (new_g + heuristic(nr, nc), new_g, nr, nc))

    return []  # No path found


# ---------------------------------------------------------------------------
# Helper utilities
# ---------------------------------------------------------------------------

def _remove_small_blobs(
    mask_u8: np.ndarray,
    min_area_px: float,
) -> np.ndarray:
    """Return a copy of mask_u8 with blobs smaller than min_area_px removed."""
    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask_u8, connectivity=8
    )
    result = mask_u8.copy()
    for lbl in range(1, n_labels):
        if stats[lbl, cv2.CC_STAT_AREA] < min_area_px:
            result[labels == lbl] = 0
    return result


def _find_auto_endpoint(
    navigable: np.ndarray,
    northmost: bool,
) -> Optional[tuple[int, int]]:
    """Find the northernmost (or southernmost) ice-free midpoint column.

    Parameters
    ----------
    navigable : (H, W) bool — True where navigable.
    northmost : If True, search from top (row 0); else from bottom.

    Returns
    -------
    (row, col) or None if no navigable pixel found.
    """
    rows = range(navigable.shape[0]) if northmost else range(navigable.shape[0] - 1, -1, -1)
    for r in rows:
        cols = np.where(navigable[r])[0]
        if len(cols) > 0:
            mid_col = int((int(cols[0]) + int(cols[-1])) // 2)
            return (r, mid_col)
    return None


# ---------------------------------------------------------------------------
# Main public API
# ---------------------------------------------------------------------------

def find_route(
    mask: np.ndarray,
    georect: GeoRect,
    ship_beam_m: float = 15.0,
    ship_length_m: float = 90.0,
    ship_speed_kn: float = 3.0,
    min_floe_area_m2: float = 0.0,
    w_dist: float = 1.0,
    w_ice: float = 2.0,
    routing_scale: float = 0.1,
    start_latlon: Optional[tuple[float, float]] = None,
    end_latlon: Optional[tuple[float, float]] = None,
    simplify_m: Optional[float] = None,
) -> RouteResult:
    """Find the optimal north-to-south ship route through sea ice.

    Parameters
    ----------
    mask : (H, W) uint8 binary ice mask; 255 = ice, 0 = water.
    georect : GeoRect from ``georectify()``.
    ship_beam_m : Ship beam in metres; clearance = beam / 2. Default 15.0.
    ship_length_m : Ship length in metres (reserved). Default 90.0.
    ship_speed_kn : Speed in knots for ETA calculation. Default 3.0.
    min_floe_area_m2 : Ice blobs smaller than this are treated as open water.
    w_dist : Weight for step distance in the cost function. Default 1.0.
    w_ice : Weight for ice proximity penalty. Default 2.0.
    routing_scale : Downscale factor for the routing grid. Default 0.1.
    start_latlon : Optional (lat, lon) for start; auto-detected if None.
    end_latlon : Optional (lat, lon) for end; auto-detected if None.
    simplify_m : RDP simplification tolerance in metres (default = beam / 2).

    Returns
    -------
    RouteResult with waypoints_latlon, total_distance_m, eta_s, navigable_fraction.
    """
    if not _CV2_AVAILABLE:
        raise ImportError("opencv-python is required for find_route()")
    if not _SCIPY_AVAILABLE:
        raise ImportError("scipy is required for find_route()")

    if simplify_m is None:
        simplify_m = ship_beam_m / 2.0

    ship_speed_ms = ship_speed_kn * _KN_TO_MS

    # ---- GSD at routing scale ----
    gsd_full = georect.gsd_m
    gsd_proc = gsd_full / routing_scale  # GSD in metres at routing scale

    # ---- Downscale mask ----
    H_orig, W_orig = mask.shape[:2]
    H_proc = max(1, int(round(H_orig * routing_scale)))
    W_proc = max(1, int(round(W_orig * routing_scale)))

    binary_full = (mask > 0).astype(np.uint8) * 255

    # ---- Remove small floes from full-res mask ----
    if min_floe_area_m2 > 0.0:
        min_area_px = min_floe_area_m2 / (gsd_full * gsd_full)
        binary_full = _remove_small_blobs(binary_full, min_area_px)

    # Downscale (nearest neighbour to preserve binary values)
    ice_proc = cv2.resize(
        binary_full, (W_proc, H_proc), interpolation=cv2.INTER_NEAREST
    )
    ice_bool = ice_proc > 0  # True = ice

    # ---- Dilate for ship clearance ----
    clearance_px = max(1, int(math.ceil((ship_beam_m / 2.0) / gsd_proc)))
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (clearance_px * 2 + 1, clearance_px * 2 + 1)
    )
    dilated = cv2.dilate(ice_proc, kernel, iterations=1)
    dilated_bool = dilated > 0  # True = ice + clearance zone

    navigable = ~dilated_bool  # True = safe to travel

    nav_fraction = float(navigable.sum()) / navigable.size

    # ---- Ice proximity distance transform ----
    # distance from each water pixel to nearest ice pixel (in pixels)
    dist_to_ice_px = distance_transform_edt(~ice_bool)

    # Cost map: w_ice / (dist_m + 1) where dist_m = dist_px * gsd_proc
    dist_to_ice_m = dist_to_ice_px * gsd_proc
    proximity_cost = w_ice / (dist_to_ice_m + 1.0)
    # Zero out cost inside ice (should not be visited anyway)
    proximity_cost[dilated_bool] = 0.0

    # ---- Start / end points ----
    if start_latlon is None:
        start_rc = _find_auto_endpoint(navigable, northmost=True)
    else:
        start_rc = _latlon_to_pixel(start_latlon[0], start_latlon[1],
                                    georect, H_proc, W_proc)

    if end_latlon is None:
        end_rc = _find_auto_endpoint(navigable, northmost=False)
    else:
        end_rc = _latlon_to_pixel(end_latlon[0], end_latlon[1],
                                  georect, H_proc, W_proc)

    if start_rc is None or end_rc is None:
        return RouteResult(
            waypoints_latlon=[],
            total_distance_m=0.0,
            eta_s=0.0,
            navigable_fraction=nav_fraction,
        )

    # Clamp to navigable grid
    if not navigable[start_rc]:
        start_rc = _nearest_navigable(navigable, start_rc)
    if not navigable[end_rc]:
        end_rc = _nearest_navigable(navigable, end_rc)
    if start_rc is None or end_rc is None:
        return RouteResult(
            waypoints_latlon=[],
            total_distance_m=0.0,
            eta_s=0.0,
            navigable_fraction=nav_fraction,
        )

    # ---- A* pathfinding ----
    path_rc = _astar(navigable, proximity_cost, start_rc, end_rc)

    if not path_rc:
        return RouteResult(
            waypoints_latlon=[],
            total_distance_m=0.0,
            eta_s=0.0,
            navigable_fraction=nav_fraction,
        )

    # ---- Path length calculation (in metres) ----
    path_arr = np.array(path_rc, dtype=np.float64)  # (N, 2) [row, col]
    diffs = np.diff(path_arr, axis=0)
    step_dists = np.linalg.norm(diffs, axis=1) * gsd_proc
    total_dist_m = float(step_dists.sum())

    # ---- Simplify path ----
    # Work in pixel space; convert epsilon from metres to pixels
    epsilon_px = simplify_m / gsd_proc

    if _GPD_AVAILABLE and len(path_rc) >= 2:
        # Build pixel-space LineString (col, row) and simplify
        path_colrow = [(c, r) for r, c in path_rc]
        line_px = LineString(path_colrow)
        simplified = line_px.simplify(epsilon_px, preserve_topology=False)
        wp_colrow = list(simplified.coords)
    else:
        # Fallback: keep every point
        wp_colrow = [(c, r) for r, c in path_rc]

    # ---- Convert simplified waypoints to lat/lon ----
    wp_u = np.array([p[0] for p in wp_colrow], dtype=np.float64)  # col
    wp_v = np.array([p[1] for p in wp_colrow], dtype=np.float64)  # row

    # Scale pixel coordinates back to full-res before lat/lon conversion
    wp_u_full = wp_u / routing_scale
    wp_v_full = wp_v / routing_scale

    lat_wp, lon_wp = _pixel_to_latlon(wp_u_full, wp_v_full, georect)
    waypoints = [(float(lat_wp[i]), float(lon_wp[i])) for i in range(len(wp_u))]

    eta_s = total_dist_m / ship_speed_ms if ship_speed_ms > 0 else 0.0

    return RouteResult(
        waypoints_latlon=waypoints,
        total_distance_m=round(total_dist_m, 1),
        eta_s=round(eta_s, 1),
        navigable_fraction=round(nav_fraction, 4),
    )


# ---------------------------------------------------------------------------
# GeoPackage output
# ---------------------------------------------------------------------------

def route_to_geopackage(
    result: RouteResult,
    path: str | Path,
) -> None:
    """Write a RouteResult to a GeoPackage with 'route' and 'waypoints' layers.

    Parameters
    ----------
    result : RouteResult from find_route().
    path : Output .gpkg file path.
    """
    if not _GPD_AVAILABLE:
        raise ImportError("geopandas and shapely are required for route_to_geopackage()")

    if not result.waypoints_latlon:
        raise ValueError("RouteResult has no waypoints — no route found")

    # Convert (lat, lon) → (lon, lat) for GeoJSON/shapely
    coords_lonlat = [(lon, lat) for lat, lon in result.waypoints_latlon]

    # Route layer: single LINESTRING
    line = LineString(coords_lonlat)
    route_gdf = gpd.GeoDataFrame(
        [{
            "total_distance_m": result.total_distance_m,
            "eta_minutes": round(result.eta_s / 60.0, 1),
            "geometry": line,
        }],
        crs="EPSG:4326",
    )

    # Waypoints layer: POINT per waypoint
    wp_rows = [
        {"seq": i, "geometry": Point(lon, lat)}
        for i, (lat, lon) in enumerate(result.waypoints_latlon)
    ]
    wp_gdf = gpd.GeoDataFrame(wp_rows, crs="EPSG:4326")

    path = str(path)
    route_gdf.to_file(path, layer="route", driver="GPKG")
    wp_gdf.to_file(path, layer="waypoints", driver="GPKG")


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _latlon_to_pixel(
    lat: float,
    lon: float,
    georect: GeoRect,
    H_proc: int,
    W_proc: int,
) -> tuple[int, int]:
    """Inverse-bilinear approximate: lat/lon → (row, col) in the routing grid.

    Uses a simple affine approximation from the corner GCPs.
    """
    W_full = georect.camera.width or georect.metadata.image_width
    H_full = georect.camera.height or georect.metadata.image_height

    lat_TL, lon_TL = georect.corners_latlon["TL"]
    lat_TR, lon_TR = georect.corners_latlon["TR"]
    lat_BL, lon_BL = georect.corners_latlon["BL"]
    lat_BR, lon_BR = georect.corners_latlon["BR"]

    lat_top = (lat_TL + lat_TR) / 2
    lat_bot = (lat_BL + lat_BR) / 2
    lon_lft = (lon_TL + lon_BL) / 2
    lon_rgt = (lon_TR + lon_BR) / 2

    if abs(lat_bot - lat_top) < 1e-12 or abs(lon_rgt - lon_lft) < 1e-12:
        return (H_proc // 2, W_proc // 2)

    s = (lat - lat_top) / (lat_bot - lat_top)
    t = (lon - lon_lft) / (lon_rgt - lon_lft)

    row = int(round(s * (H_proc - 1)))
    col = int(round(t * (W_proc - 1)))
    row = max(0, min(H_proc - 1, row))
    col = max(0, min(W_proc - 1, col))
    return (row, col)


def _nearest_navigable(
    navigable: np.ndarray,
    rc: tuple[int, int],
) -> Optional[tuple[int, int]]:
    """Return the nearest navigable pixel to rc using distance transform."""
    if not _SCIPY_AVAILABLE:
        return None
    dist = distance_transform_edt(~navigable)
    # Find pixel with minimum distance to rc that is navigable
    nav_rows, nav_cols = np.where(navigable)
    if len(nav_rows) == 0:
        return None
    dists = np.hypot(nav_rows - rc[0], nav_cols - rc[1])
    idx = int(np.argmin(dists))
    return (int(nav_rows[idx]), int(nav_cols[idx]))
