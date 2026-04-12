"""Direct georectification of nadir drone imagery.

Theory
------
Direct georectification projects every image pixel onto a flat reference
plane (the ground/water surface) using known camera position (GPS) and
orientation (gimbal angles), without requiring ground control points.

Coordinate frames
-----------------
  Camera frame (C):  x→right (cols), y→down (rows), z→scene (optical axis)
  Body frame (B):    NED aligned when gimbal is level and pointing North
  Navigation frame:  NED (North–East–Down), locally flat-Earth approximation

Rotation convention (DJI gimbal angles)
----------------------------------------
  Yaw   (ψ): rotation about Down axis;    0 = North, +90 = East (clockwise)
  Pitch (θ): rotation about East axis;    0 = level, −90 = nadir
  Roll  (φ): rotation about North axis;   0 = level, +ve = right-wing-down

  Applied order: R = Rz(ψ) · Ry(θ) · Rx(φ)   (extrinsic rotations, ZYX)

  R rotates vectors from Camera frame to NED frame:  v_NED = R · v_C

Ground-plane intersection
--------------------------
For a pixel (u,v), the ray direction in NED is:

    d_NED = R · K⁻¹ · [u, v, 1]ᵀ   (then normalised)

Scale factor to the horizontal ground plane at depth h (flying height):

    t = h / d_NED[2]      (Down component; positive = towards ground)

Ground point relative to camera (in NED metres):

    ΔP = t · d_NED

Geographic coordinates:

    Δlat = ΔP[0] / R_earth
    Δlon = ΔP[1] / (R_earth · cos(lat₀))

Reference: Linder (2016) "Digital Photogrammetry", Springer, ch. 7.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .camera import CameraModel
from .metadata import ImageMetadata

# WGS-84 mean Earth radius (metres)
_R_EARTH = 6_371_000.0


# ---------------------------------------------------------------------------
# Result
# ---------------------------------------------------------------------------

@dataclass
class GeoRect:
    """Georectification result for one image.

    Pixel grid
    ----------
    ``lat_grid`` and ``lon_grid`` are (height × width) arrays giving the
    geographic coordinate of every pixel centre.

    Corner coordinates
    ------------------
    ``corners_latlon`` is a dict with keys 'TL', 'TR', 'BL', 'BR' and
    values (lat, lon) in decimal degrees.

    Footprint polygon
    -----------------
    ``footprint_latlon`` is a closed ring [(lat,lon), ...] suitable for
    GeoJSON / folium, ordered TL → TR → BR → BL → TL.
    """

    lat_grid: np.ndarray      # (H, W)
    lon_grid: np.ndarray      # (H, W)
    corners_latlon: dict[str, tuple[float, float]]
    footprint_latlon: list[tuple[float, float]]
    gsd_m: float              # approximate ground sampling distance (m/px)
    flying_height_m: float
    metadata: ImageMetadata
    camera: CameraModel


# ---------------------------------------------------------------------------
# Core function
# ---------------------------------------------------------------------------

def georectify(
    meta: ImageMetadata,
    camera: CameraModel,
    surface_altitude_m: float = 0.0,
    full_grid: bool = True,
) -> GeoRect:
    """Compute geographic coordinates for every pixel in a nadir image.

    Parameters
    ----------
    meta:
        Position and orientation from ``metadata.read_metadata()``.
    camera:
        Intrinsic model from any ``camera.from_*`` loader.
    surface_altitude_m:
        Elevation of the ground/water surface (metres AMSL).
        Flying height = meta.altitude_abs_m − surface_altitude_m.
        For ocean surveys this is typically 0.  When using DJI relative
        altitude (recommended), pass ``surface_altitude_m=0`` and ensure
        ``meta.altitude_rel_m`` is populated — ``flying_height_m`` is used
        automatically.
    full_grid:
        If True, compute lat/lon for every pixel (can be large for 4K images).
        If False, compute only the four corners; ``lat_grid`` / ``lon_grid``
        will be None.

    Returns
    -------
    GeoRect with pixel-level coordinate arrays and corner/footprint info.
    """
    # Flying height above the reference plane
    h = meta.flying_height_m  # uses relative altitude when available

    if h <= 0.0:
        raise ValueError(f"Flying height must be positive, got {h} m")

    W = camera.width  or meta.image_width
    H = camera.height or meta.image_height
    if W is None or H is None:
        raise ValueError("Image dimensions unknown — set camera.width/height or pass via metadata")

    # --- Rotation matrix: Camera → NED ---
    yaw   = meta.gimbal_yaw   or 0.0
    pitch = meta.gimbal_pitch or -90.0   # assume nadir if missing
    roll  = meta.gimbal_roll  or 0.0
    R = _rotation_camera_to_ned(yaw, pitch, roll)

    # --- Camera-origin geographic position ---
    lat0 = np.radians(meta.latitude)
    lon0 = np.radians(meta.longitude)

    # --- Inverse intrinsic matrix ---
    K_inv = np.linalg.inv(camera.K)

    if full_grid:
        # Build pixel coordinate meshgrid
        us = np.arange(W, dtype=np.float64)   # columns
        vs = np.arange(H, dtype=np.float64)   # rows
        uu, vv = np.meshgrid(us, vs)           # (H, W) each

        lat_grid, lon_grid = _project_pixels(uu, vv, K_inv, R, h, lat0, lon0)
    else:
        lat_grid = lon_grid = None  # type: ignore[assignment]

    # --- Corner coordinates (always computed) ---
    corners_px = {
        "TL": (0.0,     0.0),
        "TR": (W - 1.0, 0.0),
        "BR": (W - 1.0, H - 1.0),
        "BL": (0.0,     H - 1.0),
    }
    corners_latlon: dict[str, tuple[float, float]] = {}
    for name, (u, v) in corners_px.items():
        lat_c, lon_c = _project_pixels(
            np.array([[u]]), np.array([[v]]), K_inv, R, h, lat0, lon0
        )
        corners_latlon[name] = (float(lat_c[0, 0]), float(lon_c[0, 0]))

    footprint = [
        corners_latlon["TL"],
        corners_latlon["TR"],
        corners_latlon["BR"],
        corners_latlon["BL"],
        corners_latlon["TL"],  # close ring
    ]

    # GSD: at nadir, approximately pixel_pitch * h / f
    # Use (fx + fy) / 2 as effective focal length in pixels
    f_px = (camera.fx + camera.fy) / 2.0
    gsd = h / f_px  # metres per pixel (exact at image centre for nadir)

    return GeoRect(
        lat_grid=lat_grid,
        lon_grid=lon_grid,
        corners_latlon=corners_latlon,
        footprint_latlon=footprint,
        gsd_m=round(gsd, 4),
        flying_height_m=h,
        metadata=meta,
        camera=camera,
    )


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _rotation_camera_to_ned(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    """Build 3×3 rotation matrix that maps camera-frame vectors to NED.

    DJI angle convention:
      Yaw   (ψ): about Down (Z_NED); 0 = North, +90 = East
      Pitch (θ): about East (Y_NED); 0 = horizontal, −90 = nadir
      Roll  (φ): about North (X_NED); 0 = level, +ve = right-wing-down

    The camera body frame has:
      x_C → right (pixel columns)
      y_C → down  (pixel rows)
      z_C → forward / into scene

    When level (pitch=0, roll=0) and heading North (yaw=0):
      z_C (optical axis) points North → X_NED
      x_C (right) points East       → Y_NED
      y_C (down)  points Down       → Z_NED

    For nadir (pitch=−90°), the optical axis points Down → Z_NED.

    The combined rotation: R_C→NED = Rz(ψ) · Ry(θ) · Rx(φ) · R_body0
    where R_body0 maps the initial camera alignment to the NED frame.
    """
    ψ = np.radians(yaw_deg)
    θ = np.radians(pitch_deg)
    φ = np.radians(roll_deg)

    # Elementary rotations (active, column vectors)
    Rz = np.array([[ np.cos(ψ), -np.sin(ψ), 0],
                   [ np.sin(ψ),  np.cos(ψ), 0],
                   [ 0,          0,          1]], dtype=np.float64)

    Ry = np.array([[ np.cos(θ),  0, np.sin(θ)],
                   [ 0,          1, 0         ],
                   [-np.sin(θ),  0, np.cos(θ)]], dtype=np.float64)

    Rx = np.array([[1, 0,          0         ],
                   [0, np.cos(φ), -np.sin(φ) ],
                   [0, np.sin(φ),  np.cos(φ) ]], dtype=np.float64)

    # Rotation from body frame (level, heading North) to NED:
    # x_C→E, y_C→Down, z_C→North  at zero angles
    # i.e.   [N, E, D] = R_body0 · [x_C, y_C, z_C]
    #        N = z_C   → col [0,0,1]
    #        E = x_C   → col [1,0,0]
    #        D = y_C   → col [0,1,0]
    R_body0 = np.array([[0, 0, 1],
                        [1, 0, 0],
                        [0, 1, 0]], dtype=np.float64)

    return Rz @ Ry @ Rx @ R_body0


def _project_pixels(
    uu: np.ndarray,    # (H, W) column indices
    vv: np.ndarray,    # (H, W) row indices
    K_inv: np.ndarray, # 3×3
    R: np.ndarray,     # 3×3  Camera→NED
    h: float,          # flying height (m)
    lat0: float,       # camera latitude (radians)
    lon0: float,       # camera longitude (radians)
) -> tuple[np.ndarray, np.ndarray]:
    """Project pixel coordinates to geographic lat/lon (decimal degrees).

    Returns (lat_deg, lon_deg) arrays of same shape as uu/vv.
    """
    orig_shape = uu.shape

    # Flatten to (3, N) homogeneous pixel vectors
    ones = np.ones_like(uu.ravel())
    px_h = np.stack([uu.ravel(), vv.ravel(), ones], axis=0)  # (3, N)

    # Ray directions in camera frame
    rays_C = K_inv @ px_h  # (3, N)

    # Ray directions in NED frame
    rays_NED = R @ rays_C  # (3, N)

    # Scale to ground plane: t = h / D_component (index 2 = Down in NED)
    D_comp = rays_NED[2, :]  # Down component
    # Avoid division by zero (shouldn't happen for nadir imagery)
    D_comp = np.where(np.abs(D_comp) < 1e-9, 1e-9, D_comp)
    t = h / D_comp  # (N,)

    # NED offset from camera to ground point (metres)
    delta_N = rays_NED[0, :] * t
    delta_E = rays_NED[1, :] * t

    # Convert metre offsets to geographic coordinates
    delta_lat = delta_N / _R_EARTH
    delta_lon = delta_E / (_R_EARTH * np.cos(lat0))

    lat_deg = np.degrees(lat0 + delta_lat).reshape(orig_shape)
    lon_deg = np.degrees(lon0 + delta_lon).reshape(orig_shape)

    return lat_deg, lon_deg
