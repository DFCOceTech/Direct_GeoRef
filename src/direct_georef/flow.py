"""Dense optical flow computation and velocity field analysis.

Computes per-pixel velocity fields from consecutive image pairs using
Farneback optical flow, converts pixel displacements to georeferenced
velocities in the NED (North–East–Down) frame, and derives vorticity
for wake vortex analysis.

Coordinate conventions
----------------------
  Flow array:  shape (H, W, 2), [..., 0] = column displacement (u),
                                          [..., 1] = row displacement (v)
  NED velocity: shape (H, W, 2), [..., 0] = Northward (m/s),
                                            [..., 1] = Eastward (m/s)
  Vorticity:   ω = ∂v_E/∂N − ∂v_N/∂E  (positive = anticlockwise, rad/s)
"""

from __future__ import annotations

from typing import Optional

import numpy as np

try:
    import cv2  # type: ignore
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False


# ---------------------------------------------------------------------------
# Pixel → NED transformation
# ---------------------------------------------------------------------------

def pixel_to_ned_matrix(
    R_cam2ned: np.ndarray,
    fx: float,
    fy: float,
    flying_height_m: float,
    scale: float = 1.0,
) -> np.ndarray:
    """Build the 2×2 matrix that maps pixel displacements to NED metre offsets.

    For a small displacement (du, dv) in pixel coordinates:
        [ΔN, ΔE] = M @ [du, dv]

    Parameters
    ----------
    R_cam2ned : 3×3 rotation matrix (camera frame → NED).
    fx, fy    : focal lengths in pixels (at full resolution).
    flying_height_m : AGL height in metres.
    scale     : image downscale factor (e.g. 0.25 for quarter resolution).
                Focal lengths are scaled internally.

    Returns
    -------
    M : (2, 2) float64 array.
    """
    fx_s = fx * scale
    fy_s = fy * scale
    M = flying_height_m * np.array([
        [R_cam2ned[0, 0] / fx_s,  R_cam2ned[0, 1] / fy_s],
        [R_cam2ned[1, 0] / fx_s,  R_cam2ned[1, 1] / fy_s],
    ], dtype=np.float64)
    return M


# ---------------------------------------------------------------------------
# Dense optical flow
# ---------------------------------------------------------------------------

def compute_flow(
    gray1: np.ndarray,
    gray2: np.ndarray,
    pyr_scale: float = 0.5,
    levels: int = 4,
    winsize: int = 25,
    iterations: int = 3,
    poly_n: int = 7,
    poly_sigma: float = 1.5,
) -> np.ndarray:
    """Compute Farneback dense optical flow between two grayscale frames.

    Parameters
    ----------
    gray1, gray2 : uint8 grayscale arrays (H, W).

    Returns
    -------
    flow : (H, W, 2) float32; [..., 0] = column (u), [..., 1] = row (v).
    """
    if not _CV2_AVAILABLE:
        raise ImportError("opencv-python is required for optical flow")

    return cv2.calcOpticalFlowFarneback(
        gray1, gray2, None,
        pyr_scale=pyr_scale,
        levels=levels,
        winsize=winsize,
        iterations=iterations,
        poly_n=poly_n,
        poly_sigma=poly_sigma,
        flags=0,
    )


def flow_to_ned_velocity(
    flow: np.ndarray,
    M_px2ned: np.ndarray,
    dt: float,
) -> np.ndarray:
    """Convert pixel flow to NED velocity field.

    Parameters
    ----------
    flow      : (H, W, 2) pixel displacements per frame (u, v).
    M_px2ned  : (2, 2) pixel → NED metre matrix from ``pixel_to_ned_matrix``.
    dt        : inter-frame interval in seconds.

    Returns
    -------
    vel_ned : (H, W, 2) float64; [..., 0] = vN (m/s), [..., 1] = vE (m/s).
    """
    H, W = flow.shape[:2]
    uv = flow.reshape(-1, 2).astype(np.float64)  # (N, 2) [u, v]
    ned = uv @ M_px2ned.T                          # (N, 2) [ΔN, ΔE] per frame
    return (ned / dt).reshape(H, W, 2)


# ---------------------------------------------------------------------------
# Smoothing
# ---------------------------------------------------------------------------

def smooth_velocity(
    vel_ned: np.ndarray,
    sigma_px: float = 5.0,
    mask: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Gaussian-smooth a velocity field, optionally ignoring masked pixels.

    Parameters
    ----------
    vel_ned  : (H, W, 2) NED velocity array.
    sigma_px : Gaussian sigma in pixels.
    mask     : bool (H, W) — True where pixels are valid. NaN is written to
               invalid pixels before smoothing so they don't bleed into valid
               regions (approximate boundary handling).

    Returns
    -------
    Smoothed (H, W, 2) array; invalid pixels set to NaN.
    """
    from scipy.ndimage import gaussian_filter  # type: ignore

    out = np.empty_like(vel_ned, dtype=np.float64)
    for c in range(2):
        ch = vel_ned[..., c].astype(np.float64).copy()
        if mask is not None:
            ch[~mask] = 0.0
        sm = gaussian_filter(ch, sigma=sigma_px)
        if mask is not None:
            # Normalise by smoothed mask weight to avoid edge bias
            wt = gaussian_filter(mask.astype(np.float64), sigma=sigma_px)
            sm = np.where(wt > 0.01, sm / wt, np.nan)
            sm[~mask] = np.nan
        out[..., c] = sm
    return out


# ---------------------------------------------------------------------------
# Vorticity
# ---------------------------------------------------------------------------

def compute_vorticity(
    vel_ned: np.ndarray,
    gsd_n: float,
    gsd_e: float,
) -> np.ndarray:
    """Compute vertical vorticity from a 2D NED velocity field.

    ω_z = ∂v_E/∂N  −  ∂v_N/∂E

    Positive ω means anticlockwise rotation (cyclonic in NH).

    Parameters
    ----------
    vel_ned : (H, W, 2) — [..., 0] = vN, [..., 1] = vE, in m/s.
    gsd_n   : grid spacing in the North direction (m/pixel — row direction).
    gsd_e   : grid spacing in the East direction  (m/pixel — col direction).

    Returns
    -------
    omega : (H, W) float64 in s⁻¹.
    """
    vN = vel_ned[..., 0]  # (H, W)
    vE = vel_ned[..., 1]  # (H, W)

    # ∂vN/∂E: gradient along columns (East direction)
    dvN_dE = np.gradient(vN, gsd_e, axis=1)
    # ∂vE/∂N: gradient along rows (North direction — note: rows increase downward)
    #   row index increases downward = decreasing N, so flip sign
    dvE_dN = -np.gradient(vE, gsd_n, axis=0)

    return dvE_dN - dvN_dE


# ---------------------------------------------------------------------------
# Relative velocity (iceberg reference frame)
# ---------------------------------------------------------------------------

def make_relative(
    vel_ned: np.ndarray,
    ref_vel: np.ndarray,
) -> np.ndarray:
    """Subtract a reference velocity from a field.

    Parameters
    ----------
    vel_ned : (H, W, 2) absolute NED velocity field.
    ref_vel : (2,) reference velocity [vN, vE] in m/s (e.g. iceberg velocity).

    Returns
    -------
    (H, W, 2) relative velocity field.
    """
    return vel_ned - ref_vel[np.newaxis, np.newaxis, :]


# ---------------------------------------------------------------------------
# Temporal aggregation
# ---------------------------------------------------------------------------

def time_mean(
    fields: list[np.ndarray],
    weights: Optional[list[float]] = None,
) -> np.ndarray:
    """Compute (weighted) time mean of a list of (H, W, ...) arrays.

    NaN values are excluded from the mean (nanmean semantics).
    """
    stack = np.stack(fields, axis=0)  # (T, H, W, ...)
    if weights is None:
        return np.nanmean(stack, axis=0)
    w = np.array(weights, dtype=np.float64)
    w = w / w.sum()
    return np.nansum(stack * w[:, np.newaxis, np.newaxis], axis=0)


# ---------------------------------------------------------------------------
# Drone motion stabilization (homography-based)
# ---------------------------------------------------------------------------

def estimate_drone_homography(
    gray1: np.ndarray,
    gray2: np.ndarray,
    water_mask: np.ndarray,
    n_features: int = 2000,
    ransac_thresh: float = 3.0,
    min_matches: int = 10,
) -> Optional[np.ndarray]:
    """Estimate apparent drone motion as a homography from ORB feature matching
    restricted to water (non-ice) pixels.

    Keypoints are detected and matched only within the water mask, so ice motion
    does not bias the estimate.  The homography captures translation, unrecorded
    yaw/pitch drift, and minor scale change — all sources of IMU error.

    Parameters
    ----------
    gray1, gray2  : uint8 grayscale frames (H, W).
    water_mask    : bool (H, W) — True where pixels are open water (not ice).
    n_features    : max ORB features per frame.
    ransac_thresh : RANSAC reprojection threshold in pixels.
    min_matches   : minimum good matches required; returns None if not reached.

    Returns
    -------
    H : (3, 3) float64 homography (frame1 → frame2 due to drone motion),
        or None if estimation failed.
    """
    if not _CV2_AVAILABLE:
        raise ImportError("opencv-python is required")

    orb = cv2.ORB_create(nfeatures=n_features)
    mask_u8 = water_mask.astype(np.uint8) * 255

    kp1, des1 = orb.detectAndCompute(gray1, mask_u8)
    kp2, des2 = orb.detectAndCompute(gray2, mask_u8)

    if des1 is None or des2 is None:
        return None
    if len(kp1) < min_matches or len(kp2) < min_matches:
        return None

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    raw = bf.knnMatch(des1, des2, k=2)
    good = [m for m, n in raw if m.distance < 0.75 * n.distance]

    if len(good) < min_matches:
        return None

    src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

    H, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, ransac_thresh)
    return H  # None if RANSAC failed


def homography_flow(
    H: np.ndarray,
    height: int,
    width: int,
) -> np.ndarray:
    """Convert a homography to a per-pixel displacement field.

    At each pixel (x, y) the drone-induced displacement is H·[x,y,1]ᵀ − [x,y].

    Parameters
    ----------
    H      : (3, 3) homography.
    height, width : frame dimensions.

    Returns
    -------
    flow : (height, width, 2) float32;
           [..., 0] = column displacement (u), [..., 1] = row displacement (v).
    """
    xx, yy = np.meshgrid(np.arange(width, dtype=np.float32),
                          np.arange(height, dtype=np.float32))
    pts = np.stack([xx.ravel(), yy.ravel()], axis=1).reshape(-1, 1, 2)
    warped = cv2.perspectiveTransform(pts, H.astype(np.float32)).reshape(height, width, 2)
    return warped - np.stack([xx, yy], axis=-1)


def stabilize_flow(
    flow: np.ndarray,
    H: Optional[np.ndarray],
) -> np.ndarray:
    """Remove estimated drone motion from an optical flow field.

    Parameters
    ----------
    flow : (H, W, 2) Farneback optical flow (u, v) in pixels.
    H    : (3, 3) homography from ``estimate_drone_homography``, or None.
           If None, flow is returned unchanged (no stabilization applied).

    Returns
    -------
    Stabilized (H, W, 2) flow array.
    """
    if H is None:
        return flow
    drone = homography_flow(H, flow.shape[0], flow.shape[1])
    return flow - drone
