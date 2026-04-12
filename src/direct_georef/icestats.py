"""Ice blob statistics.

Computes per-blob shape statistics (area, major/minor axes, orientation,
centroid) and scene-level aggregates from a binary ice detection mask.

REQ coverage: REQ-ICE-001 through REQ-ICE-007
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

try:
    import cv2  # type: ignore
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False

from .georectify import GeoRect
from .vector import _pixel_to_latlon


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class BlobStats:
    """Shape statistics for one ice blob.

    Fields
    ------
    blob_id : per-image identifier; 0 = largest blob
    area_m2 : georeferenced area in m² (area_px × gsd_m²)
    major_axis_m : length of the major ellipse axis (metres)
    minor_axis_m : length of the minor ellipse axis (metres)
    orientation_deg : angle of major axis from positive image x-axis (columns),
        counterclockwise, in [0, 180°); as returned by cv2.fitEllipse.
        To convert to bearing from North: subtract from (90° + gimbal_yaw).
    centroid_lat : geographic latitude of centroid (decimal degrees)
    centroid_lon : geographic longitude of centroid (decimal degrees)
    """
    blob_id: int
    area_m2: float
    major_axis_m: float
    minor_axis_m: float
    orientation_deg: float
    centroid_lat: float
    centroid_lon: float


@dataclass
class SceneStats:
    """Scene-level ice statistics aggregated across all blobs above threshold.

    Fields
    ------
    total_ice_area_m2 : sum of all blob areas
    ice_fraction      : total_ice_area_m2 / scene_area_m2
    blob_count        : number of blobs above min_area_m2
    mean_blob_area_m2 : arithmetic mean of individual blob areas
    max_blob_area_m2  : area of the largest blob
    min_blob_area_m2  : area of the smallest blob above threshold
    """
    total_ice_area_m2: float
    ice_fraction: float
    blob_count: int
    mean_blob_area_m2: float
    max_blob_area_m2: float
    min_blob_area_m2: float


# ---------------------------------------------------------------------------
# Main function
# ---------------------------------------------------------------------------

def compute_ice_stats(
    mask: np.ndarray,
    georect: GeoRect,
    min_area_m2: float = 0.0,
) -> tuple[list[BlobStats], SceneStats]:
    """Compute per-blob and scene-level ice statistics.

    Parameters
    ----------
    mask : (H, W) uint8 array; 255 = ice, 0 = water.
    georect : GeoRect from ``georectify()``.
    min_area_m2 : Blobs with georeferenced area below this threshold are excluded
        from both the blob list and the scene aggregates.

    Returns
    -------
    blobs : list of BlobStats, sorted largest-first by area.
    scene : SceneStats aggregated over all blobs above threshold.
    """
    if not _CV2_AVAILABLE:
        raise ImportError("opencv-python is required for compute_ice_stats()")

    binary = (mask > 0).astype(np.uint8) * 255
    mask_wh = (binary.shape[1], binary.shape[0])  # (width, height)

    # Effective GSD at mask scale
    W_full = georect.camera.width or georect.metadata.image_width
    scale = binary.shape[1] / W_full if W_full else 1.0
    gsd = georect.gsd_m / scale  # GSD at mask resolution

    # Connected components
    n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        binary, connectivity=8
    )

    H, W = binary.shape
    scene_area_m2 = H * W * gsd * gsd

    blobs: list[BlobStats] = []

    for lbl in range(1, n_labels):  # skip background (0)
        area_px = int(stats[lbl, cv2.CC_STAT_AREA])
        area_m2 = area_px * gsd * gsd

        if area_m2 < min_area_m2:
            continue

        cx, cy = centroids[lbl]  # (col, row) from OpenCV

        # Geographic centroid (pass mask dims so downscaled masks map correctly)
        lat_arr, lon_arr = _pixel_to_latlon(
            np.array([cx]), np.array([cy]), georect, _wh=mask_wh
        )
        cent_lat = float(lat_arr[0])
        cent_lon = float(lon_arr[0])

        # Ellipse fitting for axes and orientation
        blob_mask = (labels == lbl).astype(np.uint8) * 255
        contours, _ = cv2.findContours(blob_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        major_axis_m = float("nan")
        minor_axis_m = float("nan")
        orientation_deg = float("nan")

        if contours:
            contour = max(contours, key=cv2.contourArea)
            if len(contour) >= 5:
                try:
                    (_cx, _cy), (axis_a, axis_b), angle = cv2.fitEllipse(contour)
                    major_axis_m = max(axis_a, axis_b) * gsd
                    minor_axis_m = min(axis_a, axis_b) * gsd
                    orientation_deg = float(angle)
                except cv2.error:
                    pass

        blobs.append(BlobStats(
            blob_id=len(blobs),
            area_m2=round(area_m2, 2),
            major_axis_m=round(major_axis_m, 2) if not math.isnan(major_axis_m) else float("nan"),
            minor_axis_m=round(minor_axis_m, 2) if not math.isnan(minor_axis_m) else float("nan"),
            orientation_deg=round(orientation_deg, 2) if not math.isnan(orientation_deg) else float("nan"),
            centroid_lat=round(cent_lat, 7),
            centroid_lon=round(cent_lon, 7),
        ))

    # Sort largest-first; re-assign blob_id
    blobs.sort(key=lambda b: b.area_m2, reverse=True)
    for i, b in enumerate(blobs):
        b.blob_id = i

    # Scene aggregates
    if blobs:
        areas = [b.area_m2 for b in blobs]
        total = sum(areas)
        scene = SceneStats(
            total_ice_area_m2=round(total, 2),
            ice_fraction=round(total / scene_area_m2, 6) if scene_area_m2 > 0 else 0.0,
            blob_count=len(blobs),
            mean_blob_area_m2=round(total / len(blobs), 2),
            max_blob_area_m2=round(max(areas), 2),
            min_blob_area_m2=round(min(areas), 2),
        )
    else:
        scene = SceneStats(
            total_ice_area_m2=0.0,
            ice_fraction=0.0,
            blob_count=0,
            mean_blob_area_m2=0.0,
            max_blob_area_m2=0.0,
            min_blob_area_m2=0.0,
        )

    return blobs, scene
