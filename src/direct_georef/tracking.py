"""Ice blob detection and multi-frame trajectory tracking.

Pipeline
--------
1. IceDetector  : segment each frame into ice blobs using Otsu threshold
                  + morphological cleanup + connected components.
2. TrajectoryLinker : match blobs across frames using a greedy nearest-
                      centroid assignment with optional gap-bridging.
3. Trajectory   : named sequence of per-frame blob observations.

The large iceberg is identified automatically as the largest blob (by area)
and is tracked separately from the smaller ice pieces.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

try:
    import cv2  # type: ignore
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class IceBlob:
    """A single detected ice region in one frame."""
    frame_idx: int
    centroid_rc: tuple[float, float]   # (row, col) at processing scale
    area_px: int                        # area at processing scale
    bbox_rc: tuple[int, int, int, int]  # (r_min, c_min, r_max, c_max)

    @property
    def centroid_xy(self) -> tuple[float, float]:
        """(col, row) ordering for plotting."""
        return (self.centroid_rc[1], self.centroid_rc[0])


@dataclass
class Trajectory:
    """A sequence of IceBlob observations linked across frames."""
    track_id: int
    is_iceberg: bool = False
    blobs: list[IceBlob] = field(default_factory=list)

    # ------------------------------------------------------------------
    def centroid_array(self) -> np.ndarray:
        """(N, 2) array of (row, col) centroids, one per observation."""
        return np.array([b.centroid_rc for b in self.blobs])

    def frame_array(self) -> np.ndarray:
        """(N,) array of frame indices."""
        return np.array([b.frame_idx for b in self.blobs])

    def area_array(self) -> np.ndarray:
        """(N,) array of blob areas in pixels."""
        return np.array([b.area_px for b in self.blobs])

    def centroid_at(self, frame_idx: int) -> Optional[tuple[float, float]]:
        """Return (row, col) centroid for the given frame, or None."""
        for b in self.blobs:
            if b.frame_idx == frame_idx:
                return b.centroid_rc
        return None

    def __len__(self) -> int:
        return len(self.blobs)


# ---------------------------------------------------------------------------
# Ice detector
# ---------------------------------------------------------------------------

class IceDetector:
    """Detect ice blobs in a grayscale image using Otsu thresholding.

    Parameters
    ----------
    min_area_px : Minimum blob area in pixels (at the processing scale).
    morph_close : Morphological closing iterations (fills small holes).
    morph_open  : Morphological opening iterations (removes small noise).
    kernel_size : Structuring element diameter for morphological ops.
    """

    def __init__(
        self,
        min_area_px: int = 50,
        morph_close: int = 2,
        morph_open: int = 1,
        kernel_size: int = 5,
    ) -> None:
        if not _CV2_AVAILABLE:
            raise ImportError("opencv-python is required for IceDetector")
        self._min_area = min_area_px
        self._morph_close = morph_close
        self._morph_open = morph_open
        self._kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )

    def detect(self, gray: np.ndarray, frame_idx: int) -> tuple[list[IceBlob], np.ndarray]:
        """Detect ice blobs in a uint8 grayscale image.

        Parameters
        ----------
        gray : (H, W) uint8 grayscale.
        frame_idx : frame index for labelling blobs.

        Returns
        -------
        blobs : list of IceBlob (sorted largest-first by area).
        mask  : (H, W) uint8 binary mask (255 = ice).
        """
        # Otsu threshold
        _, mask = cv2.threshold(gray, 0, 255,
                                cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Morphological cleanup
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                self._kernel, iterations=self._morph_close)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                self._kernel, iterations=self._morph_open)

        # Connected components
        n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask, connectivity=8
        )

        blobs: list[IceBlob] = []
        for lbl in range(1, n_labels):  # skip background (0)
            area = int(stats[lbl, cv2.CC_STAT_AREA])
            if area < self._min_area:
                continue
            cx, cy = centroids[lbl]          # (col, row) from OpenCV
            x0 = int(stats[lbl, cv2.CC_STAT_LEFT])
            y0 = int(stats[lbl, cv2.CC_STAT_TOP])
            w  = int(stats[lbl, cv2.CC_STAT_WIDTH])
            h  = int(stats[lbl, cv2.CC_STAT_HEIGHT])
            blobs.append(IceBlob(
                frame_idx=frame_idx,
                centroid_rc=(float(cy), float(cx)),
                area_px=area,
                bbox_rc=(y0, x0, y0 + h, x0 + w),
            ))

        blobs.sort(key=lambda b: b.area_px, reverse=True)
        return blobs, mask


# ---------------------------------------------------------------------------
# Trajectory linker
# ---------------------------------------------------------------------------

class TrajectoryLinker:
    """Link IceBlob detections across frames into Trajectory objects.

    Uses greedy nearest-centroid assignment with a configurable search radius.
    Trajectories can bridge short gaps (frames where a blob was not detected).

    Parameters
    ----------
    max_dist_px : Maximum centroid-to-centroid distance (pixels) to allow a
                  match.  Blobs farther apart start a new trajectory.
    max_gap     : Maximum number of consecutive frames a trajectory may be
                  unobserved before it is terminated.
    min_length  : Minimum number of observations to keep a trajectory.
    iceberg_area_ratio : Blobs with area > this fraction of the largest blob
                  in the first frame are considered part of the iceberg set.
                  The single largest blob is always labelled iceberg.
    """

    def __init__(
        self,
        max_dist_px: float = 30.0,
        max_gap: int = 3,
        min_length: int = 5,
        iceberg_area_ratio: float = 0.5,
    ) -> None:
        self._max_dist = max_dist_px
        self._max_gap = max_gap
        self._min_length = min_length
        self._iceberg_ratio = iceberg_area_ratio

    # ------------------------------------------------------------------
    def link(
        self,
        all_blobs: list[list[IceBlob]],
    ) -> tuple[list[Trajectory], Trajectory]:
        """Link blobs into trajectories.

        Parameters
        ----------
        all_blobs : list of length T, each element a list of IceBlob for
                    that frame (sorted largest-first).

        Returns
        -------
        trajectories : all trajectories with length >= min_length
                       (iceberg excluded).
        iceberg      : single Trajectory for the large iceberg.
        """
        from scipy.optimize import linear_sum_assignment  # type: ignore

        # Identify iceberg area threshold from first non-empty frame
        iceberg_area_thresh = 0
        for blobs in all_blobs:
            if blobs:
                iceberg_area_thresh = blobs[0].area_px * self._iceberg_ratio
                break

        # Active tracks: list of (Trajectory, last_seen_frame, gap_count)
        active: list[tuple[Trajectory, int, int]] = []
        finished: list[Trajectory] = []
        next_id = 0

        for frame_idx, blobs in enumerate(all_blobs):
            if not blobs:
                # Age all active tracks
                active = [(t, ls, g + 1) for t, ls, g in active]
                active = [(t, ls, g) for t, ls, g in active
                          if g <= self._max_gap]
                continue

            # Centroids for Hungarian assignment
            if active:
                pred_cents = np.array([
                    t.blobs[-1].centroid_rc for t, _, _ in active
                ], dtype=np.float64)
                new_cents = np.array(
                    [b.centroid_rc for b in blobs], dtype=np.float64
                )

                # Cost matrix: Euclidean distance
                diff = pred_cents[:, np.newaxis, :] - new_cents[np.newaxis, :, :]
                cost = np.linalg.norm(diff, axis=2)  # (n_active, n_new)

                row_ind, col_ind = linear_sum_assignment(cost)
                matched_new = set()
                new_active = []

                for r, c in zip(row_ind, col_ind):
                    if cost[r, c] <= self._max_dist:
                        traj, _, _ = active[r]
                        traj.blobs.append(blobs[c])
                        new_active.append((traj, frame_idx, 0))
                        matched_new.add(c)

                # Unmatched active → age gap
                matched_active = {r for r, c in zip(row_ind, col_ind)
                                  if cost[r, c] <= self._max_dist}
                for i, (t, ls, g) in enumerate(active):
                    if i not in matched_active:
                        new_g = g + 1
                        if new_g <= self._max_gap:
                            new_active.append((t, ls, new_g))
                        else:
                            finished.append(t)

                # Unmatched new → start new trajectory
                for c, blob in enumerate(blobs):
                    if c not in matched_new:
                        traj = Trajectory(track_id=next_id)
                        traj.blobs.append(blob)
                        new_active.append((traj, frame_idx, 0))
                        next_id += 1

                active = new_active

            else:
                # No active tracks — start one per blob
                for blob in blobs:
                    traj = Trajectory(track_id=next_id)
                    traj.blobs.append(blob)
                    active.append((traj, frame_idx, 0))
                    next_id += 1

        # Collect remaining active
        for t, _, _ in active:
            finished.append(t)

        # Label iceberg: longest trajectory whose median area exceeds threshold
        # (the iceberg is present in nearly all frames and is always largest)
        finished.sort(key=lambda t: np.median(t.area_array()), reverse=True)
        iceberg_traj = finished[0]
        iceberg_traj.is_iceberg = True

        # Filter small trajectories
        small_ice = [t for t in finished[1:]
                     if len(t) >= self._min_length]

        return small_ice, iceberg_traj


# ---------------------------------------------------------------------------
# Centroid velocity from trajectory
# ---------------------------------------------------------------------------

def trajectory_velocity(
    traj: Trajectory,
    M_px2ned: np.ndarray,
    dt: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute frame-by-frame NED velocity from centroid displacements.

    Parameters
    ----------
    traj : Trajectory with at least 2 observations.
    M_px2ned : (2, 2) pixel → NED transform.
    dt : inter-frame interval (seconds).

    Returns
    -------
    mid_frames : (N-1,) frame indices at the midpoint of each step.
    vel_ned    : (N-1, 2) velocity [vN, vE] in m/s.
    """
    cents = traj.centroid_array()  # (N, 2) in (row, col)
    frames = traj.frame_array()    # (N,)

    # Only use consecutive frames to avoid gap artefacts
    mask = np.diff(frames) == 1
    if not mask.any():
        return np.array([]), np.array([]).reshape(0, 2)

    displacements = np.diff(cents, axis=0)[mask]   # (M, 2) in [Δrow, Δcol]
    # Reorder to [Δcol, Δrow] = [Δu, Δv] for M_px2ned
    duv = displacements[:, ::-1]                    # (M, 2) in [Δu, Δv]
    ned = duv @ M_px2ned.T                          # (M, 2) in [ΔN, ΔE] per frame
    vel_ned = ned / dt

    mid_frames = ((frames[:-1] + frames[1:]) / 2)[mask]
    return mid_frames, vel_ned
