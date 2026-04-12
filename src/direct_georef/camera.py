"""Camera intrinsic model and calibration file loaders.

Represents a pinhole camera with optional radial/tangential distortion.
Supports loading calibration parameters from:
  - Sensor spec : focal length (mm) + physical sensor dimensions (mm) — REQ-CAM-005
  - OpenCV      : YAML or JSON produced by cv2.FileStorage
  - Agisoft     : XML exported from Metashape camera calibration
  - Pix4D       : _pix4d_camera_internals.csv or camera.xml
  - EXIF        : Focal length + sensor DB lookup (DJI and Phase One models)

All parameters are stored in pixel units referenced to the image coordinate
system where (0,0) is the top-left corner of the top-left pixel.

Coordinate conventions
----------------------
  u : column index (→ right)
  v : row index    (↓ down)
  K : 3×3 matrix [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np


# ---------------------------------------------------------------------------
# Dataclass
# ---------------------------------------------------------------------------

@dataclass
class CameraModel:
    """Pinhole camera with optional Brown-Conrady distortion.

    Parameters
    ----------
    fx, fy : focal lengths in pixels
    cx, cy : principal point in pixels
    k1, k2, k3 : radial distortion coefficients
    p1, p2     : tangential distortion coefficients
    width, height : sensor dimensions in pixels
    label : human-readable description of the calibration source
    """

    fx: float
    fy: float
    cx: float
    cy: float
    k1: float = 0.0
    k2: float = 0.0
    k3: float = 0.0
    p1: float = 0.0
    p2: float = 0.0
    width: Optional[int] = None
    height: Optional[int] = None
    label: str = "unknown"

    # ------------------------------------------------------------------
    # Convenience accessors
    # ------------------------------------------------------------------

    @property
    def K(self) -> np.ndarray:
        """3×3 intrinsic matrix."""
        return np.array([
            [self.fx,  0.0,    self.cx],
            [0.0,      self.fy, self.cy],
            [0.0,      0.0,    1.0],
        ], dtype=np.float64)

    @property
    def dist_coeffs(self) -> np.ndarray:
        """OpenCV-style distortion vector [k1, k2, p1, p2, k3]."""
        return np.array([self.k1, self.k2, self.p1, self.p2, self.k3], dtype=np.float64)

    @property
    def has_distortion(self) -> bool:
        return any(v != 0.0 for v in (self.k1, self.k2, self.k3, self.p1, self.p2))

    def pixel_to_ray(self, u: float, v: float) -> np.ndarray:
        """Return unit direction vector in camera frame for pixel (u, v).

        Camera frame: x→right, y→down, z→into scene (optical axis).
        No distortion correction is applied here; use undistort_image()
        before calling this on distorted pixels.
        """
        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy
        d = np.array([x, y, 1.0], dtype=np.float64)
        return d / np.linalg.norm(d)

    def __repr__(self) -> str:
        return (
            f"CameraModel(fx={self.fx:.2f}, fy={self.fy:.2f}, "
            f"cx={self.cx:.2f}, cy={self.cy:.2f}, "
            f"distortion={'yes' if self.has_distortion else 'no'}, "
            f"label='{self.label}')"
        )


# ---------------------------------------------------------------------------
# Known sensor database (physical width × height in mm)
# ---------------------------------------------------------------------------

# Sensor sizes for common DJI cameras (mm)
_DJI_SENSORS: dict[str, tuple[float, float]] = {
    "FC300C":  (6.17,  4.55),   # Phantom 3 Adv/Pro (1/2.3")
    "FC300X":  (6.17,  4.55),   # Phantom 3 (1/2.3")
    "FC330":   (6.17,  4.55),   # Phantom 4 (1/2.3")
    "FC6310":  (13.2,  8.8),    # Phantom 4 Pro (1")
    "FC220":   (6.17,  4.55),   # Mavic Pro (1/2.3")
    "FC350":   (17.3,  13.0),   # Inspire 1 X5
    "FC550":   (17.3,  13.0),   # Inspire 1 X5R
    "L1D-20c": (13.2,  8.8),    # Mavic 2 Pro (1")
    "FC7303":  (6.3,   4.7),    # Mini 2
}

# Sensor sizes for Phase One aerial cameras (mm) — REQ-CAM-006
_AERIAL_SENSORS: dict[str, tuple[float, float]] = {
    "IXM-100":    (43.9, 32.9),   # 100 MP BSI medium-format (11664 × 8750 px)
    "IXM-50":     (43.9, 32.9),   # 50 MP variant, same sensor die
    "IXM-RS150F": (53.4, 40.0),   # 150 MP (16352 × 12288 px)
    "iXM-100":    (43.9, 32.9),   # alternate capitalisation
    "iXM-50":     (43.9, 32.9),
}


# ---------------------------------------------------------------------------
# Loaders
# ---------------------------------------------------------------------------

def from_sensor_spec(
    focal_length_mm: float,
    sensor_width_mm: float,
    sensor_height_mm: float,
    image_width: int,
    image_height: int,
) -> CameraModel:
    """Build CameraModel from physical sensor specification.

    Use this when the focal length is known from operator records or a
    separate calibration report, and the sensor dimensions are available
    from the manufacturer's datasheet.  No EXIF or database lookup required.

    This is the preferred loader for non-DJI systems (e.g. Phase One iXM)
    where focal length is not embedded in EXIF.  (REQ-CAM-005)

    Parameters
    ----------
    focal_length_mm : nominal or calibrated focal length in millimetres
    sensor_width_mm : physical sensor width in millimetres
    sensor_height_mm : physical sensor height in millimetres
    image_width, image_height : image dimensions in pixels

    Returns
    -------
    CameraModel with principal point at image centre (no distortion assumed).
    """
    px = sensor_width_mm  / image_width   # mm per pixel, x
    py = sensor_height_mm / image_height  # mm per pixel, y
    fx = focal_length_mm / px
    fy = focal_length_mm / py
    cx = image_width  / 2.0
    cy = image_height / 2.0

    return CameraModel(
        fx=fx, fy=fy, cx=cx, cy=cy,
        width=image_width, height=image_height,
        label=f"sensor_spec ({focal_length_mm:.1f}mm / {sensor_width_mm}×{sensor_height_mm}mm)",
    )


def from_exif(
    focal_length_mm: float,
    image_width: int,
    image_height: int,
    sensor_width_mm: Optional[float] = None,
    sensor_height_mm: Optional[float] = None,
    model: str = "",
) -> CameraModel:
    """Build CameraModel from EXIF focal length and sensor dimensions.

    Sensor size is resolved in priority order:
      1. Explicit ``sensor_width_mm`` / ``sensor_height_mm`` arguments
      2. Known sensor lookup by ``model`` (DJI database, then Phase One database)
      3. Raises ValueError if no sensor size can be determined

    Parameters
    ----------
    focal_length_mm : focal length in millimetres (from EXIF FocalLength tag)
    image_width, image_height : sensor dimensions in pixels
    sensor_width_mm, sensor_height_mm : physical sensor size (optional)
    model : camera model string for database lookup (e.g. "FC300C" or "IXM-100")
    """
    # Resolve sensor size
    sw, sh = sensor_width_mm, sensor_height_mm
    if sw is None or sh is None:
        clean = model.replace("\x00", "").strip()
        key = clean.split()[0] if clean else ""
        if key in _DJI_SENSORS:
            sw, sh = _DJI_SENSORS[key]
        elif key in _AERIAL_SENSORS:
            sw, sh = _AERIAL_SENSORS[key]

    if sw is None or sh is None:
        raise ValueError(
            f"Cannot determine sensor size for model '{model}'. "
            "Pass sensor_width_mm and sensor_height_mm explicitly, or use from_sensor_spec()."
        )

    # Pixel pitch (mm/px)
    px = sw / image_width
    py = sh / image_height

    fx = focal_length_mm / px
    fy = focal_length_mm / py
    cx = image_width  / 2.0
    cy = image_height / 2.0

    return CameraModel(
        fx=fx, fy=fy, cx=cx, cy=cy,
        width=image_width, height=image_height,
        label=f"EXIF+sensor ({model or 'unknown'})",
    )


def from_opencv(cal_path: str | Path) -> CameraModel:
    """Load calibration from an OpenCV YAML or JSON FileStorage file.

    Expected keys: ``camera_matrix`` (3×3) and ``dist_coeffs`` (1×5 or 5×1).
    Optionally: ``image_width``, ``image_height``.
    """
    import cv2  # type: ignore

    cal_path = Path(cal_path)
    fs = cv2.FileStorage(str(cal_path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(f"Cannot open calibration file: {cal_path}")

    K_node = fs.getNode("camera_matrix")
    D_node = fs.getNode("dist_coeffs")
    if K_node.empty() or D_node.empty():
        raise ValueError("calibration file missing 'camera_matrix' or 'dist_coeffs'")

    K = K_node.mat()
    D = D_node.mat().ravel()

    w_node = fs.getNode("image_width")
    h_node = fs.getNode("image_height")
    w = int(w_node.real()) if not w_node.empty() else None
    h = int(h_node.real()) if not h_node.empty() else None
    fs.release()

    return CameraModel(
        fx=float(K[0, 0]), fy=float(K[1, 1]),
        cx=float(K[0, 2]), cy=float(K[1, 2]),
        k1=float(D[0]) if len(D) > 0 else 0.0,
        k2=float(D[1]) if len(D) > 1 else 0.0,
        p1=float(D[2]) if len(D) > 2 else 0.0,
        p2=float(D[3]) if len(D) > 3 else 0.0,
        k3=float(D[4]) if len(D) > 4 else 0.0,
        width=w, height=h,
        label=f"OpenCV ({cal_path.name})",
    )


def from_agisoft_xml(cal_path: str | Path) -> CameraModel:
    """Load calibration from an Agisoft Metashape camera calibration XML.

    Supports both legacy (Photoscan) and current Metashape export formats.
    Handles ``f``, ``cx``, ``cy``, ``k1``–``k3``, ``p1``–``p2`` elements.
    In Agisoft, ``cx``/``cy`` are offsets from the principal point
    (image centre), so we convert: cx_pixel = width/2 + cx, cy_pixel = height/2 + cy.
    """
    cal_path = Path(cal_path)
    tree = ET.parse(cal_path)
    root = tree.getroot()

    def _float(tag: str, default: float = 0.0) -> float:
        el = root.find(f".//{tag}")
        return float(el.text) if el is not None and el.text else default

    f  = _float("f")
    cx = _float("cx")
    cy = _float("cy")
    k1 = _float("k1")
    k2 = _float("k2")
    k3 = _float("k3")
    p1 = _float("p1")
    p2 = _float("p2")

    w_el = root.find(".//width")
    h_el = root.find(".//height")
    w = int(w_el.text) if w_el is not None and w_el.text else None
    h = int(h_el.text) if h_el is not None and h_el.text else None

    # Agisoft cx/cy are offsets from image centre
    cx_px = (w / 2.0 + cx) if w else cx
    cy_px = (h / 2.0 + cy) if h else cy

    return CameraModel(
        fx=f, fy=f,           # Agisoft uses single f (square pixels assumed)
        cx=cx_px, cy=cy_px,
        k1=k1, k2=k2, k3=k3, p1=p1, p2=p2,
        width=w, height=h,
        label=f"Agisoft ({cal_path.name})",
    )


def from_pix4d_csv(cal_path: str | Path) -> CameraModel:
    """Load from a Pix4D *_pix4d_camera_internals.csv file.

    Column format (row 2 onward):
      F, Px, Py, K1, K2, K3, T1, T2, imageWidth, imageHeight
    where F is focal length in pixels, Px/Py are principal point offsets
    from image centre in pixels.
    """
    import csv

    cal_path = Path(cal_path)
    with open(cal_path) as fh:
        rows = list(csv.reader(fh))

    # Skip header rows until we find numeric data
    data_row = None
    for row in rows:
        try:
            float(row[0])
            data_row = row
            break
        except (ValueError, IndexError):
            continue

    if data_row is None:
        raise ValueError(f"No numeric data found in {cal_path}")

    vals = [float(v) for v in data_row]
    f, px, py = vals[0], vals[1], vals[2]
    k1, k2, k3 = vals[3], vals[4], vals[5]
    p1, p2 = vals[6], vals[7]
    w = int(vals[8]) if len(vals) > 8 else None
    h = int(vals[9]) if len(vals) > 9 else None

    cx_px = (w / 2.0 + px) if w else px
    cy_px = (h / 2.0 + py) if h else py

    return CameraModel(
        fx=f, fy=f,
        cx=cx_px, cy=cy_px,
        k1=k1, k2=k2, k3=k3, p1=p1, p2=p2,
        width=w, height=h,
        label=f"Pix4D ({cal_path.name})",
    )
