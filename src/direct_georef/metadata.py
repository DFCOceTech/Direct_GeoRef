"""Extract GPS position and camera orientation from drone image metadata.

Reads standard EXIF GPS tags plus DJI-specific XMP fields.
Supports any drone that follows the drone-dji XMP namespace convention
(Phantom 3/4, Mavic series, Inspire, Mini, etc.).

Returned orientation angles follow the DJI convention:
  Yaw   : degrees from North, clockwise (0 = North, 90 = East, -90/270 = West)
  Pitch : degrees; 0 = horizontal, -90 = straight down (nadir)
  Roll  : degrees; 0 = level, positive = right-wing-down
"""

from __future__ import annotations

import datetime
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from PIL import Image
from PIL.ExifTags import GPSTAGS, TAGS


# ---------------------------------------------------------------------------
# Timelapse sort helper
# ---------------------------------------------------------------------------

def sort_by_timestamp(paths: list[Path]) -> list[Path]:
    """Return image paths sorted by EXIF DateTimeOriginal.

    DJI timelapse sequences use a rolling file counter that wraps at 9999,
    so filename order is not reliable.  This function reads the EXIF
    timestamp from every file and returns them in chronological order.

    Parameters
    ----------
    paths : list of Path objects pointing to JPEG images.

    Returns
    -------
    Sorted list of Path objects.
    """
    def _exif_datetime(p: Path) -> datetime.datetime:
        img = Image.open(p)
        exif_raw = img._getexif() or {}
        img.close()
        exif = {TAGS.get(k, k): v for k, v in exif_raw.items()}
        dt_str = exif.get("DateTimeOriginal") or exif.get("DateTime") or ""
        try:
            return datetime.datetime.strptime(dt_str, "%Y:%m:%d %H:%M:%S")
        except ValueError:
            return datetime.datetime.min   # put unreadable files first

    return sorted(paths, key=_exif_datetime)


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------

@dataclass
class ImageMetadata:
    """Position and orientation extracted from one image file."""

    # File
    path: str

    # GPS (always present if extraction succeeded)
    latitude: float        # decimal degrees, positive = North
    longitude: float       # decimal degrees, positive = East
    altitude_abs_m: float  # EXIF GPSAltitude (absolute, AMSL)

    # DJI XMP — may be absent on non-DJI cameras
    altitude_rel_m: Optional[float] = None   # relative above takeoff point
    gimbal_yaw: Optional[float] = None       # camera heading (deg from N, CW)
    gimbal_pitch: Optional[float] = None     # camera tilt (deg; -90 = nadir)
    gimbal_roll: Optional[float] = None      # camera roll (deg)
    flight_yaw: Optional[float] = None       # drone body heading
    flight_pitch: Optional[float] = None     # drone body pitch
    flight_roll: Optional[float] = None      # drone body roll

    # Camera info
    make: str = ""
    model: str = ""
    focal_length_mm: Optional[float] = None
    focal_length_35mm: Optional[float] = None
    image_width: Optional[int] = None
    image_height: Optional[int] = None

    # Derived convenience property
    @property
    def flying_height_m(self) -> float:
        """Best estimate of AGL height: relative altitude when available,
        otherwise absolute altitude (proxy; less accurate for sea-level terrain).
        """
        return self.altitude_rel_m if self.altitude_rel_m is not None else self.altitude_abs_m

    @property
    def has_orientation(self) -> bool:
        """True if gimbal yaw/pitch/roll are all available."""
        return all(v is not None for v in (self.gimbal_yaw, self.gimbal_pitch, self.gimbal_roll))


# ---------------------------------------------------------------------------
# Public function
# ---------------------------------------------------------------------------

def read_metadata(image_path: str | Path) -> ImageMetadata:
    """Read GPS position and camera orientation from a JPEG image.

    Parameters
    ----------
    image_path:
        Path to the JPEG file.

    Returns
    -------
    ImageMetadata with all available fields populated.

    Raises
    ------
    ValueError
        If GPS coordinates cannot be read from the file.
    """
    path = Path(image_path)
    img = Image.open(path)

    exif_raw = img._getexif() or {}
    exif = {TAGS.get(k, k): v for k, v in exif_raw.items()}

    # --- GPS ---
    gps_raw = exif.get("GPSInfo") or {}
    gps = {GPSTAGS.get(k, k): v for k, v in gps_raw.items()}

    if not gps:
        raise ValueError(f"No GPS data found in {path.name}")

    lat = _parse_dms(gps["GPSLatitude"], gps.get("GPSLatitudeRef", "N"))
    lon = _parse_dms(gps["GPSLongitude"], gps.get("GPSLongitudeRef", "E"))
    alt_abs = float(gps.get("GPSAltitude", 0.0))

    # --- Camera info from EXIF ---
    make  = str(exif.get("Make",  "")).replace("\x00", "").strip()
    model = str(exif.get("Model", "")).replace("\x00", "").strip()
    focal_mm = exif.get("FocalLength")
    focal_35 = exif.get("FocalLengthIn35mmFilm")
    w = exif.get("ExifImageWidth") or exif.get("ImageWidth")
    h = exif.get("ExifImageHeight") or exif.get("ImageHeight")

    meta = ImageMetadata(
        path=str(path),
        latitude=lat,
        longitude=lon,
        altitude_abs_m=alt_abs,
        make=make,
        model=model,
        focal_length_mm=float(focal_mm) if focal_mm is not None else None,
        focal_length_35mm=float(focal_35) if focal_35 is not None else None,
        image_width=int(w) if w is not None else None,
        image_height=int(h) if h is not None else None,
    )

    # --- DJI XMP ---
    _parse_dji_xmp(path, meta)

    return meta


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _parse_dms(dms_tuple, ref: str) -> float:
    """Convert EXIF (degrees, minutes, seconds) rational to decimal degrees."""
    d, m, s = float(dms_tuple[0]), float(dms_tuple[1]), float(dms_tuple[2])
    dd = d + m / 60.0 + s / 3600.0
    if ref in ("S", "W"):
        dd = -dd
    return dd


def _parse_dji_xmp(path: Path, meta: ImageMetadata) -> None:
    """Parse DJI XMP block and populate orientation fields in-place."""
    with open(path, "rb") as f:
        raw = f.read()

    xmp_start = raw.find(b"<x:xmpmeta")
    if xmp_start == -1:
        return
    xmp_end = raw.find(b"</x:xmpmeta>", xmp_start) + len(b"</x:xmpmeta>")
    xmp = raw[xmp_start:xmp_end].decode("utf-8", errors="replace")

    def _attr(name: str) -> Optional[float]:
        m = re.search(rf'{name}="([+-]?\d+(?:\.\d+)?)"', xmp)
        return float(m.group(1)) if m else None

    meta.altitude_rel_m = _attr("drone-dji:RelativeAltitude")
    meta.gimbal_yaw   = _attr("drone-dji:GimbalYawDegree")
    meta.gimbal_pitch = _attr("drone-dji:GimbalPitchDegree")
    meta.gimbal_roll  = _attr("drone-dji:GimbalRollDegree")
    meta.flight_yaw   = _attr("drone-dji:FlightYawDegree")
    meta.flight_pitch = _attr("drone-dji:FlightPitchDegree")
    meta.flight_roll  = _attr("drone-dji:FlightRollDegree")
