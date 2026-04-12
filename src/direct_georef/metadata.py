"""Extract GPS position and camera orientation from drone image metadata.

Reads standard EXIF GPS tags plus vendor XMP fields.  Supports:

  DJI (Phantom 3/4, Mavic, Inspire, Mini, …)
      XMP namespace: ``drone-dji:``
      Fields: GimbalYawDegree, GimbalPitchDegree, GimbalRollDegree,
              FlightYawDegree, FlightPitchDegree, FlightRollDegree,
              RelativeAltitude

  Phase One / Orthodrone (iXM series, processed via Capture One)
      XMP namespaces: ``aerialgps:`` (primary) and ``Camera:`` (fallback)
      aerialgps fields: GPSIMUYaw, GPSIMUPitch, GPSIMURoll (rational strings,
                        e.g. "3015708/10000"); GPSIMUYawRef
      Camera fields: Yaw, Pitch, Roll (floating-point degrees)
      Note: aerialgps:GPSIMUPitch uses the same convention as DJI
            (0° = horizontal, −90° = nadir).  Camera:Pitch is nadir-relative
            (0° = nadir); converted by: gimbal_pitch = Camera:Pitch − 90°.

All returned orientation angles follow the NED convention used by georectify:
  Yaw   : degrees from True North, clockwise (0 = N, 90 = E); 0–360 or ±180
  Pitch : degrees; 0 = horizontal, −90 = straight down (nadir)
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

    # Orientation — populated from DJI or Phase One XMP; None if absent
    altitude_rel_m: Optional[float] = None   # relative above takeoff point (DJI) or ATO (Phase One)
    gimbal_yaw: Optional[float] = None       # camera heading, deg from True N, CW
    gimbal_pitch: Optional[float] = None     # camera tilt, deg; 0 = level, -90 = nadir
    gimbal_roll: Optional[float] = None      # camera roll, deg; 0 = level, +ve = right-wing-down
    flight_yaw: Optional[float] = None       # platform body heading
    flight_pitch: Optional[float] = None     # platform body pitch
    flight_roll: Optional[float] = None      # platform body roll

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

    # --- XMP (namespace-dispatched) ---
    _parse_xmp(path, meta)

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


def _eval_rational(s: str) -> Optional[float]:
    """Parse a rational string like '3015708/10000' or a plain float string.

    Returns None if the string cannot be parsed.
    """
    s = s.strip()
    if "/" in s:
        parts = s.split("/", 1)
        try:
            return float(parts[0]) / float(parts[1])
        except (ValueError, ZeroDivisionError):
            return None
    try:
        return float(s)
    except ValueError:
        return None


def _parse_xmp(path: Path, meta: ImageMetadata) -> None:
    """Read XMP block from a JPEG, detect vendor namespace, dispatch to parser."""
    with open(path, "rb") as f:
        raw = f.read()

    xmp_start = raw.find(b"<x:xmpmeta")
    if xmp_start == -1:
        return
    xmp_end = raw.find(b"</x:xmpmeta>", xmp_start) + len(b"</x:xmpmeta>")
    xmp = raw[xmp_start:xmp_end].decode("utf-8", errors="replace")

    if "drone-dji:" in xmp:
        _parse_dji_xmp(xmp, meta)
    elif "aerialgps:" in xmp or 'xmlns:Camera="http://www.phaseone.com' in xmp:
        _parse_phaseone_xmp(xmp, meta)
    # else: no recognised orientation namespace — orientation fields stay None


def _parse_dji_xmp(xmp: str, meta: ImageMetadata) -> None:
    """Populate orientation from DJI ``drone-dji:`` XMP namespace.

    REQ-META-002
    """
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


def _parse_phaseone_xmp(xmp: str, meta: ImageMetadata) -> None:
    """Populate orientation from Phase One / Orthodrone XMP namespaces.

    Primary source: ``aerialgps:`` namespace (GPSIMUYaw/Pitch/Roll).
    Values are rational strings (e.g. "3015708/10000") or plain floats.

    Fallback: ``Camera:`` namespace (Yaw/Pitch/Roll as plain floats).
    Camera:Pitch uses nadir-relative convention (0 = nadir); converted to
    DJI convention by: gimbal_pitch = Camera:Pitch - 90°.

    REQ-META-003, REQ-META-004, REQ-META-006
    """
    def _rational_attr(name: str) -> Optional[float]:
        """Match name="value" where value may be a rational like '3015708/10000'."""
        m = re.search(rf'{re.escape(name)}="([^"]+)"', xmp)
        return _eval_rational(m.group(1)) if m else None

    def _float_attr(name: str) -> Optional[float]:
        m = re.search(rf'{re.escape(name)}="([+-]?\d+(?:\.\d+)?)"', xmp)
        return float(m.group(1)) if m else None

    # --- Primary: aerialgps: namespace ---
    imu_yaw   = _rational_attr("aerialgps:GPSIMUYaw")
    imu_pitch = _rational_attr("aerialgps:GPSIMUPitch")
    imu_roll  = _rational_attr("aerialgps:GPSIMURoll")

    # Warn if yaw reference is magnetic (M); True (T) is assumed
    yaw_ref_m = re.search(r'aerialgps:GPSIMUYawRef="([^"]+)"', xmp)
    if yaw_ref_m and yaw_ref_m.group(1).upper() == "M":
        import warnings
        warnings.warn(
            "aerialgps:GPSIMUYawRef is 'M' (magnetic); georectification "
            "assumes True North. Results will be offset by magnetic declination.",
            UserWarning,
            stacklevel=3,
        )

    if imu_yaw is not None:
        meta.gimbal_yaw   = imu_yaw
        meta.gimbal_pitch = imu_pitch   # same convention as DJI: 0=level, -90=nadir
        meta.gimbal_roll  = imu_roll

    # --- Fallback: Camera: namespace ---
    if meta.gimbal_yaw is None:
        cam_yaw   = _float_attr("Camera:Yaw")
        cam_pitch = _float_attr("Camera:Pitch")   # 0° = nadir (nadir-relative)
        cam_roll  = _float_attr("Camera:Roll")

        if cam_yaw is not None:
            meta.gimbal_yaw  = cam_yaw
            # Convert nadir-relative pitch to DJI convention (0=level, -90=nadir)
            meta.gimbal_pitch = (cam_pitch - 90.0) if cam_pitch is not None else None
            meta.gimbal_roll  = cam_roll

    # --- Flight body angles from aerialgps: ---
    meta.flight_yaw   = _rational_attr("aerialgps:GPSIMUFlightYaw")
    meta.flight_pitch = _rational_attr("aerialgps:GPSIMUFlightPitch")
    meta.flight_roll  = _rational_attr("aerialgps:GPSIMUFlightRoll")

    # NOTE: aerialgps:GPSAltitudeAboveTakeOff is NOT stored as altitude_rel_m.
    # Unlike DJI RelativeAltitude (which is reliable AGL), this value is relative
    # to the aircraft's takeoff point elevation, not to the terrain below the image.
    # Flying height is computed in georectify() as: altitude_abs_m - surface_altitude_m.
    # Callers who know their terrain elevation should pass it as surface_altitude_m.
