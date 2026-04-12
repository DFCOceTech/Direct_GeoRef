# Metadata Extraction — Specification

> Version: 1.1 | Status: Implemented | Last updated: 2026-04-12

## Purpose

Extract GPS position (lat/lon/altitude) and camera orientation (yaw/pitch/roll) from drone
imagery EXIF and XMP metadata. Support both DJI (`drone-dji:` namespace) and Phase One /
Orthodrone (`aerialgps:` / `Camera:` namespaces) systems, with graceful degradation when
optional fields are absent.

## Functional Requirements

### REQ-META-001: EXIF GPS Position
The system SHALL parse GPS latitude, longitude, and absolute altitude from standard EXIF
GPSInfo tags (GPSLatitude, GPSLongitude, GPSAltitude) and raise `ValueError` if GPS data
are absent.

### REQ-META-002: DJI XMP Orientation
The system SHALL parse gimbal yaw, pitch, and roll from `drone-dji:GimbalYawDegree`,
`drone-dji:GimbalPitchDegree`, and `drone-dji:GimbalRollDegree` XMP fields when present.

### REQ-META-003: Phase One XMP Orientation
The system SHALL parse camera yaw, pitch, and roll from Phase One / Orthodrone XMP
namespaces as follows:
- Primary: `aerialgps:GPSIMUYaw`, `aerialgps:GPSIMUPitch`, `aerialgps:GPSIMURoll`
- Fallback: `Camera:Yaw`, `Camera:Pitch`, `Camera:Roll`
Values encoded as rational strings (e.g. `"3015708/10000"`) SHALL be evaluated to float.

### REQ-META-004: Phase One Angle Convention
`aerialgps:GPSIMUPitch` SHALL be interpreted using the same convention as DJI gimbal pitch:
0° = horizontal, −90° = nadir. `Camera:Pitch` uses a nadir-relative convention
(0° = nadir); the system SHALL convert by subtracting 90° when using `Camera:Pitch` as
fallback: `gimbal_pitch = Camera:Pitch − 90`.

### REQ-META-005: Nadir Defaults
When gimbal pitch and roll are not available in any XMP namespace, the system SHALL default
to `gimbal_pitch = −90.0` (nadir) and `gimbal_roll = 0.0`.

### REQ-META-006: Yaw Reference
Yaw values SHALL be interpreted as True North bearing (clockwise). Magnetic yaw is not
supported; images with `YawRef="M"` SHALL log a warning.

### REQ-META-007: Camera Dimensions
The system SHALL read `ExifImageWidth` / `ExifImageHeight` (or `PixelXDimension` /
`PixelYDimension`) from EXIF and store them in `ImageMetadata.image_width/height`.

### REQ-META-008: Make / Model
The system SHALL read `Make` and `Model` from EXIF IFD0 when present. Absence is not an
error; empty strings are stored.

### REQ-META-009: Focal Length
The system SHALL read `FocalLength` and `FocalLengthIn35mmFilm` from EXIF ExifIFD when
present. Absence is not an error; `None` is stored.

## Acceptance Scenarios

### SCENARIO-META-001: DJI image with full XMP
**GIVEN** a DJI JPEG with `drone-dji:GimbalYawDegree`, `GimbalPitchDegree`, `GimbalRollDegree`
**WHEN** `read_metadata()` is called
**THEN** `ImageMetadata.gimbal_yaw/pitch/roll` are populated from DJI XMP

### SCENARIO-META-002: Phase One image — aerialgps namespace
**GIVEN** a Phase One JPEG with `aerialgps:GPSIMUYaw=301.57`, `GPSIMUPitch=-90.807`, `GPSIMURoll=0.238`
**WHEN** `read_metadata()` is called
**THEN** `gimbal_yaw=301.57`, `gimbal_pitch=-90.807`, `gimbal_roll=0.238`

### SCENARIO-META-003: Phase One image — Camera namespace fallback
**GIVEN** a Phase One JPEG with `Camera:Yaw=301.57`, `Camera:Pitch=0.807` (nadir-relative), `Camera:Roll=0.238`
**WHEN** `read_metadata()` is called and `aerialgps:` fields are absent
**THEN** `gimbal_yaw=301.57`, `gimbal_pitch=-89.193` (= 0.807 − 90), `gimbal_roll=0.238`

### SCENARIO-META-004: No orientation XMP → nadir defaults
**GIVEN** a JPEG with GPS but no gimbal XMP
**WHEN** `read_metadata()` is called
**THEN** `gimbal_pitch=None`, `gimbal_roll=None` (caller applies defaults in `georectify()`)

### SCENARIO-META-005: Missing GPS
**GIVEN** a JPEG with no GPSInfo EXIF block
**WHEN** `read_metadata()` is called
**THEN** `ValueError` is raised

## Implementation Status (2026-04-12)

**Status**: Implemented

### What's Built
- `src/direct_georef/metadata.py:read_metadata()` — covers REQ-META-001, 007, 008, 009
- `src/direct_georef/metadata.py:_parse_dji_xmp()` — covers REQ-META-002
- `src/direct_georef/metadata.py:_parse_phaseone_xmp()` — covers REQ-META-003, 004, 006
- Nadir defaults applied in `georectify()` — covers REQ-META-005

### Deviations from Spec
- REQ-META-006 warning for magnetic yaw: not yet implemented (all known images use True)

### Deferred
- None
