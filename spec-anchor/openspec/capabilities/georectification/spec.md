# Georectification — Specification

> Version: 1.1 | Status: Implemented | Last updated: 2026-04-12

## Purpose

Project every pixel of a nadir or near-nadir drone image onto a flat reference plane using
known GPS position and camera orientation, without ground control points. Export the result
as a georeferenced GeoTIFF (WGS-84 or UTM) and/or an interactive Folium HTML map.

## Functional Requirements

### REQ-GEO-001: Flat-Earth Ray Casting
The system SHALL project pixels to ground using the flat-Earth approximation:
ray direction in NED = R · K⁻¹ · [u, v, 1]ᵀ; intersection at depth h = flying_height_m.

### REQ-GEO-002: Rotation Convention (NED)
The system SHALL use ZYX extrinsic rotations (yaw → pitch → roll) to build the
camera-to-NED rotation matrix, with conventions:
- Yaw: 0° = North, clockwise positive; accepts 0–360° and ±180° ranges
- Pitch: 0° = horizontal, −90° = nadir
- Roll: 0° = level, positive = right-wing-down

### REQ-GEO-003: Flying Height Calculation
The system SHALL compute flying height as:
- `altitude_rel_m` when available (DJI relative altitude, AGL)
- `altitude_abs_m − surface_altitude_m` otherwise (AMSL minus user-supplied terrain)
The `surface_altitude_m` parameter (default 0.0) SHALL be applied when `altitude_rel_m`
is None. (Fixes prior bug where parameter was declared but unused.)

### REQ-GEO-004: Full Pixel Grid
When `full_grid=True`, the system SHALL compute lat/lon for every pixel and return them
as `(H × W)` NumPy arrays in `GeoRect.lat_grid` / `lon_grid`.

### REQ-GEO-005: Corner Coordinates
The system SHALL always compute geographic coordinates for the four image corners
(TL, TR, BR, BL) regardless of `full_grid`.

### REQ-GEO-006: GeoTIFF Export
The system SHALL write source pixels without resampling using a rotated affine transform
fitted to the four corner GCPs. Output SHALL support any EPSG via the `epsg` parameter.

### REQ-GEO-007: UTM Output
When `epsg` is a UTM zone EPSG code, the system SHALL reproject corner coordinates to that
CRS using pyproj before writing the GeoTIFF.

### REQ-GEO-008: GSD Estimate
The system SHALL compute and report an approximate ground sampling distance:
`gsd_m ≈ flying_height_m / mean(fx, fy)`.

## Acceptance Scenarios

### SCENARIO-GEO-001: Nadir image, sea-level surface
**GIVEN** lat=78.819°N, lon=−0.674°W, alt_abs=841 m, surface=0 m, yaw=301.57°,
  pitch=−90.807°, roll=0.238°, fx=9300 px, fy=9309 px, 11664×8750 px
**WHEN** `georectify(meta, camera, surface_altitude_m=0)` is called
**THEN** flying_height=841 m; GSD ≈ 0.090 m/px; footprint ≈ 1054 m × 791 m

### SCENARIO-GEO-002: surface_altitude_m applied when altitude_rel_m is None
**GIVEN** meta.altitude_abs_m=500, meta.altitude_rel_m=None, surface_altitude_m=100
**WHEN** `georectify()` is called
**THEN** flying_height = 400 m (not 500 m)

### SCENARIO-GEO-003: GeoTIFF in UTM 30N
**GIVEN** a GeoRect result and epsg=32630
**WHEN** `to_geotiff(result, image_path, output_path, epsg=32630)` is called
**THEN** output .tif has CRS EPSG:32630; corner coordinates in metres

### SCENARIO-GEO-004: Missing pitch/roll → nadir defaults applied
**GIVEN** meta.gimbal_pitch=None, meta.gimbal_roll=None
**WHEN** `georectify()` is called
**THEN** pitch=−90°, roll=0° used; optical axis points straight down

## Implementation Status (2026-04-12)

**Status**: Implemented

### What's Built
- `src/direct_georef/georectify.py:georectify()` — covers REQ-GEO-001, 002, 004, 005, 008
- flying height fix — covers REQ-GEO-003 (bug fixed)
- `src/direct_georef/export.py:to_geotiff()` — covers REQ-GEO-006, 007

### Deviations from Spec
- None

### Deferred
- None
