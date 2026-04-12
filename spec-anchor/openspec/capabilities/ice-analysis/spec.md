# Ice Analysis — Specification

> Version: 1.0 | Status: In Progress | Last updated: 2026-04-12

## Purpose

Compute per-blob shape statistics (area, major/minor axes, orientation, centroid) and
scene-level aggregate statistics from a binary ice detection mask and a GeoRect.
Area and axis lengths are expressed in georeferenced metres using the image GSD.

## Functional Requirements

### REQ-ICE-001: compute_ice_stats()
The system SHALL provide `compute_ice_stats(mask, georect, min_area_m2)` that returns
a `(list[BlobStats], SceneStats)` tuple.

### REQ-ICE-002: BlobStats Fields
Each `BlobStats` instance SHALL contain:
- `blob_id` (int): unique identifier per blob (largest blob is id=0)
- `area_m2` (float): georeferenced blob area in m²; computed as `area_px × gsd_m²`
- `major_axis_m` (float): length of the major ellipse axis in metres
- `minor_axis_m` (float): length of the minor ellipse axis in metres
- `orientation_deg` (float): angle of major axis from positive image x-axis (columns),
  counterclockwise, in [0, 180°); as returned by `cv2.fitEllipse`
- `centroid_lat` (float): geographic latitude of blob centroid (decimal degrees)
- `centroid_lon` (float): geographic longitude of blob centroid (decimal degrees)

### REQ-ICE-003: SceneStats Fields
`SceneStats` SHALL contain:
- `total_ice_area_m2` (float): sum of all blob areas above threshold
- `ice_fraction` (float): ice area / total scene area (0–1)
- `blob_count` (int): number of blobs above the area threshold
- `mean_blob_area_m2` (float): mean of individual blob areas
- `max_blob_area_m2` (float): area of the largest blob
- `min_blob_area_m2` (float): area of the smallest blob above threshold

### REQ-ICE-004: Georeferenced Area Calculation
Blob area SHALL be computed as `area_px × gsd_m²` where `gsd_m` comes from
`GeoRect.gsd_m`. This is the approximate flat-Earth area at the image centre.

### REQ-ICE-005: fitEllipse for Axes
Major and minor axis lengths SHALL be computed from `cv2.fitEllipse`, which fits an
ellipse to the blob contour. The axes are converted from pixels to metres using `gsd_m`.
`fitEllipse` requires at least 5 contour points; blobs with fewer points SHALL report
`major_axis_m = minor_axis_m = orientation_deg = NaN`.

### REQ-ICE-006: Minimum Area Filter
`compute_ice_stats()` SHALL accept a `min_area_m2` parameter (default 0.0) and exclude
blobs below this threshold from both the blob list and the scene aggregates.

### REQ-ICE-007: Centroid Geographic Coordinates
Centroid lat/lon SHALL be computed by bilinear interpolation from `GeoRect.corners_latlon`
using the pixel centroid coordinates, or by lookup in `GeoRect.lat_grid` / `lon_grid`
if those grids are available.

## Acceptance Scenarios

### SCENARIO-ICE-001: Single rectangular blob area
**GIVEN** a 100×100 mask with a 40×20 px ice patch and gsd_m=1.0
**WHEN** `compute_ice_stats(mask, georect)` is called
**THEN** one BlobStats with area_m2 ≈ 800 m² (40×20); blob_count=1

### SCENARIO-ICE-002: fitEllipse axes reasonable
**GIVEN** a 100×100 mask with an approximately circular 30-px-radius blob
**WHEN** `compute_ice_stats(mask, georect, min_area_m2=0)` is called
**THEN** major_axis_m ≈ minor_axis_m (ratio < 1.2); both > 0

### SCENARIO-ICE-003: Min area filter
**GIVEN** two blobs: one 100 m² and one 10 m²
**WHEN** `compute_ice_stats(mask, georect, min_area_m2=50)` is called
**THEN** blob_count=1; SceneStats.total_ice_area_m2 ≈ 100

### SCENARIO-ICE-004: Scene-level aggregates
**GIVEN** three blobs with areas 100, 200, 300 m²
**WHEN** `compute_ice_stats()` is called
**THEN** total_ice_area_m2=600; mean=200; max=300; min=100; blob_count=3

## Implementation Status (2026-04-12)

**Status**: Implemented

### What's Built
- `src/direct_georef/icestats.py` — REQ-ICE-001 through REQ-ICE-007

### Deviations from Spec
- None

### Deferred
- None
