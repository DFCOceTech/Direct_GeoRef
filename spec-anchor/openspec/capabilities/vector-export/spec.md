# Vector Export — Specification

> Version: 1.0 | Status: In Progress | Last updated: 2026-04-12

## Purpose

Convert binary ice detection masks to georeferenced polygon geometries and export
them as GeoPackage files. Supports both single-image and multi-image sequence export
with a consistent schema.

## Functional Requirements

### REQ-VEC-001: ice_mask_to_polygons()
The system SHALL provide `ice_mask_to_polygons(mask, georect, min_area_m2, image_idx)`
that accepts a binary uint8 mask (255 = ice, 0 = water) and a `GeoRect` object and
returns a `GeoDataFrame` with POLYGON geometries in WGS-84 (EPSG:4326).

### REQ-VEC-002: Minimum Area Filter
`ice_mask_to_polygons()` SHALL accept a `min_area_m2` parameter (default 0.0) and
exclude any polygon whose georeferenced area is below this threshold.

### REQ-VEC-003: GeoPackage Export
The system SHALL provide `to_geopackage(gdf, path, layer)` that writes a
`GeoDataFrame` to a GeoPackage file using a user-specified layer name.

### REQ-VEC-004: Pixel-to-Geo Coordinate Transform
Pixel corners of each polygon contour SHALL be transformed to geographic coordinates
using bilinear interpolation over the four `GeoRect.corners_latlon` GCPs.
If `georect.lat_grid` is not None, the full grid SHALL be used for a nearest-pixel
lookup (more accurate for off-nadir imagery).

### REQ-VEC-005: Sequence Support
The system SHALL provide `masks_to_polygons(masks, georects, min_area_m2)` that
accepts parallel lists of masks and GeoRect objects, calls `ice_mask_to_polygons()`
for each, and returns a single merged `GeoDataFrame` with an `image_idx` integer column
indicating which image each polygon came from.

### REQ-VEC-006: GeoDataFrame Schema
Each row in the output `GeoDataFrame` SHALL have the following columns:
- `image_idx` (int): source image index (0 for single-image calls)
- `blob_id` (int): per-image blob identifier (0-based)
- `area_m2` (float): approximate georeferenced polygon area in m²
- `geometry` (Polygon): exterior ring in WGS-84

## Acceptance Scenarios

### SCENARIO-VEC-001: Single ice polygon extracted
**GIVEN** a 100×100 binary mask with a 20×20 ice patch at centre,
  and a GeoRect with known corners spanning ~0.01° lat/lon
**WHEN** `ice_mask_to_polygons(mask, georect)` is called
**THEN** one Polygon is returned; area_m2 > 0; geometry is valid

### SCENARIO-VEC-002: Min area filter excludes small patches
**GIVEN** the same mask with two patches: 20×20 and 4×4
**WHEN** `ice_mask_to_polygons(mask, georect, min_area_m2=1000)` is called
**THEN** only the large patch is returned (small one filtered out)

### SCENARIO-VEC-003: Sequence export merges images
**GIVEN** two masks each with one polygon
**WHEN** `masks_to_polygons([m1, m2], [gr1, gr2])` is called
**THEN** result has 2 rows; image_idx 0 and 1 are both present

### SCENARIO-VEC-004: GeoPackage round-trip
**GIVEN** a GeoDataFrame from ice_mask_to_polygons()
**WHEN** `to_geopackage(gdf, path, layer="ice")` is called
**THEN** the file exists and can be re-read with geopandas

## Implementation Status (2026-04-12)

**Status**: Implemented

### What's Built
- `src/direct_georef/vector.py` — REQ-VEC-001 through REQ-VEC-006

### Deviations from Spec
- None

### Deferred
- None
