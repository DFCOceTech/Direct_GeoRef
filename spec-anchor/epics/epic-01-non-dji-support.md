# Epic 01: Non-DJI System Support

> Status: Done | Last updated: 2026-04-12

## Goal
Generalize Direct_GeoRef to accept imagery from non-DJI aerial camera systems, specifically
Phase One / Orthodrone configurations that embed orientation in `aerialgps:` / `Camera:` XMP
namespaces and do not embed focal length in EXIF. Add `from_sensor_spec()` as the primary
camera model loader when EXIF focal length is absent.

## Dependencies
- Depends on: none
- Blocks: none

## Stories

| ID | Story | Status | OpenSpec Refs |
|----|-------|--------|---------------|
| S01-01 | Parse Phase One XMP orientation | Done | REQ-META-003, REQ-META-004 |
| S01-02 | Add from_sensor_spec() camera loader | Done | REQ-CAM-005, REQ-CAM-006 |
| S01-03 | Fix surface_altitude_m in georectify() | Done | REQ-GEO-003 |
| S01-04 | Georeference NREP25 Phase One image → UTM GeoTIFF | Done | REQ-GEO-006, REQ-GEO-007 |

## Acceptance Criteria
- [x] `read_metadata()` correctly extracts position + orientation from the Phase One XMP
- [x] `from_sensor_spec(35, 43.9, 32.9, 11664, 8750)` returns valid CameraModel
- [x] `georectify()` uses correct flying height when `altitude_rel_m` is None
- [x] Output GeoTIFF in EPSG:32630 (UTM Zone 30N) opens correctly in QGIS/GDAL
