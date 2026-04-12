# Changelog — Direct_GeoRef

Rolling 2-week work log. Remove entries older than 2 weeks.

## 2026-04-12 — Epic 02: Output and Analysis

### New module: `vector.py`
- `ice_mask_to_polygons(mask, georect, min_area_m2, image_idx)` — converts binary ice mask
  to a GeoDataFrame of WGS-84 POLYGON features via `cv2.findContours` + bilinear corner GCP transform
- `masks_to_polygons(masks, georects, min_area_m2)` — sequence support with `image_idx` column
- `to_geopackage(gdf, path, layer)` — GeoPackage export via geopandas
- `_pixel_to_latlon(u, v, georect)` — bilinear interpolation from 4 corner GCPs; falls back to
  full lat_grid lookup if available

### New module: `icestats.py`
- `BlobStats` dataclass: blob_id, area_m2, major_axis_m, minor_axis_m, orientation_deg, centroid_lat/lon
- `SceneStats` dataclass: total_ice_area_m2, ice_fraction, blob_count, mean/max/min blob areas
- `compute_ice_stats(mask, georect, min_area_m2)` — uses `cv2.connectedComponentsWithStats` +
  `cv2.fitEllipse`; blobs sorted largest-first; area = area_px × gsd_m²

### New module: `routing.py`
- `RouteResult` dataclass: waypoints_latlon, total_distance_m, eta_s, navigable_fraction
- `find_route(mask, georect, ...)` — A* pathfinding at configurable routing_scale;
  min_floe_area_m2 filter removes small blobs before routing; ship clearance dilation via
  `cv2.dilate`; ice proximity penalty via `scipy.ndimage.distance_transform_edt`; octile
  heuristic; RDP simplification via `shapely.LineString.simplify`; auto N/S endpoint detection
- `route_to_geopackage(result, path)` — writes "route" (LINESTRING) + "waypoints" (POINT) layers

### Dependencies
- geopandas + pyogrio added to environment.yml and installed

### Tests
- `tests/test_vector.py` — 8 tests (SCENARIO-VEC-001→004)
- `tests/test_icestats.py` — 8 tests (SCENARIO-ICE-001→004)
- `tests/test_routing.py` — 7 tests (SCENARIO-ROUTE-001→005)
- Total: 59 tests, all passing

### Spec / BMAD files written
- `spec-anchor/openspec/capabilities/vector-export/spec.md` (REQ-VEC-001→006)
- `spec-anchor/openspec/capabilities/ice-analysis/spec.md` (REQ-ICE-001→007)
- `spec-anchor/openspec/capabilities/route-planning/spec.md` (REQ-ROUTE-001→009)
- `spec-anchor/epics/epic-02-output-and-analysis.md` + stories S02-01, S02-02, S02-03
- PRD updated: FR-10→14 added; traceability updated

## 2026-04-12 — Epic 01: Non-DJI System Support

### New capability: Phase One / Orthodrone XMP parsing (`metadata.py`)
- Refactored `_parse_dji_xmp()` (now takes `xmp: str`, not `path`) and wrapped in
  `_parse_xmp()` dispatcher that detects namespace and routes to DJI or Phase One parser
- Added `_parse_phaseone_xmp()` — reads `aerialgps:GPSIMUYaw/Pitch/Roll` (rational
  strings e.g. `"3015708/10000"`) as primary; falls back to `Camera:Yaw/Pitch/Roll`
- Added `_eval_rational()` helper for rational string parsing
- `Camera:Pitch` convention converted: `gimbal_pitch = Camera:Pitch − 90°`
- `aerialgps:GPSAltitudeAboveTakeOff` intentionally NOT stored as `altitude_rel_m`;
  it is above-takeoff, not AGL — flying height falls back to `altitude_abs_m − surface_alt`
- Updated module docstring to document both DJI and Phase One support

### New loader: `from_sensor_spec()` (`camera.py`)
- Accepts focal_length_mm, sensor_width_mm, sensor_height_mm, image_width, image_height
- Primary loader for non-DJI images where EXIF focal length is absent
- Added `_AERIAL_SENSORS` dict: Phase One IXM-100, IXM-50, IXM-RS150F

### Bug fix: `surface_altitude_m` in `georectify()` (`georectify.py`)
- Parameter was declared but ignored; now applied when `altitude_rel_m is None`:
  `h = altitude_abs_m − surface_altitude_m`
- DJI images unaffected (they supply `altitude_rel_m`)

### Tests added (`tests/test_camera.py`, `tests/test_georectify.py`)
- `TestFromExif::test_phase_one_ixm100_lookup` — IXM-100 sensor DB lookup
- `TestFromSensorSpec` — 5 tests for `from_sensor_spec()`
- `TestSurfaceAltitude` — 3 tests for the `surface_altitude_m` bug fix
- `TestPhaseOneConventions` — 3 tests: nadir pitch, 360° yaw identity, full pipeline GSD
- Total: 36 tests, all passing

### Spec / BMAD files written
- `spec-anchor/openspec/capabilities/metadata-extraction/spec.md` (REQ-META-001→009)
- `spec-anchor/openspec/capabilities/camera-model/spec.md` (REQ-CAM-001→007)
- `spec-anchor/openspec/capabilities/georectification/spec.md` (REQ-GEO-001→008)
- `spec-anchor/_bmad/prd.md`, `architecture.md`, `traceability.md` — filled from templates
- `spec-anchor/epics/epic-01-non-dji-support.md` + stories S01-01, S01-02

### First validated georectification
- Image: NREP25 Phase One iXM-100, 2025-07-22, 78.8189°N, −0.6738°W
- Flying height: 841.083 m AMSL (sea level surface)
- GSD: 9.04 cm/px; footprint: ~1048 × 791 m; heading: 301.57°T
- Output: `output/NREP25_P0066781_UTM30N.tif` — 123 MB, EPSG:32630
