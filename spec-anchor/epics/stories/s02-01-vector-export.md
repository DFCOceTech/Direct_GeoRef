# S02-01: Vector Export of Ice Polygons to GeoPackage

> Status: In Progress | Epic: 02 | Last updated: 2026-04-12

## User Story

As an ice analyst, I want to export detected ice polygons as a georeferenced GeoPackage
so that I can open them in QGIS, overlay with charts, and compute statistics externally.

## OpenSpec Refs

REQ-VEC-001, REQ-VEC-002, REQ-VEC-003, REQ-VEC-004, REQ-VEC-005, REQ-VEC-006

## Acceptance Criteria

- [ ] `ice_mask_to_polygons(mask, georect)` returns a `gpd.GeoDataFrame` with CRS=EPSG:4326
- [ ] Each polygon has columns: image_idx, blob_id, area_m2, geometry
- [ ] `min_area_m2` parameter filters out small polygons
- [ ] `masks_to_polygons([m1,m2], [gr1,gr2])` produces image_idx 0 and 1 in same frame
- [ ] `to_geopackage(gdf, path)` writes a readable GeoPackage
- [ ] Coordinate accuracy: centroid within 2× GSD of expected position for synthetic test

## Implementation Notes

- New module: `src/direct_georef/vector.py`
- Polygon extraction: `cv2.findContours(mask, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)`
- Coordinate transform: `_pixel_to_latlon(u, v, georect)` — bilinear from 4 corner GCPs
- GeoPackage write: `gdf.to_file(path, layer=layer, driver="GPKG")`
- Add `geopandas` to `environment.yml`

## Test File

`tests/test_vector.py` — covers SCENARIO-VEC-001 through SCENARIO-VEC-004
