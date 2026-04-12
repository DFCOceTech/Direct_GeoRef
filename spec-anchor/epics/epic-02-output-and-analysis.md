# Epic 02: Output and Analysis

> Status: In Progress | Last updated: 2026-04-12

## Goal

Add three complementary output and analysis capabilities built on top of the existing
direct georectification and ice detection pipeline:
1. **Vector Export** — georeferenced ice polygon GeoPackages (single image + sequence)
2. **Ice Statistics** — per-blob shape metrics and scene-level aggregates
3. **Route Planning** — A* optimal N→S ship route through ice, exported as GeoPackage

## Dependencies

- Depends on: Epic 01 (georectification, ice detection)
- Blocks: none

## Stories

| ID | Story | Status | OpenSpec Refs |
|----|-------|--------|---------------|
| S02-01 | Vector export of ice polygons to GeoPackage | Done | REQ-VEC-001→006 |
| S02-02 | Ice blob statistics (area, axes, orientation) | Done | REQ-ICE-001→007 |
| S02-03 | Ship route planning through ice | Done | REQ-ROUTE-001→009 |

## Acceptance Criteria

- [x] `ice_mask_to_polygons()` produces valid WGS-84 polygon GeoDataFrame from binary mask
- [x] `masks_to_polygons()` merges a list of images into one GeoDataFrame with image_idx
- [x] `to_geopackage()` writes a readable .gpkg file
- [x] `compute_ice_stats()` returns correct area_m2 and fitEllipse axes for test blobs
- [x] `find_route()` finds a north-to-south path through a synthetic gap-in-ice scenario
- [x] `route_to_geopackage()` writes both "route" and "waypoints" layers
- [x] All 36 existing tests continue to pass (59 total, all passing)
- [x] geopandas added to environment.yml
