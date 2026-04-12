# S02-03: Ship Route Planning Through Ice

> Status: In Progress | Epic: 02 | Last updated: 2026-04-12

## User Story

As a ship navigator, I want an optimal north-to-south route computed automatically from
an ice map so I can plan the safest passage with minimum distance and ice proximity.

## OpenSpec Refs

REQ-ROUTE-001, REQ-ROUTE-002, REQ-ROUTE-003, REQ-ROUTE-004, REQ-ROUTE-005,
REQ-ROUTE-006, REQ-ROUTE-007, REQ-ROUTE-008, REQ-ROUTE-009

## Acceptance Criteria

- [ ] `find_route(mask, georect)` returns a `RouteResult` with non-empty `waypoints_latlon`
- [ ] Ship clearance dilation blocks 1-px corridors when beam is large
- [ ] `min_floe_area_m2` opens a path through an otherwise blocked scene
- [ ] ETA is computed correctly: distance / speed (1 kn ≈ 0.5144 m/s)
- [ ] `route_to_geopackage()` writes "route" (LINESTRING) and "waypoints" (POINT) layers
- [ ] Auto start/end: start is northernmost ice-free midpoint, end is southernmost

## Implementation Notes

- New module: `src/direct_georef/routing.py`
- Routing grid: `cv2.resize(mask, ..., interpolation=INTER_NEAREST)` at routing_scale
- Min floe filter: `cv2.connectedComponentsWithStats` → remove blobs < threshold before dilating
- Clearance dilation: `cv2.dilate` with `ceil(beam_m/2/gsd_proc)` iterations
- Proximity: `scipy.ndimage.distance_transform_edt(~dilated_mask)` (distance from water to ice)
- A*: heapq-based; parent dict for path reconstruction; octile heuristic
- Simplification: `shapely.LineString(path_px).simplify(epsilon_px)` in pixel space,
  then convert simplified vertices to lat/lon
- GeoPackage: `gpd.GeoDataFrame` with LINESTRING and POINT layers

## Test File

`tests/test_routing.py` — covers SCENARIO-ROUTE-001 through SCENARIO-ROUTE-005
