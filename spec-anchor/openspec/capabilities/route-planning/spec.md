# Route Planning — Specification

> Version: 1.0 | Status: In Progress | Last updated: 2026-04-12

## Purpose

Find an optimal north-to-south ship route through sea ice using A* pathfinding on a
georeferenced occupancy grid derived from a binary ice detection mask. The route avoids
ice floes above a configurable size threshold, accounts for ship beam clearance, and
is exported as a GeoPackage with LINESTRING and POINT layers.

## Functional Requirements

### REQ-ROUTE-001: find_route()
The system SHALL provide `find_route(mask, georect, ...)` that returns a `RouteResult`
containing: `waypoints_latlon` (list of (lat, lon) tuples), `total_distance_m` (float),
`eta_s` (float, estimated travel time in seconds at ship_speed_kn).

### REQ-ROUTE-002: A* Pathfinding
The system SHALL implement A* search on a 2D 8-connected grid at a user-configurable
`routing_scale` (default 0.1 = 10% of original image dimensions).

### REQ-ROUTE-003: Ship Clearance Buffer
Before pathfinding, the system SHALL dilate the ice mask by
`ceil(ship_beam_m / 2 / gsd_proc)` pixels (half-beam clearance), where `gsd_proc` is
the GSD at the routing scale. Ship beam defaults to 15 m.

### REQ-ROUTE-004: Minimum Floe Size Filter
The system SHALL accept `min_floe_area_m2` (default 0.0). Ice blobs with georeferenced
area below this threshold SHALL be removed from the mask before pathfinding (treated as
navigable water). This allows the ship to pass through fragmented brash ice.

### REQ-ROUTE-005: Auto Start/End Detection
When `start_latlon` or `end_latlon` are not provided, the system SHALL automatically:
- Start: northernmost row in the navigable mask that contains any navigable pixel;
  start column = midpoint of the navigable pixel range in that row.
- End: southernmost such row; end column = midpoint of the navigable pixel range.

### REQ-ROUTE-006: Weighted Cost Function
Step cost from pixel (r,c) to neighbor (r',c') SHALL be:
  `step_dist_px * gsd_proc * w_dist + w_ice / (dist_to_ice_m + 1)`
where `dist_to_ice_m` is the Euclidean distance in metres from (r',c') to the nearest
ice pixel (from the distance transform), `w_dist` defaults to 1.0, `w_ice` to 2.0.

### REQ-ROUTE-007: Distance Transform for Ice Proximity
The system SHALL compute `scipy.ndimage.distance_transform_edt` on the inverted
(dilated) ice mask to obtain per-pixel distance to nearest ice in pixels.

### REQ-ROUTE-008: Path Simplification
The raw A* pixel path SHALL be simplified using the Ramer-Douglas-Peucker (RDP)
algorithm via `shapely.geometry.LineString.simplify()` with
`epsilon = ship_beam_m / 2` (default 7.5 m) before converting to geographic coordinates.

### REQ-ROUTE-009: GeoPackage Output
`route_to_geopackage(result, georect, path)` SHALL write two layers to a GeoPackage:
- `"route"`: single LINESTRING feature with attributes
  `total_distance_m` and `eta_minutes`
- `"waypoints"`: POINT features with `seq` (sequence number) attribute
Both layers SHALL use CRS WGS-84 (EPSG:4326).

## Ship Parameters (defaults)

| Parameter | Default | Notes |
|-----------|---------|-------|
| ship_beam_m | 15.0 | vessel beam; clearance = beam/2 |
| ship_length_m | 90.0 | reserved for future use (turning radius) |
| ship_speed_kn | 3.0 | 1 knot ≈ 0.5144 m/s |
| routing_scale | 0.1 | 10% of image; ~90 cm/px for NREP25 at 841 m |

## Acceptance Scenarios

### SCENARIO-ROUTE-001: Route found through a gap
**GIVEN** a 50×50 mask with a vertical ice wall at columns 20–30 with a gap at rows 20–30,
  GeoRect with synthetic corners
**WHEN** `find_route(mask, georect, routing_scale=1.0)` is called
**THEN** a non-empty waypoints list is returned; first waypoint is north of last

### SCENARIO-ROUTE-002: Ship clearance removes marginal path
**GIVEN** a corridor 1 px wide (narrower than clearance)
**WHEN** `find_route()` is called with ship_beam_m large enough to close the corridor
**THEN** `RouteResult.waypoints_latlon` is empty (no navigable path)

### SCENARIO-ROUTE-003: Min floe filter opens a path
**GIVEN** an ice mask with only small blobs (< min_floe_area_m2)
**WHEN** `find_route(mask, georect, min_floe_area_m2=10000)` is called
**THEN** the path is found (all blobs removed as too small)

### SCENARIO-ROUTE-004: ETA calculation
**GIVEN** a route with total_distance_m = 1543 m, ship_speed_kn = 3.0
**WHEN** RouteResult is returned
**THEN** eta_s ≈ 1000 s  (1543 m / 1.543 m/s)

### SCENARIO-ROUTE-005: GeoPackage output round-trip
**GIVEN** a valid RouteResult
**WHEN** `route_to_geopackage(result, georect, path)` is called
**THEN** file exists; both "route" and "waypoints" layers are readable

## Implementation Status (2026-04-12)

**Status**: Implemented

### What's Built
- `src/direct_georef/routing.py` — REQ-ROUTE-001 through REQ-ROUTE-009

### Deviations from Spec
- None

### Deferred
- None
