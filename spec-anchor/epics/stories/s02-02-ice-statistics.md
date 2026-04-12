# S02-02: Ice Blob Statistics

> Status: In Progress | Epic: 02 | Last updated: 2026-04-12

## User Story

As an ice analyst, I want per-blob shape statistics (area, major/minor axes, orientation)
and scene-level aggregates so I can quantify ice coverage and floe morphology.

## OpenSpec Refs

REQ-ICE-001, REQ-ICE-002, REQ-ICE-003, REQ-ICE-004, REQ-ICE-005, REQ-ICE-006, REQ-ICE-007

## Acceptance Criteria

- [ ] `compute_ice_stats(mask, georect)` returns `(list[BlobStats], SceneStats)`
- [ ] `area_m2 = area_px × gsd_m²` is correct for a 40×20 px blob with gsd_m=1.0
- [ ] `major_axis_m` and `minor_axis_m` are > 0 for a circular blob
- [ ] `min_area_m2` filter excludes small blobs from both list and scene aggregates
- [ ] `SceneStats.ice_fraction` is between 0 and 1
- [ ] Centroid lat/lon within 2×GSD of expected position for synthetic blob

## Implementation Notes

- New module: `src/direct_georef/icestats.py`
- Connected components: `cv2.connectedComponentsWithStats`
- Ellipse fitting: `cv2.fitEllipse(contour)` — requires ≥ 5 points
- Area: `area_px × gsd_m²`; Axes: `ellipse_axes_px × gsd_m`
- Scene area: `(mask.shape[0] × mask.shape[1]) × gsd_m²`
- Blobs with < 5 contour points: axes and orientation reported as NaN

## Test File

`tests/test_icestats.py` — covers SCENARIO-ICE-001 through SCENARIO-ICE-004
