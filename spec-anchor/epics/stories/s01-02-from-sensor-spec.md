# S01-02: Add from_sensor_spec() Camera Loader

> Status: Done | Epic: 01 | Last updated: 2026-04-12

## Description
Add `from_sensor_spec()` to `camera.py` as a general loader that takes focal length (mm)
and physical sensor dimensions (mm × mm) directly, without requiring a camera model
database lookup or EXIF focal length tag. Also add Phase One iXM entries to the sensor DB.

## OpenSpec References
- Spec: `openspec/capabilities/camera-model/spec.md`
- Requirements: REQ-CAM-005, REQ-CAM-006
- Scenarios: SCENARIO-CAM-001, SCENARIO-CAM-002, SCENARIO-CAM-003

## Acceptance Criteria
- [x] `from_sensor_spec(35, 43.9, 32.9, 11664, 8750)` returns CameraModel with fx≈9300, fy≈9309
- [x] `from_exif(35, 11664, 8750, model="IXM-100")` succeeds via Phase One sensor lookup
- [x] Existing DJI sensor lookup unchanged

## Tasks
1. Add `from_sensor_spec()` function
2. Add `_AERIAL_SENSORS` dict with Phase One iXM entries
3. Update `from_exif()` to check `_AERIAL_SENSORS` fallback after `_DJI_SENSORS`

## Implementation Notes
- **Key files**: `src/direct_georef/camera.py`
- **Deviations**: none
