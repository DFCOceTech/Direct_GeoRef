# S01-01: Parse Phase One XMP Orientation

> Status: Done | Epic: 01 | Last updated: 2026-04-12

## Description
Add `_parse_phaseone_xmp()` to `metadata.py` so that `read_metadata()` populates
`gimbal_yaw/pitch/roll` from Phase One / Orthodrone `aerialgps:` and `Camera:` XMP
namespaces, using the same angle convention as the existing DJI path.

## OpenSpec References
- Spec: `openspec/capabilities/metadata-extraction/spec.md`
- Requirements: REQ-META-003, REQ-META-004, REQ-META-005, REQ-META-006
- Scenarios: SCENARIO-META-002, SCENARIO-META-003, SCENARIO-META-004

## Acceptance Criteria
- [x] `gimbal_yaw=301.57`, `gimbal_pitch=-90.807`, `gimbal_roll=0.238` for the NREP25 test image
- [x] Rational string values (e.g. `"3015708/10000"`) correctly parsed to float
- [x] Falls back to `Camera:` namespace when `aerialgps:` fields absent
- [x] Converts `Camera:Pitch` from nadir-relative to DJI-convention by subtracting 90°
- [x] DJI images unaffected (namespace detection prevents cross-parsing)

## Tasks
1. Add `_parse_phaseone_xmp(path, meta)` function
2. Update `_parse_xmp()` dispatcher to call Phase One parser when `aerialgps:` namespace detected
3. Add `_eval_rational(s)` helper for rational string parsing
4. Update module docstring to document non-DJI support

## Implementation Notes
- **Key files**: `src/direct_georef/metadata.py`
- **Deviations**: none
