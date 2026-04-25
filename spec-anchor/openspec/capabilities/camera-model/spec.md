# Camera Model — Specification

> Version: 1.2 | Status: Implemented | Last updated: 2026-04-25

## Purpose

Represent a pinhole camera with optional Brown-Conrady distortion. Provide loaders that
build a `CameraModel` from calibration files or from physical sensor specifications, without
requiring the camera make/model to be embedded in EXIF.

## Functional Requirements

### REQ-CAM-001: Pinhole Model
The system SHALL represent the camera as a pinhole model with parameters
`fx, fy, cx, cy` (pixels) and distortion coefficients `k1, k2, k3, p1, p2`.

### REQ-CAM-002: Intrinsic Matrix
The system SHALL expose `CameraModel.K` as a 3×3 NumPy array and `dist_coeffs` as an
OpenCV-compatible 5-element array `[k1, k2, p1, p2, k3]`.

### REQ-CAM-003: File Loaders
The system SHALL provide loaders for:
- `from_opencv(cal_path)` — OpenCV YAML/JSON `FileStorage` format
- `from_agisoft_xml(cal_path)` — Agisoft Metashape XML calibration
- `from_pix4d_csv(cal_path)` — Pix4D `_pix4d_camera_internals.csv`

### REQ-CAM-004: EXIF + Sensor Lookup
The system SHALL provide `from_exif(focal_length_mm, image_width, image_height, model=...)`
that resolves sensor size via a built-in database of known DJI and Phase One camera models
and converts focal length in mm to pixels. The DJI sensor database SHALL include at minimum:
FC300C/FC300X/FC330 (Phantom 3/4, 1/2.3"), FC6310 (Phantom 4 Pro, 1"), FC220 (Mavic Pro),
FC350/FC550 (Inspire X5), L1D-20c (Mavic 2 Pro, 1"), FC7303 (Mini 2), FC3582 (Mini 3, 1/1.3").

### REQ-CAM-005: Explicit Sensor Spec
The system SHALL provide `from_sensor_spec(focal_length_mm, sensor_width_mm,
sensor_height_mm, image_width, image_height)` that builds a `CameraModel` without
requiring a file or camera model lookup. This is the primary loader when focal length
is known from operator records rather than EXIF.

### REQ-CAM-006: Phase One iXM Sensor Database
The system SHALL include entries for Phase One iXM cameras in the sensor database:
- `IXM-100`: 43.9 × 32.9 mm (100 MP, BSI medium-format)
- `IXM-50`: 43.9 × 32.9 mm
- `IXM-RS150F`: 53.4 × 40.0 mm

### REQ-CAM-007: Principal Point Default
When no explicit principal point is available, the system SHALL default to image centre:
`cx = image_width / 2`, `cy = image_height / 2`.

## Acceptance Scenarios

### SCENARIO-CAM-001: from_sensor_spec — Phase One iXM-100 at 35 mm
**GIVEN** focal_length_mm=35, sensor_width_mm=43.9, sensor_height_mm=32.9,
  image_width=11664, image_height=8750
**WHEN** `from_sensor_spec()` is called
**THEN** fx ≈ 9300 px, fy ≈ 9309 px, cx=5832, cy=4375 within ±1 px

### SCENARIO-CAM-002: from_exif — Phase One IXM-100 lookup
**GIVEN** focal_length_mm=35, model="IXM-100", image_width=11664, image_height=8750
**WHEN** `from_exif()` is called
**THEN** same result as SCENARIO-CAM-001

### SCENARIO-CAM-003: from_sensor_spec — unknown sensor, no db lookup needed
**GIVEN** any focal_length_mm, sensor dimensions, and image dimensions
**WHEN** `from_sensor_spec()` is called
**THEN** CameraModel is returned with correct fx/fy/cx/cy; no error

### SCENARIO-CAM-004: from_exif — unknown model, no sensor size → error
**GIVEN** focal_length_mm=35, model="UNKNOWN-XYZ", no sensor_width/height args
**WHEN** `from_exif()` is called
**THEN** `ValueError` is raised naming the model

### SCENARIO-CAM-005: from_exif — DJI Mini 3 (FC3582) lookup
**GIVEN** focal_length_mm=6.72, model="FC3582", image_width=4032, image_height=3024
**WHEN** `from_exif()` is called
**THEN** fx ≈ 2808 px, fy ≈ 2807 px, cx=2016, cy=1512 within ±1 px (sensor 9.65 × 7.24 mm)

## Implementation Status (2026-04-25)

**Status**: Implemented

### What's Built
- `src/direct_georef/camera.py:CameraModel` — covers REQ-CAM-001, 002
- `src/direct_georef/camera.py:from_opencv/agisoft_xml/pix4d_csv()` — covers REQ-CAM-003
- `src/direct_georef/camera.py:from_exif()` — covers REQ-CAM-004
- `src/direct_georef/camera.py:from_sensor_spec()` — covers REQ-CAM-005
- Phase One entries in `_AERIAL_SENSORS` dict — covers REQ-CAM-006
- Principal point default — covers REQ-CAM-007
- `_DJI_SENSORS["FC3582"] = (9.65, 7.24)` — extends REQ-CAM-004 for DJI Mini 3 (SCENARIO-CAM-005)

### Deviations from Spec
- None

### Deferred
- None
