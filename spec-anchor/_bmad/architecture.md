# Architecture — Direct_GeoRef

> Version: 1.1 | Status: Living Document | Last updated: 2026-04-12
> **Last Reconciled: 2026-04-12** — Review required if >30 days stale

## System Context

```
[JPEG Image]──EXIF/XMP──► metadata.py ──► ImageMetadata
[Cal. File] ─────────────► camera.py ───► CameraModel
                                               │
                                               ▼
                                        georectify.py ──► GeoRect
                                               │
                                    ┌──────────┴──────────┐
                                    ▼                      ▼
                               export.py              export.py
                               (GeoTIFF)              (Folium HTML)
```

## Component Architecture

| Module | Responsibility |
|--------|---------------|
| `metadata.py` | EXIF GPS parsing; DJI and Phase One XMP orientation extraction |
| `camera.py` | Pinhole camera model; loaders for OpenCV/Agisoft/Pix4D/sensor-spec |
| `georectify.py` | Flat-earth ray casting; pixel→ground coordinate projection |
| `export.py` | GeoTIFF (rasterio) and Folium HTML export |
| `flow.py` | High-level pipeline orchestration |
| `tracking.py` | Multi-image optical flow (ice/feature tracking) |

## Data Flows

### Single-image Direct Georectification
1. `read_metadata(image_path)` → `ImageMetadata` (lat, lon, alt, yaw, pitch, roll, image dims)
2. `from_sensor_spec(focal_mm, sensor_w_mm, sensor_h_mm, px_w, px_h)` → `CameraModel`
3. `georectify(meta, camera, surface_altitude_m)` → `GeoRect` (corner coords, pixel grid, GSD)
4. `to_geotiff(result, image_path, output_path, epsg=32630)` → `.tif`

## Deployment Topology

- Local workstation (macOS Apple Silicon)
- conda environment `direct-georef` (Python 3.11, conda-forge)
- Jupyter Lab for interactive use; `pytest` for CI

## Security Architecture

- No network access at runtime; all inputs are local files
- No authentication required

## Architectural Decision Records

### ADR-001: Flat-Earth Ray-Casting (No DEM)
- **Status**: Accepted
- **Context**: Direct georectification requires ground elevation. A full DEM integration adds significant complexity and data dependencies.
- **Decision**: Use a user-supplied scalar `surface_altitude_m` (default 0 = sea level).
- **Rationale**: Adequate for coastal/marine surveys where terrain is flat or known. Rangefinder AGL supersedes AMSL when available.
- **Consequences**: Error grows with terrain relief and off-nadir angle. Document limitation clearly.

### ADR-002: Phase One XMP via `aerialgps:` Namespace
- **Status**: Accepted
- **Context**: Phase One / Orthodrone cameras embed orientation in `aerialgps:GPSIMU*` and `Camera:*` XMP fields instead of `drone-dji:Gimbal*`.
- **Decision**: Parse `aerialgps:GPSIMUYaw/Pitch/Roll` as primary; `Camera:Yaw/Pitch/Roll` as fallback. Map to the same `gimbal_yaw/pitch/roll` fields used by DJI.
- **Rationale**: `aerialgps:` values are in the same convention as DJI gimbal angles (pitch 0=horizontal, −90=nadir). No rotation matrix changes required.
- **Consequences**: Yaw range 0–360° (Phase One) vs ±180° (DJI) both work identically with trig functions.

### ADR-003: `surface_altitude_m` Bug Fix
- **Status**: Accepted
- **Context**: `georectify()` declared `surface_altitude_m` parameter but did not apply it to the flying height calculation, making it useless for non-DJI images without relative altitude.
- **Decision**: Apply `h = meta.altitude_abs_m - surface_altitude_m` when `altitude_rel_m` is None.
- **Rationale**: Correctness. DJI images unaffected (they use `altitude_rel_m`).
- **Consequences**: Breaking change only if callers passed `surface_altitude_m` and relied on it being ignored.
