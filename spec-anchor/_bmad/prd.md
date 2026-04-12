# Product Requirements Document — Direct_GeoRef

> Version: 1.1 | Status: Living Document | Last updated: 2026-04-12

## Functional Requirements

| ID | Requirement | OpenSpec Capability |
|----|-------------|---------------------|
| FR-01 | Extract GPS position (lat/lon/altitude) from drone imagery EXIF | `metadata-extraction` |
| FR-02 | Extract camera orientation (yaw/pitch/roll) from DJI XMP metadata | `metadata-extraction` |
| FR-03 | Extract camera orientation from Phase One / Orthodrone XMP metadata | `metadata-extraction` |
| FR-04 | Load camera intrinsic model from calibration file (OpenCV, Agisoft, Pix4D) | `camera-model` |
| FR-05 | Build camera intrinsic model from EXIF focal length + known sensor dimensions | `camera-model` |
| FR-06 | Build camera intrinsic model from explicit focal length (mm) + sensor spec (mm) | `camera-model` |
| FR-07 | Direct georectify a nadir/near-nadir image to a flat reference plane | `georectification` |
| FR-08 | Export georectified result as a GeoTIFF in WGS-84 or UTM projection | `georectification` |
| FR-09 | Export an interactive Folium HTML map of the georectified result | `georectification` |

## Non-Functional Requirements

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-01 | GeoTIFF corner placement accuracy (flat terrain, nadir) | < 1 × GSD |
| NFR-02 | Processing time for a 100 MP image (full pixel grid) | < 120 s on Apple Silicon |
| NFR-03 | No dependency on proprietary SDKs or closed tools | Pure Python + conda-forge |

## Interface Contracts

| Interface | Protocol | Notes |
|-----------|----------|-------|
| Input imagery | JPEG with EXIF+XMP | Minimum: GPS lat/lon/alt in EXIF GPSInfo |
| Calibration files | OpenCV YAML/JSON, Agisoft XML, Pix4D CSV | At least one required if EXIF focal length absent |
| GeoTIFF output | GeoTIFF (LZW compressed) | WGS-84 (EPSG:4326) or UTM |

| FR-10 | Export georeferenced ice polygons as GeoPackage (single image) | `vector-export` |
| FR-11 | Export georeferenced ice polygons for a sequence of images | `vector-export` |
| FR-12 | Compute per-blob ice statistics: area, major/minor axes, orientation, centroid | `ice-analysis` |
| FR-13 | Compute scene-level ice aggregate statistics (coverage, blob count, size dist.) | `ice-analysis` |
| FR-14 | Find optimal N→S ship route through ice; export as GeoPackage | `route-planning` |

## OpenSpec Capability Mapping

| PRD Requirement | OpenSpec Capability | Epic |
|-----------------|---------------------|------|
| FR-01, FR-02, FR-03 | `metadata-extraction` | Epic 01 |
| FR-04, FR-05, FR-06 | `camera-model` | Epic 01 |
| FR-07, FR-08, FR-09 | `georectification` | Epic 01 |
| FR-10, FR-11 | `vector-export` | Epic 02 |
| FR-12, FR-13 | `ice-analysis` | Epic 02 |
| FR-14 | `route-planning` | Epic 02 |
