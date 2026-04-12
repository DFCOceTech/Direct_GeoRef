# Traceability Matrix — Direct_GeoRef

> Status: Living Document | Last updated: 2026-04-12

## PRD → OpenSpec → Epic → Implementation Status

| PRD Req | OpenSpec Capability | Epic | Impl Status | Notes |
|---------|---------------------|------|-------------|-------|
| FR-01 | `metadata-extraction` | Epic 01 | Done | `metadata.py:read_metadata()` |
| FR-02 | `metadata-extraction` | Epic 01 | Done | `metadata.py:_parse_dji_xmp()` |
| FR-03 | `metadata-extraction` | Epic 01 | Done | `metadata.py:_parse_phaseone_xmp()` |
| FR-04 | `camera-model` | Epic 01 | Done | `camera.py:from_opencv/agisoft/pix4d()` |
| FR-05 | `camera-model` | Epic 01 | Done | `camera.py:from_exif()` |
| FR-06 | `camera-model` | Epic 01 | Done | `camera.py:from_sensor_spec()` |
| FR-07 | `georectification` | Epic 01 | Done | `georectify.py:georectify()` |
| FR-08 | `georectification` | Epic 01 | Done | `export.py:to_geotiff()` |
| FR-09 | `georectification` | Epic 01 | Done | `export.py:to_folium_map()` |

**Impl Status values**: Done | Partial | In Progress | Not Started | Deferred | Untested

## NFR Verification

| NFR | Verification Method | Status | Notes |
|-----|---------------------|--------|-------|
| NFR-01 | Compare corner coords against known GCPs | Untested | No GCPs available for test image |
| NFR-02 | Time `georectify()` on 11664×8750 image | Untested | |
| NFR-03 | Inspect imports; no proprietary libs | Done | All deps on conda-forge |

| FR-10 | `vector-export` | Epic 02 | Done | `vector.py:ice_mask_to_polygons()`, `to_geopackage()` |
| FR-11 | `vector-export` | Epic 02 | Done | `vector.py:masks_to_polygons()` |
| FR-12 | `ice-analysis` | Epic 02 | Done | `icestats.py:compute_ice_stats()` → `BlobStats` |
| FR-13 | `ice-analysis` | Epic 02 | Done | `icestats.py:compute_ice_stats()` → `SceneStats` |
| FR-14 | `route-planning` | Epic 02 | Done | `routing.py:find_route()`, `route_to_geopackage()` |

## Epic Dependency Graph

```
Epic 01 (metadata + camera + georectify)
    └─▶ Epic 02 (vector export + ice stats + route planning)
```
