# Known Issues & Lessons Learned — Direct_GeoRef

> Last updated: 2026-04-12

## Active Issues

### PIL DecompressionBombWarning on 100 MP images
- **Symptom**: `PIL.Image.DecompressionBombWarning: Image size (102060000 pixels) exceeds limit`
- **Root Cause**: Pillow's default limit is 89 MP; Phase One iXM-100 is 102 MP
- **Workaround**: Set `PIL.Image.MAX_IMAGE_PIXELS = None` before calling `read_metadata()`
  or `to_geotiff()` in scripts
- **Fix**: Add a MAX_IMAGE_PIXELS override in the flow.py pipeline runner

### No DEM integration — terrain elevation must be supplied manually
- **Symptom**: Flying height computed as `altitude_abs_m − surface_altitude_m` where
  `surface_altitude_m` is a scalar (default 0 = sea level)
- **Root Cause**: By design (ADR-001); flat-earth approximation
- **Workaround**: Pass `surface_altitude_m=<terrain_elevation_m>` to `georectify()`
- **Fix**: Planned — optional DEM lookup via rasterio

### Focal length absent from Phase One EXIF processed via Capture One
- **Symptom**: `meta.focal_length_mm is None` for Phase One images
- **Root Cause**: Capture One does not write `FocalLength` to the exported JPEG EXIF
- **Workaround**: Use `from_sensor_spec(focal_mm, sw_mm, sh_mm, W, H)` with
  operator-supplied focal length
- **Fix**: None needed — `from_sensor_spec()` is the correct loader for this workflow

## Lessons Learned

### Phase One `GPSAltitudeAboveTakeOff` ≠ AGL
At first, `altitude_rel_m` was set from `aerialgps:GPSAltitudeAboveTakeOff`. This gave
791.534 m instead of the correct 841.083 m (GPS AMSL). The takeoff elevation was ~50 m
above sea level, so `above_takeoff = AMSL − takeoff_elevation = 841 − 50 = 791 m`.
Lesson: above-takeoff altitude is not flying-height-above-terrain. Only set `altitude_rel_m`
from DJI `RelativeAltitude`, which is a barometrically-corrected AGL.

### Corner-average ≠ image centre for wide-angle off-nadir cameras
A test checking that the footprint centroid (average of 4 corners) was close to GPS for a
near-nadir camera failed with 112 m error on a wide-angle test camera (4000×3000 px,
fx=2340). The perspective projection of widely-spaced corners is non-linear, so the average
diverges from the image centre projection. Fix: use a narrow-FOV (tiny) test image when
verifying nadir-approximation claims via corner centroid.

### `lon_distance_m` must convert degrees to radians before multiplying by R_EARTH
A test computing longitude distance as `delta_lon_deg × R_EARTH × cos(lat)` instead of
`delta_lon_rad × R_EARTH × cos(lat)` yielded 57× inflated errors. Always use
`np.radians()` or the `_dist_m()` helper.
