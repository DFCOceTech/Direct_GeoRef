"""Direct georectification of nadir drone imagery.

Supports DJI and Phase One / Orthodrone aerial camera systems.

Quick start
-----------
    from direct_georef.metadata import read_metadata
    from direct_georef.camera import from_sensor_spec
    from direct_georef.georectify import georectify
    from direct_georef.export import to_geotiff

    meta   = read_metadata("image.jpg")
    camera = from_sensor_spec(focal_length_mm=35, sensor_width_mm=43.9,
                               sensor_height_mm=32.9, image_width=11664,
                               image_height=8750)
    result = georectify(meta, camera, surface_altitude_m=0)
    to_geotiff(result, "image.jpg", "output/image.tif", epsg=32630)
"""
