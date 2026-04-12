"""Export georectification results to GeoTIFF and Folium maps.

GeoTIFF
-------
Writes a GeoTIFF with the original RGB pixel values and a WGS-84
geographic coordinate system (EPSG:4326).  The affine transform is computed
directly from the four corner ground control points (GCPs) so the source
pixels are written without resampling — no edge-smearing artifacts.  The
resulting file has a rotated transform (non-north-up), which QGIS, ArcGIS,
and GDAL handle transparently.

Folium map
----------
Produces an interactive HTML map centred on the image centre with:
  - OpenStreetMap / Esri satellite tile layers
  - The georectified image as a semi-transparent overlay
  - A polygon showing the image footprint
  - A marker at the camera GPS position
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
from PIL import Image

from .georectify import GeoRect


# ---------------------------------------------------------------------------
# GeoTIFF export
# ---------------------------------------------------------------------------

def to_geotiff(
    result: GeoRect,
    image_path: str | Path,
    output_path: str | Path,
    epsg: int = 4326,
) -> Path:
    """Write a georectified GeoTIFF from the image and its GeoRect result.

    The affine transform is fitted to the four corner GCPs, so source pixels
    are written without any resampling.  The output has a rotated (non-north-up)
    transform — no edge smearing or KD-tree artifacts.

    Parameters
    ----------
    result:
        GeoRect produced by ``georectify()``.  ``full_grid`` is not required;
        corners are sufficient.
    image_path:
        Source JPEG/PNG.
    output_path:
        Destination .tif path.
    epsg:
        Output CRS. 4326 = WGS-84 lat/lon (default); pass a UTM zone EPSG
        (e.g. 32632) to write in metres.

    Returns
    -------
    Path to the written GeoTIFF.
    """
    try:
        import rasterio
        from rasterio.crs import CRS
        from rasterio.control import GroundControlPoint
        from rasterio.transform import from_gcps
    except ImportError as exc:
        raise ImportError("rasterio is required for GeoTIFF export") from exc

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    src_img = np.array(Image.open(image_path).convert("RGB"))
    H, W = src_img.shape[:2]

    corners = result.corners_latlon  # {TL, TR, BR, BL} → (lat, lon)

    # Optionally reproject corner coords to target CRS
    if epsg != 4326:
        from pyproj import Transformer
        tr = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)

        def _xy(name):
            lat, lon = corners[name]
            x, y = tr.transform(lon, lat)
            return x, y
    else:
        def _xy(name):
            lat, lon = corners[name]
            return lon, lat   # rasterio GCP: x=lon, y=lat

    # Four GCPs at image corners (row, col, x, y)
    gcps = [
        GroundControlPoint(row=0,     col=0,     x=_xy("TL")[0], y=_xy("TL")[1]),
        GroundControlPoint(row=0,     col=W - 1, x=_xy("TR")[0], y=_xy("TR")[1]),
        GroundControlPoint(row=H - 1, col=0,     x=_xy("BL")[0], y=_xy("BL")[1]),
        GroundControlPoint(row=H - 1, col=W - 1, x=_xy("BR")[0], y=_xy("BR")[1]),
    ]

    # Fit an affine transform from the GCPs (exact for 4 coplanar points)
    transform = from_gcps(gcps)
    out_crs = CRS.from_epsg(epsg)

    with rasterio.open(
        output_path,
        "w",
        driver="GTiff",
        height=H,
        width=W,
        count=3,
        dtype="uint8",
        crs=out_crs,
        transform=transform,
        compress="lzw",
    ) as dst:
        for band in range(3):
            dst.write(src_img[:, :, band], band + 1)

    return output_path


# ---------------------------------------------------------------------------
# Folium interactive map
# ---------------------------------------------------------------------------

def to_folium_map(
    result: GeoRect,
    image_path: str | Path,
    output_path: Optional[str | Path] = None,
    image_opacity: float = 0.7,
    thumbnail_max_px: int = 800,
) -> "folium.Map":  # type: ignore[name-defined]
    """Create a Folium interactive map with the georectified image overlay.

    Parameters
    ----------
    result:
        GeoRect from ``georectify()``.
    image_path:
        Source JPEG/PNG (used for the image overlay).
    output_path:
        If provided, save the map as an HTML file at this path.
    image_opacity:
        Transparency of the image overlay (0 = transparent, 1 = opaque).
    thumbnail_max_px:
        Resize the image to this max dimension for the overlay (reduces
        HTML file size; use None to embed full resolution).

    Returns
    -------
    folium.Map object.
    """
    try:
        import folium
        import base64
        import io
    except ImportError as exc:
        raise ImportError("folium is required for interactive map export") from exc

    lat_c = result.metadata.latitude
    lon_c = result.metadata.longitude

    # --- Build map centred on image ---
    m = folium.Map(
        location=[lat_c, lon_c],
        zoom_start=17,
        tiles=None,
    )

    folium.TileLayer("OpenStreetMap", name="OpenStreetMap").add_to(m)
    folium.TileLayer(
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="Esri",
        name="Esri Satellite",
    ).add_to(m)

    # --- Image overlay ---
    img = Image.open(image_path).convert("RGB")
    if thumbnail_max_px:
        img.thumbnail((thumbnail_max_px, thumbnail_max_px), Image.LANCZOS)

    buf = io.BytesIO()
    img.save(buf, format="PNG")
    b64 = base64.b64encode(buf.getvalue()).decode()
    img_uri = f"data:image/png;base64,{b64}"

    # folium.raster_layers.ImageOverlay expects [[lat_sw, lon_sw], [lat_ne, lon_ne]]
    # but for a rotated image we use a custom overlay via JavaScript.
    # For simplicity with nadir imagery, use the bounding box.
    corners = result.corners_latlon
    lat_sw = min(c[0] for c in corners.values())
    lat_ne = max(c[0] for c in corners.values())
    lon_sw = min(c[1] for c in corners.values())
    lon_ne = max(c[1] for c in corners.values())

    folium.raster_layers.ImageOverlay(
        image=img_uri,
        bounds=[[lat_sw, lon_sw], [lat_ne, lon_ne]],
        opacity=image_opacity,
        name="Image overlay",
        cross_origin=False,
    ).add_to(m)

    # --- Footprint polygon ---
    # folium uses (lat, lon) ordering
    fp_latlon = [(lat, lon) for lat, lon in result.footprint_latlon]
    folium.Polygon(
        locations=fp_latlon,
        color="red",
        weight=2,
        fill=False,
        tooltip="Image footprint",
        name="Footprint",
    ).add_to(m)

    # --- Corner labels ---
    for name, (lat, lon) in result.corners_latlon.items():
        folium.CircleMarker(
            location=[lat, lon],
            radius=4,
            color="red",
            fill=True,
            fill_color="white",
            tooltip=f"{name}: {lat:.6f}°, {lon:.6f}°",
        ).add_to(m)

    # --- Camera position marker ---
    folium.Marker(
        location=[lat_c, lon_c],
        tooltip=(
            f"Camera: {lat_c:.6f}°N, {lon_c:.6f}°\n"
            f"Alt: {result.flying_height_m:.1f} m AGL\n"
            f"GSD: {result.gsd_m:.3f} m/px"
        ),
        icon=folium.Icon(color="blue", icon="camera", prefix="fa"),
    ).add_to(m)

    folium.LayerControl().add_to(m)

    if output_path:
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        m.save(str(output_path))

    return m
