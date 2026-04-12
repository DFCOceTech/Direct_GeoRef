#!/usr/bin/env python3
"""NREP25 single-image analysis pipeline.

Runs on:  data/2025-07-22_NREP25_Ascend SAR Pass 7-41_UTM-07-41-06_P0066781.jpg

Steps
-----
1. Read metadata + build Phase One camera model
2. Direct georectification (corners only)
3. Ice detection at 25% scale — ALL floes detected
4. Vector export  → output/NREP25_ice_polygons.gpkg
     · size_class : "obstacle" (≥ AREA_THRESHOLD_M2) or "small_floe" (< threshold)
     · collision_risk : True for small floes inside the planned ship corridor
5. Ice statistics → printed report for all blobs + breakdown by size class
6. Ship route     → output/NREP25_ship_route.gpkg
     · Routing ignores floes below AREA_THRESHOLD_M2 (treated as open water)
     · Corridor = route LINESTRING buffered by ship_beam / 2
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import PIL.Image
PIL.Image.MAX_IMAGE_PIXELS = None  # suppress 100 MP decompression warning

import cv2
import math
import numpy as np

sys.path.insert(0, str(Path(__file__).parent / "src"))

from direct_georef.camera import from_sensor_spec
from direct_georef.georectify import georectify
from direct_georef.icestats import compute_ice_stats
from direct_georef.metadata import read_metadata
from direct_georef.routing import find_route, route_to_geopackage
from direct_georef.tracking import IceDetector
from direct_georef.vector import ice_mask_to_polygons, to_geopackage

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

IMAGE_PATH = Path("data/2025-07-22_NREP25_Ascend SAR Pass 7-41_UTM-07-41-06_P0066781.jpg")
OUTPUT_DIR = Path("output")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

SCALE = 0.25          # processing scale for ice detection (25% → ~2916×2188 px)
MIN_BLOB_AREA_PX = 10 # minimum blob at processing scale
MIN_ICE_AREA_M2 = 100 # area threshold: below → "small_floe"; above → "obstacle"

# Ship parameters
SHIP_BEAM_M   = 15.0
SHIP_LEN_M    = 90.0
SHIP_SPEED_KN = 3.0
MIN_FLOE_M2   = MIN_ICE_AREA_M2  # floes below this are treated as open water in routing
ROUTING_SCALE = 0.20              # A* grid scale (~1.8 m/px for NREP25 at 841 m)

# Display colours (hex, QGIS-compatible in data-defined colour column)
COLOUR_OBSTACLE   = "#1565C0"   # dark blue  — large obstacle floe
COLOUR_SMALL      = "#78909C"   # blue-grey  — small floe, not in route corridor
COLOUR_COLLISION  = "#C62828"   # dark red   — small floe in ship corridor

# ---------------------------------------------------------------------------
# 1. Metadata + camera
# ---------------------------------------------------------------------------

print("=" * 60)
print("NREP25 Single-Image Analysis Pipeline")
print("=" * 60)

t0 = time.time()
print(f"\n[1/6] Reading metadata …")
meta = read_metadata(IMAGE_PATH)
print(f"      GPS:       {meta.latitude:.6f}°N  {meta.longitude:.6f}°E")
print(f"      Altitude:  {meta.altitude_abs_m:.1f} m AMSL")
print(f"      Yaw / Pitch / Roll: {meta.gimbal_yaw:.2f}° / {meta.gimbal_pitch:.3f}° / {meta.gimbal_roll:.3f}°")

camera = from_sensor_spec(35.0, 43.9, 32.9, 11664, 8750)
print(f"      Camera:    {camera.label}")

# ---------------------------------------------------------------------------
# 2. Georectification
# ---------------------------------------------------------------------------

print("\n[2/6] Georectifying (corners only) …")
result = georectify(meta, camera, surface_altitude_m=0.0, full_grid=False)
gsd_proc = result.gsd_m / SCALE
print(f"      Flying height: {result.flying_height_m:.1f} m  |  GSD: {result.gsd_m*100:.1f} cm/px")
print(f"      Footprint TL:  {result.corners_latlon['TL'][0]:.6f}°N  {result.corners_latlon['TL'][1]:.6f}°E")
print(f"      Footprint BR:  {result.corners_latlon['BR'][0]:.6f}°N  {result.corners_latlon['BR'][1]:.6f}°E")

# ---------------------------------------------------------------------------
# 3. Ice detection — ALL floes
# ---------------------------------------------------------------------------

print(f"\n[3/6] Detecting ice at {SCALE:.0%} scale (min_blob={MIN_BLOB_AREA_PX} px) …")
t_det = time.time()
img_full = np.array(PIL.Image.open(IMAGE_PATH).convert("L"))
gray = cv2.resize(img_full, (0, 0), fx=SCALE, fy=SCALE)
H_proc, W_proc = gray.shape

detector = IceDetector(
    min_area_px=MIN_BLOB_AREA_PX,
    morph_close=3,
    morph_open=2,
    kernel_size=7,
)
blobs, mask = detector.detect(gray, frame_idx=0)
ice_px   = int((mask > 0).sum())
ice_frac = ice_px / (H_proc * W_proc)
print(f"      Grid: {W_proc}×{H_proc} px  |  GSD_proc: {gsd_proc:.2f} m/px")
print(f"      Raw blobs: {len(blobs)}  |  Ice coverage: {ice_frac*100:.1f}%  ({ice_px:,} px)")
print(f"      Detection time: {time.time()-t_det:.1f} s")

# ---------------------------------------------------------------------------
# 4. Vector export — ALL floes, classified and coloured
# ---------------------------------------------------------------------------

print(f"\n[4/6] Building georeferenced polygon layer (all floes) …")
t_vec = time.time()

# Export with no area filter — include every detected blob
gdf = ice_mask_to_polygons(mask, result, min_area_m2=0.0)

# Classify by size threshold
gdf["size_class"] = gdf["area_m2"].apply(
    lambda a: "obstacle" if a >= MIN_ICE_AREA_M2 else "small_floe"
)

# Placeholder collision column — filled after routing
gdf["collision_risk"] = False

# Assign base display colour
gdf["display_colour"] = gdf["size_class"].map({
    "obstacle":  COLOUR_OBSTACLE,
    "small_floe": COLOUR_SMALL,
})

# Counts per class
n_obstacle  = int((gdf["size_class"] == "obstacle").sum())
n_small     = int((gdf["size_class"] == "small_floe").sum())
print(f"      Total polygons: {len(gdf)}")
print(f"        Obstacles  (≥ {MIN_ICE_AREA_M2:.0f} m²): {n_obstacle}")
print(f"        Small floes (< {MIN_ICE_AREA_M2:.0f} m²): {n_small}")
print(f"      Export time: {time.time()-t_vec:.1f} s")

# ---------------------------------------------------------------------------
# 5. Ice statistics
# ---------------------------------------------------------------------------

print("\n[5/6] Computing ice statistics …")

# --- All blobs (no area filter) ---
all_blobs, scene_all = compute_ice_stats(mask, result, min_area_m2=0.0)

# --- Obstacle blobs only ---
obs_blobs, scene_obs = compute_ice_stats(mask, result, min_area_m2=MIN_ICE_AREA_M2)

# Scene area (m²)
scene_area_m2 = H_proc * W_proc * gsd_proc * gsd_proc

print()
print("  ── All Detected Floes ───────────────────────────────")
print(f"     Count:              {scene_all.blob_count:>8d}")
print(f"     Total area:         {scene_all.total_ice_area_m2:>12,.0f} m²  ({scene_all.total_ice_area_m2/1e6:.4f} km²)")
print(f"     Ice fraction:       {scene_all.ice_fraction*100:>10.2f} %")
print(f"     Largest floe:       {scene_all.max_blob_area_m2:>12,.0f} m²")
print(f"     Smallest floe:      {scene_all.min_blob_area_m2:>12,.1f} m²")
print(f"     Mean floe area:     {scene_all.mean_blob_area_m2:>12,.1f} m²")
print()
print(f"  ── Obstacles (≥ {MIN_ICE_AREA_M2:.0f} m²) ─────────────────────────")
print(f"     Count:              {scene_obs.blob_count:>8d}")
print(f"     Total area:         {scene_obs.total_ice_area_m2:>12,.0f} m²")
print(f"     Ice fraction:       {scene_obs.ice_fraction*100:>10.2f} %")
if scene_obs.blob_count > 0:
    print(f"     Largest:            {scene_obs.max_blob_area_m2:>12,.0f} m²")
    print(f"     Mean:               {scene_obs.mean_blob_area_m2:>12,.0f} m²")
print()
print(f"  ── Small Floes (< {MIN_ICE_AREA_M2:.0f} m²) ───────────────────────")
n_small_s  = scene_all.blob_count - scene_obs.blob_count
area_small = scene_all.total_ice_area_m2 - scene_obs.total_ice_area_m2
print(f"     Count:              {n_small_s:>8d}")
print(f"     Total area:         {area_small:>12,.0f} m²")
print(f"     Fraction of ice:    {(area_small/max(scene_all.total_ice_area_m2,1))*100:>10.2f} %")
print()

if obs_blobs:
    print("  ── Top 10 Obstacle Blobs (by area) ─────────────────")
    print(f"  {'ID':>4}  {'Area (m²)':>10}  {'Major (m)':>10}  {'Minor (m)':>9}  "
          f"{'Orient(°)':>10}  {'Lat':>11}  {'Lon':>12}")
    for b in obs_blobs[:10]:
        maj = f"{b.major_axis_m:>10.1f}" if not math.isnan(b.major_axis_m) else f"{'N/A':>10}"
        mi  = f"{b.minor_axis_m:>9.1f}"  if not math.isnan(b.minor_axis_m) else f"{'N/A':>9}"
        ori = f"{b.orientation_deg:>10.1f}" if not math.isnan(b.orientation_deg) else f"{'N/A':>10}"
        print(f"  {b.blob_id:>4}  {b.area_m2:>10,.0f}  {maj}  {mi}  "
              f"{ori}  {b.centroid_lat:>11.6f}  {b.centroid_lon:>12.6f}")
    if len(obs_blobs) > 10:
        print(f"  … ({len(obs_blobs)-10} more obstacle blobs)")
print()

# ---------------------------------------------------------------------------
# 6. Ship route
# ---------------------------------------------------------------------------

print(f"\n[6/6] Planning ship route "
      f"(beam={SHIP_BEAM_M:.0f} m, speed={SHIP_SPEED_KN:.1f} kn, "
      f"ignore floes < {MIN_FLOE_M2:.0f} m²) …")
t_route = time.time()

route = find_route(
    mask,
    result,
    ship_beam_m=SHIP_BEAM_M,
    ship_length_m=SHIP_LEN_M,
    ship_speed_kn=SHIP_SPEED_KN,
    min_floe_area_m2=MIN_FLOE_M2,
    w_dist=1.0,
    w_ice=3.0,
    routing_scale=ROUTING_SCALE,
)
print(f"      Route planning time: {time.time()-t_route:.1f} s")
print(f"      Navigable fraction:  {route.navigable_fraction*100:.1f}%")

n_collisions = 0

if route.waypoints_latlon:
    import geopandas as gpd
    from shapely.geometry import LineString

    # Build GeoDataFrames from waypoints and perform collision check in UTM 30N
    coords_lonlat = [(lon, lat) for lat, lon in route.waypoints_latlon]
    route_line_wgs84 = LineString(coords_lonlat)
    route_gdf_wgs84 = gpd.GeoDataFrame(
        [{"total_distance_m": route.total_distance_m,
          "eta_minutes": round(route.eta_s / 60.0, 1),
          "geometry": route_line_wgs84}],
        crs="EPSG:4326",
    )

    # Reproject to UTM 30N for metric buffering
    UTM = "EPSG:32630"
    route_utm  = route_gdf_wgs84.to_crs(UTM)
    floes_utm  = gdf.to_crs(UTM)

    # Ship corridor = route buffered by beam/2
    corridor = route_utm.geometry.iloc[0].buffer(SHIP_BEAM_M / 2.0)

    # Small floes whose geometry intersects the corridor
    small_idx = floes_utm[floes_utm["size_class"] == "small_floe"].index
    collision_flags = floes_utm.loc[small_idx, "geometry"].intersects(corridor)
    collision_idx   = collision_flags[collision_flags].index

    gdf.loc[collision_idx, "collision_risk"] = True
    gdf.loc[collision_idx, "display_colour"] = COLOUR_COLLISION
    n_collisions = len(collision_idx)

    print(f"      Waypoints:           {len(route.waypoints_latlon)}")
    print(f"      Total distance:      {route.total_distance_m:,.0f} m  "
          f"({route.total_distance_m/1000:.2f} km)")
    eta_min = route.eta_s / 60
    print(f"      ETA at {SHIP_SPEED_KN:.1f} kn:       {eta_min:.1f} min  ({route.eta_s:.0f} s)")
    print()

    print("  ── Waypoints (lat, lon) ─────────────────────────────")
    step = max(1, len(route.waypoints_latlon) // 10)
    for i in range(0, len(route.waypoints_latlon), step):
        lat, lon = route.waypoints_latlon[i]
        print(f"      WP {i:>3d}: {lat:.6f}°N  {lon:.6f}°E")
    if (len(route.waypoints_latlon) - 1) % step != 0:
        lat, lon = route.waypoints_latlon[-1]
        print(f"      WP {len(route.waypoints_latlon)-1:>3d}: {lat:.6f}°N  {lon:.6f}°E  (end)")

    print()
    print(f"  ── Collision Analysis ───────────────────────────────")
    print(f"     Small floes in ship corridor: {n_collisions}")
    print(f"     (corridor width = {SHIP_BEAM_M:.0f} m, centred on route)")
    if n_collisions > 0:
        coll_gdf = gdf[gdf["collision_risk"]]
        print(f"     Collision-risk areas: {coll_gdf['area_m2'].sum():,.0f} m² total")
        print(f"     Largest at-risk floe: {coll_gdf['area_m2'].max():,.1f} m²")
    print()

    # Write ship route GeoPackage (route + waypoints layers)
    from direct_georef.routing import route_to_geopackage
    route_path = OUTPUT_DIR / "NREP25_ship_route.gpkg"
    route_to_geopackage(route, route_path)
    print(f"      Written: {route_path}  ({route_path.stat().st_size/1024:.0f} KB)")

else:
    print("      No navigable route found.")

# ---------------------------------------------------------------------------
# Write ice polygon GeoPackage (all floes, classified + collision flags)
# ---------------------------------------------------------------------------

poly_path = OUTPUT_DIR / "NREP25_ice_polygons.gpkg"
to_geopackage(gdf, poly_path, layer="ice_polygons")
print(f"\n      Ice polygons → {poly_path}  ({poly_path.stat().st_size/1024:.0f} KB)")
print(f"        {n_obstacle} obstacle floes  (≥ {MIN_ICE_AREA_M2:.0f} m², blue)")
print(f"        {n_small - n_collisions} small floes, no collision  (grey)")
print(f"        {n_collisions} small floes, collision risk  (red)")
print()
print("  Columns in ice_polygons.gpkg:")
print("    image_idx, blob_id, area_m2  — standard polygon attributes")
print("    size_class    : 'obstacle' | 'small_floe'")
print("    collision_risk: True if inside ship corridor")
print("    display_colour: hex colour for QGIS data-defined symbology")
print("      → in QGIS: Layer Properties → Symbology → Single Symbol →")
print("        Fill colour → Data defined override → Field: display_colour")

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------

print()
print("=" * 60)
print(f"Total runtime: {time.time()-t0:.1f} s")
print()
print("Output files:")
for f in sorted(OUTPUT_DIR.glob("NREP25_*.gpkg")):
    print(f"  {f}  ({f.stat().st_size/1024:.0f} KB)")
print("=" * 60)
