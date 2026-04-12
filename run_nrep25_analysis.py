#!/usr/bin/env python3
"""NREP25 single-image analysis pipeline.

Runs on:  data/2025-07-22_NREP25_Ascend SAR Pass 7-41_UTM-07-41-06_P0066781.jpg

Steps
-----
1. Read metadata + build Phase One camera model
2. Direct georectification (corners only — fast, no full pixel grid)
3. Ice detection at 25% scale (Otsu threshold + morphological cleanup)
4. Vector export  → output/NREP25_ice_polygons.gpkg
5. Ice statistics → printed report
6. Ship route     → output/NREP25_ship_route.gpkg
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

# Suppress PIL 100 MP decompression warning for Phase One iXM-100
import PIL.Image
PIL.Image.MAX_IMAGE_PIXELS = None

import cv2
import numpy as np

# Ensure src/ is importable when run from project root
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
MIN_BLOB_AREA_PX = 50 # minimum blob at processing scale
MIN_ICE_AREA_M2 = 500 # minimum georeferenced area to include in outputs (m²)

# Ship parameters
SHIP_BEAM_M   = 15.0
SHIP_LEN_M    = 90.0
SHIP_SPEED_KN = 3.0
MIN_FLOE_M2   = 500.0  # ignore floes smaller than this for routing
ROUTING_SCALE = 0.05   # A* grid scale (5% of full-res → ~583×438 px ≈ 1.8 m/px)

# ---------------------------------------------------------------------------
# 1. Metadata + camera
# ---------------------------------------------------------------------------

print("=" * 60)
print("NREP25 Single-Image Analysis Pipeline")
print("=" * 60)

t0 = time.time()
print(f"\n[1/6] Reading metadata from {IMAGE_PATH.name}...")
meta = read_metadata(IMAGE_PATH)
print(f"      GPS:       {meta.latitude:.6f}°N  {meta.longitude:.6f}°E")
print(f"      Altitude:  {meta.altitude_abs_m:.1f} m AMSL")
print(f"      Yaw:       {meta.gimbal_yaw:.2f}°T")
print(f"      Pitch:     {meta.gimbal_pitch:.3f}°")
print(f"      Roll:      {meta.gimbal_roll:.3f}°")

# Phase One iXM-100, 35 mm lens, sensor 43.9 × 32.9 mm, 11664 × 8750 px
camera = from_sensor_spec(35.0, 43.9, 32.9, 11664, 8750)
print(f"      Camera:    {camera.label}")
print(f"      fx={camera.fx:.1f} px  fy={camera.fy:.1f} px")

# ---------------------------------------------------------------------------
# 2. Georectification (corners only)
# ---------------------------------------------------------------------------

print("\n[2/6] Georectifying (corners only)...")
result = georectify(meta, camera, surface_altitude_m=0.0, full_grid=False)
print(f"      Flying height: {result.flying_height_m:.1f} m")
print(f"      GSD:           {result.gsd_m:.4f} m/px  ({result.gsd_m*100:.1f} cm/px)")
tl = result.corners_latlon["TL"]
br = result.corners_latlon["BR"]
print(f"      Footprint TL:  {tl[0]:.6f}°N  {tl[1]:.6f}°E")
print(f"      Footprint BR:  {br[0]:.6f}°N  {br[1]:.6f}°E")

# Approximate footprint size
import math
_R = 6_371_000.0
dlat_m = abs(tl[0] - br[0]) * math.pi / 180 * _R
dlon_m = abs(tl[1] - br[1]) * math.pi / 180 * _R * math.cos(math.radians(meta.latitude))
print(f"      Footprint:     {dlat_m:.0f} m × {dlon_m:.0f} m")

# ---------------------------------------------------------------------------
# 3. Ice detection
# ---------------------------------------------------------------------------

print(f"\n[3/6] Loading image and detecting ice at {SCALE:.0%} scale...")
t_det = time.time()
img_full = np.array(PIL.Image.open(IMAGE_PATH).convert("L"))
gray = cv2.resize(img_full, (0, 0), fx=SCALE, fy=SCALE)
H_proc, W_proc = gray.shape
print(f"      Processing grid: {W_proc} × {H_proc} px")

detector = IceDetector(
    min_area_px=MIN_BLOB_AREA_PX,
    morph_close=3,
    morph_open=2,
    kernel_size=7,
)
blobs, mask = detector.detect(gray, frame_idx=0)
gsd_proc = result.gsd_m / SCALE
ice_px = int((mask > 0).sum())
ice_frac = ice_px / (H_proc * W_proc)
print(f"      Detection time: {time.time()-t_det:.1f} s")
print(f"      Raw blobs detected: {len(blobs)}")
print(f"      Ice coverage (raw): {ice_frac*100:.1f}%  ({ice_px:,} px)")
print(f"      GSD at proc scale:  {gsd_proc:.2f} m/px")

# ---------------------------------------------------------------------------
# 4. Vector export
# ---------------------------------------------------------------------------

print(f"\n[4/6] Exporting ice polygons (min_area={MIN_ICE_AREA_M2:.0f} m²)...")
t_vec = time.time()
gdf = ice_mask_to_polygons(mask, result, min_area_m2=MIN_ICE_AREA_M2)
vec_path = OUTPUT_DIR / "NREP25_ice_polygons.gpkg"
to_geopackage(gdf, vec_path, layer="ice_polygons")
print(f"      Polygons exported: {len(gdf)}")
print(f"      Written: {vec_path}  ({vec_path.stat().st_size/1024:.1f} KB)")
print(f"      Export time: {time.time()-t_vec:.1f} s")

# ---------------------------------------------------------------------------
# 5. Ice statistics
# ---------------------------------------------------------------------------

print(f"\n[5/6] Computing ice statistics...")
blob_stats, scene = compute_ice_stats(mask, result, min_area_m2=MIN_ICE_AREA_M2)

print()
print("  ── Scene Statistics ─────────────────────────────────")
print(f"     Total ice area:     {scene.total_ice_area_m2:>12,.0f} m²"
      f"  ({scene.total_ice_area_m2/1e6:.4f} km²)")
print(f"     Scene area:         {H_proc*W_proc*gsd_proc**2:>12,.0f} m²"
      f"  ({H_proc*W_proc*gsd_proc**2/1e6:.4f} km²)")
print(f"     Ice fraction:       {scene.ice_fraction*100:>11.2f} %")
print(f"     Blob count:         {scene.blob_count:>12d}")
if scene.blob_count > 0:
    print(f"     Largest blob:       {scene.max_blob_area_m2:>12,.0f} m²")
    print(f"     Smallest blob:      {scene.min_blob_area_m2:>12,.0f} m²")
    print(f"     Mean blob area:     {scene.mean_blob_area_m2:>12,.0f} m²")
print()

if blob_stats:
    print("  ── Top 10 Blobs (by area) ───────────────────────────")
    print(f"  {'ID':>4}  {'Area (m²)':>12}  {'Major (m)':>10}  {'Minor (m)':>10}  "
          f"{'Orient (°)':>11}  {'Centroid lat':>13}  {'Centroid lon':>13}")
    for b in blob_stats[:10]:
        maj = f"{b.major_axis_m:>10.1f}" if not math.isnan(b.major_axis_m) else f"{'N/A':>10}"
        mi  = f"{b.minor_axis_m:>10.1f}" if not math.isnan(b.minor_axis_m) else f"{'N/A':>10}"
        ori = f"{b.orientation_deg:>11.1f}" if not math.isnan(b.orientation_deg) else f"{'N/A':>11}"
        print(f"  {b.blob_id:>4}  {b.area_m2:>12,.0f}  {maj}  {mi}  "
              f"{ori}  {b.centroid_lat:>13.6f}  {b.centroid_lon:>13.6f}")
    if len(blob_stats) > 10:
        print(f"  ... ({len(blob_stats)-10} more blobs)")
print()

# ---------------------------------------------------------------------------
# 6. Ship route
# ---------------------------------------------------------------------------

print(f"\n[6/6] Planning ship route (beam={SHIP_BEAM_M:.0f} m, speed={SHIP_SPEED_KN:.1f} kn)...")
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

if route.waypoints_latlon:
    route_path = OUTPUT_DIR / "NREP25_ship_route.gpkg"
    route_to_geopackage(route, route_path)
    print(f"      Waypoints:           {len(route.waypoints_latlon)}")
    print(f"      Total distance:      {route.total_distance_m:,.0f} m  "
          f"({route.total_distance_m/1000:.2f} km)")
    eta_min = route.eta_s / 60
    print(f"      ETA at {SHIP_SPEED_KN:.1f} kn:       {eta_min:.1f} min  "
          f"({route.eta_s:.0f} s)")
    print(f"      Written: {route_path}  ({route_path.stat().st_size/1024:.1f} KB)")
    print()
    print("  ── Waypoints (lat, lon) ─────────────────────────────")
    step = max(1, len(route.waypoints_latlon) // 10)
    for i in range(0, len(route.waypoints_latlon), step):
        lat, lon = route.waypoints_latlon[i]
        print(f"      WP {i:>3d}: {lat:.6f}°N  {lon:.6f}°E")
    last = route.waypoints_latlon[-1]
    print(f"      WP {len(route.waypoints_latlon)-1:>3d}: {last[0]:.6f}°N  {last[1]:.6f}°E  (end)")
else:
    print("      No navigable route found — scene may be fully ice-covered")
    print("      Try increasing min_floe_area_m2 or decreasing ship_beam_m")

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
