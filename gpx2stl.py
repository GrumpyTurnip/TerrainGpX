#!/usr/bin/env python3
"""
gpx2stl - turn a GPX file into a 3D-printable terrain model.

  * Real elevation from AWS Terrain Tiles (Terrarium encoding, no API key)
  * Real satellite imagery from Esri World Imagery (no API key) used to pick colours
  * Watertight, solid mesh with a flat base
  * Output: STL (optionally with per-triangle colour), 3MF with colours,
    and/or one STL per colour for multi-material printing (--split)

Usage:
    python gpx2stl.py route.gpx -o model.stl --colours 5 --3mf --split
"""
import argparse
import hashlib
import io
import math
import struct
import sys
import threading
import time
import zipfile
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import json
import re
from urllib.parse import urlencode

from PIL import Image, ImageChops, ImageDraw, ImageFilter

TILE = 256
TERRARIUM_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"
ESRI_URL = ("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/"
            "MapServer/tile/{z}/{y}/{x}")
OVERPASS_URLS = ["https://overpass-api.de/api/interpreter",
                 "https://overpass.kumi.systems/api/interpreter",
                 "https://overpass.private.coffee/api/interpreter"]
USER_AGENT = "gpx2stl/1.0 (personal terrain model generator)"


# --------------------------------------------------------------------------- GPX
def read_gpx(path):
    """Return (list of tracks/routes as [(lat, lon), ...], list of waypoints)."""
    root = ET.parse(path).getroot()
    local = lambda t: t.rsplit("}", 1)[-1]
    tracks, waypoints = [], []
    for el in root.iter():
        name = local(el.tag)
        if name in ("trkseg", "rte"):
            pts = [(float(p.get("lat")), float(p.get("lon")))
                   for p in el if local(p.tag) in ("trkpt", "rtept")]
            if pts:
                tracks.append(pts)
        elif name == "wpt":
            waypoints.append((float(el.get("lat")), float(el.get("lon"))))
    if not tracks and not waypoints:
        sys.exit("No track, route or waypoints found in the GPX file.")
    return tracks, waypoints


def make_bbox(points, pad, square, min_span_m):
    """Padded bounding box (lat_min, lat_max, lon_min, lon_max)."""
    lats = [p[0] for p in points]
    lons = [p[1] for p in points]
    clat, clon = (min(lats) + max(lats)) / 2, (min(lons) + max(lons)) / 2
    m_lat = 111320.0
    m_lon = 111320.0 * math.cos(math.radians(clat))
    h = (max(lats) - min(lats)) * m_lat
    w = (max(lons) - min(lons)) * m_lon
    w = max(w * (1 + 2 * pad), min_span_m)
    h = max(h * (1 + 2 * pad), min_span_m)
    if square:
        w = h = max(w, h)
    dlat, dlon = h / 2 / m_lat, w / 2 / m_lon
    return (max(clat - dlat, -85.0), min(clat + dlat, 85.0), clon - dlon, clon + dlon)


# ------------------------------------------------------------------ web mercator
def to_px(lon, lat, z):
    n = TILE * 2 ** z
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def bbox_px(bbox, z):
    lat0, lat1, lon0, lon1 = bbox
    x0, y0 = to_px(lon0, lat1, z)
    x1, y1 = to_px(lon1, lat0, z)
    return x0, y0, x1, y1


def tile_range(bbox, z):
    x0, y0, x1, y1 = bbox_px(bbox, z)
    mx = 2 ** z - 1
    return (max(int(x0 // TILE), 0), min(int((x1 - 1e-9) // TILE), mx),
            max(int(y0 // TILE), 0), min(int((y1 - 1e-9) // TILE), mx))


def tile_count(bbox, z):
    tx0, tx1, ty0, ty1 = tile_range(bbox, z)
    return (tx1 - tx0 + 1) * (ty1 - ty0 + 1)


def pick_zoom(bbox, target_px, zmax, max_tiles):
    best = 0
    for z in range(0, zmax + 1):
        if tile_count(bbox, z) > max_tiles:
            break
        best = z
        x0, y0, x1, y1 = bbox_px(bbox, z)
        if max(x1 - x0, y1 - y0) >= target_px:
            break
    return best



# ------------------------------------------- framing: shape, ratio and rotation of the model area
R_EARTH = 6378137.0
HALF_WORLD = math.pi * R_EARTH
HEX_REGULAR = 2.0 / math.sqrt(3.0)          # width : height of a regular flat-topped hexagon
SHAPES = ("rectangle", "hexagon", "circle")
CORNER_STYLES = ("sharp", "fillet", "chamfer")


def to_merc(lat, lon):
    return R_EARTH * math.radians(lon), R_EARTH * math.asinh(math.tan(math.radians(lat)))


def from_merc(x, y):
    return math.degrees(math.atan(math.sinh(y / R_EARTH))), math.degrees(x / R_EARTH)


def parse_ratio(txt, portrait=False):
    """'auto' -> None; '3:2' or '1.5' -> width / height (portrait swaps it)."""
    t = "auto" if txt is None else str(txt).strip().lower()
    if t in ("", "auto", "fit"):
        return None
    try:
        if ":" in t:
            a, b = t.split(":", 1)
            r = float(a) / float(b)
        else:
            r = float(t)
    except (ValueError, ZeroDivisionError):
        sys.exit(f"Bad --ratio '{txt}'. Use auto, or width:height such as 3:2.")
    if not 0.2 <= r <= 5:
        sys.exit("--ratio must be between 1:5 and 5:1.")
    return 1.0 / r if portrait else r


def convex_hull(P):
    """Andrew's monotone chain; P is (n, 2)."""
    P = np.unique(np.round(P, 3), axis=0)
    if len(P) <= 2:
        return P

    def half(points):
        h = []
        for p in points:
            while len(h) >= 2 and ((h[-1][0] - h[-2][0]) * (p[1] - h[-2][1])
                                   - (h[-1][1] - h[-2][1]) * (p[0] - h[-2][0])) <= 0:
                h.pop()
            h.append(p)
        return h
    return np.array(half(P)[:-1] + half(P[::-1])[:-1])


def _fit_extent(U, V, shape, ratio):
    """Tight half-sizes of the shape around the points (U, V), centred on their bounding box."""
    uc, vc = (U.min() + U.max()) / 2, (V.min() + V.max()) / 2
    x, y = np.abs(U - uc), np.abs(V - vc)
    if shape == "hexagon":                       # flat-topped: |y| <= b and |x|/a + |y|/(2b) <= 1, a = r*b
        r = ratio or HEX_REGULAR
        b = float(np.maximum(y, x / r + y / 2).max())
        a = r * b
    elif shape == "circle":                      # ellipse (a circle when ratio is 1)
        r = ratio or 1.0
        b = float(np.sqrt((x / r) ** 2 + y ** 2).max())
        a = r * b
    elif ratio is None:
        a, b = float(x.max()), float(y.max())
    else:
        b = float(max(y.max(), x.max() / ratio))
        a = ratio * b
    return uc, vc, 2 * a, 2 * b


def fit_frame(pm, theta, shape, ratio, pad, min_span):
    """pm: (n, 2) mercator metres. The model's top edge faces bearing `theta` (deg clockwise from north).
    Returns (xc, yc, W, H) in mercator metres."""
    t = math.radians(theta)
    sn, cs = math.sin(t), math.cos(t)
    U = pm[:, 0] * cs - pm[:, 1] * sn            # right = (cos, -sin)
    V = pm[:, 0] * sn + pm[:, 1] * cs            # up    = (sin,  cos)
    uc, vc, W, H = _fit_extent(U, V, shape, ratio)
    f = 1 + 2 * pad
    if shape == "rectangle" and ratio is None:
        W, H = max(W * f, min_span), max(H * f, min_span)
    else:
        W, H = W * f, H * f
        m = min(W, H)
        if m < min_span:
            if m <= 1e-9:
                r = ratio or (HEX_REGULAR if shape == "hexagon" else 1.0)
                H, W = (min_span, r * min_span) if r >= 1 else (min_span / r, min_span)
            else:
                W, H = W * min_span / m, H * min_span / m
    return uc * cs + vc * sn, -uc * sn + vc * cs, W, H


def best_theta(pm, shape, ratio, pad, min_span):
    """Rotation (whole degrees, -90..89) giving the smallest model area; ties go to the smallest angle."""
    hull = convex_hull(pm)
    areas = np.array([np.prod(fit_frame(hull, th, shape, ratio, pad, min_span)[2:]) for th in range(-90, 90)])
    ok = np.nonzero(areas <= areas.min() * 1.005)[0]
    return float(min((abs(-90 + int(i)), -90 + int(i)) for i in ok)[1])


class Frame:
    """The model area on the map: centre, size and rotation (mercator metres)."""

    def __init__(self, xc, yc, W, H, theta):
        self.xc, self.yc, self.W, self.H, self.theta = xc, yc, W, H, theta
        t = math.radians(theta)
        self.sn, self.cs = math.sin(t), math.cos(t)
        self.lat_c = from_merc(xc, yc)[0]
        self.k = math.cos(math.radians(self.lat_c))       # ground metres per mercator metre

    @property
    def W_g(self):
        return self.W * self.k

    @property
    def H_g(self):
        return self.H * self.k

    def xy(self, u, v):
        """Mercator position of the point u (right) and v (up) mercator metres from the centre."""
        return self.xc + u * self.cs + v * self.sn, self.yc - u * self.sn + v * self.cs

    def aabb(self, margin=0.01):
        ll = [from_merc(*self.xy(su * self.W / 2, sv * self.H / 2)) for su in (-1, 1) for sv in (-1, 1)]
        la, lo = [p[0] for p in ll], [p[1] for p in ll]
        dla, dlo = (max(la) - min(la)) * margin, (max(lo) - min(lo)) * margin
        return (max(min(la) - dla, -85.0), min(max(la) + dla, 85.0), min(lo) - dlo, max(lo) + dlo)

    def cells(self, ring, nxc, nyc):
        """(lat, lon) points -> (x, y) in grid-cell units (x right, y down, origin at the top-left corner)."""
        out = []
        for lat, lon in ring:
            x, y = to_merc(lat, lon)
            dx, dy = x - self.xc, y - self.yc
            u, v = dx * self.cs - dy * self.sn, dx * self.sn + dy * self.cs
            out.append(((u / self.W + 0.5) * nxc, (0.5 - v / self.H) * nyc))
        return out

    def coeffs(self, z, nxc, nyc, origin, vertices):
        """PIL affine coefficients mapping output pixels to canvas pixels at zoom z.
        vertices=True samples the (nxc+1) x (nyc+1) mesh vertices; False samples the nxc x nyc cell centres."""
        mpp = 2 * HALF_WORLD / (TILE * 2 ** z)
        su, sv = self.W / nxc, self.H / nyc
        x0, y0 = self.xy(-self.W / 2, self.H / 2)
        px0, py0 = (x0 + HALF_WORLD) / mpp - origin[0], (HALF_WORLD - y0) / mpp - origin[1]
        a, b = su * self.cs / mpp, -sv * self.sn / mpp
        d, e = su * self.sn / mpp, sv * self.cs / mpp
        if vertices:
            return (a, b, px0 - 0.5 * (a + b), d, e, py0 - 0.5 * (d + e)), mpp
        return (a, b, px0, d, e, py0), mpp


def pick_zoom_frame(frame, aabb, target_px, zmax, max_tiles):
    """Lowest zoom that gives target_px across the frame's longest side, within the tile budget."""
    best = 0
    for z in range(0, zmax + 1):
        if tile_count(aabb, z) > max_tiles:
            break
        best = z
        if max(frame.W, frame.H) / (2 * HALF_WORLD / (TILE * 2 ** z)) >= target_px:
            break
    return best


def build_layout(points, o, need_mask=True):
    """Everything about the model footprint that needs no map data. `o` carries: shape, ratio, portrait,
    square, rotate, pad, min_span, resolution, size, corner_style, corner_size."""
    shape = o.shape
    if shape not in SHAPES:
        sys.exit(f"--shape must be one of: {', '.join(SHAPES)}")
    ratio = 1.0 if getattr(o, "square", False) else parse_ratio(o.ratio, getattr(o, "portrait", False))
    rot = str(o.rotate).strip().lower()
    centre = getattr(o, "centre", None)
    auto = rot == "auto" and not centre
    if centre:
        # fixed position: the area is set by centre + width, not fitted to the route (for neighbouring tiles)
        try:
            theta = 0.0 if rot == "auto" else float(rot)
        except ValueError:
            sys.exit(f"Bad --rotate '{o.rotate}'. Use a number of degrees.")
        theta = (theta + 180.0) % 360.0 - 180.0
        xc, yc = to_merc(*centre)
        W = o.width_m / math.cos(math.radians(centre[0]))
        H = W / (ratio or (HEX_REGULAR if shape == "hexagon" else 1.0))
    else:
        pm = np.array([to_merc(la, lo) for la, lo in points], float)
        k0 = math.cos(math.radians(float(np.mean([p[0] for p in points]))))
        min_span = o.min_span / k0
        if auto:
            theta = best_theta(pm, shape, ratio, o.pad, min_span)
        else:
            try:
                theta = float(rot)
            except ValueError:
                sys.exit(f"Bad --rotate '{o.rotate}'. Use a number of degrees, or auto.")
            theta = (theta + 180.0) % 360.0 - 180.0
        xc, yc, W, H = fit_frame(pm, theta, shape, ratio, o.pad, min_span)
    fr = Frame(xc, yc, W, H, theta)
    if W >= H:
        nxc, nyc = o.resolution, max(2, int(round(o.resolution * H / W)))
    else:
        nyc, nxc = o.resolution, max(2, int(round(o.resolution * W / H)))
    cell_mm = o.size / max(nxc, nyc)
    style = o.corner_style if (o.corner_size > 0 and shape != "circle") else "sharp"
    poly = shape_polygon(shape, nxc * cell_mm, nyc * cell_mm, style, o.corner_size)
    mask = footprint_mask(shape, style, poly, nxc, nyc, cell_mm) if need_mask else None
    from types import SimpleNamespace
    return SimpleNamespace(frame=fr, theta=theta, auto=auto, nxc=nxc, nyc=nyc, cell_mm=cell_mm,
                           W_mm=nxc * cell_mm, H_mm=nyc * cell_mm, poly=poly, mask=mask, style=style)


def neighbours(fr, shape):
    """Centres of the tiles that sit next to this one (same size, shape and rotation).
    Directions are relative to the model (its top edge), with the compass bearing too."""
    W, H, k = fr.W_g, fr.H_g, fr.k
    if shape == "hexagon":
        offs = [("Top", 0, H), ("Upper right", .75 * W, H / 2), ("Lower right", .75 * W, -H / 2),
                ("Bottom", 0, -H), ("Lower left", -.75 * W, -H / 2), ("Upper left", -.75 * W, H / 2)]
    elif shape == "circle":
        d = math.sqrt(0.5)
        offs = [("Top", 0, H), ("Top right", d * W, d * H), ("Right", W, 0), ("Bottom right", d * W, -d * H),
                ("Bottom", 0, -H), ("Bottom left", -d * W, -d * H), ("Left", -W, 0), ("Top left", -d * W, d * H)]
    else:
        offs = [("Top", 0, H), ("Top right", W, H), ("Right", W, 0), ("Bottom right", W, -H),
                ("Bottom", 0, -H), ("Bottom left", -W, -H), ("Left", -W, 0), ("Top left", -W, H)]
    out = []
    for name, du, dv in offs:
        lat, lon = from_merc(*fr.xy(du / k, dv / k))
        out.append(dict(name=name, lat=round(lat, 6), lon=round(lon, 6),
                        bearing=round((fr.theta + math.degrees(math.atan2(du, dv))) % 360, 1)))
    return out


# Colour schemes: terrain clusters are ordered by their average height and painted along the ramp (low -> high),
# so the satellite imagery still decides WHERE the colour boundaries go. The other entries are matching
# defaults for water, roads etc. that the web page applies when you pick a scheme.
SCHEMES = {
    "satellite": dict(label="Satellite (true colour)", ramp=None, water="#3a78c2", river="#3a78c2",
                      road="#8a8a8a", path="#a0764a", building="#d6cfc4", route="#e8202a"),
    "topo": dict(label="Topographic", ramp=["#4f7942", "#8fae5b", "#d8d17a", "#c9a66b", "#a07850", "#ece6da"],
                 water="#4a86c5", river="#4a86c5", road="#5c5c5c", path="#7a4e2d", building="#f2ede4", route="#d62828"),
    "alpine": dict(label="Alpine", ramp=["#2f5d50", "#5f8d6b", "#a3b18a", "#8d8d86", "#c9c9c4", "#f4f4f0"],
                   water="#5aa0c8", river="#5aa0c8", road="#4a4a4a", path="#6b5b45", building="#e9e2d6", route="#e63946"),
    "desert": dict(label="Desert", ramp=["#7a4a2b", "#b5683c", "#d98c4f", "#e8b27a", "#f1d3a1", "#f8ebd0"],
                   water="#2e8ba6", river="#2e8ba6", road="#5b3a29", path="#8c5a3a", building="#fff4dc", route="#1d3557"),
    "autumn": dict(label="Autumn", ramp=["#3d4f2f", "#7a8b3a", "#c9a227", "#d9772b", "#a63d2a", "#f3e3c3"],
                   water="#3d7ea6", river="#3d7ea6", road="#4b3b30", path="#6e4a2e", building="#f6ecd6", route="#1b4332"),
    "forest": dict(label="Forest", ramp=["#1b3a2a", "#2d6a4f", "#52b788", "#95d5b2", "#d8f3dc"],
                   water="#3a86a8", river="#3a86a8", road="#3c3c3c", path="#7b5e3b", building="#f1ead8", route="#ff6b35"),
    "sunset": dict(label="Sunset", ramp=["#2b1b3f", "#6a2c70", "#b83b5e", "#f08a5d", "#f9d56e", "#fff3d1"],
                   water="#3c5a99", river="#3c5a99", road="#2b2b2b", path="#6a4a3a", building="#fff3d1", route="#00c2a8"),
    "blueprint": dict(label="Blueprint", ramp=["#0b2545", "#13315c", "#1f5a8a", "#3b86b8", "#8ecae6", "#e0f4ff"],
                      water="#ffd166", river="#ffd166", road="#e0f4ff", path="#8ecae6", building="#ffffff", route="#ff595e"),
    "clay": dict(label="Clay (greys)", ramp=["#3a3a3a", "#6b6b6b", "#9a9a9a", "#c4c4c4", "#ececec"],
                 water="#5b7fa6", river="#5b7fa6", road="#2b2b2b", path="#7a7a7a", building="#ffffff", route="#d62828"),
}


def apply_scheme(idx, palette, elev_cells, name):
    """Recolour terrain clusters along the scheme ramp, lowest-average-height cluster first."""
    ramp = SCHEMES[name]["ramp"]
    if not ramp:
        return palette
    k = len(palette)
    mean = np.array([elev_cells[idx == i].mean() if (idx == i).any() else np.inf for i in range(k)])
    order = np.argsort(mean, kind="stable")
    stops = np.array([parse_hex(c) for c in ramp], float)
    out = palette.copy()
    for rank, cl in enumerate(order):
        t = rank / max(1, k - 1) * (len(stops) - 1)
        i0 = min(int(t), len(stops) - 2)
        f = t - i0
        out[cl] = np.clip(stops[i0] * (1 - f) + stops[i0 + 1] * f, 0, 255).round().astype(np.uint8)
    return out


PROGRESS = False


def progress(pct, msg):
    """Machine-readable progress line for the web page (only with --progress)."""
    if PROGRESS:
        print(f"@@PROGRESS {pct:.0f} {msg}", flush=True)


# ------------------------------------------------------------------ tile fetching
def fetch_tile(url, cache_dir, session, retries=4):
    cache = Path(cache_dir) / (hashlib.sha1(url.encode()).hexdigest() + ".bin")
    if cache.exists():
        return cache.read_bytes()
    last = None
    for attempt in range(retries):
        try:
            r = session.get(url, timeout=30)
            if r.status_code == 200:
                cache.write_bytes(r.content)
                return r.content
            last = f"HTTP {r.status_code}"
            if r.status_code in (403, 404):      # won't get better by retrying
                break
        except Exception as e:  # network hiccup
            last = str(e)
        time.sleep(1.5 * (attempt + 1))
    raise RuntimeError(f"Could not fetch {url}: {last}")


def mosaic_raw(url_tpl, z, bbox, cache_dir, mode, reject=None, on_tile=None):
    """Stitch the tiles covering bbox. Returns (PIL canvas, (x, y) of its top-left corner in zoom-z pixels)."""
    cache_dir = Path(cache_dir)
    cache_dir.mkdir(parents=True, exist_ok=True)
    try:
        import requests
        session = requests.Session()
        session.headers["User-Agent"] = USER_AGENT
    except ImportError:
        session = None
    tx0, tx1, ty0, ty1 = tile_range(bbox, z)
    coords = [(x, y) for y in range(ty0, ty1 + 1) for x in range(tx0, tx1 + 1)]
    lock, done = threading.Lock(), [0]

    def get(c):
        b = fetch_tile(url_tpl.format(z=z, x=c[0], y=c[1]), cache_dir, session)
        if on_tile:
            with lock:
                done[0] += 1
                on_tile(done[0], len(coords))
        return b
    with ThreadPoolExecutor(max_workers=8) as ex:
        blobs = list(ex.map(get, coords))
    if reject is not None:
        reject(blobs)
    canvas = Image.new(mode, ((tx1 - tx0 + 1) * TILE, (ty1 - ty0 + 1) * TILE))
    for (x, y), blob in zip(coords, blobs):
        canvas.paste(Image.open(io.BytesIO(blob)).convert(mode), ((x - tx0) * TILE, (y - ty0) * TILE))
    return canvas, (tx0 * TILE, ty0 * TILE)


class NoDetail(Exception):
    """The imagery server has no real pictures at this zoom here (it sends 'not available' tiles)."""


def reject_placeholders(blobs, z):
    """Esri sends the same small 'Map data not yet available' picture for every uncovered tile at high zoom.
    Real aerial tiles are bigger and all different, so many identical small tiles means no coverage."""
    if z < 18 or not blobs:
        return
    from collections import Counter
    h = Counter(hashlib.sha1(b).digest() for b in blobs if len(b) < 9000)
    dup = max(h.values()) if h else 0
    small = sum(1 for b in blobs if len(b) < 3000)
    if (dup >= 2 and dup / len(blobs) > 0.10) or small / len(blobs) > 0.5:
        raise NoDetail(z)


def fetch_imagery(fr, aabb, args):
    """Best imagery up to --max-zoom; steps down a zoom whenever the server has no real detail there."""
    z = pick_zoom_frame(fr, aabb, args.resolution * args.imagery_oversample, args.max_zoom, args.max_imagery_tiles)
    while True:
        print(f"Fetching satellite imagery (zoom {z}, {tile_count(aabb, z)} tiles)...")
        try:
            canvas, org = mosaic_raw(args.imagery_url, z, aabb, args.cache_dir, "RGB",
                                     reject=lambda b, z=z: reject_placeholders(b, z),
                                     on_tile=lambda d, t: progress(20 + 25 * d / t, f"Downloading satellite imagery ({d}/{t} tiles)"))
            return canvas, org, z
        except (NoDetail, RuntimeError) as e:
            if z <= 15:
                raise
            why = "no detailed imagery here" if isinstance(e, NoDetail) else "tiles not available"
            print(f"  zoom {z}: {why}, trying zoom {z - 1}")
            z -= 1


# ------------------------------------------------------------------ LiDAR (WCS)
LIDAR_SOURCES = {
    "none": dict(label="Global elevation data (about 10-30 m)"),
    "ea-dtm-1m": dict(
        label="England: Environment Agency 1 m LiDAR terrain (DTM)",
        url="https://environment.data.gov.uk/spatialdata/lidar-composite-digital-terrain-model-dtm-1m/wcs",
        coverage="13787b9a-26a4-4775-8523-806d13af58fc__Lidar_Composite_Elevation_DTM_1m",
        match="Elevation_DTM_1m", native_m=1.0,
        bbox=(49.9, 55.85, -6.3, 2.1),                      # lat0, lat1, lon0, lon1 of the data (England)
        attribution="Contains Environment Agency information (c) Environment Agency and/or database right 2022. "
                    "Open Government Licence v3.0."),
}
_WCS_CRS = "http://www.opengis.net/def/crs/EPSG/0/3857"
_WCS_NEGOTIATED = {}


def http_get(url, session):
    r = session.get(url, timeout=180)
    return r.content, r.status_code


def lidar_in_range(key, lat, lon):
    src = LIDAR_SOURCES.get(key, {})
    if "bbox" not in src:
        return False
    la0, la1, lo0, lo1 = src["bbox"]
    return la0 <= lat <= la1 and lo0 <= lon <= lo1


def _is_tiff(b):
    return b[:4] in (b"II*\x00", b"MM\x00*", b"II+\x00", b"MM\x00+")


def wcs_fetch(url, cache_dir, session, retries=3):
    """GET a WCS URL, check it is a real TIFF (servers send XML errors), cache only good answers."""
    cache = Path(cache_dir) / ("wcs_" + hashlib.sha1(url.encode()).hexdigest() + ".tif")
    if cache.exists():
        return cache.read_bytes()
    last = None
    for attempt in range(retries):
        try:
            blob, status = http_get(url, session)
            if status == 200 and _is_tiff(blob):
                cache.write_bytes(blob)
                return blob
            text = re.sub(r"\s+", " ", re.sub(r"<[^>]+>", " ", blob[:1500].decode("utf-8", "replace"))).strip()[:220]
            last = f"HTTP {status}: {text or 'not a TIFF image'}"
            if 400 <= status < 500:
                break
        except Exception as e:
            last = f"could not connect to the LiDAR service ({type(e).__name__}); check your internet connection"
        time.sleep(1.5 * (attempt + 1))
    raise RuntimeError(last)


def wcs_get_url(src, coverage, sub, scale, x0, y0, x1, y1, w, h):
    q = [("service", "WCS"), ("version", "2.0.1"), ("request", "GetCoverage"), ("coverageId", coverage),
         ("format", "image/tiff"),
         ("subset", f"{sub[0]}({x0:.3f},{x1:.3f})"), ("subset", f"{sub[1]}({y0:.3f},{y1:.3f})"),
         ("subsettingCrs", _WCS_CRS), ("outputCrs", _WCS_CRS),
         ("scaleSize", f"{scale[0]}({w}),{scale[1]}({h})")]
    return src["url"] + "?" + urlencode(q, safe="(),:/")


def _read_tiff(blob):
    img = Image.open(io.BytesIO(blob))
    a = np.asarray(img, dtype=np.float32)
    return a[..., 0] if a.ndim == 3 else a


def wcs_session():
    try:
        import requests
        s = requests.Session()
        s.headers["User-Agent"] = USER_AGENT
        return s
    except ImportError:
        return None


def wcs_negotiate(key, xc, yc, m, cache_dir, session):
    """Find the axis labels this server wants for a small request (servers differ: X/Y, x/y, E/N ...).
    Returns (coverage id, subset labels, scale labels). Raises RuntimeError with the server's own message."""
    src = LIDAR_SOURCES[key]
    if key in _WCS_NEGOTIATED:
        return _WCS_NEGOTIATED[key]
    covs = [src["coverage"]]
    err = "no answer"
    for attempt in range(2):
        for cov in covs:
            for sub in (("X", "Y"), ("x", "y"), ("E", "N"), ("Long", "Lat")):
                for sc in (("i", "j"), sub):
                    url = wcs_get_url(src, cov, sub, sc, xc, yc, xc + 8 * m, yc + 8 * m, 8, 8)
                    try:
                        a = _read_tiff(wcs_fetch(url, cache_dir, session, retries=1))
                        if a.shape == (8, 8):
                            _WCS_NEGOTIATED[key] = (cov, sub, sc)
                            return _WCS_NEGOTIATED[key]
                        err = f"the server ignored the requested size (got {a.shape[1]}x{a.shape[0]})"
                    except RuntimeError as e:
                        err = str(e)
                    except Exception as e:
                        err = f"could not read the answer: {e}"
        if attempt == 0:                      # the coverage id may have changed: look it up
            try:
                blob, st = http_get(src["url"] + "?service=WCS&request=GetCapabilities&version=2.0.1", session)
                found = [c for c in re.findall(r"CoverageId>\s*([^<\s]+)\s*<", blob.decode("utf-8", "replace"))
                         if src["match"] in c]
                covs = [c for c in found if c != src["coverage"]]
            except Exception:
                covs = []
            if not covs:
                break
    raise RuntimeError(err)


def fetch_lidar(key, fr, args, on_block=None):
    """Download LiDAR heights covering the frame as one web-mercator raster at a chosen pixel size.
    Returns (float32 canvas with NaN where there is no data, zoom, origin) in the same convention as the tile mosaics,
    so it can go straight through warp()."""
    src = LIDAR_SOURCES[key]
    corners = [fr.xy(su * fr.W / 2, sv * fr.H / 2) for su in (-1, 1) for sv in (-1, 1)]
    xmin, xmax = min(c[0] for c in corners), max(c[0] for c in corners)
    ymin, ymax = min(c[1] for c in corners), max(c[1] for c in corners)
    cell_g = max(fr.W_g, fr.H_g) / max(args._nxc, args._nyc)
    m = min(max(src["native_m"], cell_g / 3), 8.0) / fr.k              # raster pixel, mercator metres
    n = ((xmax - xmin) / m + 6) * ((ymax - ymin) / m + 6)
    if n > args.lidar_max_px:
        m *= math.sqrt(n / args.lidar_max_px)
    x0, ytop = math.floor((xmin - 2 * m) / m) * m, math.ceil((ymax + 2 * m) / m) * m
    nx, ny = math.ceil((xmax + 2 * m - x0) / m), math.ceil((ytop - (ymin - 2 * m)) / m)
    session = wcs_session()
    cache_dir = Path(args.cache_dir)
    cache_dir.mkdir(parents=True, exist_ok=True)
    cov, sub, sc = wcs_negotiate(key, (xmin + xmax) / 2, (ymin + ymax) / 2, m, cache_dir, session)
    BL = 1024
    blocks = [(bx, by, min(BL, nx - bx), min(BL, ny - by)) for by in range(0, ny, BL) for bx in range(0, nx, BL)]
    canvas = np.full((ny, nx), np.nan, np.float32)
    lock, done = threading.Lock(), [0]

    def get(blk):
        bx, by, bw, bh = blk
        url = wcs_get_url(src, cov, sub, sc, x0 + bx * m, ytop - (by + bh) * m, x0 + (bx + bw) * m, ytop - by * m, bw, bh)
        a = _read_tiff(wcs_fetch(url, cache_dir, session))
        if a.shape != (bh, bw):
            raise RuntimeError(f"the server returned {a.shape[1]}x{a.shape[0]} pixels instead of {bw}x{bh}")
        a = np.where(np.isfinite(a) & (a > -500) & (a < 4500), a, np.nan).astype(np.float32)
        canvas[by:by + bh, bx:bx + bw] = a
        if on_block:
            with lock:
                done[0] += 1
                on_block(done[0], len(blocks))
    with ThreadPoolExecutor(max_workers=4) as ex:
        list(ex.map(get, blocks))
    z = math.log2(2 * HALF_WORLD / (TILE * m))
    return canvas, z, ((x0 + HALF_WORLD) / m, (HALF_WORLD - ytop) / m), m * fr.k


def lidar_check(key, lat, lon, cache_dir):
    """Small live test for the web page / command line: is this place covered, and does the service answer as expected?"""
    if key not in LIDAR_SOURCES or key == "none":
        return dict(ok=False, message="Choose a LiDAR source first.")
    src = LIDAR_SOURCES[key]
    if not lidar_in_range(key, lat, lon):
        return dict(ok=False, message="That position is outside this LiDAR source's area (England).")
    xc, yc = to_merc(lat, lon)
    k = math.cos(math.radians(lat))
    m = 1.0 / k
    session = wcs_session()
    cache_dir = Path(cache_dir)
    cache_dir.mkdir(parents=True, exist_ok=True)
    try:
        cov, sub, sc = wcs_negotiate(key, xc, yc, m, cache_dir, session)
        a = _read_tiff(wcs_fetch(wcs_get_url(src, cov, sub, sc, xc - 50 * m, yc - 50 * m, xc + 50 * m, yc + 50 * m, 100, 100),
                                 cache_dir, session))
    except RuntimeError as e:
        return dict(ok=False, message=f"The LiDAR service did not give usable data: {e}")
    except Exception as e:
        return dict(ok=False, message=f"The LiDAR service answered in a way I could not read: {e}")
    good = np.isfinite(a) & (a > -500) & (a < 4500)
    if good.mean() < 0.05:
        return dict(ok=False, message="The service works but has no LiDAR data at this exact spot (it covers about 99% of England).")
    return dict(ok=True, covered=round(float(good.mean()), 3), height_m=round(float(np.median(a[good])), 1),
                message=f"LiDAR is available here ({good.mean():.0%} of a 100 m test patch has data, ground about "
                        f"{np.median(a[good]):.0f} m above sea level).")


def decode_terrarium(img):
    a = np.asarray(img, dtype=np.float32)
    return a[..., 0] * 256.0 + a[..., 1] + a[..., 2] / 256.0 - 32768.0


# ---------------------------------------------------------------- synthetic (tests)
class FakeWorld:
    """Deterministic fake terrain for --synthetic. It serves fake map tiles, so the whole real pipeline
    (tile stitching, zoom choice, rotation, sampling) runs without any network access."""

    def __init__(self, ref_bbox):
        lat0, lat1, lon0, lon1 = ref_bbox
        self.cx, self.cy = to_merc((lat0 + lat1) / 2, (lon0 + lon1) / 2)
        x0, y0 = to_merc(lat0, lon0)
        x1, y1 = to_merc(lat1, lon1)
        self.span = max(abs(x1 - x0), abs(y1 - y0))
        rng = np.random.default_rng(3)
        self.bumps = [(rng.uniform(-.5, .5) * self.span, rng.uniform(-.5, .5) * self.span,
                       rng.uniform(.1, .3) * self.span, rng.uniform(150, 600)) for _ in range(7)]
        g = np.linspace(-0.75, 0.75, 60) * self.span
        gx, gy = np.meshgrid(g + self.cx, g + self.cy)
        e = self.elev(gx, gy)
        self.lo, self.hi = float(e.min()), float(e.max())

    def elev(self, xm, ym):
        x, y = xm - self.cx, ym - self.cy
        e = np.zeros(x.shape, np.float64)
        for bx, by, r, a in self.bumps:
            e += a * np.exp(-((x - bx) ** 2 + (y - by) ** 2) / (2 * r * r))
        return e + 1.2 * np.sin(x * 0.011) * np.cos(y * 0.013) + 0.8 * np.sin(x * 0.047 + y * 0.031)

    def rgb(self, xm, ym):
        t = (self.elev(xm, ym) - self.lo) / (self.hi - self.lo + 1e-9)
        out = np.empty(t.shape + (3,), np.float32)
        out[:] = (60, 110, 50)                                           # grass
        out[t < 0.12] = (40, 80, 150)                                    # water
        out[(t >= 0.55) & (t < 0.8)] = (120, 105, 90)                    # rock
        out[t >= 0.8] = (235, 235, 240)                                  # snow
        out += 6 * (np.sin(xm * 0.31) * np.cos(ym * 0.27))[..., None]
        return np.clip(out, 0, 255).astype(np.uint8)

    def http_get(self, url, session):
        """Fake EA WCS: answers GetCoverage with a float32 TIFF, a nodata strip on the east side (to test the fallback)."""
        from urllib.parse import parse_qs, urlparse
        q = {k.lower(): v for k, v in parse_qs(urlparse(url).query).items()}
        if q.get("request", [""])[0].lower() == "getcapabilities":
            return b"<CoverageId>" + b"x__Lidar_Composite_Elevation_DTM_1m" + b"</CoverageId>", 200
        subs = [re.match(r"(\w+)\(([-\d.]+),([-\d.]+)\)", v).groups() for v in q["subset"]]
        if [s[0] for s in subs] != ["X", "Y"]:
            return b"<ExceptionReport>Unknown axis label</ExceptionReport>", 400   # like a real server that wants other labels
        (_, x0, x1), (_, y0, y1) = subs
        w, h = (int(n) for n in re.findall(r"\((\d+)\)", q["scalesize"][0]))
        x0, x1, y0, y1 = float(x0), float(x1), float(y0), float(y1)
        X = x0 + (np.arange(w) + 0.5) * (x1 - x0) / w
        Y = y1 - (np.arange(h) + 0.5) * (y1 - y0) / h
        X, Y = np.meshgrid(X, Y)
        e = self.elev(X, Y) + 0.6 * np.sin(X * 0.9) * np.cos(Y * 0.7)        # extra fine relief the global data lacks
        e[X > self.cx + 0.3 * self.span] = -9999.0                             # outside the survey
        buf = io.BytesIO()
        Image.fromarray(e.astype(np.float32)).save(buf, "TIFF")
        return buf.getvalue(), 200

    def fetch(self, url, cache_dir, session, retries=4):
        nums = [int(n) for n in re.findall(r"/(\d+)(?=[/.]|$)", url)][-3:]
        if "terrarium" in url:
            z, x, y = nums
        else:                                                            # imagery URLs are z/y/x
            z, y, x = nums
        mpp = 2 * HALF_WORLD / (TILE * 2 ** z)
        X, Y = np.meshgrid((x * TILE + np.arange(TILE) + 0.5) * mpp - HALF_WORLD,
                           HALF_WORLD - (y * TILE + np.arange(TILE) + 0.5) * mpp)
        if "terrarium" in url:                                           # same encoding as the real tiles
            v = self.elev(X, Y) + 32768.0
            arr = np.stack([np.floor(v / 256), np.floor(v) % 256, np.floor((v - np.floor(v)) * 256)], -1).astype(np.uint8)
        else:
            arr = self.rgb(X, Y)
        buf = io.BytesIO()
        Image.fromarray(arr).save(buf, "PNG")
        return buf.getvalue()


# ------------------------------------------------------------ OpenStreetMap features
ROAD_CLASSES = ["motorway", "trunk", "primary", "secondary", "tertiary", "unclassified",
                "residential", "living_street", "motorway_link", "trunk_link", "primary_link"]
PATH_CLASSES = ["track", "path", "footway", "cycleway", "bridleway", "pedestrian", "steps"]
ROAD_WIDTH_FACTOR = {"motorway": 1.6, "trunk": 1.6, "primary": 1.5, "secondary": 1.25, "tertiary": 1.1}


def fetch_osm(bbox, water, rivers, buildings, cache_dir, roads=False, paths=False):
    """Download water, waterways, roads, paths and buildings for the bbox from the Overpass API (cached).
    Each kind is fetched separately, so one busy answer only costs that layer, and the progress bar says what is happening."""
    import requests
    bb = f"{bbox[0]:.5f},{bbox[2]:.5f},{bbox[1]:.5f},{bbox[3]:.5f}"
    groups = []
    wr = []
    if water:
        wr += [f'way["natural"="water"]({bb});', f'relation["natural"="water"]({bb});']
    if rivers:
        wr += [f'way["waterway"~"^(river|stream|canal)$"]({bb});']
    if wr:
        groups.append(("water and rivers", wr))
    if roads or paths:
        classes = (ROAD_CLASSES if roads else []) + (PATH_CLASSES if paths else [])
        groups.append(("roads and paths", [f'way["highway"~"^({"|".join(classes)})$"]({bb});']))
    if buildings:
        groups.append(("buildings", [f'way["building"]({bb});', f'relation["building"]({bb});']))
    cache = Path(cache_dir)
    cache.mkdir(parents=True, exist_ok=True)
    elements, failed, session = [], [], requests.Session()
    for gi, (label, parts) in enumerate(groups):
        base = 56 + 3 * gi
        query = f'[out:json][timeout:60];({"".join(parts)});out geom qt;'
        cf = cache / ("osm_" + hashlib.sha1(query.encode()).hexdigest() + ".json")
        if cf.exists():
            elements += json.loads(cf.read_text()).get("elements", [])
            continue
        last, got, n, slow = None, False, 0, 0
        total = 2 * len(OVERPASS_URLS)
        for attempt in range(2):
            slow = 0
            for url in OVERPASS_URLS:
                n += 1
                host = url.split("/")[2]
                progress(base, f"Fetching {label} from OpenStreetMap ({host}, try {n}/{total})")
                print(f"OpenStreetMap: asking {host} for {label} (try {n}/{total})", flush=True)
                try:
                    r = session.post(url, data={"data": query}, headers={"User-Agent": USER_AGENT}, timeout=75)
                    if r.status_code == 200:
                        data = r.json()
                        cf.write_text(json.dumps(data))
                        elements += data.get("elements", [])
                        got = True
                        break
                    last = f"{host}: HTTP {r.status_code}" + (" (busy)" if r.status_code in (429, 504) else "")
                except Exception as e:
                    timed_out = "Timeout" in type(e).__name__
                    slow += timed_out
                    last = f"{host}: " + ("no answer within 75 s" if timed_out else str(e)[:80])
                print(f"  {last}", flush=True)
            if got or slow == len(OVERPASS_URLS):      # every server too slow: asking again straight away won't help
                break
            time.sleep(3 * (attempt + 1))
        if not got:
            failed.append(label)
            print(f"Could not get {label} from OpenStreetMap ({last}); that layer is left out. "
                  f"The public servers are sometimes busy: try again in a few minutes.", flush=True)
    if groups and len(failed) == len(groups):
        raise RuntimeError("Could not reach OpenStreetMap (all servers busy or unreachable). Try again in a few minutes, "
                           "or untick water, rivers, roads, paths and buildings.")
    return {"elements": elements}


def building_height(tags):
    for key in ("height", "building:height"):
        m = re.match(r"\s*([0-9.]+)", tags.get(key, ""))
        if m:
            return float(m.group(1))
    m = re.match(r"\s*([0-9.]+)", tags.get("building:levels", ""))
    return float(m.group(1)) * 3.2 if m else 7.0


def stitch(ways):
    """Join way fragments end-to-end into rings (multipolygon members come in pieces)."""
    ways = [list(w) for w in ways if len(w) >= 2]
    rings = []
    while ways:
        ring = ways.pop()
        changed = True
        while ring[0] != ring[-1] and changed:
            changed = False
            for i, w in enumerate(ways):
                if w[0] == ring[-1]:
                    ring = ring + w[1:]
                elif w[-1] == ring[-1]:
                    ring = ring + w[::-1][1:]
                elif w[-1] == ring[0]:
                    ring = w[:-1] + ring
                elif w[0] == ring[0]:
                    ring = w[::-1][:-1] + ring
                else:
                    continue
                ways.pop(i)
                changed = True
                break
        if len(ring) >= 4:
            rings.append(ring)
    return rings


def parse_osm(data):
    """Overpass JSON -> {'water': [...], 'rivers': [...], 'buildings': [...]} (lat, lon rings/lines)."""
    out = {"water": [], "rivers": [], "buildings": [], "roads": []}
    for el in data.get("elements", []):
        tags = el.get("tags", {})
        if el["type"] == "way":
            g = [(p["lat"], p["lon"]) for p in el.get("geometry", [])]
            if len(g) < 2:
                continue
            closed = g[0] == g[-1] and len(g) >= 4
            if tags.get("highway") in ROAD_CLASSES + PATH_CLASSES:
                out["roads"].append((tags["highway"], g))
            if tags.get("waterway") in ("river", "stream", "canal"):
                out["rivers"].append(g)
            if tags.get("natural") == "water" and closed:
                out["water"].append({"outer": [g], "inner": []})
            if "building" in tags and closed:
                out["buildings"].append({"outer": [g], "inner": [], "h": building_height(tags)})
        elif el["type"] == "relation":
            outers, inners = [], []
            for m in el.get("members", []):
                if m.get("type") != "way" or not m.get("geometry"):
                    continue
                g = [(p["lat"], p["lon"]) for p in m["geometry"]]
                (inners if m.get("role") == "inner" else outers).append(g)
            feat = {"outer": stitch(outers), "inner": stitch(inners)}
            if not feat["outer"]:
                continue
            if tags.get("natural") == "water":
                out["water"].append(feat)
            if "building" in tags:
                out["buildings"].append({**feat, "h": building_height(tags)})
    return out


def synthetic_osm(bbox):
    """Fake lake, river and buildings for offline testing."""
    lat0, lat1, lon0, lon1 = bbox
    P = lambda fy, fx: (lat1 - (lat1 - lat0) * fy, lon0 + (lon1 - lon0) * fx)
    lake = [P(0.62, 0.10), P(0.58, 0.25), P(0.70, 0.32), P(0.82, 0.22), P(0.78, 0.08), P(0.62, 0.10)]
    isle = [P(0.69, 0.16), P(0.72, 0.20), P(0.74, 0.15), P(0.69, 0.16)]
    river = [P(0.0, 0.55), P(0.25, 0.50), P(0.5, 0.58), P(0.75, 0.52), P(1.0, 0.60)]
    blds = []
    for i in range(4):
        for j in range(3):
            fy, fx = 0.30 + i * 0.045, 0.78 + j * 0.05
            ring = [P(fy, fx), P(fy, fx + 0.025), P(fy + 0.025, fx + 0.025), P(fy + 0.025, fx), P(fy, fx)]
            blds.append({"outer": [ring], "inner": [], "h": 6.0 + 4 * ((i + j) % 3)})
    road = [P(0.10, 0.0), P(0.30, 0.35), P(0.38, 0.70), P(0.55, 0.85), P(0.60, 1.0)]
    lane = [P(0.30, 0.74), P(0.45, 0.74), P(0.45, 0.95)]
    return {"water": [{"outer": [lake], "inner": [isle]}], "rivers": [river], "buildings": blds,
            "roads": [("primary", road), ("residential", lane),
                      ("footway", [P(0.80, 0.40), P(0.60, 0.55), P(0.45, 0.62)])]}


def _cells(ring, fr, nxc, nyc):
    return fr.cells(ring, nxc, nyc)


def raster_polys(feats, fr, nxc, nyc):
    """Boolean mask of cells covered by polygons (holes respected)."""
    img = Image.new("L", (nxc, nyc), 0)
    for f in feats:
        outers = [_cells(r, fr, nxc, nyc) for r in f["outer"]]
        inners = [_cells(r, fr, nxc, nyc) for r in f["inner"]]
        if inners:
            tmp = Image.new("L", (nxc, nyc), 0)
            d = ImageDraw.Draw(tmp)
            for r in outers:
                d.polygon(r, fill=255, outline=255)
            for r in inners:
                d.polygon(r, fill=0)
            img = ImageChops.lighter(img, tmp)
        else:
            d = ImageDraw.Draw(img)
            for r in outers:
                d.polygon(r, fill=255, outline=255)
    return np.asarray(img) > 0


def raster_lines(lines, fr, nxc, nyc, width_cells):
    img = Image.new("L", (nxc, nyc), 0)
    d = ImageDraw.Draw(img)
    for ln in lines:
        d.line(_cells(ln, fr, nxc, nyc), fill=255, width=max(1, int(round(width_cells))), joint="curve")
    return np.asarray(img) > 0


def raster_roads(roads, fr, nxc, nyc, base_cells):
    """Cells covered by roads; bigger road classes are drawn wider."""
    img = Image.new("L", (nxc, nyc), 0)
    d = ImageDraw.Draw(img)
    for cls, ln in roads:
        w = max(1, int(round(base_cells * ROAD_WIDTH_FACTOR.get(cls, 1.0))))
        d.line(_cells(ln, fr, nxc, nyc), fill=255, width=w, joint="curve")
    return np.asarray(img) > 0


def raster_building_heights(blds, fr, nxc, nyc, mm_per_m, min_mm):
    """Per-cell building height in mm (0 where there is no building). Taller buildings win overlaps."""
    img = Image.new("F", (nxc, nyc), 0.0)
    d = ImageDraw.Draw(img)
    for b in sorted(blds, key=lambda b: b["h"]):
        h = max(b["h"] * mm_per_m, min_mm)
        for r in b["outer"]:
            d.polygon(_cells(r, fr, nxc, nyc), fill=h, outline=h)
        for r in b["inner"]:
            d.polygon(_cells(r, fr, nxc, nyc), fill=0.0)
    return np.asarray(img, dtype=np.float32)


# ------------------------------------------------------------------ points of interest
POI_TYPES = ["castle", "house", "tower", "church", "flag", "pin"]
POI_COLOURS = {"castle": "#b9b2a3", "house": "#c9774f", "tower": "#a8a8a8",
               "church": "#d8d2c2", "flag": "#e8202a", "pin": "#ff3b30"}
POI_RADIUS = {"castle": 6.0, "house": 3.6, "tower": 2.8, "church": 5.5, "flag": 2.2, "pin": 2.4}  # mm at size 1
NOMINATIM_URL = "https://nominatim.openstreetmap.org/search"


def geocode(query, cache_dir):
    """Place name -> (lat, lon, display name) via OpenStreetMap Nominatim (cached, 1 request/second)."""
    import requests
    cache = Path(cache_dir)
    cache.mkdir(parents=True, exist_ok=True)
    cf = cache / ("geo_" + hashlib.sha1(query.lower().encode()).hexdigest() + ".json")
    if cf.exists():
        hit = json.loads(cf.read_text())
    else:
        try:
            r = requests.get(NOMINATIM_URL, params={"q": query, "format": "json", "limit": 1},
                             headers={"User-Agent": USER_AGENT}, timeout=30)
            r.raise_for_status()
            res = r.json()
        except Exception as e:
            sys.exit(f"Could not look up '{query}' ({e}). Use latitude,longitude instead, e.g. 50.6395,-2.0576")
        if not res:
            sys.exit(f"No place found for '{query}'. Try a more specific name, or use latitude,longitude.")
        hit = {"lat": float(res[0]["lat"]), "lon": float(res[0]["lon"]), "name": res[0].get("display_name", query)}
        cf.write_text(json.dumps(hit))
        time.sleep(1.1)
    return hit["lat"], hit["lon"], hit["name"]


def resolve_pois(specs, waypoint_type, waypoints, cache_dir):
    """Specs look like 'castle|Corfe Castle|#b9b2a3|1.2' or 'house|50.61,-2.05'. Returns a list of dicts."""
    pois = []
    for spec in specs or []:
        parts = [p.strip() for p in spec.split("|")]
        if len(parts) < 2 or parts[0].lower() not in POI_TYPES or not parts[1]:
            sys.exit(f"Bad --poi '{spec}'. Use type|place or lat,lon[|#colour[|size]]; types: {', '.join(POI_TYPES)}")
        kind = parts[0].lower()
        m = re.fullmatch(r"(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)", parts[1])
        if m:
            lat, lon = float(m.group(1)), float(m.group(2))
        else:
            lat, lon, name = geocode(parts[1], cache_dir)
            print(f"  {kind}: '{parts[1]}' -> {name[:70]} ({lat:.5f}, {lon:.5f})")
        colour = parts[2] if len(parts) > 2 and re.fullmatch(r"#[0-9a-fA-F]{6}", parts[2]) else POI_COLOURS[kind]
        try:
            size = min(max(float(parts[3]), 0.4), 4.0) if len(parts) > 3 and parts[3] else 1.0
        except ValueError:
            size = 1.0
        pois.append({"type": kind, "lat": lat, "lon": lon, "rgb": parse_hex(colour), "size": size})
    if waypoint_type:
        for lat, lon in waypoints:
            pois.append({"type": waypoint_type, "lat": lat, "lon": lon,
                         "rgb": parse_hex(POI_COLOURS[waypoint_type]), "size": 1.0})
    return pois


def _orient(v, t):
    """Make every triangle of a convex solid face outward."""
    v = np.asarray(v, float)
    t = np.asarray(t, int).copy()
    p = v[t]
    n = np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0])
    flip = (n * (p.mean(1) - v.mean(0))).sum(1) < 0
    t[flip] = t[flip][:, [0, 2, 1]]
    return v, t


def _quads(q):
    return [(a, b, c) for a, b, c, d in q for (a, b, c) in ((a, b, c), (a, c, d))]


def _box(cx, cy, w, d, z0, z1):
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - d / 2, cy + d / 2
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    return _orient(v, _quads([(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]))


def _cyl(cx, cy, r, z0, z1, n=18):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = [(cx + r * np.cos(t), cy + r * np.sin(t)) for t in a]
    v = [(x, y, z0) for x, y in ring] + [(x, y, z1) for x, y in ring] + [(cx, cy, z0), (cx, cy, z1)]
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris += [(i, j, n + j), (i, n + j, n + i), (2 * n, j, i), (2 * n + 1, n + i, n + j)]
    return _orient(v, tris)


def _cone(cx, cy, r, z0, z1, n=18, rot=0.0):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False) + rot
    v = [(cx + r * np.cos(t), cy + r * np.sin(t), z0) for t in a] + [(cx, cy, z1), (cx, cy, z0)]
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris += [(i, j, n), (n + 1, j, i)]
    return _orient(v, tris)


def _gable(cx, cy, w, d, z0, zr):
    """Roof prism: ridge runs along y."""
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - d / 2, cy + d / 2
    v = [(x0, y0, z0), (x1, y0, z0), (cx, y0, zr), (x0, y1, z0), (x1, y1, z0), (cx, y1, zr)]
    return _orient(v, [(0, 1, 2), (3, 4, 5)] + _quads([(0, 1, 4, 3), (0, 2, 5, 3), (1, 2, 5, 4)]))


def build_poi(kind, size=1.0):
    """Stylised marker around (0, 0) standing on z = 0. Sizes in mm; not to map scale."""
    if kind == "house":
        pieces = [_box(0, 0, 6, 5, 0, 3), _gable(0, 0, 6.6, 5.6, 3, 5.4)]
    elif kind == "tower":
        pieces = [_cyl(0, 0, 2.0, 0, 9), _cone(0, 0, 2.5, 9, 12.5)]
    elif kind == "castle":
        pieces = [_box(0, 4.9, 11, 1.2, 0, 3), _box(0, -4.9, 11, 1.2, 0, 3),
                  _box(4.9, 0, 1.2, 11, 0, 3), _box(-4.9, 0, 1.2, 11, 0, 3),
                  _box(0, 0, 4.2, 4.2, 0, 7.5), _box(0, 0, 4.8, 4.8, 7.5, 8.3)]
        for sx in (-1, 1):
            for sy in (-1, 1):
                pieces += [_cyl(sx * 5, sy * 5, 1.7, 0, 5.2), _cone(sx * 5, sy * 5, 2.1, 5.2, 7.2)]
    elif kind == "church":
        pieces = [_box(0, -1.2, 4.6, 7.5, 0, 3.6), _gable(0, -1.2, 5.2, 7.5, 3.6, 5.6),
                  _box(0, 3.4, 3.0, 3.0, 0, 8.0), _cone(0, 3.4, 2.3, 8.0, 12.5, n=4, rot=np.pi / 4)]
    elif kind == "flag":
        pieces = [_cyl(0, 0, 0.5, 0, 9.5, n=10), _box(1.7, 0, 3.4, 0.6, 6.0, 9.2), _cyl(0, 0, 1.4, 0, 0.8, n=14)]
    else:  # pin
        pieces = [_cone(0, 0, 2.2, 0, 9.0, n=14)]
    verts, tris, off = [], [], 0
    for v, t in pieces:
        verts.append(v * size)
        tris.append(t + off)
        off += len(v)
    return np.vstack(verts), np.vstack(tris)


# ------------------------------------------------------------------------ colours
def kmeans(x, k, seed=0, iters=30):
    rng = np.random.default_rng(seed)
    x = x.astype(np.float32)
    s = x[rng.choice(len(x), 50000, replace=False)] if len(x) > 50000 else x
    c = [s[rng.integers(len(s))]]
    for _ in range(1, k):
        d = np.min([((s - ci) ** 2).sum(1) for ci in c], axis=0)
        p = d / d.sum() if d.sum() > 0 else None
        c.append(s[rng.choice(len(s), p=p)])
    c = np.array(c, dtype=np.float32)
    for _ in range(iters):
        lab = ((s[:, None, :] - c[None]) ** 2).sum(2).argmin(1)
        for j in range(k):
            m = lab == j
            if m.any():
                c[j] = s[m].mean(0)
    return c


def assign(x, centres, chunk=100000):
    out = np.empty(len(x), np.int32)
    for i in range(0, len(x), chunk):
        d = ((x[i:i + chunk, None, :].astype(np.float32) - centres[None]) ** 2).sum(2)
        out[i:i + chunk] = d.argmin(1)
    return out


def quantise(rgb_img, k, clean_passes):
    """Cluster imagery into k colours. Returns (index grid, palette uint8 (k,3))."""
    arr = np.asarray(rgb_img, dtype=np.float32)
    h, w, _ = arr.shape
    flat = arr.reshape(-1, 3)
    centres = kmeans(flat, k)
    lab = assign(flat, centres)
    # order palette dark -> light so indices are stable & meaningful
    lum = centres @ np.array([0.299, 0.587, 0.114], np.float32)
    order = np.argsort(lum)
    remap = np.empty(k, np.int32)
    remap[order] = np.arange(k)
    idx = remap[lab].reshape(h, w)
    palette = np.clip(centres[order], 0, 255).round().astype(np.uint8)
    for _ in range(clean_passes):  # remove speckle: 3x3 majority filter
        idx = np.asarray(Image.fromarray(idx.astype(np.uint8)).filter(ImageFilter.ModeFilter(3)))
    return idx.astype(np.int32), palette


def parse_hex(s):
    s = s.lstrip("#")
    return tuple(int(s[i:i + 2], 16) for i in (0, 2, 4))


# ------------------------------------------------------------------------ helpers
def gauss_smooth(a, sigma):
    if sigma <= 0:
        return a
    r = int(3 * sigma) + 1
    x = np.arange(-r, r + 1)
    k = np.exp(-x ** 2 / (2 * sigma ** 2))
    k /= k.sum()
    h, w = a.shape
    p = np.pad(a, ((0, 0), (r, r)), mode="edge")
    a = sum(k[i] * p[:, i:i + w] for i in range(2 * r + 1))
    p = np.pad(a, ((r, r), (0, 0)), mode="edge")
    return sum(k[i] * p[i:i + h, :] for i in range(2 * r + 1))


def warp(img, fr, z, origin, nxc, nyc, vertices, shrink_to, shrink_filter):
    """Sample a tile canvas (PIL, mode F or RGB) through the possibly rotated frame, onto the mesh
    vertices ((nxc+1) x (nyc+1)) or the cell centres (nxc x nyc). Source pixels are first averaged down
    to about `shrink_to` pixels per cell so fine data doesn't alias."""
    (a, b, c, d, e, f), mpp = fr.coeffs(z, nxc, nyc, origin, vertices)
    per_cell = (fr.W / nxc) / mpp
    if per_cell > shrink_to:
        k = shrink_to / per_cell
        w0, h0 = img.size
        img = img.resize((max(4, round(w0 * k)), max(4, round(h0 * k))), shrink_filter)
        fx, fy = img.width / w0, img.height / h0
        a, b, c, d, e, f = a * fx, b * fx, c * fx, d * fy, e * fy, f * fy
    arr = np.asarray(img)
    arr = np.pad(arr, [(2, 2), (2, 2)] + [(0, 0)] * (arr.ndim - 2), mode="edge")   # clamp at the canvas edge
    out = Image.fromarray(arr).transform((nxc + 1, nyc + 1) if vertices else (nxc, nyc), Image.AFFINE,
                                         (a, b, c + 2, d, e, f + 2), resample=Image.BICUBIC)
    return out


def rasterise_trail(tracks, fr, size_cells, width_cells):
    """Boolean mask (ny_c, nx_c) of cells touched by the GPX path."""
    nx_c, ny_c = size_cells
    img = Image.new("L", (nx_c, ny_c), 0)
    d = ImageDraw.Draw(img)
    for pts in tracks:
        xy = fr.cells(pts, nx_c, ny_c)
        if len(xy) == 1:
            xy = xy * 2
        d.line(xy, fill=255, width=max(1, int(round(width_cells))), joint="curve")
    return np.asarray(img) > 0


def shape_polygon(shape, W, H, style, size):
    """Outline of the model (mm), counter-clockwise or clockwise, with corners filleted or chamfered.
    Works for any convex polygon, so rectangles and hexagons share one code path."""
    if shape == "circle":
        a = np.linspace(0, 2 * math.pi, 144, endpoint=False)
        return np.stack([W / 2 + W / 2 * np.cos(a), H / 2 + H / 2 * np.sin(a)], 1)
    if shape == "hexagon":
        P = np.array([(W * .25, 0), (W * .75, 0), (W, H / 2), (W * .75, H), (W * .25, H), (0, H / 2)], float)
    else:
        P = np.array([(0, 0), (W, 0), (W, H), (0, H)], float)
    if style == "sharp" or size <= 0:
        return P
    n, out = len(P), []
    for k in range(n):
        p, a, b = P[k], P[k - 1], P[(k + 1) % n]
        e1, e2 = a - p, b - p
        l1, l2 = float(np.linalg.norm(e1)), float(np.linalg.norm(e2))
        e1, e2 = e1 / l1, e2 / l2
        half = math.acos(float(np.clip(e1 @ e2, -1, 1))) / 2
        lim = 0.49 * min(l1, l2)                       # never eat more than half of an edge
        if style == "chamfer":
            d = min(size, lim)
            out += [p + e1 * d, p + e2 * d]
        else:                                          # fillet: circular arc tangent to both edges
            t = min(size / math.tan(half), lim)
            r = t * math.tan(half)
            A, B = p + e1 * t, p + e2 * t
            C = p + (e1 + e2) / float(np.linalg.norm(e1 + e2)) * (r / math.sin(half))
            a0, a1 = math.atan2(A[1] - C[1], A[0] - C[0]), math.atan2(B[1] - C[1], B[0] - C[0])
            da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
            steps = max(4, int(abs(da) / (math.pi / 2) * 24))
            out += [C + r * np.array([math.cos(a0 + da * u), math.sin(a0 + da * u)]) for u in np.linspace(0, 1, steps)]
    return np.array(out)


def footprint_mask(shape, style, poly, nxc, nyc, cell_mm):
    """Grid cells whose centre lies inside the outline polygon (convex)."""
    if shape == "rectangle" and style == "sharp":
        return np.ones((nyc, nxc), bool)
    X, Y = np.meshgrid((np.arange(nxc) + 0.5) * cell_mm, (np.arange(nyc) + 0.5) * cell_mm)
    if shape == "circle":
        Wm, Hm = nxc * cell_mm, nyc * cell_mm
        return ((X - Wm / 2) / (Wm / 2)) ** 2 + ((Y - Hm / 2) / (Hm / 2)) ** 2 <= 1.0
    nxt = np.roll(poly, -1, axis=0)
    sgn = 1.0 if 0.5 * np.sum(poly[:, 0] * nxt[:, 1] - nxt[:, 0] * poly[:, 1]) > 0 else -1.0
    inside = np.ones(X.shape, bool)
    for p0, p1 in zip(poly, nxt):
        ex, ey = p1 - p0
        if abs(ex) + abs(ey) < 1e-12:
            continue
        inside &= sgn * (ex * (Y - p0[1]) - ey * (X - p0[0])) >= -1e-9
    return inside


# -------------------------------------------------------------------------- mesh
def build_part(mask, nx, ny):
    """
    Closed solid for the cells in `mask` (shape (ny-1, nx-1)). Top vertices are shared grid
    vertices; bottom vertices sit at z=0. Walls are emitted wherever a neighbouring cell is
    outside the mask, so parts of different colours fit together perfectly.
    Vertex ids: top = r*nx + c, bottom = top + nx*ny.  Returns (tris (T,3), cell_flat_id (T,)).
    """
    nyc, nxc = mask.shape
    N = nx * ny
    VI = np.arange(N).reshape(ny, nx)
    rr, cc = np.nonzero(mask)
    a, b, d, e = VI[rr, cc], VI[rr, cc + 1], VI[rr + 1, cc], VI[rr + 1, cc + 1]
    cid = rr * nxc + cc
    tris, cells = [], []

    def add(t, c):
        tris.append(np.asarray(t).T if np.ndim(t) == 2 and np.shape(t)[0] == 3 else t)
        cells.append(c)

    # top (normal +z) and bottom (normal -z)
    for t in ((a, d, e), (a, e, b), (a + N, e + N, d + N), (a + N, b + N, e + N)):
        tris.append(np.stack(t, 1)); cells.append(cid)

    P = np.pad(mask, 1)
    outside = {
        "n": ~P[rr, cc + 1], "s": ~P[rr + 2, cc + 1],
        "w": ~P[rr + 1, cc], "e": ~P[rr + 1, cc + 2],
    }
    edges = {"n": (a, b), "e": (b, e), "s": (e, d), "w": (d, a)}  # interior on the right
    for side, (t0, t1) in edges.items():
        m = outside[side]
        t0, t1, c = t0[m], t1[m], cid[m]
        b0, b1 = t0 + N, t1 + N
        tris.append(np.stack((t0, b1, b0), 1)); cells.append(c)
        tris.append(np.stack((t0, t1, b1), 1)); cells.append(c)
    return np.concatenate(tris).astype(np.int64), np.concatenate(cells)


def compact(verts, tris):
    used, inv = np.unique(tris, return_inverse=True)
    return verts[used], inv.reshape(tris.shape)


# ----------------------------------------------------------------------- writers
def write_stl(path, verts, tris, colours=None):
    p = verts[tris].astype(np.float32)
    n = np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)
    dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    arr = np.zeros(len(tris), dt)
    arr["n"], arr["v"] = n, p
    header = b"gpx2stl terrain"
    if colours is not None:
        c = colours.astype(np.uint16)
        # VisCAM/SolidView convention: bit15 = colour valid, R in bits 0-4, G 5-9, B 10-14
        arr["a"] = 0x8000 | (c[:, 0] >> 3) | ((c[:, 1] >> 3) << 5) | ((c[:, 2] >> 3) << 10)
        header += b" COLOR=ON"
    with open(path, "wb") as f:
        f.write(header.ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        f.write(arr.tobytes())


_CT = ('<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/'
       'content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.'
       'relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-'
       '3dmodel+xml"/></Types>')
_RELS = ('<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/'
         '2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/'
         '3dmanufacturing/2013/01/3dmodel"/></Relationships>')


def _xml_vertices(v, chunk=100000):
    for i in range(0, len(v), chunk):
        yield "".join('<vertex x="%.4f" y="%.4f" z="%.4f"/>' % tuple(r) for r in v[i:i + chunk].tolist())


def _xml_triangles(t, extra=None, chunk=100000):
    for i in range(0, len(t), chunk):
        rows = t[i:i + chunk].tolist()
        if extra is None:
            yield "".join('<triangle v1="%d" v2="%d" v3="%d"/>' % tuple(r) for r in rows)
        else:
            ex = extra[i:i + chunk].tolist()
            yield "".join('<triangle v1="%d" v2="%d" v3="%d" pid="1" p1="%d"/>' % (r[0], r[1], r[2], k)
                          for r, k in zip(rows, ex))


def write_3mf(path, verts, tris, tri_colour_idx, palette):
    cols = "".join('<m:color color="#%02X%02X%02X"/>' % tuple(c) for c in palette.tolist())
    head = ('<?xml version="1.0" encoding="UTF-8"?><model unit="millimeter" xml:lang="en-US" '
            'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" '
            'xmlns:m="http://schemas.microsoft.com/3dmanufacturing/material/2015/02">'
            f'<resources><m:colorgroup id="1">{cols}</m:colorgroup><object id="2" type="model"><mesh><vertices>')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", _CT)
        z.writestr("_rels/.rels", _RELS)
        with z.open("3D/3dmodel.model", "w", force_zip64=True) as f:
            f.write(head.encode())
            for chunk in _xml_vertices(verts):
                f.write(chunk.encode())
            f.write(b"</vertices><triangles>")
            for chunk in _xml_triangles(tris, tri_colour_idx):
                f.write(chunk.encode())
            f.write(b'</triangles></mesh></object></resources><build><item objectid="2"/></build></model>')


def write_3mf_parts(path, parts):
    """One 3MF where each colour is its own closed part, grouped into a single object.
    parts: list of (name, verts, tris, (r, g, b)). Slicers (Bambu Studio, OrcaSlicer, PrusaSlicer)
    show them as separate parts of one object, ready to assign a filament to each."""
    bases = "".join('<base name="%s" displaycolor="#%02X%02X%02X"/>' % (n, r, g, b) for n, _, _, (r, g, b) in parts)
    asm_id = len(parts) + 2
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", _CT)
        z.writestr("_rels/.rels", _RELS)
        with z.open("3D/3dmodel.model", "w", force_zip64=True) as f:
            f.write(('<?xml version="1.0" encoding="UTF-8"?><model unit="millimeter" xml:lang="en-US" '
                     'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
                     f'<resources><basematerials id="1">{bases}</basematerials>').encode())
            for i, (n, v, t, _) in enumerate(parts):
                f.write(f'<object id="{i + 2}" name="{n}" type="model" pid="1" pindex="{i}"><mesh><vertices>'.encode())
                for chunk in _xml_vertices(v):
                    f.write(chunk.encode())
                f.write(b"</vertices><triangles>")
                for chunk in _xml_triangles(t):
                    f.write(chunk.encode())
                f.write(b"</triangles></mesh></object>")
            comps = "".join(f'<component objectid="{i + 2}"/>' for i in range(len(parts)))
            f.write((f'<object id="{asm_id}" name="terrain" type="model"><components>{comps}</components></object>'
                     f'</resources><build><item objectid="{asm_id}"/></build></model>').encode())


def write_surface_obj(path, top_v, mask, idx, palette, nx, ny):
    """Top surface only (no walls, no underside): an OBJ with one material per colour, plus its MTL."""
    nyc, nxc = mask.shape
    VI = np.arange(nx * ny).reshape(ny, nx)
    rr, cc = np.nonzero(mask)
    a, b, d, e = VI[rr, cc], VI[rr, cc + 1], VI[rr + 1, cc], VI[rr + 1, cc + 1]
    tris = np.concatenate([np.stack((a, d, e), 1), np.stack((a, e, b), 1)])
    col = np.concatenate([idx[rr, cc], idx[rr, cc]]) if idx is not None else np.zeros(len(tris), int)
    verts, tris = compact(top_v, tris)
    mtl = path.with_suffix(".mtl")
    cols = sorted(set(int(c) for c in col))
    with open(mtl, "w") as fh:
        for k in cols:
            r, g, b_ = (palette[k] / 255.0) if idx is not None else (0.67, 0.67, 0.67)
            fh.write(f"newmtl colour{k + 1}\nKd {r:.4f} {g:.4f} {b_:.4f}\nKa 0 0 0\nKs 0 0 0\n\n")
    with open(path, "w") as fh:
        fh.write(f"# surface only, units mm, Z is up\nmtllib {mtl.name}\n")
        np.savetxt(fh, verts, fmt="v %.4f %.4f %.4f")
        for k in cols:
            fh.write(f"usemtl colour{k + 1}\n")
            np.savetxt(fh, tris[col == k] + 1, fmt="f %d %d %d")
    print(f"Wrote {path} and {mtl.name}  (top surface only, {len(tris):,} triangles, Z up)")


def write_preview(path, idx, palette, Zcell, scale=4, footprint=None, theta=0.0):
    gy, gx = np.gradient(Zcell)
    shade = np.clip(0.75 + (-gx - gy) * 0.8, 0.45, 1.2)
    rgb = np.clip(palette[idx].astype(np.float32) * shade[..., None], 0, 255).astype(np.uint8)
    alpha = np.full(idx.shape, 255, np.uint8) if footprint is None else (footprint * 255).astype(np.uint8)
    img = Image.fromarray(np.dstack([rgb, alpha]))
    img = img.resize((img.width * scale, img.height * scale), Image.NEAREST)
    # north arrow: the top edge faces bearing theta, so north points -theta from "up"
    d = ImageDraw.Draw(img)
    r = max(14, min(img.size) // 14)
    cx, cy = r + 12, r + 12
    dx, dy = -math.sin(math.radians(theta)), -math.cos(math.radians(theta))
    tip, tail = (cx + dx * r, cy + dy * r), (cx - dx * r, cy - dy * r)
    for width, col in ((5, (255, 255, 255, 255)), (2, (30, 30, 30, 255))):
        d.line([tail, tip], fill=col, width=width)
        for sgn in (-1, 1):
            hx, hy = -dy * sgn, dx * sgn
            d.line([tip, (tip[0] - dx * r * .45 + hx * r * .3, tip[1] - dy * r * .45 + hy * r * .3)], fill=col, width=width)
    d.text((tip[0] + dx * 8 - 3, tip[1] + dy * 8 - 5), "N", fill=(30, 30, 30, 255))
    img.save(path)


# -------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="GPX -> 3D terrain STL (real elevation + satellite colours)")
    ap.add_argument("gpx", nargs="?", help="GPX file (optional when --centre and --width-m are given)")
    ap.add_argument("-o", "--output", help="output .stl (default: <gpx name>.stl)")
    ap.add_argument("--size", type=float, default=150, help="longest side of the model in mm (default 150)")
    ap.add_argument("--resolution", type=int, default=400,
                    help="grid cells along the longest side (default 400; 200 = quick draft, 700+ = very fine; "
                         "bigger = more detail but larger files)")
    ap.add_argument("--colours", "--colors", type=int, default=4, dest="colours",
                    help="number of terrain colours from satellite imagery; 0 or 1 = no colour (default 4)")
    ap.add_argument("--vertical-exaggeration", type=float, default=1.5)
    ap.add_argument("--base", type=float, default=4.0, help="solid base thickness in mm (default 4)")
    ap.add_argument("--pad", type=float, default=0.15, help="extra margin around the route, fraction (default 0.15)")
    ap.add_argument("--min-span", type=float, default=1500, help="minimum model extent in metres (default 1500)")
    ap.add_argument("--shape", choices=SHAPES, default="rectangle",
                    help="outline of the model: rectangle or hexagon (flat-topped; rotate 30 or 90 for pointy)")
    ap.add_argument("--ratio", default="auto",
                    help="width:height of the outline, e.g. 3:2, 1:1, 16:9 (default auto: fits the route; "
                         "a regular hexagon for --shape hexagon)")
    ap.add_argument("--centre", "--center", default=None, metavar="LAT,LON", dest="centre",
                    help="fix the model centre instead of fitting the route (for neighbouring tiles; needs --width-m)")
    ap.add_argument("--width-m", type=float, default=None, help="with --centre: ground width of the model in metres")
    ap.add_argument("--base-elev", type=float, default=None,
                    help="elevation in metres that sits at the top of the base; set the same value on neighbouring tiles "
                         "so their heights match (default: the lowest point of this model)")
    ap.add_argument("--lidar", choices=list(LIDAR_SOURCES), default="none",
                    help="use real LiDAR terrain where available instead of the global elevation data (England: ea-dtm-1m)")
    ap.add_argument("--lidar-max-px", type=float, default=16e6, help=argparse.SUPPRESS)
    ap.add_argument("--lidar-check", default=None, metavar="LAT,LON",
                    help="just test whether the LiDAR service answers and covers this point, then exit")
    ap.add_argument("--scheme", choices=list(SCHEMES), default="satellite",
                    help="terrain colour scheme: true satellite colours, or a palette painted by height (default satellite)")
    ap.add_argument("--progress", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--skin", type=float, default=0.0, metavar="MM",
                    help="with --split / --3mf-parts: make each colour a thin layer this thick on top of one plain base "
                         "(less filament changing) instead of colouring all the way down")
    ap.add_argument("--surface-obj", action="store_true",
                    help="also write the top surface only, with colours, as an OBJ + MTL (for Blender and renderers)")
    ap.add_argument("--minecraft", choices=("none", "java", "bedrock"), default="none",
                    help="also write a Minecraft map: Java .schem, or Bedrock .mcpack of .mcstructure tiles")
    ap.add_argument("--mc-size", type=int, default=None, help="Minecraft map size in blocks, longest side (default: your grid, up to 400)")
    ap.add_argument("--mc-version", default="1.20", choices=("1.21", "1.20", "1.19", "1.18", "1.17"),
                    help="Java version the .schem is written for; it also works in newer versions (default 1.20)")
    ap.add_argument("--portrait", action="store_true", help="swap --ratio to portrait, e.g. 3:2 becomes 2:3")
    ap.add_argument("--square", action="store_true", help="same as --ratio 1:1")
    ap.add_argument("--rotate", default="0",
                    help="which compass bearing faces the top edge of the model, degrees clockwise from north "
                         "(0 = north at the top, 90 = east at the top), or 'auto' for the tightest fit to the route")
    ap.add_argument("--corner-style", choices=CORNER_STYLES, default="sharp",
                    help="outline corners: sharp, fillet (rounded) or chamfer (cut)")
    ap.add_argument("--corner-size", type=float, default=0,
                    help="fillet radius or chamfer length in mm (default 0 = sharp)")
    ap.add_argument("--corner-radius", type=float, default=0, help=argparse.SUPPRESS)   # old name for a fillet size
    ap.add_argument("--smooth", type=float, default=None,
                    help="elevation smoothing in grid cells (default: automatic, 0.8 or more at high detail; 0 = none)")
    ap.add_argument("--clean", type=int, default=1, help="colour speckle-removal passes (default 1)")
    ap.add_argument("--clip-sea", action="store_true", help="clamp elevations below 0 m to 0")
    ap.add_argument("--water", action="store_true", help="lakes/ponds from OpenStreetMap: coloured and slightly recessed")
    ap.add_argument("--rivers", action="store_true", help="rivers, canals and streams from OpenStreetMap: coloured channels")
    ap.add_argument("--buildings", action="store_true", help="OpenStreetMap buildings, raised (best for small areas)")
    ap.add_argument("--roads", action="store_true", help="roads from OpenStreetMap: coloured and slightly raised")
    ap.add_argument("--paths", action="store_true", help="footpaths, tracks, bridleways and cycleways from OpenStreetMap")
    ap.add_argument("--path-width", type=float, default=0.6, help="footpath width on the model in mm (default 0.6)")
    ap.add_argument("--path-colour", default="#a0764a", help="footpath colour (default brown)")
    ap.add_argument("--road-width", type=float, default=0.9, help="road width on the model in mm (default 0.9)")
    ap.add_argument("--road-raise", type=float, default=0.3, help="road height above the land in mm (default 0.3)")
    ap.add_argument("--river-colour", default="#3a78c2", help="river colour (default blue)")
    ap.add_argument("--road-colour", default="#8a8a8a", help="road colour (default grey)")
    ap.add_argument("--sea", action="store_true", help="treat areas at/below 0 m elevation as sea (coloured as water)")
    ap.add_argument("--water-depth", type=float, default=0.8, help="how far lakes sit below the land, mm (default 0.8)")
    ap.add_argument("--river-width", type=float, default=1.0, help="minimum river width on the model, mm (default 1.0)")
    ap.add_argument("--river-depth", type=float, default=0.5, help="river channel depth, mm (default 0.5)")
    ap.add_argument("--building-min-height", type=float, default=0.8, help="minimum building height, mm (default 0.8)")
    ap.add_argument("--water-colour", default="#3a78c2", help="lake/water colour (default blue)")
    ap.add_argument("--building-colour", default="#d6cfc4", help="building colour (default stone)")
    ap.add_argument("--max-building-cell-m", type=float, default=20.0,
                    help="skip buildings if one grid cell covers more than this many metres (default 20)")
    ap.add_argument("--poi", action="append", default=[], metavar="SPEC",
                    help="point of interest: 'type|place or lat,lon[|#colour[|size]]'; repeatable. "
                         "Types: castle, house, tower, church, flag, pin")
    ap.add_argument("--waypoint-poi", choices=POI_TYPES, default=None,
                    help="also place a marker of this type at every waypoint in the GPX file")
    ap.add_argument("--no-trail", action="store_true", help="don't mark the GPX route on the model")
    ap.add_argument("--trail-width", type=float, default=1.6, help="route width in mm (default 1.6)")
    ap.add_argument("--trail-raise", type=float, default=0.6, help="route emboss height in mm (default 0.6)")
    ap.add_argument("--trail-colour", default="#e8202a", help="route colour (default red)")
    ap.add_argument("--3mf", dest="threemf", action="store_true", help="also write a coloured .3mf")
    ap.add_argument("--split", action="store_true",
                    help="also write one closed STL per colour (for multi-material slicers)")
    ap.add_argument("--3mf-parts", dest="threemf_parts", action="store_true",
                    help="also write a 3MF where each colour is a separate part (best for multi-colour slicers)")
    ap.add_argument("--view-mesh", action="store_true",
                    help="for large models, also write a lighter <name>_view.stl for quick on-screen preview "
                         "(used by the web app; not meant for printing)")
    ap.add_argument("--no-colour-in-stl", action="store_true", help="plain STL without colour attributes")
    ap.add_argument("--max-zoom", type=int, default=19, help="max imagery zoom (default 19; steps down where the server has no detail)")
    ap.add_argument("--imagery-oversample", type=float, default=4.0, help="imagery pixels per grid cell to aim for (default 4)")
    ap.add_argument("--max-imagery-tiles", type=int, default=500, help="tile budget for imagery (default 500)")
    ap.add_argument("--max-elev-zoom", type=int, default=15, help="max elevation tile zoom (default 15)")
    ap.add_argument("--max-tiles", type=int, default=200)
    ap.add_argument("--imagery-url", default=ESRI_URL, help="XYZ tile template for imagery ({z}/{x}/{y})")
    ap.add_argument("--cache-dir", default=str(Path.home() / ".cache" / "gpx2stl"))
    ap.add_argument("--synthetic", action="store_true", help="use fake terrain (offline testing only)")
    args = ap.parse_args()
    if args.corner_radius > 0 and args.corner_size <= 0:
        args.corner_style, args.corner_size = "fillet", args.corner_radius

    global PROGRESS
    PROGRESS = args.progress
    if args.lidar_check:
        la, lo = (float(v) for v in args.lidar_check.split(","))
        r = lidar_check(args.lidar if args.lidar != "none" else "ea-dtm-1m", la, lo, args.cache_dir)
        print(r["message"])
        sys.exit(0 if r["ok"] else 1)
    if args.centre:
        try:
            la, lo = (float(v) for v in args.centre.replace(";", ",").split(","))
            assert -85 <= la <= 85 and -180 <= lo <= 180
        except (ValueError, AssertionError):
            sys.exit("--centre must be LAT,LON, e.g. 53.07,-4.08")
        args.centre = (la, lo)
        if not args.width_m or args.width_m < 100:
            sys.exit("--centre needs --width-m (the ground width in metres, at least 100)")
    elif not args.gpx:
        sys.exit("Give a GPX file, or --centre LAT,LON with --width-m.")
    if args.scheme != "satellite" and args.colours < 2:
        print("Note: colour schemes only apply when --colours is 2 or more.")
    out = Path(args.output) if args.output else Path(args.gpx or "tile.gpx").with_suffix(".stl")
    out.parent.mkdir(parents=True, exist_ok=True)
    progress(1, "Reading your route")
    tracks, wpts = read_gpx(args.gpx) if args.gpx else ([], [])
    if args.poi:
        print("Looking up points of interest...")
    pois = resolve_pois(args.poi, args.waypoint_poi, wpts, args.cache_dir)
    pts = [p for t in tracks for p in t] + wpts + [(p["lat"], p["lon"]) for p in pois]

    progress(4, "Planning the area")
    # ---- footprint: shape, ratio and rotation
    lay = build_layout(pts, args)
    fr, nxc, nyc, cell_mm = lay.frame, lay.nxc, lay.nyc, lay.cell_mm
    nx, ny = nxc + 1, nyc + 1
    aabb = fr.aabb()
    longest_m = max(fr.W_g, fr.H_g)
    ref_bbox = aabb if (args.centre or not pts) else make_bbox(pts, args.pad, False, args.min_span)   # anchors the fake world
    if args.centre and pois:                                        # markers outside this tile belong to its neighbours
        keep = []
        for p in pois:
            fx, fy = _cells([(p["lat"], p["lon"])], fr, nxc, nyc)[0]
            if 0 <= fx <= nxc and 0 <= fy <= nyc:
                keep.append(p)
        if len(keep) < len(pois):
            print(f"{len(pois) - len(keep)} marker(s) fall outside this tile and were left out.")
        pois = keep
    print(f"Area: {fr.W_g / 1000:.2f} x {fr.H_g / 1000:.2f} km {args.shape}, centred "
          f"{fr.lat_c:.4f}, {from_merc(fr.xc, fr.yc)[1]:.4f}; the top edge faces bearing {lay.theta % 360:.0f} deg"
          + (" (auto-fitted)" if lay.auto else ""))
    if lay.style != "sharp":
        print(f"Corners: {lay.style}, {args.corner_size:g} mm")

    # ---- map data
    if args.synthetic:
        _fw = FakeWorld(ref_bbox)
        globals()["fetch_tile"] = _fw.fetch
        globals()["http_get"] = _fw.http_get
        print("Using SYNTHETIC terrain (test mode).")
    progress(5, "Downloading elevation tiles")
    z_e = pick_zoom_frame(fr, aabb, args.resolution * 1.5, args.max_elev_zoom, args.max_tiles)
    while True:
        print(f"Fetching elevation tiles (zoom {z_e}, {tile_count(aabb, z_e)} tiles)...")
        try:
            ecanvas, org_e = mosaic_raw(TERRARIUM_URL, z_e, aabb, args.cache_dir, "RGB",
                                        on_tile=lambda d, t: progress(5 + 15 * d / t, f"Downloading elevation tiles ({d}/{t})"))
            break
        except RuntimeError as e:
            if z_e > 12 and ("HTTP 403" in str(e) or "HTTP 404" in str(e)):
                z_e -= 1
                print(f"  that zoom isn't available here, trying zoom {z_e}")
            else:
                raise
    elev = decode_terrarium(ecanvas).astype(np.float32)
    if args.clip_sea:
        elev = np.maximum(elev, 0)
    img_canvas = None
    if args.colours >= 2:
        img_canvas, org_i, z_i = fetch_imagery(fr, aabb, args)

    progress(45, "Building the height map")
    # ---- elevation at the mesh vertices, through the (possibly rotated) frame
    E = np.asarray(warp(Image.fromarray(elev), fr, z_e, org_e, nxc, nyc, True, 1.5, Image.BICUBIC),
                   dtype=np.float32)
    # Source elevation pixels are bigger than grid cells when detail is high; smooth in proportion so
    # source-pixel stair-steps don't show up as ripples. --smooth overrides this.
    up = (2 * HALF_WORLD / (TILE * 2 ** z_e)) / (fr.W / nxc)       # grid cells per source pixel
    sigma = args.smooth if args.smooth is not None else min(max(0.8, 0.4 * up), 4.0)
    E = gauss_smooth(E, sigma)

    if args.lidar != "none":
        args._nxc, args._nyc = nxc, nyc
        la_c, lo_c = from_merc(fr.xc, fr.yc)
        if not (lidar_in_range(args.lidar, la_c, lo_c) or args.synthetic):
            print("LiDAR: this area is outside the source's coverage (England), so the global elevation data is used.")
        else:
            progress(46, "Looking for LiDAR data")
            try:
                lc, lz, lorg, lpx = fetch_lidar(args.lidar, fr, args, on_block=lambda d, t: progress(
                    46 + 8 * d / t, f"Downloading LiDAR ({d}/{t} blocks)"))
            except RuntimeError as e:
                print(f"LiDAR unavailable ({e}). The global elevation data is used instead.")
            else:
                valid = np.isfinite(lc)
                Ev = np.asarray(warp(Image.fromarray(np.where(valid, lc, 0).astype(np.float32)), fr, lz, lorg, nxc, nyc,
                                     True, 1.5, Image.BICUBIC), np.float32)
                Wv = np.asarray(warp(Image.fromarray(valid.astype(np.float32)), fr, lz, lorg, nxc, nyc,
                                     True, 1.5, Image.BICUBIC), np.float32)
                ok = Wv > 0.85
                if ok.mean() < 0.02:
                    print("LiDAR: no data covers this area, so the global elevation data is used.")
                else:
                    El = Ev / np.maximum(Wv, 1e-3)
                    off = float(np.median(El[ok] - E[ok]))          # the two datasets use slightly different height references
                    El = np.where(ok, El, E + off)
                    upl = (2 * HALF_WORLD / (TILE * 2 ** lz)) / (fr.W / nxc)
                    sig_l = args.smooth if args.smooth is not None else min(max(0.6, 0.4 * upl), 4.0)
                    E = gauss_smooth(El, sig_l)
                    sigma = sig_l
                    if args.clip_sea:
                        E = np.maximum(E, 0)
                    print(f"LiDAR: {LIDAR_SOURCES[args.lidar]['label']}; {ok.mean():.0%} of the area has LiDAR "
                          f"({lpx:.1f} m pixels)" + ("" if ok.mean() > 0.98 else ", the rest uses global data")
                          + f"; height offset between datasets {off:+.1f} m")
                    print(LIDAR_SOURCES[args.lidar]["attribution"])

    sea_m = None
    if args.sea:
        Ec = (E[:-1, :-1] + E[1:, :-1] + E[:-1, 1:] + E[1:, 1:]) / 4
        sea_m = Ec <= 0.5
        E = np.maximum(E, 0)
    s = args.size / longest_m                       # mm per metre
    E0 = float(args.base_elev) if args.base_elev is not None else float(E.min())
    Z = args.base + (E - E0) * s * args.vertical_exaggeration
    if Z.min() < 0.8:
        print(f"Note: ground lower than the base elevation {E0:.0f} m was raised to a 0.8 mm minimum thickness.")
        Z = np.maximum(Z, 0.8)
    print(f"Model: {longest_m / 1000:.2f} km across, relief {E.max() - E.min():.0f} m, "
          f"scale 1:{1000 / s:,.0f}, base elevation {E0:.0f} m")
    cell_m = longest_m / max(nxc, nyc)
    print(f"Grid: {nxc} x {nyc} cells; each is {cell_mm:.2f} mm on the model = {cell_m:.1f} m on the ground "
          f"(smoothing {sigma:.1f} cells)")
    if cell_m < 8:
        print("Note: cells are finer than typical elevation data (about 10-30 m), so extra terrain detail is "
              "interpolated; colours, roads and buildings still gain detail.")
    if cell_mm < 0.2:
        print("Note: cells are under 0.2 mm, finer than most FDM printers can reproduce (fine for resin).")

    # ---- OpenStreetMap features: water, rivers, buildings
    water_m = np.zeros((nyc, nxc), bool) if sea_m is None else sea_m.copy()
    river_m = np.zeros((nyc, nxc), bool)
    road_m = np.zeros((nyc, nxc), bool)
    path_m = np.zeros((nyc, nxc), bool)
    bld_h = np.zeros((nyc, nxc), np.float32)
    want_bld = args.buildings
    cell_m = longest_m / max(nxc, nyc)
    if want_bld and cell_m > args.max_building_cell_m:
        need = int(longest_m / args.max_building_cell_m) + 1
        print(f"Buildings skipped: each grid cell covers {cell_m:.0f} m on the ground, larger than most buildings. "
              f"Raise detail to about {need} cells, or use a smaller area.")
        want_bld = False
    if args.water or args.rivers or args.roads or args.paths or want_bld:
        progress(56, "Fetching map features (water, roads, buildings)")
        if args.synthetic:
            feats = synthetic_osm(ref_bbox)
        else:
            print("Fetching OpenStreetMap data (water / rivers / buildings)...")
            feats = parse_osm(fetch_osm(aabb, args.water, args.rivers, want_bld, args.cache_dir,
                                        roads=args.roads, paths=args.paths))
        if args.water and feats["water"]:
            water_m |= raster_polys(feats["water"], fr, nxc, nyc)
        if args.rivers and feats["rivers"]:
            river_m = raster_lines(feats["rivers"], fr, nxc, nyc, args.river_width / cell_mm)
        if args.roads and feats["roads"]:
            roads = [r for r in feats["roads"] if r[0] in ROAD_CLASSES]
            road_m = raster_roads(roads, fr, nxc, nyc, args.road_width / cell_mm)
        if args.paths and feats["roads"]:
            paths = [r for r in feats["roads"] if r[0] in PATH_CLASSES]
            path_m = raster_roads(paths, fr, nxc, nyc, args.path_width / cell_mm)
        if want_bld and feats["buildings"]:
            bld_h = raster_building_heights(feats["buildings"], fr, nxc, nyc,
                                            s * args.vertical_exaggeration, args.building_min_height)
        print(f"Found: {len(feats['water'])} water bodies, {len(feats['rivers'])} waterways, "
              f"{len(feats['buildings'])} buildings, {len(feats['roads'])} roads")
    bld_m = bld_h > 0

    # ---- terrain colours
    use_colour = args.colours >= 2
    progress(60, "Working out the colours")
    if use_colour:
        im = warp(img_canvas, fr, z_i, org_i, nxc, nyc, False, 1.0, Image.LANCZOS)
        idx, palette = quantise(im, args.colours, args.clean)
        if args.scheme != "satellite":
            Ec_ = (E[:-1, :-1] + E[1:, :-1] + E[:-1, 1:] + E[1:, 1:]) / 4
            palette = apply_scheme(idx, palette, Ec_, args.scheme)
            print(f"Colour scheme: {SCHEMES[args.scheme]['label']}")
    else:
        idx, palette = np.zeros((nyc, nxc), np.int32), np.array([[170, 170, 170]], np.uint8)
    n_terrain = len(palette)

    # layers painted on top of the terrain colours, in order
    if use_colour:
        for mask, rgb in ((water_m, parse_hex(args.water_colour)), (river_m, parse_hex(args.river_colour)),
                          (path_m, parse_hex(args.path_colour)), (road_m, parse_hex(args.road_colour)), (bld_m, parse_hex(args.building_colour))):
            if mask.any():
                rgb = np.array(rgb, np.uint8)
                hit = np.nonzero((palette == rgb).all(1))[0]       # same colour as an existing layer -> same part
                if len(hit):
                    k = int(hit[0])
                else:
                    palette = np.vstack([palette, rgb])
                    k = len(palette) - 1
                idx = np.where(mask, k, idx)

    progress(66, "Shaping the terrain")
    # ---- heights: recessed water, carved rivers, raised buildings
    floor = max(1.0, args.base * 0.5)
    if water_m.any() and args.water_depth > 0:
        Wp = np.pad(water_m, 1, mode="edge")
        deep = Wp[:-1, :-1] & Wp[:-1, 1:] & Wp[1:, :-1] & Wp[1:, 1:]      # vertices surrounded by water
        Z = np.maximum(Z - deep * args.water_depth, floor)
    if river_m.any() and args.river_depth > 0:
        Rp = np.pad(river_m, 1)
        touch = Rp[:-1, :-1] | Rp[:-1, 1:] | Rp[1:, :-1] | Rp[1:, 1:]
        Z = np.maximum(Z - touch * args.river_depth, floor)
    lift_m = road_m | path_m
    if lift_m.any() and args.road_raise > 0:
        Kp = np.pad(lift_m, 1)
        touch = Kp[:-1, :-1] | Kp[:-1, 1:] | Kp[1:, :-1] | Kp[1:, 1:]
        Z = Z + touch * args.road_raise
    if bld_m.any():
        Hp = np.pad(bld_h, 1)
        Z = Z + np.maximum.reduce([Hp[:-1, :-1], Hp[:-1, 1:], Hp[1:, :-1], Hp[1:, 1:]])

    trail = None
    if not args.no_trail and tracks:
        trail = rasterise_trail(tracks, fr, (nxc, nyc), args.trail_width / cell_mm)
        if use_colour:
            palette = np.vstack([palette, np.array(parse_hex(args.trail_colour), np.uint8)])
            idx = np.where(trail, len(palette) - 1, idx)
        if args.trail_raise > 0:
            Pm = np.pad(trail, 1)
            V = Pm[:-1, :-1] | Pm[:-1, 1:] | Pm[1:, :-1] | Pm[1:, 1:]   # vertices touching a trail cell
            Z = Z + V * args.trail_raise

    # ---- points of interest: flatten a small plateau under each, then stand a stylised model on it
    poi_meshes = []
    if pois:
        xs_v = np.arange(nx) * cell_mm
        ys_v = (ny - 1 - np.arange(ny)) * cell_mm
        for p in pois:
            fx, fy = _cells([(p["lat"], p["lon"])], fr, nxc, nyc)[0]
            rad = POI_RADIUS[p["type"]] * p["size"]
            cx = min(max(fx * cell_mm, rad), nxc * cell_mm - rad)
            cy = min(max((nyc - fy) * cell_mm, rad), nyc * cell_mm - rad)
            region = (np.abs(ys_v - cy) <= rad + cell_mm)[:, None] & (np.abs(xs_v - cx) <= rad + cell_mm)[None, :]
            z0 = float(np.median(Z[region]))
            Z[region] = z0
            v, t = build_poi(p["type"], p["size"])
            k = 0
            if use_colour:
                rgb = np.array(p["rgb"], np.uint8)
                hit = np.nonzero((palette == rgb).all(1))[0]
                if len(hit):
                    k = int(hit[0])
                else:
                    palette = np.vstack([palette, rgb])
                    k = len(palette) - 1
            poi_meshes.append((v + np.array([cx, cy, z0]), t, k))
        print(f"Placed {len(poi_meshes)} point(s) of interest (stylised markers, not to map scale)")

    progress(72, "Building the 3D mesh")
    # ---- mesh
    allv = np.zeros((2 * nx * ny, 3), np.float64)
    xs = np.arange(nx) * cell_mm
    ys = (ny - 1 - np.arange(ny)) * cell_mm
    X, Y = np.meshgrid(xs, ys)
    allv[:nx * ny] = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
    allv[nx * ny:] = np.stack([X.ravel(), Y.ravel(), np.zeros(nx * ny)], 1)

    full_mask = lay.mask
    tris, cell_ids = build_part(full_mask, nx, ny)
    verts, ctris = compact(allv, tris)
    tri_cidx = idx.ravel()[cell_ids]
    if poi_meshes:
        off, vs, ts, cs = len(verts), [verts], [ctris], [tri_cidx]
        for v, t, k in poi_meshes:
            vs.append(v)
            ts.append(t + off)
            cs.append(np.full(len(t), k))
            off += len(v)
        verts, ctris, tri_cidx = np.vstack(vs), np.vstack(ts), np.concatenate(cs)
    colours = palette[tri_cidx] if (use_colour and not args.no_colour_in_stl) else None
    progress(80, "Writing the STL file")
    write_stl(out, verts, ctris, colours)
    print(f"Wrote {out}  ({len(ctris):,} triangles, "
          f"{xs.max():.0f} x {ys.max():.0f} x {Z.max():.1f} mm)")

    VIEW_TRIS = 1_000_000
    if args.view_mesh and len(ctris) > VIEW_TRIS:
        # Same model on a coarser grid (points of interest, fillet and colours included), so the browser
        # can show a 3D view without downloading or drawing millions of triangles.
        f = int(math.ceil(math.sqrt(len(ctris) / VIEW_TRIS)))
        nxp, nyp = max(2, round(nxc / f)), max(2, round(nyc / f))
        xi = np.linspace(0, nxc, nxp + 1).round().astype(int)
        yi = np.linspace(0, nyc, nyp + 1).round().astype(int)
        cx = np.minimum((xi[:-1] + xi[1:]) // 2, nxc - 1)
        cy = np.minimum((yi[:-1] + yi[1:]) // 2, nyc - 1)
        idx_p, mask_p = idx[np.ix_(cy, cx)], full_mask[np.ix_(cy, cx)]
        Xp, Yp = np.meshgrid(xs[xi], ys[yi])
        npv = (nxp + 1) * (nyp + 1)
        pv = np.zeros((2 * npv, 3))
        pv[:npv] = np.stack([Xp.ravel(), Yp.ravel(), Z[np.ix_(yi, xi)].ravel()], 1)
        pv[npv:] = np.stack([Xp.ravel(), Yp.ravel(), np.zeros(npv)], 1)
        pt, pcell = build_part(mask_p, nxp + 1, nyp + 1)
        pverts, ptris = compact(pv, pt)
        pcol = idx_p.ravel()[pcell]
        if poi_meshes:
            off, vs, ts, cs = len(pverts), [pverts], [ptris], [pcol]
            for v, t, k in poi_meshes:
                vs.append(v)
                ts.append(t + off)
                cs.append(np.full(len(t), k))
                off += len(v)
            pverts, ptris, pcol = np.vstack(vs), np.vstack(ts), np.concatenate(cs)
        progress(88, "Making the lighter 3D view")
        vpath = out.with_name(out.stem + "_view.stl")
        write_stl(vpath, pverts, ptris, palette[pcol] if use_colour else None)
        print(f"Wrote {vpath}  (lighter preview mesh, {len(ptris):,} triangles; use the full model for printing)")

    progress(91, "Writing colour files")
    if args.threemf and use_colour:
        p3 = out.with_suffix(".3mf")
        write_3mf(p3, verts, ctris, tri_cidx, palette)
        print(f"Wrote {p3}")
    elif args.threemf:
        print("Skipped 3MF: it only adds value when --colours is 2 or more.")

    if (args.split or args.threemf_parts) and use_colour:
        parts = []
        skin = args.skin > 0
        if skin:
            # colour layers are only the top `skin` mm; one plain base (in the most common colour, so one filament
            # does both jobs) fills the rest. Both use the same vertices, so the faces meet exactly.
            Zb = np.minimum(np.maximum(Z - args.skin, 0.4), Z - 0.2)
            skin_v = allv.copy()
            skin_v[nx * ny:, 2] = Zb.ravel()                 # colour layer: floor follows the surface down
            base_v = allv.copy()
            base_v[:nx * ny, 2] = Zb.ravel()                 # base: top surface is that same floor
            area = np.array([((idx == k) & full_mask).sum() for k in range(len(palette))])
            kb = int(area.argmax())
            tb, _ = build_part(full_mask, nx, ny)
            vb, cb = compact(base_v, tb)
            hxb = "%02x%02x%02x" % tuple(palette[kb])
            if args.split:
                pb = out.with_name(f"{out.stem}_base_{hxb}.stl")
                write_stl(pb, vb, cb)
                print(f"Wrote {pb}  (plain base, use the same filament as colour #{hxb})")
            parts.append((f"Base #{hxb}", vb, cb, tuple(int(c) for c in palette[kb])))
        for k in range(len(palette)):
            m = (idx == k) & full_mask
            pm = [(v, t) for v, t, kk in poi_meshes if kk == k]
            if not m.any() and not pm:
                continue
            vs, ts, off = [], [], 0
            if m.any():
                t, _ = build_part(m, nx, ny)
                v, ct = compact(skin_v if skin else allv, t)
                vs.append(v)
                ts.append(ct)
                off = len(v)
            for v, t in pm:
                vs.append(v)
                ts.append(t + off)
                off += len(v)
            v, ct = np.vstack(vs), np.vstack(ts)
            hx = "%02x%02x%02x" % tuple(palette[k])
            note = f"{m.sum() / full_mask.sum():.0%} of surface" + (f" + {len(pm)} marker(s)" if pm else "") \
                + (f", {args.skin:g} mm layer" if skin else "")
            if args.split:
                p = out.with_name(f"{out.stem}_part{k + 1}_{hx}.stl")
                write_stl(p, v, ct)
                print(f"Wrote {p}  ({note})")
            parts.append((f"Colour {k + 1} #{hx}", v, ct, tuple(int(c) for c in palette[k])))
        if args.threemf_parts:
            pp = out.with_name(out.stem + "_parts.3mf")
            write_3mf_parts(pp, parts)
            print(f"Wrote {pp}  ({len(parts)} parts)")
    elif args.split or args.threemf_parts:
        print("Skipped --split / --3mf-parts: they need --colours 2 or more.")

    if args.surface_obj:
        progress(94, "Writing the surface-only OBJ")
        write_surface_obj(out.with_suffix(".obj"), allv[:nx * ny], full_mask, idx if use_colour else None,
                          palette, nx, ny)

    if args.minecraft != "none":
        progress(95, "Building the Minecraft map")
        sys.path.insert(0, str(Path(__file__).resolve().parent))      # find mc_export.py next to this file
        try:
            import mc_export
        except ImportError:
            sys.exit("mc_export.py is missing. Put it in the same folder as gpx2stl.py to use the Minecraft export.")
        water_all = water_m | river_m
        mc_export.export(args, out, dict(
            idx=idx, palette=palette, n_terrain=n_terrain, mask=full_mask, E=E, E0=E0, water=water_all,
            bld_mm=bld_h, s_exag=s * args.vertical_exaggeration, longest_m=longest_m,
            exaggeration=args.vertical_exaggeration, scheme=args.scheme))

    progress(97, "Drawing the colour map")
    prev = out.with_name(out.stem + "_preview.png")
    Zc = (Z[:-1, :-1] + Z[1:, :-1] + Z[:-1, 1:] + Z[1:, 1:]) / 4
    write_preview(prev, idx, palette, Zc, scale=max(1, min(8, 1600 // max(nxc, nyc))), footprint=full_mask, theta=lay.theta)
    print(f"Wrote {prev}")
    print("Palette: " + ", ".join("#%02x%02x%02x" % tuple(c) for c in palette))

    # ---- tile data: everything needed to make the tile next door
    clat, clon = from_merc(fr.xc, fr.yc)
    info = dict(centre=dict(lat=round(clat, 6), lon=round(clon, 6)), width_m=round(fr.W_g, 2), height_m=round(fr.H_g, 2),
                ratio=round(fr.W_g / fr.H_g, 5), rotation=round(lay.theta, 3), shape=args.shape,
                corner_style=lay.style, corner_size=args.corner_size if lay.style != "sharp" else 0,
                size_mm=args.size, resolution=args.resolution, vertical_exaggeration=args.vertical_exaggeration,
                base_mm=args.base, base_elev_m=round(E0, 2), min_elev_m=round(float(E.min()), 2),
                max_elev_m=round(float(E.max()), 2), lidar=args.lidar, neighbours=neighbours(fr, args.shape))
    out.with_name(out.stem + "_tile.json").write_text(json.dumps(info, indent=1))
    progress(100, "Done")


if __name__ == "__main__":
    main()
