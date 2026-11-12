#!/usr/bin/env python3
"""
Local web front-end for gpx2stl.

    pip install -r requirements.txt
    python app.py            ->  open http://127.0.0.1:5000

Upload a GPX, pick options, get a 3D preview plus STL / 3MF / per-colour STL downloads.
Runs on your own machine (it needs internet access to fetch elevation + satellite tiles).
"""
import json
import re
import shutil
import os
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path
from types import SimpleNamespace

from flask import Flask, abort, jsonify, request, send_from_directory

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE))       # so gpx2stl can be imported for the layout preview
JOBS = BASE / "jobs"
JOBS.mkdir(exist_ok=True)
STATE = {}                    # job id -> progress / result, read by /api/status
STATE_LOCK = threading.Lock()

POI_TYPES = {"castle", "house", "tower", "church", "flag", "pin"}

app = Flask(__name__, static_folder=str(BASE / "static"), static_url_path="/static")
app.config["MAX_CONTENT_LENGTH"] = 20 * 1024 * 1024  # 20 MB GPX limit


def num(form, name, default, lo, hi, cast=float):
    raw = form.get(name, "")
    if raw == "":
        return default
    try:
        v = cast(raw)
    except ValueError:
        abort(400, f"Invalid value for {name}")
    return min(max(v, lo), hi)


def shape_args(form):
    """Validated shape / framing options shared by /api/generate and /api/plan."""
    shape = form.get("shape", "rectangle")
    if shape not in ("rectangle", "hexagon", "circle"):
        abort(400, "Invalid shape")
    ratio = (form.get("ratio", "auto") or "auto").strip().lower()[:12]
    if not re.fullmatch(r"auto|\d+(\.\d+)?(:\d+(\.\d+)?)?", ratio):
        abort(400, "Invalid ratio. Use auto, or something like 3:2")
    rotate = (form.get("rotate", "0") or "0").strip().lower()
    if rotate != "auto":
        try:
            rotate = str(min(max(float(rotate), -360.0), 360.0))
        except ValueError:
            abort(400, "Invalid rotation")
    style = form.get("corner_style", "sharp")
    if style not in ("sharp", "fillet", "chamfer"):
        abort(400, "Invalid corner style")
    centre, width_m = None, None
    if form.get("centre", "").strip():
        try:
            la, lo = (float(v) for v in form["centre"].replace(";", ",").split(","))
            if not (-85 <= la <= 85 and -180 <= lo <= 180):
                raise ValueError
        except ValueError:
            abort(400, "Centre must be latitude,longitude, e.g. 53.07,-4.08")
        centre = (la, lo)
        width_m = num(form, "width_m", 3000, 100, 200000)
    return dict(shape=shape, ratio=ratio, portrait=form.get("portrait") in ("1", "true", "on"),
                rotate=rotate, corner_style=style, corner_size=num(form, "corner_size", 0, 0, 100),
                centre=centre, width_m=width_m)


def cleanup_old_jobs(max_age_s=24 * 3600):
    now = time.time()
    for d in JOBS.iterdir():
        if d.is_dir() and now - d.stat().st_mtime > max_age_s:
            shutil.rmtree(d, ignore_errors=True)
            with STATE_LOCK:
                STATE.pop(d.name, None)


def find_index():
    """index.html normally lives in static/, but accept it next to app.py too."""
    for d in (BASE / "static", BASE):
        if (d / "index.html").is_file():
            return d
    return None


@app.get("/")
def index():
    d = find_index()
    if d is None:
        return ("index.html not found. Put it in a folder called 'static' next to app.py "
                f"(looked in {BASE / 'static'} and {BASE}).", 500)
    return send_from_directory(d, "index.html")


@app.get("/api/schemes")
def schemes():
    import gpx2stl as g
    return jsonify({k: dict(label=v["label"], ramp=v["ramp"],
                            colours={c: v[c] for c in ("water", "river", "road", "path", "building", "route")})
                    for k, v in g.SCHEMES.items()})


@app.get("/api/lidar-sources")
def lidar_sources():
    import gpx2stl as g
    return jsonify([dict(key=k, label=v["label"], attribution=v.get("attribution", "")) for k, v in g.LIDAR_SOURCES.items()])


@app.post("/api/lidar-check")
def lidar_check():
    import gpx2stl as g
    key = request.form.get("source", "ea-dtm-1m")
    try:
        lat, lon = float(request.form["lat"]), float(request.form["lon"])
    except (KeyError, ValueError):
        abort(400, "Need a latitude and longitude")
    if key not in g.LIDAR_SOURCES:
        abort(400, "Unknown LiDAR source")
    return jsonify(g.lidar_check(key, lat, lon, str(Path.home() / ".cache" / "gpx2stl")))


def _run_job(job_id, cmd, job):
    """Runs the generator in the background, keeping percentage + message up to date for /api/status."""
    st = STATE[job_id]
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                cwd=str(BASE), env=env)
    except OSError as e:
        with STATE_LOCK:
            st.update(state="error", error=f"Could not start the generator: {e}")
        return
    st["proc"] = proc
    timer = threading.Timer(1200, proc.kill)
    timer.start()
    lines = []
    for line in proc.stdout:
        line = line.rstrip("\n")
        m = re.match(r"@@PROGRESS (\d+) (.*)", line)
        with STATE_LOCK:
            if m:
                st["pct"], st["message"] = int(m.group(1)), m.group(2)
            else:
                lines.append(line)
                st["log"] = "\n".join(lines[-60:])
    code = proc.wait()
    timer.cancel()
    log = "\n".join(lines)
    with STATE_LOCK:
        if st.get("cancelled"):
            st.update(state="cancelled", log=log[-2000:])
            return
        if code != 0:
            err = "Timed out after 20 minutes. Try a lower detail setting." if code == -9 else "Generation failed"
            st.update(state="error", error=err, log=log[-2000:])
            return
    files = []
    for p in sorted(job.iterdir()):
        if p.name in ("input.gpx", "terrain_tile.json"):
            continue
        kind = ("model" if p.name == "terrain.stl" else
                "viewmesh" if p.name == "terrain_view.stl" else
                "part" if p.suffix == ".stl" else
                "3mf" if p.suffix == ".3mf" else
                "preview" if p.suffix == ".png" else "other")
        files.append({"name": p.name, "kind": kind, "size": p.stat().st_size,
                      "url": f"/jobs/{job_id}/{p.name}"})
    tile = None
    try:
        tile = json.loads((job / "terrain_tile.json").read_text())
    except (OSError, ValueError):
        pass
    with STATE_LOCK:
        st.update(state="done", pct=100, message="Done", files=files, log=log, tile=tile)


@app.post("/api/generate")
def generate():
    cleanup_old_jobs()
    if not (BASE / "gpx2stl.py").is_file():
        return jsonify(error=f"gpx2stl.py is missing. Put it in the same folder as app.py ({BASE}) "
                             "and try again."), 500
    form = request.form
    sa = shape_args(form)
    f = request.files.get("gpx")
    if f and not f.filename.lower().endswith(".gpx"):
        abort(400, "Please upload a .gpx file")
    if not f and not sa["centre"]:
        abort(400, "Please upload a .gpx file, or enter a centre position")

    job_id = uuid.uuid4().hex[:12]
    job = JOBS / job_id
    job.mkdir()
    cmd = [sys.executable, str(BASE / "gpx2stl.py")]
    if f:
        gpx_path = job / "input.gpx"
        f.save(gpx_path)
        cmd.append(str(gpx_path))

    colours = num(form, "colours", 4, 0, 12, int)
    import gpx2stl as g
    scheme = form.get("scheme", "satellite")
    if scheme not in g.SCHEMES:
        abort(400, "Unknown colour scheme")
    lidar = form.get("lidar", "none")
    if lidar not in g.LIDAR_SOURCES:
        abort(400, "Unknown LiDAR source")
    cmd += [
        "-o", str(job / "terrain.stl"), "--progress", "--scheme", scheme, "--lidar", lidar,
        "--size", str(num(form, "size", 150, 30, 400)),
        "--resolution", str(num(form, "resolution", 400, 40, 1500, int)),
        "--colours", str(colours),
        "--vertical-exaggeration", str(num(form, "exaggeration", 1.5, 0.5, 6)),
        "--base", str(num(form, "base", 4, 1, 20)),
        "--water-depth", str(num(form, "water_depth", 0.8, 0, 5)),
        "--river-width", str(num(form, "river_width", 1.0, 0.4, 5)),
        "--road-width", str(num(form, "road_width", 0.9, 0.4, 4)),
        "--path-width", str(num(form, "path_width", 0.6, 0.3, 3)),
        "--view-mesh",
        "--pad", str(num(form, "pad", 0.15, 0, 1)),
        "--trail-width", str(num(form, "trail_width", 1.6, 0.4, 6)),
        "--trail-raise", str(num(form, "trail_raise", 0.6, 0, 3)),
    ]
    for field, flag, default in (("trail_colour", "--trail-colour", "#e8202a"),
                                 ("water_colour", "--water-colour", "#3a78c2"),
                                 ("river_colour", "--river-colour", "#3a78c2"),
                                 ("road_colour", "--road-colour", "#8a8a8a"),
                                 ("path_colour", "--path-colour", "#a0764a"),
                                 ("building_colour", "--building-colour", "#d6cfc4")):
        c = form.get(field, default)
        if re.fullmatch(r"#[0-9a-fA-F]{6}", c):
            cmd += [flag, c]
    # points of interest: JSON list of {type, location, colour, size}
    try:
        pois = json.loads(form.get("pois", "[]"))
    except ValueError:
        pois = []
    for p in (pois if isinstance(pois, list) else [])[:10]:
        if not isinstance(p, dict) or p.get("type") not in POI_TYPES:
            continue
        loc = re.sub(r"[|\x00-\x1f]", " ", str(p.get("location", ""))).strip()[:120]
        if not loc:
            continue
        colour = p.get("colour", "") if re.fullmatch(r"#[0-9a-fA-F]{6}", str(p.get("colour", ""))) else ""
        try:
            size = min(max(float(p.get("size", 1)), 0.4), 4.0)
        except (TypeError, ValueError):
            size = 1.0
        cmd += ["--poi", f"{p['type']}|{loc}|{colour}|{size}"]
    if form.get("waypoint_poi") in POI_TYPES and f:
        cmd += ["--waypoint-poi", form["waypoint_poi"]]

    cmd += ["--shape", sa["shape"], "--ratio", sa["ratio"], "--rotate", sa["rotate"],
            "--corner-style", sa["corner_style"], "--corner-size", str(sa["corner_size"])]
    if sa["portrait"]:
        cmd.append("--portrait")
    if sa["centre"]:
        cmd += ["--centre", "%.6f,%.6f" % sa["centre"], "--width-m", str(sa["width_m"])]
    if form.get("base_elev", "").strip():
        try:
            cmd += ["--base-elev", str(float(form["base_elev"]))]
        except ValueError:
            abort(400, "Base elevation must be a number of metres")

    if form.get("smooth", "") != "":
        cmd += ["--smooth", str(num(form, "smooth", 0.8, 0, 6))]

    for key, flag in (("no_trail", "--no-trail"), ("clip_sea", "--clip-sea"),
                      ("threemf", "--3mf"), ("threemf_parts", "--3mf-parts"),
                      ("water", "--water"), ("rivers", "--rivers"), ("roads", "--roads"), ("paths", "--paths"), ("buildings", "--buildings"), ("sea", "--sea"),
                      ("split", "--split"), ("synthetic", "--synthetic")):
        if form.get(key) in ("1", "true", "on"):
            cmd.append(flag)

    with STATE_LOCK:
        STATE[job_id] = dict(state="running", pct=0, message="Starting", log="", started=time.time(),
                             coloured=colours >= 2)
    threading.Thread(target=_run_job, args=(job_id, cmd, job), daemon=True).start()
    return jsonify(job=job_id)


@app.get("/api/status/<job_id>")
def status(job_id):
    with STATE_LOCK:
        st = STATE.get(job_id)
        if st is None:
            abort(404)
        out = {k: v for k, v in st.items() if k != "proc"}
    out["elapsed"] = round(time.time() - out["started"])
    return jsonify(out)


@app.post("/api/cancel/<job_id>")
def cancel(job_id):
    with STATE_LOCK:
        st = STATE.get(job_id)
        if st is None:
            abort(404)
        st["cancelled"] = True
        proc = st.get("proc")
    if proc and proc.poll() is None:
        proc.kill()
    return jsonify(ok=True)


@app.post("/api/plan")
def plan():
    """Footprint preview: size, grid, triangle estimate and outline for the chosen shape, without map data."""
    import numpy as np
    import gpx2stl as g
    form = request.form
    sa = shape_args(form)
    f = request.files.get("gpx")
    tmp = None
    try:
        if f:
            tmp = JOBS / f"plan_{uuid.uuid4().hex[:10]}.gpx"
            f.save(tmp)
            tracks, wpts = g.read_gpx(str(tmp))
        elif sa["centre"]:      # fixed position: no route needed
            tracks, wpts = [], []
        else:        # no file yet: show the shape around an example 3 x 2 km route
            tracks, wpts = [[(51.5, -0.1), (51.5 + 0.018, -0.1 + 0.0433)]], []
    except SystemExit as e:
        abort(400, str(e.code))
    except Exception:
        abort(400, "Could not read that GPX file")
    finally:
        if tmp is not None:
            tmp.unlink(missing_ok=True)

    res = num(form, "resolution", 400, 40, 1500, int)
    o = SimpleNamespace(square=False, pad=num(form, "pad", 0.15, 0, 1), min_span=1500, resolution=res,
                        size=num(form, "size", 150, 30, 400), **sa)
    pts = [p for t in tracks for p in t] + wpts
    try:
        lay = g.build_layout(pts, o, need_mask=False)
        small = g.build_layout(pts, SimpleNamespace(**{**vars(o), "resolution": min(res, 300)}))
    except SystemExit as e:
        abort(400, str(e.code))
    m = small.mask
    P = np.pad(m, 1)
    edges = sum(int((m & ~P[sl]).sum()) for sl in (np.s_[:-2, 1:-1], np.s_[2:, 1:-1], np.s_[1:-1, :-2], np.s_[1:-1, 2:]))
    k = res / max(small.nxc, small.nyc)
    fr = lay.frame

    def norm(pts_):
        step = max(1, len(pts_) // 800)
        return [[round(x, 4), round(y, 4)] for x, y in fr.cells(pts_[::step], 1, 1)]
    clat, clon = g.from_merc(fr.xc, fr.yc)
    return jsonify(
        lidar_range={k: g.lidar_in_range(k, clat, clon) for k in g.LIDAR_SOURCES if k != "none"},
        centre=[round(clat, 6), round(clon, 6)], neighbours=g.neighbours(fr, sa["shape"]), fixed=bool(sa["centre"]),
        theta=lay.theta, auto=lay.auto, nominal=(f is None and not sa["centre"]), W_m=fr.W_g, H_m=fr.H_g,
        nxc=lay.nxc, nyc=lay.nyc, cell_mm=lay.cell_mm, cell_m=max(fr.W_g, fr.H_g) / max(lay.nxc, lay.nyc),
        size_mm=[lay.W_mm, lay.H_mm], tris=int(4 * m.sum() * k * k + 2 * edges * k),
        outline=[[round(x / lay.W_mm, 4), round(y / lay.H_mm, 4)] for x, y in lay.poly],
        route=[norm(t) for t in tracks], waypoints=norm(wpts) if wpts else [])


@app.get("/jobs/<job_id>/<name>")
def job_file(job_id, name):
    if not re.fullmatch(r"[0-9a-f]{12}", job_id) or not re.fullmatch(r"[\w.\-]+", name):
        abort(404)
    return send_from_directory(JOBS / job_id, name, as_attachment=request.args.get("dl") == "1")


@app.errorhandler(400)
def bad_request(e):
    return jsonify(error=getattr(e, "description", "Bad request")), 400


if __name__ == "__main__":
    print("Open http://127.0.0.1:5000")
    app.run(host="127.0.0.1", port=5000, debug=False)
