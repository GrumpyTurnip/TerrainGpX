# gpx2stl

Turn a GPX file (track, route or waypoints) into a 3D-printable terrain model.

- Elevation: AWS Terrain Tiles (free, no key)
- Colours: k-means clustering of Esri World Imagery satellite tiles (free, no key; check Esri's terms for anything beyond personal use)
- Output: watertight solid STL, optional coloured 3MF, optional one-STL-per-colour for multi-material printing

## Install
    pip install -r requirements.txt

## Use
    python gpx2stl.py route.gpx -o model.stl                 # 4 colours (default)
    python gpx2stl.py route.gpx -o model.stl --colours 0     # plain, no colour
    python gpx2stl.py route.gpx -o model.stl --colours 6 --3mf --split
    python gpx2stl.py route.gpx --size 200 --resolution 300 --vertical-exaggeration 2 --square

Key options: `--colours N` (0/1 = none), `--size` (mm, longest side), `--resolution` (grid cells; more = finer,
bigger file), `--vertical-exaggeration`, `--base` (mm), `--pad`, `--square`, `--no-trail`, `--trail-colour`,
`--trail-raise`, `--clip-sea`. Run `-h` for all. A `<name>_preview.png` is always written so you can check
the colour map before printing.

## Colour outputs
- **STL colour attributes** (default when colours >= 2): per-triangle colour in the VisCAM/SolidView style; only
  some viewers read it. Use `--no-colour-in-stl` for a plain STL.
- **3MF** (`--3mf`): per-triangle colours via the 3MF materials extension; the most portable colour format.
- **`--split`**: one closed solid per colour, sharing exact boundaries. Import all parts into your slicer as one
  object and assign a filament to each. Parts run from the base to the surface, so colour changes go through
  the full height. Parts can touch diagonally at single edges; slicers handle this, but run a repair if yours complains.

## Notes
- Data is cached in `~/.cache/gpx2stl`. `--synthetic` makes fake terrain for offline testing only.

## Web app
    pip install -r requirements.txt
    python app.py        # then open http://127.0.0.1:5000

Upload a GPX, adjust the sliders, and get an interactive 3D preview (drag to rotate, scroll to zoom), a colour
map tab, and downloads. It runs only on your machine and needs internet for the tiles (the 3D viewer also loads
three.js from a CDN). Tick "Demo mode" to test without internet.

## Shape, angle, ratio and corners
- `--shape rectangle|hexagon`: the footprint of the model. The hexagon is flat-top (flat edges at top and bottom); rotate by 30 or 90 for a pointy-top one.
- `--ratio auto|1:1|3:2|16:9|W:H` (default auto = fits your route); `--portrait` swaps it; `--square` is the same as `--ratio 1:1`.
  A hexagon with ratio auto is regular; other ratios stretch it.
- `--rotate DEGREES|auto`: the compass bearing that faces the top edge of the model (0 = north up, 90 = east up).
  `auto` picks the angle that fits the route in the smallest area. The preview PNG and the web outline show a north arrow.
- `--corner-style sharp|fillet|chamfer` with `--corner-size MM`: rounded or cut corners, on both rectangle and hexagon.
  `--corner-radius MM` still works (same as fillet). Edges are stepped at the size of one grid cell, so raise `--resolution` for smoother curves.
- The web page has a live outline preview of the footprint, route and north arrow, plus the exact size, grid and triangle estimate.

## Large models and the 3D view
- A small area at a high `--resolution` gives a huge file (1.5 km at 1200 cells is about 5.8 million triangles, 290 MB). Cells finer than the
  elevation data (10-30 m) only sharpen colours, roads and buildings.
- `--view-mesh` also writes `<name>_view.stl`, a lighter copy when the model has over 1 million triangles. The web page shows it in the 3D view;
  downloads are always full detail.

## Circle, fixed position and neighbouring tiles
- `--shape circle` (an oval with `--ratio`). Circles have no corners.
- `--centre LAT,LON --width-m METRES` fixes the area instead of fitting it to the route; the GPX file is then optional
  (it is only used to draw the route and markers that fall inside). `--base-elev M` sets the elevation that sits at the top of the base.
- Every run writes `<name>_tile.json` with the centre, ground size, rotation, ratio, base elevation and the centres of the
  neighbouring tiles. To make the tile next door, reuse the same `--shape --width-m --ratio --rotate --size --resolution
  --vertical-exaggeration --base-elev` and change only `--centre`. In the web page, the "This tile & its neighbours" buttons fill all of that in.
  Rectangles and hexagons tile exactly; circles only touch. Corner rounding or chamfering leaves small gaps at the joins.
  Each tile smooths its own terrain, so a very faint seam is possible at the join.

## Colour schemes
`--scheme satellite|topo|alpine|desert|autumn|forest|sunset|blueprint|clay`. Satellite keeps true colours. The others keep the imagery's
grouping of areas but repaint each group along a palette by average height (lowest group first). The web page also applies matching colours
for water, rivers, roads, paths, buildings and the route when you pick one; you can still change them.

## Progress
The web page shows a percentage bar, the current step (including tile counts while downloading), elapsed time and a Cancel button.
The generator prints `@@PROGRESS pct message` lines when run with `--progress` (used by the page; hidden from normal use).
Jobs run in the background (`/api/generate` returns a job id; `/api/status/<id>` and `/api/cancel/<id>`).

## LiDAR terrain (England)
`--lidar ea-dtm-1m` uses the Environment Agency's national 1 m LiDAR terrain (DTM, bare ground without buildings or trees) instead of the
global elevation data. It covers roughly 99% of England, is free under the Open Government Licence v3.0, and is read over the agency's WCS service
(environment.data.gov.uk). In the web page it is the "Elevation data" drop-down in step 3, with a "Check this area" button.
- It is requested at a pixel size matched to your model (never finer than 1 m, never coarser than 8 m), capped at 16 million pixels.
- Where LiDAR has gaps (sea, outside England, unsurveyed patches) the global data fills in, shifted to the same height reference.
- LiDAR only shows when cells are small: under about 5 m on the ground (for example 1.5 km at 400 cells, or 3 km at 700). Large areas look the same.
- If the service can't be reached the generator says so and carries on with global data.
- `python gpx2stl.py --lidar-check 52.2053,0.1218` tests the service and the point without making a model.
- Attribution: Contains Environment Agency information (c) Environment Agency and/or database right 2022.
- More regions (Wales, Scotland) can be added as extra entries in `LIDAR_SOURCES`.

## Web resources used
- Elevation: AWS Terrain Tiles (s3.amazonaws.com/elevation-tiles-prod/terrarium); optional LiDAR from environment.data.gov.uk (England)
- Imagery: Esri World Imagery (server.arcgisonline.com)
- 3D preview in the web app: three.js from cdnjs.cloudflare.com and cdn.jsdelivr.net

## Per-colour outputs (summary)
- `--split` / "One STL file per colour": a separate closed STL for each colour, all aligned in the same coordinates.
- `--3mf-parts` / "3MF with each colour as a separate part": one 3MF file holding every colour as its own part of a
  single object, named by colour. Open it in Bambu Studio, OrcaSlicer or PrusaSlicer and assign a filament per part.
- `--3mf`: one object with per-triangle painted colours (for viewers that read 3MF colours).

## Water, rivers and buildings (OpenStreetMap)
- `--water`: lakes and ponds, coloured and recessed by `--water-depth` mm.
- `--rivers`: rivers, canals and streams as coloured channels (`--river-width`, `--river-depth`).
- `--buildings`: building footprints raised to their real height (from OSM `height` / `building:levels`, default 7 m,
  scaled like the terrain, minimum `--building-min-height` mm). Skipped automatically when one grid cell covers more than `--max-building-cell-m` metres (default 20), because
  buildings would be smaller than a cell; the message tells you what `--resolution` to use, or pick a smaller area.
- `--sea`: treats anything at or below 0 m elevation as sea. Off by default because some inland areas are below sea level.
- Colours are `--water-colour` and `--building-colour`; they become their own parts in `--split` / `--3mf-parts`.
- Data: Overpass API (overpass-api.de, falling back to overpass.kumi.systems), cached locally.
  Map data (c) OpenStreetMap contributors, ODbL. OSM coverage varies by region.

## Roads and per-feature colours
- `--roads`: OpenStreetMap roads (motorway down to residential), coloured and raised by `--road-raise` mm.
  Bigger roads are drawn wider than small ones (`--road-width` is the base width). 
- Every layer has its own colour: `--water-colour`, `--river-colour`, `--road-colour`, `--building-colour`, `--trail-colour`
  (hex, e.g. `#ff8800`). In the web app these are the colour swatches next to each toggle.
- Each feature colour becomes its own part in `--split` / `--3mf-parts`. Terrain colours still come from the satellite imagery.

## Footpaths and points of interest
- `--paths` is now its own layer (footpaths, tracks, bridleways, cycleways) with `--path-colour` and `--path-width`;
  `--roads` no longer includes them. `--road-paths` has been removed.
- `--poi "type|place or lat,lon|#colour|size"` (repeatable) adds a marker. Types: castle, house, tower, church, flag, pin.
  Examples: `--poi "castle|Corfe Castle"`, `--poi "house|50.6395,-2.0576|#ff9900|1.3"`.
  Place names are looked up with OpenStreetMap Nominatim (cached; if it finds the wrong place, use lat,lon).
- `--waypoint-poi pin` puts a marker on every waypoint in the GPX file.
- Markers are stylised icons about 3-12 mm tall, not to map scale. The terrain under each is flattened into a small
  plateau so the marker sits flush; the area shown grows to include every marker.
- Each marker colour is a separate part in `--split` / `--3mf-parts`; a marker matching an existing colour joins that part.
- Place lookups: (c) OpenStreetMap contributors, via nominatim.openstreetmap.org.

## Detail and quality
- `--resolution N` sets the grid cells along the longest side; the default is now **400**. Rough guide for a 150 mm model:
  200 = quick draft (0.75 mm cells), 400 = good (0.38 mm), 700 = very fine (0.21 mm), 1000 = maximum useful (0.15 mm, resin printers).
  Triangles are about 4 x cells, so 1000 cells is about 2.7 million triangles and a 130 MB STL. The web page shows a live estimate
  of triangles, file size and metres-per-cell, and has Draft / High / Ultra / Max presets.
- Elevation is sampled exactly at each mesh vertex (previously half a cell off), from the finest tiles available (zoom 15, falling back
  automatically). `--smooth` is automatic by default (it grows with detail so source-data stair-steps do not show as ripples);
  set a number of cells to override, or 0 for raw data.
- The real limit on terrain detail is the elevation data itself (roughly 10-30 m per sample, depending on the region). Beyond about
  8 m per cell, extra detail mostly improves colours, roads, paths, buildings and edges rather than landforms.
- Below about 0.2 mm per cell is finer than most FDM printers can reproduce.
- The web 3D view uses a lighter copy for big models (see above); the downloads stay full detail.
- `--max-elev-zoom` (default 15), `--max-zoom` (imagery, now default **19**, about 0.3 m per pixel) and `--max-tiles` (elevation, default 200) control how fine the source tiles can be.
- Imagery aims for 4 pixels per grid cell (`--imagery-oversample`) within a budget of `--max-imagery-tiles` (default 500). Where Esri has no real
  detail at a zoom (it sends identical grey 'not available' tiles), the tool steps down one zoom at a time and prints what it did.
