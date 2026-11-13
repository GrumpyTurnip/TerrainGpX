"""
Minecraft export for gpx2stl.

  Java Edition    -> <name>_minecraft_java.schem   (Sponge schematic v2; paste with WorldEdit / FAWE, or open in Litematica)
  Bedrock Edition -> <name>_bedrock.mcpack          (behaviour pack holding the map as 64 x 64 block .mcstructure tiles)
                     <name>_bedrock_commands.txt    (the /structure load commands that put the tiles together)

One block per grid cell by default (set --mc-size to change the size in blocks). Heights keep the model's vertical
exaggeration relative to the horizontal scale, squeezed if needed to fit Minecraft's height limit.
Colours become the nearest Minecraft block. Points of interest are not exported.
"""
import gzip
import json
import struct
import uuid
import zipfile

import numpy as np

# name (Java), name (Bedrock), rgb, Bedrock states, used for satellite terrain colours too ("natural")
_B = [
    ("grass_block", "grass_block", (113, 160, 58), {}, True),
    ("moss_block", "moss_block", (89, 109, 45), {}, True),
    ("podzol", "podzol", (104, 76, 36), {}, True),
    ("dirt", "dirt", (134, 96, 67), {}, True),
    ("coarse_dirt", "coarse_dirt", (119, 85, 59), {}, True),
    ("clay", "clay", (160, 166, 179), {}, True),
    ("sand", "sand", (219, 207, 163), {}, True),
    ("red_sand", "red_sand", (190, 102, 33), {}, True),
    ("sandstone", "sandstone", (216, 203, 155), {}, True),
    ("snow_block", "snow", (249, 254, 254), {}, True),
    ("stone", "stone", (125, 125, 125), {}, True),
    ("andesite", "andesite", (136, 136, 137), {}, True),
    ("diorite", "diorite", (188, 188, 188), {}, True),
    ("granite", "granite", (149, 103, 85), {}, True),
    ("cobblestone", "cobblestone", (127, 127, 127), {}, True),
    ("gravel", "gravel", (131, 127, 126), {}, True),
    ("tuff", "tuff", (108, 109, 102), {}, True),
    ("calcite", "calcite", (223, 224, 220), {}, True),
    ("smooth_stone", "smooth_stone", (158, 158, 158), {}, True),
    ("white_terracotta", "white_terracotta", (209, 177, 161), {}, True),
    ("orange_terracotta", "orange_terracotta", (161, 83, 37), {}, True),
    ("yellow_terracotta", "yellow_terracotta", (186, 133, 36), {}, True),
    ("brown_terracotta", "brown_terracotta", (77, 51, 36), {}, True),
    ("red_terracotta", "red_terracotta", (143, 61, 46), {}, True),
    ("green_terracotta", "green_terracotta", (76, 83, 42), {}, True),
    ("light_gray_terracotta", "light_gray_terracotta", (135, 106, 97), {}, True),
    ("gray_terracotta", "gray_terracotta", (57, 42, 35), {}, True),
]
_CONCRETE = {
    "white": (207, 213, 214), "orange": (224, 97, 0), "magenta": (169, 48, 159), "light_blue": (35, 137, 198),
    "yellow": (240, 175, 21), "lime": (94, 168, 24), "pink": (213, 101, 142), "gray": (54, 57, 61),
    "light_gray": (125, 125, 115), "cyan": (21, 119, 136), "purple": (100, 31, 156), "blue": (45, 47, 143),
    "brown": (96, 59, 31), "green": (73, 91, 36), "red": (142, 32, 32), "black": (8, 10, 15),
}
BLOCKS = _B + [(f"{n}_concrete", f"{n}_concrete", rgb, {}, False) for n, rgb in _CONCRETE.items()]

JAVA_DATA_VERSIONS = {"1.21": 3953, "1.20": 3465, "1.19": 3337, "1.18": 2975, "1.17": 2730}
BEDROCK_BLOCK_VERSION = 18168865        # 1.21.60.33 in the Bedrock structure palette encoding; newer games upgrade it
MAX_HEIGHT = 300                        # Minecraft's build height is 384 (-64..320); stay comfortably inside it
TILE = 64


# ------------------------------------------------------------------ NBT writer (Java: big-endian + gzip, Bedrock: little-endian)
def _str(s, e):
    b = s.encode("utf-8")
    return struct.pack(e + "H", len(b)) + b


_IDS = {"byte": 1, "short": 2, "int": 3, "long": 4, "bytearr": 7, "str": 8, "list": 9, "comp": 10, "intarr": 11}


def _payload(t, v, e):
    if t == "byte":
        return struct.pack(e + "b", v)
    if t == "short":
        return struct.pack(e + "h", v)
    if t == "int":
        return struct.pack(e + "i", v)
    if t == "long":
        return struct.pack(e + "q", v)
    if t == "str":
        return _str(v, e)
    if t == "bytearr":
        return struct.pack(e + "i", len(v)) + bytes(v)
    if t == "intarr":
        return struct.pack(e + "i", len(v)) + np.asarray(v, e + "i4").tobytes()
    if t == "comp":
        out = []
        for name, (tt, vv) in v.items():
            out.append(struct.pack("B", _IDS[tt]) + _str(name, e) + _payload(tt, vv, e))
        return b"".join(out) + b"\x00"
    if t == "list":                         # v = (element type, items) ; items may be a numpy int array
        et, items = v
        if et == "int" and isinstance(items, np.ndarray):
            return struct.pack("B", 3) + struct.pack(e + "i", len(items)) + items.astype(e + "i4").tobytes()
        if not len(items):
            return struct.pack("B", 0) + struct.pack(e + "i", 0)
        return (struct.pack("B", _IDS[et]) + struct.pack(e + "i", len(items))
                + b"".join(_payload(et, it, e) for it in items))
    raise ValueError(t)


def nbt_bytes(root_name, comp, little):
    e = "<" if little else ">"
    return b"\x0a" + _str(root_name, e) + _payload("comp", comp, e)


# ------------------------------------------------------------------ block choice
def _near(rgb, pool):
    r, g, b = (float(c) for c in rgb)
    best, bd = None, 1e18
    for blk in pool:
        r2, g2, b2 = blk[2]
        rb = (r + r2) / 2
        d = (2 + rb / 256) * (r - r2) ** 2 + 4 * (g - g2) ** 2 + (2 + (255 - rb) / 256) * (b - b2) ** 2
        if d < bd:
            best, bd = blk, d
    return best


_DIRTY = {"grass_block", "moss_block", "podzol", "dirt", "coarse_dirt", "clay"}
_SANDY = {"sand", "red_sand", "sandstone"}


def build_volume(d, mc_size, log):
    """Returns (volume uint8 [y, z, x] of palette indices, java palette names, bedrock palette entries, info)."""
    idx, palette, mask = d["idx"], d["palette"], d["mask"]
    nyc, nxc = mask.shape
    f = mc_size / max(nxc, nyc)
    mnx, mny = max(8, round(nxc * f)), max(8, round(nyc * f))
    ix = np.minimum(((np.arange(mnx) + 0.5) * nxc / mnx).astype(int), nxc - 1)
    iy = np.minimum(((np.arange(mny) + 0.5) * nyc / mny).astype(int), nyc - 1)
    rs = np.ix_(iy, ix)
    g = d["longest_m"] / max(mnx, mny)                      # ground metres per block

    E = d["E"]
    Ec = ((E[:-1, :-1] + E[1:, :-1] + E[:-1, 1:] + E[1:, 1:]) / 4)[rs]
    base = 4
    vscale = d["exaggeration"] / g                           # blocks per metre
    relief = max(float(Ec.max()) - d["E0"], 1.0)
    squeezed = False
    if base + relief * vscale > MAX_HEIGHT - 12:
        vscale = (MAX_HEIGHT - 12 - base) / relief
        squeezed = True
    top = np.clip(base + np.round((Ec - d["E0"]) * vscale).astype(int), 1, None)
    msk = mask[rs]
    water = d["water"][rs] & msk
    bld = d["bld_mm"][rs] > 0
    hb = np.where(bld, np.maximum(2, np.round(d["bld_mm"][rs] / d["s_exag"] * vscale)), 0).astype(int)
    bld &= ~water

    # chosen block for every palette colour
    nat = d["scheme"] == "satellite"
    pool_nat = [b for b in BLOCKS if b[4]]
    blk_of = []
    for k, rgb in enumerate(palette):
        blk_of.append(_near(rgb, pool_nat if (nat and k < d["n_terrain"]) else BLOCKS))
    names = ["air", "stone", "dirt", "sandstone", "gravel", "water"]
    blocks = {"air": None, "stone": ("stone", "stone", (0, 0, 0), {}, True), "dirt": ("dirt", "dirt", (0, 0, 0), {}, True),
              "sandstone": ("sandstone", "sandstone", (0, 0, 0), {}, True), "gravel": ("gravel", "gravel", (0, 0, 0), {}, True),
              "water": ("water", "water", (0, 0, 0), {"liquid_depth": 0}, True)}

    def bid(blk):
        if blk[0] not in blocks:
            blocks[blk[0]] = blk
            names.append(blk[0])
        return names.index(blk[0])
    surf_k = np.array([bid(b) for b in blk_of], np.uint8)

    def filler(blk):
        n = blk[0]
        return names.index("dirt") if n in _DIRTY else names.index("sandstone") if n in _SANDY else bid(blk)
    fill_k = np.array([filler(b) for b in blk_of], np.uint8)
    ci = idx[rs]
    surf, fill = surf_k[ci], fill_k[ci]
    GRAVEL, WATER, STONE = names.index("gravel"), names.index("water"), names.index("stone")

    ground = top.copy()
    st = np.where(water, np.maximum(top - 3, 1), top + hb)   # last solid block of each column
    wt = np.where(water, top - 1, -1)                          # top of the water (lakes and rivers are two deep)
    surf = np.where(water, GRAVEL, surf)
    fill = np.where(water, GRAVEL, fill)
    H = int(max(st.max(), wt.max())) + 2
    vol = np.zeros((H, mny, mnx), np.uint8)
    for y in range(H):
        sl = np.zeros((mny, mnx), np.uint8)
        solid = msk & (y <= st)
        sl[solid] = STONE
        near = solid & (y > st - 4)
        sl[near] = fill[near]
        s_top = solid & (y == st)
        sl[s_top] = surf[s_top]
        wm = msk & (y > st) & (y <= wt)
        sl[wm] = WATER
        bm = bld & (y > ground) & (y <= st)
        sl[bm] = surf[bm]
        vol[y] = sl
    info = dict(size=(mnx, H, mny), block_m=g, vscale=vscale, squeezed=squeezed, base=base,
                relief_blocks=int(top.max() - top.min()))
    java = [("minecraft:water[level=0]" if n == "water" else f"minecraft:{n}") for n in names]
    bedrock = []
    for n in names:
        if n == "air":
            bedrock.append(("minecraft:air", {}))
        else:
            blk = blocks[n]
            bedrock.append((f"minecraft:{blk[1]}", blk[3]))
    return vol, java, bedrock, info


# ------------------------------------------------------------------ Java: Sponge schematic v2
def write_schem(path, vol, java_palette, data_version, name):
    H, L, W = vol.shape
    assert len(java_palette) < 128                       # one varint byte per block
    comp = {
        "Version": ("int", 2),
        "DataVersion": ("int", data_version),
        "Metadata": ("comp", {"Name": ("str", name), "Author": ("str", "gpx2stl"),
                              "WEOffsetX": ("int", 0), "WEOffsetY": ("int", 0), "WEOffsetZ": ("int", 0)}),
        "Width": ("short", W), "Height": ("short", H), "Length": ("short", L),
        "Offset": ("intarr", [0, 0, 0]),
        "PaletteMax": ("int", len(java_palette)),
        "Palette": ("comp", {n: ("int", i) for i, n in enumerate(java_palette)}),
        "BlockData": ("bytearr", vol.reshape(-1).tobytes()),       # index = x + z*W + y*W*L
        "BlockEntities": ("list", ("comp", [])),
    }
    with gzip.open(path, "wb", compresslevel=6) as fh:
        fh.write(nbt_bytes("Schematic", comp, little=False))


# ------------------------------------------------------------------ Bedrock: .mcstructure tiles in a behaviour pack
def _structure(vol_tile, bedrock_palette):
    H, L, W = vol_tile.shape
    primary = vol_tile.transpose(2, 0, 1).reshape(-1).astype(np.int32)       # x outermost, z fastest
    palette = [{"name": ("str", n), "states": ("comp", {k: ("int", v) for k, v in st.items()}),
                "version": ("int", BEDROCK_BLOCK_VERSION)} for n, st in bedrock_palette]
    comp = {
        "format_version": ("int", 1),
        "size": ("list", ("int", np.array([W, H, L], np.int32))),
        "structure": ("comp", {
            "block_indices": ("list", ("list", [("int", primary), ("int", np.full_like(primary, -1))])),
            "entities": ("list", ("comp", [])),
            "palette": ("comp", {"default": ("comp", {
                "block_palette": ("list", ("comp", palette)),
                "block_position_data": ("comp", {})})}),
        }),
        "structure_world_origin": ("list", ("int", np.array([0, 0, 0], np.int32))),
    }
    return nbt_bytes("", comp, little=True)


def write_bedrock_pack(path, cmd_path, vol, bedrock_palette, stem):
    H, L, W = vol.shape
    ns = "gpx2stl"
    cmds = []
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for r0 in range(0, L, TILE):
            for c0 in range(0, W, TILE):
                tile = vol[:, r0:r0 + TILE, c0:c0 + TILE]
                if not tile.any():
                    continue
                name = f"r{r0 // TILE}_c{c0 // TILE}"
                z.writestr(f"structures/{ns}/{name}.mcstructure", _structure(tile, bedrock_palette))
                cmds.append(f"/structure load {ns}:{name} ~{c0 if c0 else ''} ~ ~{r0 if r0 else ''}")
        manifest = {
            "format_version": 2,
            "header": {"name": f"gpx2stl map {stem}", "description": "Terrain tiles made by gpx2stl",
                       "uuid": str(uuid.uuid4()), "version": [1, 0, 0], "min_engine_version": [1, 21, 0]},
            "modules": [{"type": "data", "uuid": str(uuid.uuid4()), "version": [1, 0, 0]}],
        }
        z.writestr("manifest.json", json.dumps(manifest, indent=2))
    cmd_path.write_text(
        "Bedrock map: how to place it\n"
        "1. Double-click the .mcpack to import it, then add it to a world as a BEHAVIOR PACK (World > Behavior Packs).\n"
        "   Turn cheats on.\n"
        "2. Stand where the north-west corner of the map should be, at the height you want the BOTTOM of the map\n"
        "   (the world floor is y=-64; stand on it or build up from there).\n"
        "3. Paste each line below into the chat, one after another, from that same spot. Each places a 64 x 64 block tile.\n"
        "   East is +x, south is +z, so row 0 is the north edge.\n\n" + "\n".join(cmds) + "\n")
    return len(cmds)


# ------------------------------------------------------------------ entry point
def export(args, out, d, log=print):
    """d: idx, palette, n_terrain, mask, E, E0, water, bld_mm, s_exag, longest_m, exaggeration, scheme."""
    n = max(d["mask"].shape)
    size = args.mc_size or min(n, 400)
    size = max(32, min(size, 1000))
    W_est = size
    while True:                                  # keep the block volume sensible (about 200 million blocks at most)
        f = size / n
        wv, lv = round(d["mask"].shape[1] * f), round(d["mask"].shape[0] * f)
        if wv * lv * 220 <= 200e6 or size <= 64:
            break
        size = int(size * 0.9)
    if size != W_est:
        log(f"Minecraft: map size reduced to {size} blocks to keep the file manageable.")
    vol, java, bedrock, info = build_volume(d, size, log)
    mnx, H, mny = info["size"]
    log(f"Minecraft map: {mnx} x {mny} blocks, {H} blocks tall; one block = {info['block_m']:.1f} m across, "
        f"vertically one block = {1 / info['vscale']:.1f} m"
        + (" (squeezed to fit Minecraft's height limit)" if info["squeezed"] else ""))
    written = []
    if args.minecraft == "java":
        key = args.mc_version if args.mc_version in JAVA_DATA_VERSIONS else "1.20"
        p = out.with_name(out.stem + "_minecraft_java.schem")
        write_schem(p, vol, java, JAVA_DATA_VERSIONS[key], out.stem)
        log(f"Wrote {p}  (Java schematic; paste with WorldEdit //schem load then //paste)")
        written.append(p)
    else:
        p = out.with_name(out.stem + "_bedrock.mcpack")
        c = out.with_name(out.stem + "_bedrock_commands.txt")
        k = write_bedrock_pack(p, c, vol, bedrock, out.stem)
        log(f"Wrote {p}  ({k} tiles) and {c}")
        written += [p, c]
    return written
