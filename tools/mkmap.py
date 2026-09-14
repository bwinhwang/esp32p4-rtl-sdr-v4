#!/usr/bin/env python3
"""Bake the map under the display into main/map_data.h.

Land (Natural Earth 10m land + minor islands, lakes cut out) and airports
(OurAirports), both public domain, clipped to a box around the antenna in
sdkconfig and stored as 0.1 km offsets from it, so rerun this after moving the
antenna. Downloads go through http(s)_proxy like curl.
"""

import argparse
import csv
import math
import os
import re
import sys
import urllib.request
import json

NE = "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/"
SOURCES = {
    "ne_10m_land.geojson":          NE + "ne_10m_land.geojson",
    "ne_10m_minor_islands.geojson": NE + "ne_10m_minor_islands.geojson",
    "ne_10m_lakes.geojson":         NE + "ne_10m_lakes.geojson",
    "airports.csv": "https://davidmegginson.github.io/ourairports-data/airports.csv",
}
# Natural Earth ranks lakes by importance; anything coarser than this is a
# pond at the display's 2-20 km per cell.
MAX_LAKE_RANK = 7
UNIT_KM = 0.1


def fetch(cache, name):
    path = os.path.join(cache, name)
    if not os.path.exists(path):
        print(f"fetching {name}", file=sys.stderr)
        os.makedirs(cache, exist_ok=True)
        urllib.request.urlretrieve(SOURCES[name], path)
    return path


def rings(geom):
    t = geom["type"]
    if t == "Polygon":      return geom["coordinates"]
    if t == "MultiPolygon": return [r for p in geom["coordinates"] for r in p]
    return []


def clip_ring(pts, half_x, half_y):
    """Sutherland-Hodgman against the box; the result runs along the box edge
    where the ring left it, which is what keeps even-odd filling right."""
    def clip(pts, inside, cross):
        out = []
        for a, b in zip(pts, pts[1:] + pts[:1]):
            ia, ib = inside(a), inside(b)
            if ia: out.append(a)
            if ia != ib: out.append(cross(a, b))
        return out

    def cut_x(x):
        return lambda a, b: (x, a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0]))

    def cut_y(y):
        return lambda a, b: (a[0] + (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]), y)

    for inside, cross in ((lambda p: p[0] >= -half_x, cut_x(-half_x)),
                          (lambda p: p[0] <=  half_x, cut_x(half_x)),
                          (lambda p: p[1] >= -half_y, cut_y(-half_y)),
                          (lambda p: p[1] <=  half_y, cut_y(half_y))):
        pts = clip(pts, inside, cross)
        if len(pts) < 3:
            return []
    return pts


def thin(pts, min_km):
    """Drop points closer than a fraction of a cell to the one kept before."""
    out = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= min_km:
            out.append(p)
    if len(out) > 1 and math.hypot(out[0][0] - out[-1][0], out[0][1] - out[-1][1]) < min_km:
        out.pop()
    return out


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--sdkconfig", default=os.path.join(root, "sdkconfig"))
    ap.add_argument("--out", default=os.path.join(root, "main", "map_data.h"))
    ap.add_argument("--cache", default=os.path.join(root, "tools", ".cache"))
    ap.add_argument("--half-ew", type=float, default=450, help="km east/west kept (default 450)")
    ap.add_argument("--half-ns", type=float, default=300, help="km north/south kept (default 300)")
    args = ap.parse_args()

    cfg = open(args.sdkconfig).read()
    lat0 = float(re.search(r'^CONFIG_ADSB_RX_LAT="([^"]+)"', cfg, re.M).group(1))
    lon0 = float(re.search(r'^CONFIG_ADSB_RX_LON="([^"]+)"', cfg, re.M).group(1))
    kx = 111.32 * math.cos(math.radians(lat0))       # same projection as update_range()
    ky = 111.32

    def km(lon, lat):
        return ((lon - lon0) * kx, (lat - lat0) * ky)

    # A coarse degree box first so the big files are not projected whole.
    dlat = args.half_ns / ky + 0.5
    dlon = args.half_ew / kx + 0.5

    polys = []
    for name in ("ne_10m_land.geojson", "ne_10m_minor_islands.geojson", "ne_10m_lakes.geojson"):
        for f in json.load(open(fetch(args.cache, name)))["features"]:
            rank = f["properties"].get("scalerank")
            if name.startswith("ne_10m_lakes") and rank is not None and float(rank) > MAX_LAKE_RANK:
                continue
            for ring in rings(f["geometry"]):
                if not any(abs(lat - lat0) <= dlat and abs(lon - lon0) <= dlon for lon, lat in ring):
                    continue
                pts = clip_ring([km(lon, lat) for lon, lat in ring], args.half_ew, args.half_ns)
                if pts:
                    pts = thin(pts, 0.5)
                    if len(pts) >= 3:
                        polys.append(pts)

    apts = []
    for r in csv.DictReader(open(fetch(args.cache, "airports.csv"), encoding="utf-8")):
        if r["type"] not in ("large_airport", "medium_airport"): continue
        if r["scheduled_service"] != "yes" or not r["iata_code"]:  continue
        x, y = km(float(r["longitude_deg"]), float(r["latitude_deg"]))
        if abs(x) <= args.half_ew and abs(y) <= args.half_ns:
            apts.append((x, y, r["iata_code"], r["name"]))
    apts.sort(key=lambda a: math.hypot(a[0], a[1]))

    def u(v):
        v = round(v / UNIT_KM)
        assert -32768 <= v <= 32767, v
        return v

    npts = sum(len(p) for p in polys)
    with open(args.out, "w") as o:
        o.write(f"/* Generated by tools/mkmap.py for {lat0:.6f},{lon0:.6f} -- do not edit.\n"
                f" * Natural Earth 10m land + minor islands, lakes to scalerank {MAX_LAKE_RANK} as holes,\n"
                f" * OurAirports (scheduled large/medium), +-{args.half_ew:.0f} km E-W, +-{args.half_ns:.0f} km N-S. */\n"
                f"#pragma once\n#include <stdint.h>\n\n"
                f"#define MAP_LAT     {lat0:.6f}f\n#define MAP_LON     {lon0:.6f}f\n"
                f"#define MAP_UNIT_KM {UNIT_KM}f\n"
                f"#define MAP_HALF_EW_KM {args.half_ew:.1f}f\n#define MAP_HALF_NS_KM {args.half_ns:.1f}f\n"
                f"#define MAP_NPOLYS  {len(polys)}\n#define MAP_NPTS    {npts}\n#define MAP_NAPTS   {len(apts)}\n\n"
                f"typedef struct {{ int16_t x, y; }} map_pt_t;               /* east, north of the antenna */\n"
                f"typedef struct {{ int16_t x, y; char code[4]; }} map_apt_t;\n\n")
        o.write("/* Polygon i is the closed ring MAP_PTS[MAP_POLY_START[i] .. MAP_POLY_START[i+1]);\n"
                " * land is where a point is inside an odd number of them. */\n")
        o.write("static const uint16_t MAP_POLY_START[MAP_NPOLYS + 1] = {")
        start = 0
        for i, p in enumerate(polys):
            o.write(("\n    " if i % 12 == 0 else " ") + f"{start},")
            start += len(p)
        o.write(f"\n    {start}\n}};\n\n")
        o.write("static const map_pt_t MAP_PTS[MAP_NPTS] = {")
        i = 0
        for p in polys:
            for x, y in p:
                o.write(("\n    " if i % 8 == 0 else " ") + f"{{{u(x)},{u(y)}}},")
                i += 1
        o.write("\n};\n\n")
        o.write("/* Nearest first. */\nstatic const map_apt_t MAP_APTS[MAP_NAPTS] = {\n")
        for x, y, code, name in apts:
            o.write(f"    {{{u(x):6d},{u(y):6d}, \"{code}\"}},   /* {name} */\n")
        o.write("};\n")
    print(f"{args.out}: {len(polys)} polygons, {npts} points, {len(apts)} airports, "
          f"~{(npts * 4 + len(polys) * 2 + len(apts) * 8) / 1024:.1f} KB", file=sys.stderr)


if __name__ == "__main__":
    main()
