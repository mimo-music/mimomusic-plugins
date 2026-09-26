#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Mettronic GmbH
"""Plots the CSVs written by contour_preset_probe (see that file's header).

For every test signal one PNG is written: one column per configuration
("saved" = the preset exactly as exported, plus the one-knob variants), three
rows per column:
  1. input (grey) and output (orange) waveform over a ~70 ms steady-state window,
  2. the gain the plugin applied (dB, estimated as |out/in| where |in| is big
     enough to divide by, global output gain removed),
  3. the output spectrum (dB re a full-scale sine).

Only needs Pillow (no numpy/matplotlib):  python3 plot_contour_probe.py <dir>
"""
import csv
import glob
import math
import os
import sys

from PIL import Image, ImageDraw, ImageFont

W_COL, H_ROW = 620, 250
MARGIN_L, MARGIN_R, MARGIN_T, MARGIN_B = 58, 12, 26, 26
BG = (20, 21, 25)
GRID = (48, 51, 58)
TXT = (170, 176, 186)
C_IN = (120, 126, 138)
C_OUT = (250, 170, 70)
C_GAIN = (120, 210, 170)
C_SPEC = (120, 190, 250)


def font(size):
    for p in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "/usr/share/fonts/dejavu/DejaVuSans.ttf"):
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


F_SMALL, F_MED, F_BIG = font(11), font(13), font(17)


def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def panel(draw, x0, y0, title, xlabels, ylabels, xmap, ymap, xtext=None):
    """Frame + grid. xmap/ymap map a data value to pixel inside the plot rect."""
    px0, py0 = x0 + MARGIN_L, y0 + MARGIN_T
    px1, py1 = x0 + W_COL - MARGIN_R, y0 + H_ROW - MARGIN_B
    draw.rectangle([px0, py0, px1, py1], outline=GRID)
    draw.text((x0 + MARGIN_L, y0 + 6), title, fill=TXT, font=F_SMALL)
    for v in xlabels:
        x = xmap(v)
        draw.line([x, py0, x, py1], fill=GRID)
        draw.text((x - 10, py1 + 4), (xtext(v) if xtext else "%g" % v), fill=TXT, font=F_SMALL)
    for v in ylabels:
        y = ymap(v)
        draw.line([px0, y, px1, y], fill=GRID)
        draw.text((x0 + 4, y - 6), ("%g" % v), fill=TXT, font=F_SMALL)
    return px0, py0, px1, py1


def polyline(draw, pts, color, width=1):
    if len(pts) >= 2:
        draw.line(pts, fill=color, width=width)


def draw_case(img, col, name, rows, spec_rows):
    d = ImageDraw.Draw(img)
    x0 = col * W_COL
    n = len(rows)
    # steady-state 70 ms window starting at 0.5 s
    t_start = 500.0
    t_len = 70.0
    win = [r for r in rows if t_start <= float(r["t_ms"]) < t_start + t_len]
    peak = max(1e-6, max(abs(float(r["in"])) for r in rows))
    amp = max(0.05, peak * 1.15)

    # 1. waveform
    y0 = 48
    px0, py0, px1, py1 = panel(
        d, x0, y0, "input (grey) / output (orange, output gain removed), %d..%d ms" % (t_start, t_start + t_len),
        [t_start + t_len * k / 5 for k in range(6)], [-amp, 0, amp],
        lambda t: x0 + MARGIN_L + (t - t_start) / t_len * (W_COL - MARGIN_L - MARGIN_R),
        lambda v: y0 + MARGIN_T + (1 - (v + amp) / (2 * amp)) * (H_ROW - MARGIN_T - MARGIN_B))
    xm = lambda t: px0 + (t - t_start) / t_len * (px1 - px0)
    ym = lambda v: py1 - (v + amp) / (2 * amp) * (py1 - py0)
    polyline(d, [(xm(float(r["t_ms"])), ym(float(r["in"]))) for r in win], C_IN, 1)
    polyline(d, [(xm(float(r["t_ms"])), ym(float(r["out_norm"]))) for r in win], C_OUT, 1)

    # 2. gain
    y1 = 48 + H_ROW
    gains = [float(r["gain_db"]) for r in win if r["gain_db"] != ""]
    glo, ghi = (min(gains), max(gains)) if gains else (-1, 1)
    glo, ghi = math.floor(min(glo, -0.5)) - 1, math.ceil(max(ghi, 0.5)) + 1
    step = max(1, (ghi - glo) // 4)
    px0, py0, px1, py1 = panel(
        d, x0, y1, "gain applied (dB) -- jumps here are the 'crush'",
        [t_start + t_len * k / 5 for k in range(6)], list(range(int(glo), int(ghi) + 1, int(step))),
        lambda t: x0 + MARGIN_L + (t - t_start) / t_len * (W_COL - MARGIN_L - MARGIN_R),
        lambda v: y1 + MARGIN_T + (1 - (v - glo) / (ghi - glo)) * (H_ROW - MARGIN_T - MARGIN_B))
    xm = lambda t: px0 + (t - t_start) / t_len * (px1 - px0)
    ym = lambda v: py1 - (v - glo) / (ghi - glo) * (py1 - py0)
    seg = []
    for r in win:
        if r["gain_db"] == "":
            polyline(d, seg, C_GAIN, 2); seg = []
        else:
            seg.append((xm(float(r["t_ms"])), ym(float(r["gain_db"]))))
    polyline(d, seg, C_GAIN, 2)

    # 3. spectrum (log frequency 20 Hz..20 kHz, -120..0 dB)
    y2 = 48 + 2 * H_ROW
    fl, fh = math.log10(20.0), math.log10(20000.0)
    px0, py0, px1, py1 = panel(
        d, x0, y2, "output spectrum (dB, 0 = full-scale sine)",
        [math.log10(f) for f in (20, 100, 1000, 10000)], [-120, -90, -60, -30, 0],
        lambda lf: x0 + MARGIN_L + (lf - fl) / (fh - fl) * (W_COL - MARGIN_L - MARGIN_R),
        lambda v: y2 + MARGIN_T + (1 - (v + 120) / 120.0) * (H_ROW - MARGIN_T - MARGIN_B),
        xtext=lambda lf: {1: "10", 2: "100", 3: "1k", 4: "10k"}.get(round(lf), "%g Hz" % (10 ** lf)) if abs(lf - round(lf)) < 1e-6 else "20")
    for lab, f in (("20", 20), ("100", 100), ("1k", 1000), ("10k", 10000)):
        pass
    xm = lambda f: px0 + (math.log10(max(f, 20.0)) - fl) / (fh - fl) * (px1 - px0)
    ym = lambda v: py1 - (max(-120.0, min(0.0, v)) + 120.0) / 120.0 * (py1 - py0)
    pts, last_px = [], -1
    for r in spec_rows:
        f = float(r["freq_hz"])
        if f < 20 or f > 20000:
            continue
        px = int(xm(f))
        if px == last_px and pts:
            pts[-1] = (px, min(pts[-1][1], ym(float(r["db"]))))  # keep the strongest bin per pixel column
        else:
            pts.append((px, ym(float(r["db"]))))
            last_px = px
    polyline(d, pts, C_SPEC, 1)

    d.text((x0 + MARGIN_L, 26), name, fill=(235, 238, 242), font=F_MED)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    d = sys.argv[1]
    cases = {}
    for path in sorted(glob.glob(os.path.join(d, "*__*.csv"))):
        base = os.path.basename(path)[:-4]
        if base.endswith("_spectrum"):
            continue
        sig, cfg = base.split("__", 1)
        cases.setdefault(sig, []).append((cfg, path))
    order = ["saved", "trigfilter_off", "atk1_rel5ms"]
    nice = {"saved": "AS SAVED (your preset)", "trigfilter_off": "trigger filter OFF",
            "atk1_rel5ms": "attack 1 ms / release 5 ms"}
    for sig, lst in cases.items():
        lst.sort(key=lambda c: order.index(c[0]) if c[0] in order else 99)
        img = Image.new("RGB", (W_COL * len(lst), 48 + 3 * H_ROW), BG)
        dr = ImageDraw.Draw(img)
        dr.text((10, 4), "signal: %s" % sig, fill=(250, 250, 250), font=F_BIG)
        for col, (cfg, path) in enumerate(lst):
            rows = load(path)
            spec = load(path[:-4] + "_spectrum.csv")
            draw_case(img, col, nice.get(cfg, cfg), rows, spec)
        out = os.path.join(d, "%s.png" % sig)
        img.save(out)
        print("wrote", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
