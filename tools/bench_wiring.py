#!/usr/bin/env python3
"""Writes the wiring figures of the DMA bench (docs/assurance/bench/figures/wiring-<board>.svg and .dark.svg).

    python3 tools/bench_wiring.py            # write the figures
    python3 tools/bench_wiring.py --check    # fail when a committed figure differs from what this writes

One light and one dark SVG per board, in the Kokeb figure colours (the tokens of the shared figure kit), sized for a
phone (360 px wide). The pins are the bench pins of tests/target/dma/main/bench.hpp; a change there is a change here.
SPDX-License-Identifier: MIT
Copyright (c) 2026 The port-esp32 authors
"""

from __future__ import annotations

import argparse
import pathlib
import sys
from xml.sax.saxutils import escape

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT = ROOT / "docs" / "assurance" / "bench" / "figures"

# The figure kit's tokens (light, dark).
BASE = {
    "canvas": ("#FFFFFF", "#0E131A"), "frame": ("#E2E7EE", "#283241"), "card": ("#FFFFFF", "#151C26"),
    "ink": ("#0E1726", "#E8EDF3"), "muted": ("#566070", "#9AA6B5"), "line": ("#C9D1DB", "#3A4657"),
}
TONES = {  # strong, text, tint, soft
    "brand": (("#9A4A17", "#7A360E", "#FBEFE6", "#E9C3A6"), ("#F4B942", "#F8D27A", "#2A2113", "#6B5424")),
    "blue": (("#2563EB", "#1E429F", "#EAF2FF", "#BCD2F9"), ("#60A5FA", "#BFDBFE", "#13233A", "#2B4C78")),
    "green": (("#15803D", "#14532D", "#EAF7EF", "#B2E2C2"), ("#4ADE80", "#BBF7D0", "#11281A", "#285A3A")),
    "amber": (("#C26A05", "#8A3C0B", "#FFF5E3", "#F1CF93"), ("#FBBF24", "#FDE68A", "#2C2310", "#6A5220")),
    "slate": (("#64748B", "#334155", "#F1F4F8", "#CBD5E1"), ("#94A3B8", "#E2E8F0", "#1B232E", "#3A4656")),
}

BOARDS = {
    "esp32c6": {
        "title": "ESP32-C6 devkit", "spi": "SPI2", "sclk": 19, "mosi": 18, "miso": 20, "cs": 21,
        "adc": (0, 1, 4), "items": "SPI and ADC items",
    },
    "esp32s3": {
        "title": "ESP32-S3 devkit", "spi": "SPI2", "sclk": 12, "mosi": 11, "miso": 13, "cs": 10,
        "adc": (1, 2, 3), "items": "SPI and ADC items (PSRAM item if fitted)",
    },
    "esp32": {
        "title": "ESP32 devkit", "spi": "SPI3 (VSPI)", "sclk": 18, "mosi": 23, "miso": 19, "cs": 5,
        "adc": None, "items": "SPI items, T8 receive rounding",
    },
}

W = 360


def tok(name: str, dark: bool) -> str:
    if name in BASE:
        return BASE[name][1 if dark else 0]
    tone, part = name.split(".")
    return TONES[tone][1 if dark else 0][("strong", "text", "tint", "soft").index(part)]


def figure(board: str, spec: dict, dark: bool) -> str:
    t = lambda n: tok(n, dark)  # noqa: E731
    parts: list[str] = []
    y = 20

    def text(x: float, yy: float, s: str, size: int = 13, colour: str = "ink", weight: str = "400",
             anchor: str = "start") -> None:
        parts.append(f'<text x="{x}" y="{yy}" font-size="{size}" font-weight="{weight}" fill="{t(colour)}" '
                     f'text-anchor="{anchor}">{escape(s)}</text>')

    text(16, y + 14, "BENCH WIRING", 11, "brand.text", "700")
    text(16, y + 36, spec["title"], 18, "ink", "700")
    text(16, y + 56, spec["items"], 12, "muted")
    y += 76
    # The board: one row per pin used.
    rows = [("SCLK", spec["sclk"], "blue", "analyser CH0, optional"),
            ("MOSI", spec["mosi"], "blue", "jumper to MISO"),
            ("MISO", spec["miso"], "blue", "jumper from MOSI"),
            ("CS", spec["cs"], "blue", "analyser CH1, optional")]
    if spec["adc"]:
        taps = ("tap 1, 0.8 V", "tap 2, 1.65 V", "tap 3, 2.5 V")
        for i, pin in enumerate(spec["adc"]):
            rows.append((f"ADC{i}", pin, "green", taps[i]))
    rows += [("3V3", None, "amber", "divider top"), ("GND", None, "slate", "divider bottom"),
             ("USB", None, "slate", "console 115200 Bd")]
    row_h = 30
    box_h = 34 + row_h * len(rows)
    parts.append(f'<rect x="16" y="{y}" width="132" height="{box_h}" rx="10" fill="{t("card")}" '
                 f'stroke="{t("line")}" stroke-width="1.5"/>')
    text(82, y + 22, spec["spi"], 12, "muted", "600", "middle")
    ry = y + 34
    mosi_y = miso_y = 0.0
    for label, pin, tone, note in rows:
        cy = ry + row_h / 2
        pin_txt = f"GPIO{pin}" if pin is not None else ""
        parts.append(f'<rect x="26" y="{ry + 4}" width="112" height="{row_h - 8}" rx="6" fill="{t(tone + ".tint")}" '
                     f'stroke="{t(tone + ".soft")}"/>')
        text(34, cy + 4.5, label, 12, tone + ".text", "700")
        text(132, cy + 4.5, pin_txt, 12, tone + ".text", "400", "end")
        parts.append(f'<line x1="148" y1="{cy}" x2="182" y2="{cy}" stroke="{t(tone + ".strong")}" stroke-width="2"/>')
        parts.append(f'<circle cx="148" cy="{cy}" r="3" fill="{t(tone + ".strong")}"/>')
        text(188, cy + 4.5, note, 12, "ink")
        if label == "MOSI":
            mosi_y = cy
        if label == "MISO":
            miso_y = cy
        ry += row_h
    # The loopback jumper.
    parts.append(f'<path d="M182 {mosi_y} C 172 {mosi_y + 4}, 172 {miso_y - 4}, 182 {miso_y}" fill="none" '
                 f'stroke="{t("brand.strong")}" stroke-width="3"/>')
    y += box_h + 20
    if spec["adc"]:
        text(16, y + 12, "Divider: four 10 kOhm in series, 3V3 to GND;", 12, "muted")
        text(16, y + 28, "the inner nodes are taps 1 to 3.", 12, "muted")
        y += 40
    text(16, y + 12, "Jumper under 10 cm for 8 MHz SCLK.", 12, "muted")
    y += 28
    h = y
    title = f"{spec['title']}: bench wiring"
    desc = (f"{spec['title']} wiring for the DMA bench: {spec['spi']} with MOSI GPIO{spec['mosi']} jumpered to MISO "
            f"GPIO{spec['miso']}, SCLK GPIO{spec['sclk']}, CS GPIO{spec['cs']}"
            + (", three ADC inputs on a resistor divider" if spec["adc"] else "") + ", USB console.")
    head = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {h}" width="{W}" height="{h}" role="img" '
            f'aria-labelledby="t-{board} d-{board}" font-family="Inter, Segoe UI, Helvetica, Arial, sans-serif">'
            f'<title id="t-{board}">{escape(title)}</title><desc id="d-{board}">{escape(desc)}</desc>'
            f'<rect width="{W}" height="{h}" rx="14" fill="{t("canvas")}" stroke="{t("frame")}"/>')
    return head + "".join(parts) + "</svg>\n"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    bad = []
    OUT.mkdir(parents=True, exist_ok=True)
    for board, spec in BOARDS.items():
        for dark in (False, True):
            path = OUT / f"wiring-{board}{'.dark' if dark else ''}.svg"
            svg = figure(board, spec, dark)
            if a.check:
                if not path.is_file() or path.read_text(encoding="utf-8") != svg:
                    bad.append(str(path.relative_to(ROOT)))
            else:
                path.write_text(svg, encoding="utf-8")
    for b in bad:
        print(f"bench_wiring: {b} differs from tools/bench_wiring.py (run it and commit)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
