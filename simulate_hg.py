#!/usr/bin/env python3
"""
simulate_hg.py — Dry-run the Holy Grail exposure engine offline.

Drives the real holygrail.py with synthetic meter shots derived from
an astro scene model so the tracker converges exactly as it would during
a live sequence.  Lets you verify ISO/shutter/aperture/WB transitions
before committing to a shoot.

Usage (run from the project root):
    python3 simulate_hg.py                           # tonight, 12h, default settings
    python3 simulate_hg.py --start "2026-08-18 18:30" --hours 14
    python3 simulate_hg.py --no-feedback             # astro-only, no tracker
    python3 simulate_hg.py --step 10                 # print every 10th frame
    python3 simulate_hg.py --json > out.json         # export for plotting
"""

import sys, math, datetime, argparse, json, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from zoneinfo import ZoneInfo
from holygrail import HolyGrailController, HGSettings

# ── Ground-truth scene model ──────────────────────────────────────────────────
# Normalized pixel EV (what the scene would produce at anchor settings).
# Calibrate NIGHT from your actual observation: "25s / ISO 4600 / f4 = good exposure"
# → anchor at 1/500, ISO 100 → camera_ev_offset = log2(25*500)+log2(46) ≈ 19.1
# → normalized = raw_pixel_ev(≈10) - 19.1 ≈ -9.1
_SCENE_EV_BY_PHASE = {
    "day":       12.5,
    "golden":     9.0,
    "twilight":   4.0,
    "night":     -9.1,
}

def _scene_ev(sun_alt: float) -> float:
    """Interpolate ground-truth normalized scene EV from sun altitude."""
    if   sun_alt >  10: return _SCENE_EV_BY_PHASE["day"]
    elif sun_alt >   0:
        t = (10 - sun_alt) / 10.0
        return _SCENE_EV_BY_PHASE["day"]     * (1-t) + _SCENE_EV_BY_PHASE["golden"]    * t
    elif sun_alt >  -6:
        t = -sun_alt / 6.0
        return _SCENE_EV_BY_PHASE["golden"]  * (1-t) + _SCENE_EV_BY_PHASE["twilight"]  * t
    elif sun_alt > -18:
        t = (-6 - sun_alt) / 12.0
        return _SCENE_EV_BY_PHASE["twilight"] * (1-t) + _SCENE_EV_BY_PHASE["night"]    * t
    return _SCENE_EV_BY_PHASE["night"]

def _synthetic_frame(raw_pixel_ev: float, w: int = 64, h: int = 48) -> np.ndarray:
    """
    Return a uint8 RGB image whose P50 luminance matches raw_pixel_ev.
    Pixel EV formula: ev = log2((lum/255)^2.2 / 0.18) + 12
    → lum = 255 * (0.18 * 2^(ev-12))^(1/2.2)
    """
    linear = 0.18 * (2.0 ** (raw_pixel_ev - 12.0))
    linear = max(1e-9, min(1.0, linear))
    lum    = int(255.0 * (linear ** (1.0 / 2.2)))
    lum    = max(2, min(253, lum))
    img    = np.full((h, w, 3), lum, dtype=np.uint8)
    rng    = np.random.default_rng(lum)       # reproducible noise
    noise  = rng.integers(-4, 5, (h, w, 3))
    return np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)


def simulate(hg: HolyGrailController, start: datetime.datetime,
             hours: float, with_feedback: bool = True,
             noise_ev: float = 0.12) -> list[dict]:
    """
    Step through the sequence, optionally pushing synthetic meter shots.

    Returns one dict per frame with all key exposure parameters.
    """
    rows    = []
    t       = start
    end     = start + datetime.timedelta(hours=hours)
    rng     = np.random.default_rng(1234)

    while t < end:
        params   = hg.get_next_shot_parameters(now=t)
        interval = params.get("interval", 15.0)
        phase    = params.get("phase", "day")
        sun_alt  = params.get("sun_alt", 0.0)

        if with_feedback:
            shutter_s  = params["shutter_s"]
            iso        = params["iso"]
            anchor_s   = hg.settings.anchor_shutter_s
            anchor_iso = hg.settings.anchor_iso

            true_scene_ev  = _scene_ev(sun_alt)
            noisy_scene_ev = true_scene_ev + rng.normal(0, noise_ev)

            if anchor_s and anchor_iso:
                cam_offset   = (math.log2(max(shutter_s, 1e-9) / anchor_s) +
                                math.log2(max(iso, 1)          / anchor_iso))
                raw_pixel_ev = noisy_scene_ev + cam_offset
            else:
                cam_offset   = 0.0
                raw_pixel_ev = noisy_scene_ev

            synthetic = _synthetic_frame(raw_pixel_ev)
            hg.push_meter_shot(synthetic, camera_ev_offset=cam_offset)

        rows.append({
            "time":       t.strftime("%H:%M"),
            "phase":      phase,
            "sun_alt":    round(sun_alt, 1),
            "ev_final":   round(params["ev_final"],  3),
            "ev_astro":   round(params["ev_astro"],  3),
            "shutter":    params["shutter"],
            "shutter_s":  params["shutter_s"],
            "iso":        params["iso"],
            "aperture":   params["aperture"],
            "kelvin":     params["kelvin"],
            "pixel_w":    round(params["pixel_weight"], 2),
            "astro_w":    round(params["astro_weight"], 2),
            "interval":   round(interval, 1),
            "r_squared":  round(params.get("r_squared", 0), 3),
            "slope_ma":   round(params.get("slope_ma",  0), 4),
        })

        t += datetime.timedelta(seconds=interval)

    return rows


# ── Output formatters ─────────────────────────────────────────────────────────

_PHASE_COLOR = {
    "day": "\033[33m", "golden": "\033[31m",
    "twilight": "\033[35m", "night": "\033[34m",
}
_RESET = "\033[0m"

def _bar(val: float, lo: float, hi: float, width: int = 20, char: str = "█") -> str:
    pct = max(0.0, min(1.0, (val - lo) / max(hi - lo, 1e-9)))
    filled = int(round(pct * width))
    return char * filled + "·" * (width - filled)

def print_table(rows: list[dict], step: int = 1, color: bool = True) -> None:
    hdr = (f"{'Time':6}  {'Phase':8}  {'Alt':>6}  {'EV':>7}  "
           f"{'Shutter':>10}  {'ISO':>6}  {'f/':>5}  {'K':>5}  "
           f"{'Pw':>4}  {'R²':>5}  {'Int':>5}")
    print(hdr)
    print("─" * len(hdr))

    prev_phase = None
    for r in rows[::step]:
        if r["phase"] != prev_phase:
            if prev_phase is not None:
                print()
            prev_phase = r["phase"]

        c = _PHASE_COLOR.get(r["phase"], "") if color else ""
        rs = _RESET if color else ""
        print(
            f"{c}{r['time']:6}{rs}  "
            f"{r['phase']:8}  "
            f"{r['sun_alt']:+6.1f}  "
            f"{r['ev_final']:+7.2f}  "
            f"{r['shutter']:>10}  "
            f"{r['iso']:6}  "
            f"{r['aperture']:5.1f}  "
            f"{r['kelvin']:5}  "
            f"{r['pixel_w']:4.2f}  "
            f"{r['r_squared']:5.3f}  "
            f"{r['interval']:5.1f}s"
        )


def print_graph(rows: list[dict], step: int = 1) -> None:
    """Mini terminal graph of shutter speed and ISO over time."""
    sampled = rows[::step]
    print("\n── Shutter (log scale, ← faster  longer →) ──────────────────────────────")
    for r in sampled:
        t_s = r["shutter_s"]
        bar = _bar(math.log2(max(t_s, 1/8000)), math.log2(1/8000), math.log2(30), 40)
        print(f"  {r['time']:6} {r['phase']:8}  {bar}  {r['shutter']:>10}")

    print("\n── ISO (log scale) ───────────────────────────────────────────────────────")
    for r in sampled:
        bar = _bar(math.log2(max(r['iso'], 100)), math.log2(100), math.log2(6400), 40)
        print(f"  {r['time']:6} {r['phase']:8}  {bar}  ISO {r['iso']}")

    print("\n── White balance (Kelvin) ────────────────────────────────────────────────")
    for r in sampled:
        bar = _bar(r['kelvin'], 3500, 6000, 40)
        print(f"  {r['time']:6} {r['phase']:8}  {bar}  {r['kelvin']}K")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--start",    default=None,
                    help="Start datetime 'YYYY-MM-DD HH:MM' (default: today 17:00)")
    ap.add_argument("--hours",    type=float, default=12.0,
                    help="Duration in hours (default 12)")
    ap.add_argument("--step",     type=int,   default=5,
                    help="Print every Nth frame (default 5)")
    ap.add_argument("--no-feedback", action="store_true",
                    help="Disable synthetic meter shots — astro model only")
    ap.add_argument("--noise",    type=float, default=0.12,
                    help="Simulated meter shot noise in EV stops (default 0.12)")
    ap.add_argument("--graph",    action="store_true",
                    help="Print terminal bar charts after the table")
    ap.add_argument("--json",     action="store_true",
                    help="Output JSON (for piping to a plotter)")
    ap.add_argument("--no-color", action="store_true")

    # HG settings overrides
    ap.add_argument("--lat",              type=float, default=49.8951)
    ap.add_argument("--lon",              type=float, default=-97.1384)
    ap.add_argument("--tz",                           default="America/Winnipeg")
    ap.add_argument("--anchor-shutter",   type=float, default=1/500,
                    help="Anchor shutter seconds (default 1/500 = 0.002)")
    ap.add_argument("--anchor-iso",       type=int,   default=100)
    ap.add_argument("--anchor-ev",        type=float, default=12.5,
                    help="Pixel EV at anchor calibration (default 12.5)")
    ap.add_argument("--iso-max-night",    type=int,   default=6400)
    ap.add_argument("--iso-native-high",  type=int,   default=640,
                    help="Dual-native ISO jump point (0=disable, 640=Sony A7III)")
    ap.add_argument("--shutter-max-night",type=float, default=25.0)
    ap.add_argument("--aperture-day",     type=float, default=5.6)
    ap.add_argument("--aperture-night",   type=float, default=4.0)
    ap.add_argument("--interval-day",     type=float, default=35.0)
    ap.add_argument("--interval-golden",  type=float, default=15.0)
    ap.add_argument("--interval-twilight",type=float, default=15.0)
    ap.add_argument("--interval-night",   type=float, default=35.0)
    args = ap.parse_args()

    tz = ZoneInfo(args.tz)
    if args.start:
        start = datetime.datetime.fromisoformat(args.start).replace(tzinfo=tz)
    else:
        now = datetime.datetime.now(tz)
        start = now.replace(hour=17, minute=0, second=0, microsecond=0)

    s = HGSettings(
        lat=args.lat, lon=args.lon, tz=args.tz,
        anchor_shutter_s=args.anchor_shutter,
        anchor_iso=args.anchor_iso,
        anchor_ev=args.anchor_ev,
        iso_max_night=args.iso_max_night,
        iso_native_high=args.iso_native_high,
        shutter_max_night=args.shutter_max_night,
        aperture_day=args.aperture_day,
        aperture_night=args.aperture_night,
        interval_day=args.interval_day,
        interval_golden=args.interval_golden,
        interval_twilight=args.interval_twilight,
        interval_night=args.interval_night,
    )
    hg = HolyGrailController(s)

    # In no-feedback mode, clear the anchor so the astro cold-start path runs
    # instead of holding at anchor_ev forever. This shows what the astro model
    # would plan across the sequence without any pixel corrections.
    if args.no_feedback:
        hg.settings.anchor_shutter_s = None
        hg.settings.anchor_iso       = None
        hg.settings.anchor_ev        = None

    if not args.json:
        print(f"\n{'─'*70}")
        print(f"  HG Simulation  |  {start.strftime('%Y-%m-%d %H:%M %Z')}  +{args.hours:.0f}h")
        print(f"  Anchor: {args.anchor_shutter:.4f}s / ISO{args.anchor_iso} / EV{args.anchor_ev}")
        print(f"  Night max: {args.shutter_max_night:.0f}s / ISO{args.iso_max_night}")
        print(f"  Aperture: f/{args.aperture_day} → f/{args.aperture_night}")
        print(f"  {'With' if not args.no_feedback else 'WITHOUT'} synthetic tracker feedback")
        print(f"{'─'*70}\n")

    rows = simulate(hg, start, args.hours,
                    with_feedback=not args.no_feedback,
                    noise_ev=args.noise)

    if args.json:
        print(json.dumps(rows, indent=2))
        return

    print_table(rows, step=args.step, color=not args.no_color)

    if args.graph:
        print_graph(rows, step=args.step)

    print(f"\n  {len(rows)} frames  ·  "
          f"step={args.step}  ·  "
          f"noise={args.noise}EV  ·  "
          f"{'feedback ON' if not args.no_feedback else 'feedback OFF'}")


if __name__ == "__main__":
    main()
