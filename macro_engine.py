#!/usr/bin/env python3
"""
macro_engine.py — Macro Focus Stack / 3D Scan Engine for PiSlider

Supports two sub-modes:
  SCAN  — even angular spacing, full 360° coverage, science-first
  ART   — easing curves, partial arcs, cinematic movement

Sequence loop (outer → inner):
  Project
    Orbit (one physical rig setup, one rotation_axis_angle)
      Rotation position (pan motor, outer loop)
        Exposure slot (relay state + camera settings)
          Focus increment (rail motor, inner loop)
            Capture

Rail always returns to start_mm at max speed between stacks.
sequence.json is written incrementally — every completed stack is marked done.
project.json is updated at orbit start and orbit completion.
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import datetime
from dataclasses import dataclass, field, asdict
from typing import Optional, List, Dict, Any, Callable

import numpy as np
from distributions import CURVE_FUNCTIONS, normalize
from slider import TimelapseTrajectoryPlayer

logger = logging.getLogger("PiSlider.Macro")

LEAD_SCREW_PITCH_MM  = 2.0
STEPS_PER_MM         = 800.0    # 200-step NEMA17 × 1/8 microstep / 2mm pitch ← matches hardware.py
PAN_STEPS_PER_DEG    = 66.667   # matches hardware.py: 200 × 8 × 15 / 360
TILT_STEPS_PER_DEG   = 133.333  # matches hardware.py: 200 × 8 × 30 / 360
RAIL_RETURN_VELOCITY = 50000    # TMC2209 VACTUAL units — fast return, no capture

# Physical speed of the focus rail at RAIL_RETURN_VELOCITY.
# TMC2209 VACTUAL formula: speed_usteps_s = VACTUAL × f_clk / 2^24 (f_clk ≈ 12 MHz)
# At STEPS_PER_MM microsteps/mm: speed_mm_s = speed_usteps_s / STEPS_PER_MM
# RAIL_RETURN_VELOCITY=50000 → ≈44.7 mm/s (confirmed against TMC2209 datasheet)
RAIL_RETURN_MM_S = RAIL_RETURN_VELOCITY * 12_000_000 / (2**24 * STEPS_PER_MM)  # ≈ 44.7 mm/s


def ease_in_out_cubic(t: float) -> float:
    """
    Smooth cubic ease-in-out easing function.
    Reduces jarring starts/stops for calibration stability.

    t: 0.0 (start) to 1.0 (end)
    Returns: eased position (0.0 to 1.0)
    """
    if t < 0.5:
        return 4 * t * t * t
    else:
        p = 2 * t - 2
        return 0.5 * p * p * p + 1


# ─── DATACLASSES ──────────────────────────────────────────────────────────────

@dataclass
class MattingConfig:
    """
    Triangulation (three-background) matting configuration.
    When enabled, each focus position captures up to three frames:
      <frame_id>_black  — subject against OLED black background
      <frame_id>_white  — subject against colour-temperature-matched white background
      <frame_id>_grey   — optional mid-tone grey (white dimmed to bg_brightness %)
    Alpha is computed offline from the pair or triplet.
    """
    enabled:         bool  = False
    black_settle_ms: int   = 300   # OLED off → on settle (true black is instant; extra safety margin)
    white_settle_ms: int   = 200   # white → black settle is slower — but black is captured first
    kelvin:          int   = 5500  # CT of the white background to match specimen lighting


@dataclass
class ExposureSlot:
    """One lighting / camera treatment applied at every focus position."""
    id:               str   = "slot_A"
    label:            str   = "diffuse"
    enabled:          bool  = True
    relay1:           bool  = False
    relay2:           bool  = False
    relay_settle_ms:  int   = 0      # wait BEFORE shutter after relay fires
    relay_release_ms: int   = 0      # wait AFTER shutter before relay releases
    iso:              int   = 400
    shutter_s:        float = 1/125
    kelvin:           int   = 5500
    ae:               bool  = False  # True = let camera auto-expose this slot
    awb:              bool  = False
    # Triangulation matting: set to "black", "white", or "grey" to command the
    # OLED phone background before this slot's capture.  Leave empty for no change.
    # kelvin sets the CT of white/grey; bg_brightness (0-100 %) dims the white
    # level for grey (50 = mid-tone grey card equivalent, ~18% reflectance).
    bg_color:         str   = ""
    bg_settle_ms:     int   = 300   # ms to wait after bg transition before shutter
    # brightness is controlled manually on the phone — not commanded per-slot


@dataclass
class LensProfile:
    name:               str   = "unknown"
    lens_type:          str   = "macro"   # telecentric | macro | other
    magnification:      float = 1.0
    working_distance_mm: float = 0.0
    sensor_pixel_um:    float = 3.92     # sensor pixel pitch in µm (Sony APS-C default)
    image_width_px:     int   = 6000     # sensor width for COLMAP cameras.txt
    image_height_px:    int   = 4000     # sensor height for COLMAP cameras.txt
    notes:              str   = ""


@dataclass
class MacroSession:
    """Complete configuration for one orbit sequence."""

    # Identity
    project_name:   str  = "macro_project"
    orbit_label:    str  = "orbit_001"
    session_mode:   str  = "scan"   # scan | art

    # Scan geometry
    scan_type:      str  = "orbit"   # orbit | grid_2d | geodesic
    # grid_2d: pan+tilt motors position the SPECIMEN; divides pan/tilt ranges into
    # a columns × rows grid of discrete viewing angles.
    pan_cols:       int  = 4         # columns along pan axis
    tilt_rows:      int  = 3         # rows along tilt axis
    grid_snake:     bool = True      # snake (boustrophedon) row order to minimise travel

    # Rig geometry for COLMAP pose computation
    pan_axis_tilt_deg: float = 90.0  # pan shaft tilt from vertical toward camera (degrees)
                                     # 90° = vertical axis (default), <90° = tilted toward camera

    # Multi-orbit tracking (for remount sessions — see orbit_label for naming)
    orbit_number:   int  = 1         # which physical remount this is (1, 2, 3…)
    orbit_notes:    str  = ""        # free-text description, e.g. "top of specimen"

    # Stereo 3D capture (for VR and 3D video)
    stereo_enabled: bool  = False    # True = capture left/right pairs for 3D video
    stereo_offset_deg: float = 3.0   # pan offset between left and right eyes (degrees)
                                     # Typical: 2°-5° for comfortable viewing

    # LEGO registration mount
    # A 4×4 LEGO brick used as the specimen base allows precise, repeatable
    # remounting at 0°/90°/180°/270° rotations around the vertical axis.
    # The inter-orbit transform is known exactly from the LEGO rotation angle,
    # so COLMAP poses from different orbits can be merged analytically.
    # Stud pitch = 8.0 mm; 4×4 block = 32 mm × 32 mm (rotation axis through centre).
    use_lego_mount:          bool  = False  # True if specimen is mounted on LEGO brick
    lego_rotation_deg:       float = 0.0   # rotation of LEGO relative to orbit 1 (0/90/180/270)
    lego_block:              str   = "4x4" # "4x4" | "2x4" | "2x2" etc. (metadata only)
    lego_stud_pitch_mm:      float = 8.0   # do not change — LEGO standard
    lego_rotation_axis_offset_mm: float = 0.0  # lateral offset of block centre from
                                                # pan motor axis (measure if block isn't centred)

    # Rail (focus axis — slider motor) — absolute step tracking from home position
    rail_start_steps:    int = 0        # absolute steps from home at start of stack
    rail_end_steps:      int = 4000     # absolute steps from home at end of stack
    images_per_stack:    int = 9        # number of focus images per stack (primary input)
    step_increment_steps: int = 250     # calculated: steps between consecutive focus images

    # Rotation stage (pan motor)
    rotation_mode:      str   = "full"   # full | range
    rotation_start_deg: float = 0.0
    rotation_end_deg:   float = 360.0
    num_stacks:         int   = 36
    rotation_easing:    str   = "even"   # any key from distributions.CURVE_FUNCTIONS
    rotation_axis_angle_deg: float = 90.0   # physical tilt of rotation axis (metadata)
    rotation_axis_description: str = "vertical"

    # Aux axis (tilt motor — optional creative use)
    aux_enabled:    bool  = False
    aux_label:      str   = "aux"
    aux_start_deg:  float = 0.0
    aux_end_deg:    float = 0.0
    aux_easing:     str   = "even"
    aux_soft_min:   float = -90.0
    aux_soft_max:   float =  90.0

    # Exposure slots (up to 2 for now, designed for easy extension)
    slots: List[ExposureSlot] = field(default_factory=lambda: [ExposureSlot()])

    # Timing
    vibe_delay_s:   float = 0.5    # anti-vibration settle after each motor move
    exp_margin_s:   float = 0.2    # extra wait after shutter for DNG write

    # Camera source (mirrors main app state)
    active_camera:  str  = "picam"

    # Lens metadata (saved to project.json)
    lens: LensProfile = field(default_factory=LensProfile)

    # Triangulation matting (two-background, OLED phone)
    matting: MattingConfig = field(default_factory=MattingConfig)

    # Storage
    save_path:      str  = "/home/tim/Pictures/PiSlider"


# ─── DERIVED CALCULATIONS ─────────────────────────────────────────────────────

def rail_frame_count(session: MacroSession) -> int:
    """Number of focus steps across the rail range.

    With absolute step tracking, this is always equal to images_per_stack.
    """
    return session.images_per_stack


def rail_step_mm(session: MacroSession) -> float:
    """Convert step increment to mm for logging/display purposes."""
    if session.step_increment_steps == 0:
        return 0.0
    return session.step_increment_steps / STEPS_PER_MM


def rotation_angles(session: MacroSession) -> List[float]:
    """Compute the list of rotation angles for all stacks."""
    n = session.num_stacks
    if n <= 0:
        return []
    if n == 1:
        return [session.rotation_start_deg]

    if session.rotation_mode == "full":
        # Even spacing around full 360° — last point does NOT repeat start
        return [session.rotation_start_deg + i * 360.0 / n for i in range(n)]
    else:
        # Range mode — include both endpoints
        span = session.rotation_end_deg - session.rotation_start_deg
        # Apply easing to the spacing
        weights = normalize(
            CURVE_FUNCTIONS.get(session.rotation_easing,
                                CURVE_FUNCTIONS["even"])(n)
        )
        angles = [session.rotation_start_deg]
        for w in weights[:-1]:
            angles.append(angles[-1] + w * span)
        angles[-1] = session.rotation_end_deg   # clamp last to exact end
        return angles[:n]


def aux_positions(session: MacroSession) -> List[float]:
    """
    Compute the list of aux (tilt) axis positions for orbit mode.

    If aux_enabled is False, returns num_stacks copies of aux_start_deg.
    If aux_enabled is True, sweeps from aux_start_deg to aux_end_deg
    using the aux_easing curve (same logic as rotation_angles for the pan axis).
    """
    n = session.num_stacks
    if n <= 0:
        return []
    if not session.aux_enabled:
        return [session.aux_start_deg] * n
    if n == 1:
        return [session.aux_start_deg]
    span = session.aux_end_deg - session.aux_start_deg
    weights = normalize(
        CURVE_FUNCTIONS.get(session.aux_easing, CURVE_FUNCTIONS["even"])(n)
    )
    angles = [session.aux_start_deg]
    for w in weights[:-1]:
        angles.append(angles[-1] + w * span)
    angles[-1] = session.aux_end_deg
    return angles[:n]


def steps_to_mm(steps: int) -> float:
    """Convert rail steps to millimetres using STEPS_PER_MM constant."""
    return steps / STEPS_PER_MM


def generate_scan_keyframes(session: MacroSession) -> List[Dict[str, Any]]:
    """Generate a flat list of keyframes for macro scan mode.

    Each keyframe dict contains:
        - "pan": pan angle in degrees
        - "tilt": tilt angle in degrees (or aux axis)
        - "rail_steps": absolute rail position in steps from home for each frame
        - "stack_index": index of the focus stack (0‑based)
        - "frame_index": index within the stack (0‑based)
    The order matches the execution order of the existing MacroEngine.
    """
    is_grid = session.scan_type == "grid_2d"
    angles = rotation_angles(session) if not is_grid else []
    aux_pos = aux_positions(session) if not is_grid else []
    grid_pos = grid_positions(session) if is_grid else []
    frames_per_stack = rail_frame_count(session)
    direction = 1 if session.rail_end_steps >= session.rail_start_steps else -1
    # Pre‑compute rail step positions for a single stack
    rail_steps_stack = [
        session.rail_start_steps + i * session.step_increment_steps * direction
        for i in range(frames_per_stack)
    ]
    # Ensure last step lands exactly on end
    if rail_steps_stack:
        rail_steps_stack[-1] = session.rail_end_steps
    keyframes: List[Dict[str, Any]] = []
    if is_grid:
        for stack_idx, (pan_deg, tilt_deg) in enumerate(grid_pos):
            for frame_idx, rail_steps in enumerate(rail_steps_stack):
                keyframes.append({
                    "pan": pan_deg,
                    "tilt": tilt_deg,
                    "rail_steps": rail_steps,
                    "stack_index": stack_idx,
                    "frame_index": frame_idx,
                })
    else:
        for stack_idx, rot_deg in enumerate(angles):
            a_deg = aux_pos[stack_idx] if session.aux_enabled else 0.0
            for frame_idx, rail_steps in enumerate(rail_steps_stack):
                keyframes.append({
                    "pan": rot_deg,
                    "tilt": a_deg,
                    "rail_steps": rail_steps,
                    "stack_index": stack_idx,
                    "frame_index": frame_idx,
                })
    return keyframes




def compute_geodesic_grid(total_stacks: int, pan_min: float, pan_max: float,
                         tilt_min: float, tilt_max: float,
                         pan_axis_tilt_deg: float = 90.0) -> tuple:
    """
    Compute optimal pan_cols and tilt_rows for even surface-area coverage.

    A naive rectangular (pan × tilt) grid produces dense clustering near the
    poles and sparse coverage at the equator — the opposite of what we want.
    The corrected weight for each tilt row accounts for the actual circumference
    of the pan arc at that tilt, which depends on BOTH the tilt motor angle AND
    the physical pan axis orientation.

    Derivation:
        For a pan axis unit vector k_pan and a camera at [0, 0, -D], the specimen
        surface point at motor angles (pan=θ, tilt=φ) traces a circle as θ varies.
        The radius of that circle (which sets the pan sample density needed) is:
            radius = |cos(φ_rad + alpha)|
        where alpha = radians(90 - pan_axis_tilt_deg) converts the UI angle to the
        same convention used by _colmap_pose().

        For a vertical pan axis (pan_axis_tilt_deg=90): alpha=0, radius=|cos(φ)|.
        The effective equator (widest circle, needs most samples) is at tilt=0°. ✓

        For a 45°-tilted axis (pan_axis_tilt_deg=45): alpha=45°, radius=|cos(φ+45°)|.
        The effective equator shifts to tilt=-45° (specimen must be tilted back to
        present its widest cross-section to the camera).

    Args:
        total_stacks:        desired total number of stacks
        pan_min, pan_max:    pan range (degrees)
        tilt_min, tilt_max:  tilt range (degrees)
        pan_axis_tilt_deg:   pan shaft angle (90°=vertical, <90°=toward camera)

    Returns:
        (pan_cols, tilt_rows) such that pan_cols × tilt_rows ≈ total_stacks
    """
    # Alpha converts UI pan axis angle to radians from vertical (same as _colmap_pose)
    alpha_rad = math.radians(90.0 - pan_axis_tilt_deg)

    # Start with rough estimate for tilt rows
    tilt_rows = max(2, int(math.sqrt(total_stacks / 2)))

    # Compute tilt angles
    if tilt_rows == 1:
        tilt_angles = [(tilt_min + tilt_max) / 2]
    else:
        tilt_span = tilt_max - tilt_min
        tilt_angles = [tilt_min + j * tilt_span / (tilt_rows - 1)
                       for j in range(tilt_rows)]

    # Effective pan arc radius at each tilt row, accounting for pan axis tilt.
    # |cos(tilt_rad + alpha_rad)| gives the circumference weight for that latitude.
    # Clamp to minimum so no row gets zero samples even near poles.
    weights = [max(0.05, abs(math.cos(math.radians(tilt) + alpha_rad)))
               for tilt in tilt_angles]
    total_weight = sum(weights)

    # Distribute stacks proportionally: more samples at wide latitudes (equator)
    if total_weight > 0:
        stacks_per_tilt = [int(round(w / total_weight * total_stacks))
                          for w in weights]
    else:
        stacks_per_tilt = [total_stacks // tilt_rows] * tilt_rows

    # Average pan_cols across tilt rows (single-column grids use this for the whole grid)
    pan_cols = max(1, int(round(sum(stacks_per_tilt) / tilt_rows)))

    return (pan_cols, tilt_rows)


# ── Node placement engine ─────────────────────────────────────────────────────
#
# Places an arbitrary number of capture nodes evenly over the reachable surface
# and orders them into a continuous serpentine path.
#
# Distribution: a Fibonacci (golden-angle) cap. Elevation is uniform in sin(e),
# which is equal-area, so nodes are DENSER around the middle and sparser at the
# poles. A naive spiral that steps elevation linearly oversamples the poles by
# about 1.7x. Measured nearest-neighbour CoV ~0.01, matching a subdivided
# icosphere while accepting any node count rather than 12/42/162/642.
#
# Ordering: golden-angle order puts consecutive nodes ~137 deg apart, so
# successive frames share no view. Re-sorting into elevation bands and snaking
# through them leaves every node exactly where it was (distribution untouched)
# while cutting the largest step between consecutive captures to ~26 deg.
# Fewer bands is better: more bands means fewer nodes per band and larger
# azimuth jumps within one.

_GOLDEN_ANGLE = math.pi * (3.0 - math.sqrt(5.0))


def _pan_axis_vec(pan_axis_tilt_deg: float):
    """Pan axis unit vector and alpha, matching _colmap_pose() exactly."""
    alpha = math.radians(90.0 - pan_axis_tilt_deg)
    return np.array([0.0, math.cos(alpha), -math.sin(alpha)]), alpha


def viewpoint_for(pan_deg: float, tilt_deg: float,
                  pan_axis_tilt_deg: float = 90.0) -> np.ndarray:
    """Unit vector from the specimen toward the camera, in specimen coords."""
    k, _ = _pan_axis_vec(pan_axis_tilt_deg)
    R = _R_rodrigues(k, math.radians(pan_deg)) @ _Rx(math.radians(tilt_deg))
    c = -R.T @ np.array([0.0, 0.0, 1.0])
    n = np.linalg.norm(c)
    return c / n if n > 1e-12 else c


def motors_for_viewpoint(v, pan_axis_tilt_deg: float = 90.0):
    """Inverse kinematics: viewpoint -> (pan_deg, tilt_deg), or None.

    From  -v = Rx(-phi) @ Rpan(-theta) @ zhat.  Rx leaves x untouched, so
        u_x = -cos(alpha) sin(theta) = -v_x   =>   sin(theta) = v_x/cos(alpha)
    which also gives the reachability test: a viewpoint is only reachable when
    |v_x| <= |cos(alpha)|. Tilting the pan axis away from the camera therefore
    costs coverage - at 135 deg only ~71% of the sphere can be reached at all.
    """
    k, alpha = _pan_axis_vec(pan_axis_tilt_deg)
    ca = math.cos(alpha)
    if abs(ca) < 1e-12 or abs(v[0]) > abs(ca) + 1e-9:
        return None
    best = None
    base = math.asin(max(-1.0, min(1.0, v[0] / ca)))
    for th in (base, math.pi - base):
        u = _R_rodrigues(k, -th) @ np.array([0.0, 0.0, 1.0])
        phi = math.atan2(u[2], u[1]) - math.atan2(-v[2], -v[1])
        pan = math.degrees(th) % 360.0
        if pan > 180.0:
            pan -= 360.0
        tilt = math.degrees(phi) % 360.0
        err = float(np.degrees(np.arccos(np.clip(
            float(np.dot(viewpoint_for(pan, tilt, pan_axis_tilt_deg), v)), -1.0, 1.0))))
        if best is None or err < best[0]:
            best = (err, pan, tilt)
    return (best[1], best[2]) if best else None


def fibonacci_nodes(n: int, elev_min_deg: float, elev_max_deg: float):
    """n unit vectors spread evenly over an elevation band (equal-area)."""
    n = max(1, int(n))
    z0, z1 = math.sin(math.radians(elev_min_deg)), math.sin(math.radians(elev_max_deg))
    out = []
    for i in range(n):
        z = z0 + (z1 - z0) * (i + 0.5) / n      # uniform in sin => equal area
        r = math.sqrt(max(0.0, 1.0 - z * z))
        a = i * _GOLDEN_ANGLE
        out.append(np.array([r * math.cos(a), r * math.sin(a), z]))
    return out


def serpentine_order(nodes, bands: int = 0):
    """Re-order nodes into a continuous path. Returns a list of indices.

    Picks the band count that minimises the largest angular step between
    consecutive nodes, unless one is given explicitly.
    """
    if len(nodes) < 3:
        return list(range(len(nodes)))
    P = np.array(nodes)
    elev = np.degrees(np.arcsin(np.clip(P[:, 2], -1, 1)))
    azim = np.degrees(np.arctan2(P[:, 1], P[:, 0])) % 360.0

    def build(nb):
        lo, hi = elev.min() - 1e-6, elev.max() + 1e-6
        edges = np.linspace(lo, hi, nb + 1)
        b = np.clip(np.digitize(elev, edges) - 1, 0, nb - 1)
        order, flip = [], False
        for kb in range(nb):
            idx = np.where(b == kb)[0]
            if not len(idx):
                continue
            idx = idx[np.argsort(azim[idx])]
            if flip:
                idx = idx[::-1]
            flip = not flip
            order += list(idx)
        return order

    def worst_step(order):
        Q = P[np.array(order)]
        d = np.degrees(np.arccos(np.clip(np.sum(Q[:-1] * Q[1:], axis=1), -1, 1)))
        return float(d.max()) if len(d) else 0.0

    if bands and bands > 0:
        return build(bands)
    cands = sorted({max(2, int(round(c))) for c in
                    (len(nodes) ** 0.5 / 2, len(nodes) ** 0.5, 4, 5, 6, 8, 10, 12)})
    best = None
    for nb in cands:
        o = build(nb)
        if len(o) != len(nodes):
            continue
        w = worst_step(o)
        if best is None or w < best[0]:
            best = (w, o)
    return best[1] if best else list(range(len(nodes)))


def _serpentine_motor_order(kept):
    """Serpentine through (pan band, tilt sweep); band count minimises worst step."""
    if len(kept) < 3:
        return list(range(len(kept)))
    P     = np.array([k[0] for k in kept])
    pans  = np.array([k[1] for k in kept])
    tilts = np.array([k[2] for k in kept])

    def build(nb):
        lo, hi = pans.min() - 1e-6, pans.max() + 1e-6
        edges = np.linspace(lo, hi, nb + 1)
        b = np.clip(np.digitize(pans, edges) - 1, 0, nb - 1)
        order, flip = [], False
        for kb in range(nb):
            idx = np.where(b == kb)[0]
            if not len(idx):
                continue
            idx = idx[np.argsort(tilts[idx])]
            if flip:
                idx = idx[::-1]
            flip = not flip
            order += list(idx)
        return order

    def build_elev(nb):
        elev = np.degrees(np.arcsin(np.clip(P[:, 2], -1, 1)))
        azim = np.degrees(np.arctan2(P[:, 1], P[:, 0])) % 360.0
        edges = np.linspace(elev.min() - 1e-6, elev.max() + 1e-6, nb + 1)
        b = np.clip(np.digitize(elev, edges) - 1, 0, nb - 1)
        order, flip = [], False
        for kb in range(nb):
            idx = np.where(b == kb)[0]
            if not len(idx):
                continue
            idx = idx[np.argsort(azim[idx])]
            if flip:
                idx = idx[::-1]
            flip = not flip
            order += list(idx)
        return order

    def worst(o):
        Q = P[np.array(o)]
        st = np.degrees(np.arccos(np.clip(np.sum(Q[:-1] * Q[1:], axis=1), -1, 1)))
        return float(st.max()) if len(st) else 0.0

    # Neither banding axis wins universally: banding by pan suits a tilted axis
    # (reachable region is a lune), banding by sphere elevation suits a vertical
    # one. Build both across a range of band counts and keep whichever gives the
    # smallest worst-case step between consecutive captures.
    best = None
    for mk in (build, build_elev):
        for nb in (3, 4, 5, 6, 8, 10, 12):
            o = mk(nb)
            if len(o) != len(kept):
                continue
            w = worst(o)
            if best is None or w < best[0]:
                best = (w, o)
    return best[1] if best else list(range(len(kept)))


def plan_geodesic_nodes(session: MacroSession) -> List[Dict[str, Any]]:
    """Full node plan: even distribution, reachability filter, serpentine path.

    Returns one dict per node with the motor angles and the sphere position, so
    the scan-graph page can draw the planned route before any capture happens.
    """
    n_req = max(1, int(getattr(session, "num_stacks", 1)))
    axis  = float(getattr(session, "pan_axis_tilt_deg", 90.0))
    pan_lo = min(session.rotation_start_deg, session.rotation_end_deg)
    pan_hi = max(session.rotation_start_deg, session.rotation_end_deg)
    e_lo   = float(getattr(session, "aux_start_deg", -60.0))
    e_hi   = float(getattr(session, "aux_end_deg",    60.0))
    if e_hi < e_lo:
        e_lo, e_hi = e_hi, e_lo

    # Distribute over the REACHABLE set, not over the whole sphere.
    #
    # Filtering a full-sphere distribution afterwards does not work: the
    # reachability test |v_x| <= |cos(alpha)| preferentially removes EQUATORIAL
    # nodes (those are the ones with large |v_x|), so a tilted axis ends up
    # denser at the poles - exactly backwards. Instead over-generate candidates,
    # keep only the reachable ones, then farthest-point sample from those. That
    # spreads n_req nodes evenly across whatever the rig can actually reach.
    pool = []
    for mult in (14, 30, 60):
        pool = []
        for v in fibonacci_nodes(max(n_req * mult, 400), e_lo, e_hi):
            m = motors_for_viewpoint(v, axis)
            if m is None:
                continue
            pan, tilt = m
            if pan < pan_lo - 1e-6 or pan > pan_hi + 1e-6:
                continue
            pool.append((v, pan, tilt))
        if len(pool) >= n_req * 3:
            break
    if not pool:
        return []

    if len(pool) <= n_req:
        kept = pool
    else:
        # Farthest-point sampling: repeatedly take the candidate furthest from
        # everything chosen so far. Gives near-uniform spacing over any region.
        Pp = np.array([q[0] for q in pool])
        first = int(np.argmax(Pp[:, 2]))
        chosen = [first]
        dmin = 1.0 - (Pp @ Pp[first])
        for _ in range(n_req - 1):
            nxt = int(np.argmax(dmin))
            chosen.append(nxt)
            dmin = np.minimum(dmin, 1.0 - (Pp @ Pp[nxt]))
        kept = [pool[i] for i in chosen]

    # Order the path in MOTOR space, not on the sphere.
    #
    # Banding by sphere elevation cuts across the reachable region, which for a
    # tilted pan axis is a lune rather than a cap - consecutive nodes can land on
    # disconnected arcs (measured 72 deg gaps). Banding by PAN (the cable-limited
    # axis) and sweeping TILT (free 360) within each band follows the reachable
    # shape and roughly halves the worst gap.
    order = _serpentine_motor_order(kept)
    out: List[Dict[str, Any]] = []
    for stack_i, idx in enumerate(order):
        v, pan, tilt = kept[idx]
        out.append({
            "stack":    stack_i,
            "pan_deg":  round(float(pan), 3),
            "tilt_deg": round(float(tilt), 3),
            "x": round(float(v[0]), 6),
            "y": round(float(v[1]), 6),
            "z": round(float(v[2]), 6),
            "state":    "pending",
        })
    return out


def generate_scan_positions(session: MacroSession) -> List[Dict[str, Any]]:
    """
    Generate all (pan, tilt, eye) positions for the scan, accounting for stereo mode.

    Returns list of dicts: [{"pan": deg, "tilt": deg, "eye": "left"|"right"}, ...]

    If stereo_enabled:
      Each position generates TWO entries: left eye at (pan, tilt) and right eye at (pan+offset, tilt)
    Otherwise:
      Each position generates ONE entry with eye="mono"
    """
    # Get base positions based on scan type
    if session.scan_type == "geodesic":
        # Even node placement over the reachable surface, serpentine-ordered.
        base_positions = [(n["pan_deg"], n["tilt_deg"])
                          for n in plan_geodesic_nodes(session)]
    elif session.scan_type == "grid_2d":
        base_positions = grid_positions(session)
    else:  # orbit mode
        base_positions = orbit_positions(session)

    result = []
    for pan, tilt in base_positions:
        if session.stereo_enabled:
            # Left eye
            result.append({"pan": pan, "tilt": tilt, "eye": "left"})
            # Right eye (offset pan)
            result.append({
                "pan": pan + session.stereo_offset_deg,
                "tilt": tilt,
                "eye": "right"
            })
        else:
            # Mono (single eye)
            result.append({"pan": pan, "tilt": tilt, "eye": "mono"})

    return result


def orbit_positions(session: MacroSession) -> List[tuple]:
    """
    For scan_type='orbit': return list of (pan_deg, tilt_deg) tuples.

    If aux_enabled is False (single orbit): all stacks at aux_start_deg tilt.
    If aux_enabled is True (sphere coverage): delegates to geodesic_orbit_positions()
    for an evenly distributed grid that accounts for the tilted pan axis.

    Returns num_stacks tuples in serpentine order.
    """
    if not session.aux_enabled:
        tilt = session.aux_start_deg
        return [(pan, tilt) for pan in rotation_angles(session)]
    return geodesic_orbit_positions(session)


def _fibonacci_sphere(n: int) -> List[tuple]:
    """
    Generate n approximately uniformly distributed points on the unit sphere
    using the Fibonacci / golden-angle spiral method.

    Returns list of (x, y, z) tuples, each a unit vector.
    """
    pts: List[tuple] = []
    gr  = (1.0 + math.sqrt(5.0)) / 2.0   # golden ratio ≈ 1.618
    for i in range(n):
        # Latitude: uniform in cos(theta) → uniform area per band
        theta = math.acos(1.0 - 2.0 * (i + 0.5) / n)
        phi   = 2.0 * math.pi * i / gr
        pts.append((math.sin(theta) * math.cos(phi),
                    math.sin(theta) * math.sin(phi),
                    math.cos(theta)))
    return pts


def _ik_orbit(vx: float, vy: float, vz: float,
               pan_axis_tilt_deg: float) -> List[tuple]:
    """
    Inverse kinematics: find all (pan_deg, tilt_deg) motor angle pairs
    whose viewpoint vector equals (vx, vy, vz).

    Forward model: viewpoint = (R_pan(pan) @ Rx(tilt))ᵀ @ [0,0,1]
    Constraint derived by projecting Rx(tilt)@v onto the pan-axis direction k_pan:

        vy·cos(tilt+α) − vz·sin(tilt+α) = −sin(α)

    where α = radians(90 − pan_axis_tilt_deg).  This is a single-variable
    trig equation with 0, 1, or 2 solutions per 360° period.  For each tilt
    solution, pan is recovered analytically.

    Returns list of (pan_deg, tilt_deg), primary branch (−180° < pan,tilt ≤ 180°).
    """
    alpha = math.radians(90.0 - pan_axis_tilt_deg)
    sa, ca = math.sin(alpha), math.cos(alpha)

    # ── Solve for tilt ─────────────────────────────────────────────────────────
    # A·cos(φ) + B·sin(φ) = C,  φ = tilt + alpha
    A, B, C = vy, -vz, -sa
    R2 = A * A + B * B
    if R2 < 1e-12:
        return []                          # degenerate (v along X axis)
    ratio = C / math.sqrt(R2)
    if ratio < -1.0 - 1e-9 or ratio > 1.0 + 1e-9:
        return []                          # no real solution
    ratio = max(-1.0, min(1.0, ratio))

    phi_base   = math.atan2(B, A)          # = atan2(−vz, vy)
    phi_offset = math.acos(ratio)

    results: List[tuple] = []
    for sign in (+1.0, -1.0):
        phi  = phi_base + sign * phi_offset   # = tilt + alpha  (radians)
        tilt = phi - alpha                    # tilt in radians

        # ── Recover pan from tilt ───────────────────────────────────────────────
        # w = Rx(tilt) @ v  →  R_pan(−pan) @ e_z = w
        # Rx convention: [[1,0,0],[0,cosθ,−sinθ],[0,sinθ,cosθ]]
        ct, st = math.cos(tilt), math.sin(tilt)
        wx =  vx
        wy =  vy * ct - vz * st
        wz =  vy * st + vz * ct

        # k_pan = [0, ca, −sa]
        # ez_perp = e_z − (e_z·k_pan)·k_pan = [0, sa·ca, ca²]  (precomputed)
        ez_px, ez_py, ez_pz = 0.0, sa * ca, ca * ca

        # w_perp = w − (w·k_pan)·k_pan
        wd   = wy * ca - wz * sa           # w · k_pan
        w_px = wx                          # k_pan[0] = 0
        w_py = wy - ca * wd
        w_pz = wz + sa * wd               # − (−sa)·wd

        ez_n2 = ez_px**2 + ez_py**2 + ez_pz**2
        w_n2  = w_px**2  + w_py**2  + w_pz**2

        if ez_n2 < 1e-12 or w_n2 < 1e-12:
            pan_deg = 0.0                  # at pole — pan undefined, default 0
        else:
            ez_n = math.sqrt(ez_n2)
            w_n  = math.sqrt(w_n2)
            # angle of (e_z_perp → w_perp) around k_pan = −pan
            cx = ez_py * w_pz - ez_pz * w_py
            cy = ez_pz * w_px - ez_px * w_pz
            cz = ez_px * w_py - ez_py * w_px
            sin_neg = (cy * ca + cz * (-sa)) / (ez_n * w_n)  # cross·k_pan
            cos_neg = (ez_px*w_px + ez_py*w_py + ez_pz*w_pz) / (ez_n * w_n)
            neg_pan  = math.atan2(sin_neg, cos_neg)
            pan_deg  = -math.degrees(neg_pan)

        results.append((pan_deg, math.degrees(tilt)))

    return results


def _farthest_first_select(xyzs: List[tuple], n: int) -> List[int]:
    """
    Greedy farthest-first selection: choose n indices from xyzs such that
    the minimum pairwise angular distance is maximised (≈ Poisson-disk on sphere).

    Uses dot product as the similarity metric (dot→1 = nearest on sphere).
    O(n × M) where M = len(xyzs).
    """
    m = len(xyzs)
    if n >= m:
        return list(range(m))

    # Start from the point nearest the centroid of all candidates
    cx = sum(p[0] for p in xyzs) / m
    cy = sum(p[1] for p in xyzs) / m
    cz = sum(p[2] for p in xyzs) / m
    start = min(range(m),
                key=lambda i: (xyzs[i][0]-cx)**2 + (xyzs[i][1]-cy)**2 + (xyzs[i][2]-cz)**2)

    selected = [start]
    remaining = list(range(m))
    remaining.remove(start)

    # min_dot[j] = min dot product with any already-selected point (lower = farther)
    sx, sy, sz = xyzs[start]
    min_dot = [sx*xyzs[j][0] + sy*xyzs[j][1] + sz*xyzs[j][2] for j in range(m)]

    for _ in range(n - 1):
        if not remaining:
            break
        # The remaining point farthest from all selected = lowest min_dot
        next_j = min(remaining, key=lambda j: min_dot[j])
        selected.append(next_j)
        remaining.remove(next_j)
        nx, ny, nz = xyzs[next_j]
        for j in remaining:
            d = nx*xyzs[j][0] + ny*xyzs[j][1] + nz*xyzs[j][2]
            if d < min_dot[j]:
                min_dot[j] = d

    return selected


def geodesic_orbit_positions(session: MacroSession) -> List[tuple]:
    """
    Two-phase node placement and routing for 3D orbit scans.

    Phase 1 — Placement (sphere space):
      Generate a dense Fibonacci sphere, run IK to find which points land inside
      the user's motor range, then select N of those using farthest-first (greedy
      max-min distance in XYZ).  This gives true geodesic coverage — every node
      is as far as possible from all its neighbours on the actual sphere, with no
      pole over-sampling and full compensation for the tilted pan axis.

    Phase 2 — Routing (motor space):
      A virtual serpentine stylus sweeps the motor range back and forth across
      n_rows tilt bands.  Each geodesic node is assigned the path-parameter of
      the nearest point on the stylus, then all nodes are sorted by that
      parameter.  The motor visits them in the order the stylus sweeps past —
      smooth, systematic, no backtracking.  The routing never alters node
      positions; it only determines the order they are shot.
    """
    N = session.num_stacks
    if N <= 0:
        return []
    if N == 1:
        return [(session.rotation_start_deg, session.aux_start_deg)]

    is_full      = (session.rotation_mode == "full")
    pan_min      = min(session.rotation_start_deg, session.rotation_end_deg)
    pan_max      = max(session.rotation_start_deg, session.rotation_end_deg)
    tilt_min     = min(session.aux_start_deg, session.aux_end_deg)
    tilt_max     = max(session.aux_start_deg, session.aux_end_deg)
    tilt_span    = tilt_max - tilt_min
    pan_span     = 360.0 if is_full else abs(pan_max - pan_min)
    is_full_tilt = tilt_span >= 355.0
    alpha_deg    = getattr(session, "pan_axis_tilt_deg", 45.0)

    # ── Helical stylus path ────────────────────────────────────────────────────
    # Pan is the slow axis, tilt is the fast axis — like an apple peeler.
    # The stylus spirals continuously: tilt rotates through n_turns full sweeps
    # while pan slowly advances from one edge to the other.  This is the natural
    # path for 360° tilt because tilt never reverses — it just keeps winding.
    # For partial tilt ranges the same helix still works; it just makes partial
    # turns instead of full ones.
    #
    # n_turns chosen so the between-turn spacing (in pan/sphere) ≈ within-turn
    # spacing (in tilt/sphere), giving uniform node density across the patch.

    # Pan arm always has physical cable limits — never generate positions outside
    # the configured pan range even if rotation_mode=="full".  "Full" in the
    # geodesic orbit context means full-sphere TILT coverage (aux ±180°), not a
    # 360° continuous pan revolution which the arm cannot do.
    #
    # Start from home (0° if in range) and sweep to ONE side only (the farther
    # edge).  With full 360° tilt, the two pan directions produce mirror-image
    # great-circle fans — sweeping both sides is redundant and creates two lobes.
    # Geodesic orbit: the pan ARM cannot do continuous 360° rotation (cable limits).
    # If the user selected rotation_mode="full" the start/end become 0°/360° which
    # is dangerous.  Detect this and refuse to generate out-of-range positions.
    if (pan_max - pan_min) >= 350.0:
        logger.error(
            "geodesic_orbit_positions: pan range spans ≥350° — this looks like "
            "rotation_mode='full' was used with a cable-limited pan arm.  "
            "Switch to rotation_mode='range' and set your actual physical pan limits.  "
            "Clamping pan span to 90° as an emergency fallback."
        )
        # Emergency fallback: just go from home 90° in the positive direction
        pan_min  = 0.0
        pan_max  = 90.0

    pan_home = float(np.clip(0.0, pan_min, pan_max))
    # Choose far edge: whichever of pan_min / pan_max is farthest from home
    pan_from = pan_home
    pan_to   = pan_min if abs(pan_min - pan_home) >= abs(pan_max - pan_home) else pan_max
    pan_span = abs(pan_to - pan_from)
    if pan_span < 1.0:
        logger.warning("Pan range too narrow for geodesic helix — defaulting to 90° span")
        pan_span = 90.0
        pan_to   = pan_from + pan_span

    pan_frac  = pan_span  / 360.0
    tilt_frac = tilt_span / 360.0          # fraction of a full tilt revolution
    # turns² × tilt_frac ≈ N × pan_frac  →  turns ≈ sqrt(N × pan_frac / tilt_frac)
    n_turns   = max(1, int(round(math.sqrt(N * pan_frac / max(tilt_frac, 0.05)))))

    logger.info(
        f"Helix: {n_turns} turns, pan {pan_from:.0f}°→{pan_to:.0f}°, "
        f"tilt sweeps {n_turns} × {tilt_span:.0f}° = {n_turns * tilt_span:.0f}° total"
    )

    # Sample the helix densely for arc-length integration
    SAMPLES = max(5000, N * 40)
    path_pan:  List[float] = []
    path_tilt: List[float] = []
    for k in range(SAMPLES):
        t = k / (SAMPLES - 1)
        path_pan.append(pan_from + t * (pan_to - pan_from))
        # Tilt winds continuously from 0°; sin/cos in FK handle values > 360°
        path_tilt.append(t * n_turns * tilt_span)

    # Forward kinematics → unit viewpoint vectors; accumulate sphere arc length
    vecs = [_pan_tilt_to_xyz(p, t, alpha_deg) for p, t in zip(path_pan, path_tilt)]
    arc: List[float] = [0.0]
    for i in range(1, len(vecs)):
        dot = float(np.clip(np.dot(vecs[i - 1], vecs[i]), -1.0, 1.0))
        arc.append(arc[-1] + math.acos(dot))

    total_arc = arc[-1]
    if total_arc < 1e-9:
        logger.warning("Helix path has zero arc length — returning uniform fallback")
        return [(pan_from + k * (pan_to - pan_from) / max(1, N - 1), 0.0)
                for k in range(N)]

    logger.info(
        f"Helix arc: {math.degrees(total_arc):.1f}° total, "
        f"{math.degrees(total_arc / (N - 1)):.1f}°/node spacing"
    )

    # Place N nodes at equal arc-length intervals — geodesic compensation for
    # the 45° tilt axis falls out automatically: where the helix sweeps slowly
    # across the sphere the arc budget is spent quickly, packing nodes tighter
    # in motor space; where it sweeps quickly they spread further apart.
    import bisect
    result: List[tuple] = []
    for k in range(N):
        target = total_arc * k / (N - 1)
        idx = bisect.bisect_left(arc, target)
        idx = max(1, min(len(arc) - 1, idx))
        a0, a1 = arc[idx - 1], arc[idx]
        f = (target - a0) / (a1 - a0) if a1 > a0 else 0.0
        pan  = path_pan[idx - 1]  + f * (path_pan[idx]  - path_pan[idx - 1])
        tilt = path_tilt[idx - 1] + f * (path_tilt[idx] - path_tilt[idx - 1])
        # Hard clamp: pan must never leave the configured range (safety net for
        # the pan arm which cannot do continuous 360° due to cable limits).
        pan = float(np.clip(pan, pan_min, pan_max))
        result.append((pan, tilt))

    # Tilt values are naturally monotonically increasing (unwrapped helix) —
    # _move_rotation uses delta = target − current so large absolute values are
    # fine; each step moves only a small delta.
    return result

    return _stylus_sort(selected,
                        pan_min=pan_min, pan_max=pan_max,
                        tilt_min=tilt_min, tilt_max=tilt_max,
                        n_rows=n_rows,
                        tilt_circular=is_full_tilt,
                        start_pan=0.0, start_tilt=0.0)


def _pan_tilt_to_xyz(pan_deg: float, tilt_deg: float,
                     pan_axis_tilt_deg: float) -> "np.ndarray":
    """
    Convert (pan_motor, tilt_motor) angles to a 3D unit viewpoint vector.

    The viewpoint is the direction from the specimen to the camera in specimen
    coordinates.  Since the camera is fixed at +Z and the specimen rotates by
    R = R_pan @ R_tilt, the camera appears at R⁻¹ @ [0,0,1] = Rᵀ @ [0,0,1]
    from the specimen's frame.

    This matches panTiltToXYZ() in macro-graph.html so the NN sort finds
    nearest neighbours in the same space the sphere visualises them — the two
    are otherwise mirror images and the drawn path looks random.
    """
    alpha = math.radians(90.0 - pan_axis_tilt_deg)
    k_pan = np.array([0.0, math.cos(alpha), -math.sin(alpha)])
    R = _R_rodrigues(k_pan, math.radians(pan_deg)) @ _Rx(math.radians(tilt_deg))
    return R.T @ np.array([0.0, 0.0, 1.0])   # Rᵀ @ [0,0,1] = viewpoint direction


def _stylus_sort(positions: List[tuple],
                 pan_min: float, pan_max: float,
                 tilt_min: float, tilt_max: float,
                 n_rows: int,
                 tilt_circular: bool = False,
                 start_pan: float = 0.0,
                 start_tilt: float = 0.0) -> List[tuple]:
    """
    Sort (pan, tilt) positions by a virtual serpentine stylus sweeping the motor range.

    Imagine a stylus tracing a boustrophedon (back-and-forth) path across the full
    motor range: it sweeps pan_min→pan_max (or reverse) in each of n_rows tilt bands,
    alternating direction each row.  Each geodesic node is assigned the path-parameter
    of the stylus point that passes closest to it, then all nodes are sorted by that
    parameter.  This gives smooth, spatially coherent traversal even for irregular
    (farthest-first geodesic) node placements — no nearest-neighbour clustering, no
    large jumps.

    For full 360° tilt the stylus path wraps continuously: after the top tilt row it
    steps ±25° (circular) to the next, so nodes on the "other side" of the tilt range
    are visited in the same small-step sequence rather than requiring one big leap.

    Output tilt values are unwrapped so every consecutive motor delta is ≤180°
    (_move_rotation uses delta = target − current).
    """
    if len(positions) <= 1:
        return list(positions)

    pan_span  = pan_max  - pan_min
    tilt_span = tilt_max - tilt_min
    PERIOD    = 360.0

    # ── Tilt level for each stylus row ────────────────────────────────────────
    if n_rows == 1:
        tilt_levels = [(tilt_min + tilt_max) / 2.0]
    elif tilt_circular:
        # Full circle: n_rows equally-spaced levels, endpoint NOT repeated
        step = tilt_span / n_rows
        tilt_levels = [tilt_min + r * step for r in range(n_rows)]
    else:
        step = tilt_span / (n_rows - 1)
        tilt_levels = [tilt_min + r * step for r in range(n_rows)]

    def _tilt_dist(a: float, b: float) -> float:
        d = abs(a - b)
        return min(d, PERIOD - d) if tilt_circular else d

    # ── Rotate row order to start nearest to start_tilt ───────────────────────
    s = min(range(n_rows), key=lambda r: _tilt_dist(tilt_levels[r], start_tilt))
    row_order = list(range(s, n_rows)) + list(range(0, s))
    # row_order[i] indexes tilt_levels; stylus visits them in this sequence.

    # ── First-row direction: pan end nearest start_pan ────────────────────────
    # row0_going_right=True  → stylus sweeps pan_min→pan_max (ascending progress)
    # row0_going_right=False → stylus sweeps pan_max→pan_min (descending progress)
    row0_going_right = (abs(pan_min - start_pan) <= abs(pan_max - start_pan))

    def _path_param(pan: float, tilt: float) -> float:
        """Scalar path parameter: integer part = row index, fraction = pan progress."""
        # Nearest row in the reordered sequence
        ri = min(range(n_rows),
                 key=lambda i: _tilt_dist(tilt, tilt_levels[row_order[i]]))

        # Pan progress 0→1 from pan_min to pan_max
        prog = (pan - pan_min) / pan_span if pan_span > 0 else 0.5
        prog = max(0.0, min(1.0, prog))

        # Alternate direction each row in the reordered sequence
        going_right = row0_going_right if (ri % 2 == 0) else (not row0_going_right)
        if not going_right:
            prog = 1.0 - prog          # descending → invert progress

        return float(ri) + prog

    # ── Assign parameters and sort ────────────────────────────────────────────
    keyed = sorted(zip((_path_param(p, t) for p, t in positions), positions))
    result = [pos for _, pos in keyed]

    # ── Unwrap circular tilt ──────────────────────────────────────────────────
    if tilt_circular and len(result) > 1:
        unwrapped: List[tuple] = [result[0]]
        prev_t = result[0][1]
        for pan, tilt in result[1:]:
            while tilt - prev_t >  180.0:
                tilt -= 360.0
            while tilt - prev_t < -180.0:
                tilt += 360.0
            unwrapped.append((pan, tilt))
            prev_t = tilt
        return unwrapped

    return result


def _nn_sort_motor(positions: List[tuple],
                   start_pan: float = 0.0,
                   start_tilt: float = 0.0,
                   tilt_circular: bool = False) -> List[tuple]:
    """
    Greedy nearest-neighbour sort in motor-angle (pan, tilt) space, followed by
    a 2-opt improvement pass to eliminate pathological long edges.

    Phase 1 — Greedy NN:
      Always visits the closest unvisited node next (Euclidean in pan/tilt degrees,
      with circular tilt wrapping when tilt_circular=True).  Works well for geodesic
      nodes whose pan/tilt angles don't form regular grid rows.

    Phase 2 — 2-opt improvement:
      Repeatedly tests all pairs of path edges; if swapping them (reversing the
      segment between them) reduces total travel, it does so.  This eliminates the
      worst long jumps that greedy NN leaves behind when kinematic "regions" are
      separated in motor space.  O(N²) per pass, typically 1-3 passes needed.

    For circular tilt the final output tilt values are unwrapped so consecutive
    steps are ≤180° — _move_rotation uses delta = target − current.
    """
    if len(positions) <= 2:
        return list(positions)

    TILT_PERIOD = 360.0

    def _circ_dt(dt: float) -> float:
        """Circular-tilt delta: shortest signed arc."""
        if tilt_circular:
            dt = dt % TILT_PERIOD
            if dt > TILT_PERIOD / 2.0:
                dt -= TILT_PERIOD
        return dt

    def _dist2(p1: tuple, p2: tuple) -> float:
        dp = p2[0] - p1[0]
        dt = _circ_dt(p2[1] - p1[1])
        return dp * dp + dt * dt

    # ── Phase 1: greedy NN ─────────────────────────────────────────────────────
    remaining = list(positions)
    result: List[tuple] = []
    curr = (start_pan, start_tilt)

    while remaining:
        ni = min(range(len(remaining)), key=lambda i: _dist2(curr, remaining[i]))
        curr = remaining[ni]
        result.append(remaining.pop(ni))

    # ── Phase 2: 2-opt improvement ─────────────────────────────────────────────
    # For a path (not cycle), edge (i,i+1) and (j,j+1) reversed gives:
    #   old cost: d(i,i+1) + d(j,j+1)
    #   new cost: d(i,j)   + d(i+1,j+1)
    # If new < old, reverse path[i+1 .. j].
    n = len(result)
    improved = True
    while improved:
        improved = False
        for i in range(n - 2):
            for j in range(i + 2, n - 1):
                d_old = _dist2(result[i], result[i + 1]) + _dist2(result[j], result[j + 1])
                d_new = _dist2(result[i], result[j])     + _dist2(result[i + 1], result[j + 1])
                if d_new < d_old - 1e-9:
                    result[i + 1: j + 1] = result[i + 1: j + 1][::-1]
                    improved = True

    # ── Unwrap circular tilt ───────────────────────────────────────────────────
    if tilt_circular and len(result) > 1:
        unwrapped: List[tuple] = [result[0]]
        prev_t = result[0][1]
        for pan, tilt in result[1:]:
            while tilt - prev_t >  180.0:
                tilt -= 360.0
            while tilt - prev_t < -180.0:
                tilt += 360.0
            unwrapped.append((pan, tilt))
            prev_t = tilt
        return unwrapped

    return result


def _serpentine_sort(positions: List[tuple],
                     start_pan: float = 0.0,
                     start_tilt: float = 0.0,
                     tilt_circular: bool = False) -> List[tuple]:
    """
    Sort (pan, tilt) positions in serpentine (boustrophedon) order.

    Groups positions into tilt rows, then:
      1. Rotates the row order so the row closest to start_tilt comes first.
         With tilt_circular=True (full 360° rotation), tilt distance wraps at
         ±180° so the row reorder correctly joins the last ascending row to the
         first descending row as a small step rather than a 360° jump.
      2. Within the first row, starts at the individual node closest to start_pan,
         then sweeps toward the nearer endpoint first.
      3. Alternates pan direction for each subsequent row.
    """
    if len(positions) <= 2:
        return positions

    from collections import defaultdict
    rows: dict = defaultdict(list)
    for pan, tilt in positions:
        rows[tilt].append(pan)

    sorted_tilts = sorted(rows.keys())
    tilt_period  = 360.0  # used only when tilt_circular=True

    def _tilt_dist(a: float, b: float) -> float:
        d = abs(a - b)
        return min(d, tilt_period - d) if tilt_circular else d

    # ── Rotate row order to start at the tilt row nearest start_tilt ──────────
    start_idx       = min(range(len(sorted_tilts)),
                          key=lambda i: _tilt_dist(sorted_tilts[i], start_tilt))
    reordered_tilts = sorted_tilts[start_idx:] + sorted_tilts[:start_idx]

    result: List[tuple] = []
    # going_right = True  →  current row outputs pans ascending  (left → right)
    # going_right = False →  current row outputs pans descending (right → left)
    going_right = True

    for row_idx, tilt in enumerate(reordered_tilts):
        pans = sorted(rows[tilt])   # ascending order

        if row_idx == 0:
            # Choose which endpoint to start at: pick the one closer to start_pan
            # so the very first stack requires the smallest initial motor move.
            if abs(pans[-1] - start_pan) <= abs(pans[0] - start_pan):
                # Right end (pan_max) is closer → start there, sweep left
                pans = list(reversed(pans))
                going_right = False   # this row goes right→left (descending)
            else:
                # Left end (pan_min) is closer → start there, sweep right
                going_right = True    # this row goes left→right (ascending)
        else:
            # Alternate direction each row
            going_right = not going_right
            if not going_right:       # descending: reverse so we start at pan_max
                pans = list(reversed(pans))

        for pan in pans:
            result.append((pan, tilt))

    # ── Circular-tilt unwrap ──────────────────────────────────────────────────
    # When tilt is full-circle, -180° and +180° are the same motor position.
    # _move_rotation uses delta = target - current, so a result tilt of -180°
    # after +154° would command a 334° move instead of the intended 25.71° step.
    # Unwrap consecutive tilt values so each step ≤ 180° (motor takes short path).
    if tilt_circular and len(result) > 1:
        unwrapped: List[tuple] = [result[0]]
        prev_tilt = result[0][1]
        for pan, tilt in result[1:]:
            while tilt - prev_tilt >  180.0:
                tilt -= 360.0
            while tilt - prev_tilt < -180.0:
                tilt += 360.0
            unwrapped.append((pan, tilt))
            prev_tilt = tilt
        return unwrapped

    return result


def grid_positions(session: MacroSession) -> List[tuple]:
    """
    For scan_type='grid_2d': return list of (pan_deg, tilt_deg) tuples covering
    the pan_start→pan_end × tilt_start→tilt_end grid in snake (boustrophedon) order.

    Pan spans [rotation_start_deg .. rotation_end_deg] in pan_cols steps.
    Tilt spans [aux_start_deg    .. aux_end_deg]       in tilt_rows steps.
    """
    cols = max(1, session.pan_cols)
    rows = max(1, session.tilt_rows)

    if cols == 1:
        pan_angles = [session.rotation_start_deg]
    else:
        span = session.rotation_end_deg - session.rotation_start_deg
        pan_angles = [session.rotation_start_deg + i * span / (cols - 1)
                      for i in range(cols)]

    if rows == 1:
        tilt_angles = [session.aux_start_deg]
    else:
        span = session.aux_end_deg - session.aux_start_deg
        tilt_angles = [session.aux_start_deg + j * span / (rows - 1)
                       for j in range(rows)]

    positions: List[tuple] = []
    for row_idx, tilt in enumerate(tilt_angles):
        row_pans = pan_angles if (not session.grid_snake or row_idx % 2 == 0) \
                   else list(reversed(pan_angles))
        for pan in row_pans:
            positions.append((pan, tilt))
    return positions


def stereo_multiplier(session: MacroSession) -> int:
    """Return 2 if stereo enabled (L+R), else 1."""
    return 2 if session.stereo_enabled else 1


def num_stacks_grid(session: MacroSession) -> int:
    """Total stacks in grid_2d mode = pan_cols × tilt_rows (×2 if stereo)."""
    base_stacks = max(1, session.pan_cols) * max(1, session.tilt_rows)
    return base_stacks * stereo_multiplier(session)


def depth_per_image_um(session: MacroSession) -> float:
    """
    Actual depth step between frames in micrometres.
    Calculated from step_increment_steps.
    Returns 0 if only one frame in the stack.
    """
    if session.images_per_stack <= 1:
        return 0.0
    # Convert steps to mm, then to micrometers: steps / STEPS_PER_MM * 1000
    return (session.step_increment_steps / STEPS_PER_MM) * 1000.0


def effective_pixel_um(session: MacroSession) -> float:
    """Object-plane pixel size in µm (sensor pixel / magnification)."""
    m = session.lens.magnification
    if m <= 0:
        return session.lens.sensor_pixel_um
    return session.lens.sensor_pixel_um / m


def total_image_count(session: MacroSession) -> int:
    """Total images including stereo pairs (L+R) if enabled."""
    enabled_slots = sum(1 for s in session.slots if s.enabled)
    if session.scan_type == "grid_2d":
        n_stacks = num_stacks_grid(session)  # already multiplied by stereo_multiplier
    else:
        # Orbit mode: multiply by stereo after counting base stacks
        n_stacks = session.num_stacks * stereo_multiplier(session)
    return rail_frame_count(session) * n_stacks * max(1, enabled_slots)


def estimated_storage_gb(session: MacroSession, mb_per_frame: float = 25.0) -> float:
    return total_image_count(session) * mb_per_frame / 1024.0


# ─── COLMAP POSE HELPERS ─────────────────────────────────────────────────────
#
# Coordinate system (world = specimen frame):
#   +Y  up
#   +Z  away from camera (camera is at [0, 0, -D] looking in +Z toward specimen)
#   +X  camera right
#
# Pan motor geometry:
#   The pan shaft is tilted pan_axis_tilt_deg degrees from vertical (+Y) toward
#   the camera (-Z direction).  At 45°: k_pan = [0, cos45, -sin45] = [0,√½,−√½].
#
# Tilt motor geometry:
#   The tilt motor sits ON the pan assembly, so its axis rotates with pan.
#   At pan=0 the tilt axis aligns with +X (perpendicular to k_pan in the X-Z plane).
#
# Kinematic chain:  R_spec = R_pan @ R_tilt
#   (pan rotates the whole tilt+specimen assembly; tilt acts first in pan's local frame)
#
# World-to-camera transform:
#   R_wc = R_spec   (NOT R_spec.T — see derivation below)
#   t    = [0, 0, D]  (constant for any pan/tilt)
#
# Derivation:
#   Camera in lab frame always at C_lab = [0, 0, -D].
#   World (specimen) frame rotates with specimen: C_world = R_spec^T @ C_lab.
#   COLMAP: x_cam = R_wc @ x_world + t,  camera centre: -R_wc^T @ t = C_world.
#   ⟹  R_wc = R_spec,  t = -R_spec @ C_world = -R_spec @ R_spec^T @ [0,0,-D] = [0,0,D] ✓

def _Rx(a: float) -> "np.ndarray":
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def _R_rodrigues(k: "np.ndarray", angle_rad: float) -> "np.ndarray":
    """
    Rotation matrix for angle_rad around unit axis k (Rodrigues formula).
    R = cos(θ)I + sin(θ)[k]× + (1−cos(θ)) k⊗k
    """
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    K = np.array([[ 0,     -k[2],  k[1]],
                  [ k[2],   0,    -k[0]],
                  [-k[1],  k[0],   0   ]], dtype=float)
    return c * np.eye(3) + s * K + (1 - c) * np.outer(k, k)


def _telecentric_f_px(distance_mm: float, eff_um: float) -> float:
    """
    Calibrated PINHOLE focal length (pixels) for a telecentric lens.

    A telecentric lens has orthographic (parallel-ray) projection.
    COLMAP has no orthographic model, so we use PINHOLE with the focal
    length that makes it produce the same pixel coordinates as the
    true orthographic model at depth Z = distance_mm (the working distance).

    Derivation:
        PINHOLE:     x_px = f * X / Z
        Telecentric: x_px = X / (eff_um / 1000)
        At Z = distance_mm:  f = distance_mm * 1000 / eff_um

    eff_um: effective pixel size at specimen plane in µm
            = sensor_pixel_um / magnification

    IMPORTANT: lock this value in COLMAP (--refine_focal_length 0).
    If COLMAP re-estimates focal length it will try to fit a perspective
    model, producing wrong intrinsics.
    """
    if eff_um <= 0:
        return 1_000_000.0
    return distance_mm * 1000.0 / eff_um


def _colmap_pose(pan_deg: float, tilt_deg: float, distance_mm: float,
                 pan_axis_tilt_deg: float = 90.0):
    """
    World-to-camera (R, t) for the PiSlider pan-tilt rig.

    pan_deg             : pan motor angle (degrees, positive = CCW from above)
    tilt_deg            : tilt motor angle (degrees)
    distance_mm         : camera working distance from specimen centre
    pan_axis_tilt_deg   : pan shaft angle in degrees, measured from camera direction.
                          90° = vertical pan axis (perpendicular to camera).
                          < 90° = axis tilted toward camera.
                          > 90° = axis tilted away from camera.
                          UI range 45–135°; default 90° (vertical).

    Returns (R: np.ndarray 3×3, t: np.ndarray 3) in COLMAP convention.
    """
    # Convert UI angle to radians from vertical.
    # UI: 90° = vertical (+Y axis). alpha=0 → k_pan=[0,1,0].
    # Formula: alpha = radians(90 - pan_axis_tilt_deg)
    #   pan_axis_tilt_deg=90  → alpha=  0° → k_pan=[0, 1,  0] = vertical ✓
    #   pan_axis_tilt_deg=45  → alpha= 45° → k_pan=[0, √½,-√½] = tilted toward camera ✓
    #   pan_axis_tilt_deg=135 → alpha=-45° → k_pan=[0, √½, √½] = tilted away from camera ✓
    alpha = math.radians(90.0 - pan_axis_tilt_deg)
    # Pan axis unit vector: tilted from +Y toward -Z (toward camera)
    k_pan = np.array([0.0, math.cos(alpha), -math.sin(alpha)])

    R_pan  = _R_rodrigues(k_pan, math.radians(pan_deg))
    R_tilt = _Rx(math.radians(tilt_deg))
    R = R_pan @ R_tilt                    # R_spec = R_wc
    t = np.array([0.0, 0.0, distance_mm])  # always constant
    return R, t


def _R_to_quat(R: "np.ndarray"):
    """3×3 rotation matrix → (qw, qx, qy, qz) in COLMAP convention."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0)
        return 0.25 / s, (R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        return (R[2,1]-R[1,2])/s, 0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s
    elif R[1,1] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        return (R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        return (R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s


# ─── FOLDER / JSON HELPERS ───────────────────────────────────────────────────

def project_folder(session: MacroSession) -> str:
    date_str = datetime.datetime.now().strftime("%Y-%m-%d")
    return os.path.join(session.save_path, f"{session.project_name}_{date_str}")


def orbit_folder(proj_folder: str, orbit_label: str) -> str:
    return os.path.join(proj_folder, orbit_label)


def stack_folder(orb_folder: str, stack_idx: int,
                 rot_deg: float, aux_deg: Optional[float] = None) -> str:
    rot_str = f"rot{rot_deg:+08.3f}"
    if aux_deg is not None:
        rot_str += f"_aux{aux_deg:+07.2f}"
    return os.path.join(orb_folder, f"stack_{stack_idx+1:03d}", rot_str)


def slot_folder(stk_folder: str, slot: ExposureSlot) -> str:
    # Include bg_color in dir name when set so pairs are self-documenting:
    # slot_A_black/ and slot_B_white/ rather than just slot_A/ and slot_B/
    suffix = f"_{slot.bg_color}" if slot.bg_color else ""
    return os.path.join(stk_folder, f"{slot.id}{suffix}")


def _serial(obj):
    """JSON-serialise dataclass or plain dict recursively."""
    if hasattr(obj, '__dataclass_fields__'):
        return {k: _serial(v) for k, v in asdict(obj).items()}
    if isinstance(obj, list):
        return [_serial(i) for i in obj]
    return obj


def write_sequence_json(orb_folder: str, session: MacroSession,
                        stacks_meta: List[Dict]) -> str:
    is_grid = session.scan_type == "grid_2d"
    angles  = rotation_angles(session)
    aux_pos = aux_positions(session)
    grid_pos = grid_positions(session) if is_grid else []
    doc = {
        "version":       1,
        "created":       datetime.datetime.now().isoformat(),
        "session_mode":  session.session_mode,
        "scan_type":     session.scan_type,
        "orbit_label":   session.orbit_label,
        "rotation_axis_angle_deg":        session.rotation_axis_angle_deg,
        "rotation_axis_description":      session.rotation_axis_description,
        "rail": {
            "start_steps":     session.rail_start_steps,
            "end_steps":       session.rail_end_steps,
            "step_increment":  session.step_increment_steps,
            "step_mm":         rail_step_mm(session),
            "frame_count":     rail_frame_count(session),
            "travel_steps":    abs(session.rail_end_steps - session.rail_start_steps),
            "travel_mm":       abs(session.rail_end_steps - session.rail_start_steps) / STEPS_PER_MM,
        },
        "rotation": {
            "mode":            session.rotation_mode,
            "start_deg":       session.rotation_start_deg,
            "end_deg":         session.rotation_end_deg,
            "num_stacks":      session.num_stacks,
            "easing_curve":    session.rotation_easing,
            "angles_deg":      angles,
        },
        "aux_axis": {
            "enabled":         session.aux_enabled,
            "label":           session.aux_label,
            "start_deg":       session.aux_start_deg,
            "end_deg":         session.aux_end_deg,
            "easing_curve":    session.aux_easing,
            "positions_deg":   aux_pos,
        },
        **({"grid_2d": {
            "pan_cols":        session.pan_cols,
            "tilt_rows":       session.tilt_rows,
            "grid_snake":      session.grid_snake,
            "total_positions": num_stacks_grid(session),
            "positions":       [{"pan_deg": p, "tilt_deg": t} for p, t in grid_pos],
        }} if is_grid else {}),
        "exposure_slots":  [_serial(s) for s in session.slots],
        "timing": {
            "vibe_delay_s":    session.vibe_delay_s,
            "exp_margin_s":    session.exp_margin_s,
        },
        "total_images":    total_image_count(session),
        "stacks":          stacks_meta,
    }
    path = os.path.join(orb_folder, "sequence.json")
    with open(path, "w") as f:
        json.dump(doc, f, indent=2)
    return path


def write_project_json(proj_folder: str, session: MacroSession,
                       orbits_meta: List[Dict]) -> str:
    doc = {
        "version":       1,
        "project_name":  session.project_name,
        "created":       datetime.datetime.now().isoformat(),
        "lens_profile":  _serial(session.lens),
        "rig": {
            "lead_screw_pitch_mm": LEAD_SCREW_PITCH_MM,
            "steps_per_mm":        STEPS_PER_MM,
            "pan_steps_per_deg":   PAN_STEPS_PER_DEG,
            "tilt_steps_per_deg":  TILT_STEPS_PER_DEG,
        },
        "orbits": orbits_meta,
    }
    path = os.path.join(proj_folder, "project.json")
    with open(path, "w") as f:
        json.dump(doc, f, indent=2)
    return path


# ─── COLMAP EXPORT ────────────────────────────────────────────────────────────

def export_colmap_metadata(orb_folder: str, session: MacroSession,
                            stacks_meta: List[Dict]) -> str:
    """
    Write COLMAP-compatible files to ``orb_folder/colmap/``.

    Files produced
    ──────────────
    cameras.txt  — single PINHOLE camera; large focal length approximates
                   telecentric / orthographic projection.
    images.txt   — one entry per *completed* stack with world-to-camera pose
                   derived from pan/tilt step data.
    points3D.txt — empty (no sparse SfM needed when poses are known).
    README.txt   — setup instructions for COLMAP / Gaussian splat workflow.

    Pose derivation (world = specimen frame at start of this orbit):
        k_pan = [0, cos(α), -sin(α)]  where α = pan_axis_tilt_deg
        R_wc  = R_rodrigues(k_pan, pan) @ Rx(tilt)
        t     = [0, 0, D]   (constant — camera always faces specimen origin)

    Returns the colmap/ folder path.
    """
    colmap_dir = os.path.join(orb_folder, "colmap")
    os.makedirs(colmap_dir, exist_ok=True)

    lens  = session.lens
    D_mm  = lens.working_distance_mm if lens.working_distance_mm > 0 else 200.0
    eff_um = effective_pixel_um(session)
    pan_tilt = session.pan_axis_tilt_deg

    # ── cameras.txt ──────────────────────────────────────────────────────────
    # COLMAP has no native orthographic/telecentric camera model.
    # We use PINHOLE with a *calibrated* focal length derived from physical
    # measurements, not an arbitrary large number.
    #
    # Derivation:
    #   PINHOLE projection: x_px = f * X_world / Z_world
    #   Telecentric (ortho): x_px = X_world / eff_size_mm
    #                              = X_world * 1000 / eff_um
    #   At Z_world = D (working distance), these match when:
    #     f_px = D_mm * 1000 / eff_um
    #
    # This is the focal length of a *perspective* camera whose angular scale
    # exactly matches the telecentric magnification at the working distance.
    # CRITICAL: pass --fix_existing_cameras / ba_refine_focal_length 0 to
    # prevent COLMAP from re-fitting this value as a perspective focal length.
    f_px  = _telecentric_f_px(D_mm, eff_um)
    w, h  = lens.image_width_px, lens.image_height_px
    cx, cy = w / 2.0, h / 2.0

    with open(os.path.join(colmap_dir, "cameras.txt"), "w") as fh:
        fh.write("# Camera list: CAMERA_ID MODEL WIDTH HEIGHT PARAMS[]\n")
        fh.write("# Model: PINHOLE (telecentric orthographic approximation)\n")
        fh.write(f"# Lens:  {lens.name}  {lens.magnification}x  D={D_mm:.1f} mm\n")
        fh.write(f"# Calibrated focal length: f = D_mm * 1000 / eff_um\n")
        fh.write(f"#   = {D_mm:.1f} * 1000 / {eff_um:.3f} = {f_px:.0f} px\n")
        fh.write(f"# Effective object-plane scale: {eff_um:.4f} µm/px\n")
        fh.write(f"# Physical scale: 1 px = {eff_um:.4f} µm at specimen plane\n")
        fh.write(f"# DO NOT let COLMAP refine this focal length — fix it:\n")
        fh.write(f"#   colmap bundle_adjuster --BundleAdjustment.refine_focal_length 0\n")
        fh.write(f"1 PINHOLE {w} {h} {f_px:.4f} {f_px:.4f} {cx:.1f} {cy:.1f}\n")

    # ── images.txt ───────────────────────────────────────────────────────────
    with open(os.path.join(colmap_dir, "images.txt"), "w") as fh:
        fh.write("# Image list: IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME\n")
        fh.write("#             POINTS2D[] as (X Y POINT3D_ID)\n")
        fh.write(f"# Orbit {session.orbit_number}: {session.orbit_notes or session.orbit_label}\n")
        fh.write(f"# Pan axis tilt: {pan_tilt}°,  D={D_mm:.1f} mm\n")
        fh.write(f"# World origin = specimen position at start of this orbit\n")
        img_id = 1
        for stack in stacks_meta:
            if not stack.get("completed"):
                continue
            pan_deg  = float(stack.get("rotation_deg", 0.0))
            tilt_deg = float(stack.get("tilt_deg",     0.0))
            R, t = _colmap_pose(pan_deg, tilt_deg, D_mm, pan_tilt)
            qw, qx, qy, qz = _R_to_quat(R)
            stack_id = stack.get("stack_id", f"stack_{img_id:03d}")
            img_name = f"{stack_id}/best_focus.jpg"
            fh.write(f"{img_id} {qw:.8f} {qx:.8f} {qy:.8f} {qz:.8f} "
                     f"{t[0]:.4f} {t[1]:.4f} {t[2]:.4f} 1 {img_name}\n")
            fh.write("\n")
            img_id += 1

    # ── points3D.txt ─────────────────────────────────────────────────────────
    with open(os.path.join(colmap_dir, "points3D.txt"), "w") as fh:
        fh.write("# 3D point list — empty (known-pose workflow; no sparse SfM)\n")

    # ── README.txt ───────────────────────────────────────────────────────────
    is_telecentric = lens.lens_type == "telecentric"
    # Convert step_increment_steps to micrometers, then to pixels
    step_um = (session.step_increment_steps / STEPS_PER_MM) * 1000.0
    step_px = step_um / eff_um if eff_um > 0 else 0
    lines = [
        f"COLMAP — PiSlider 3-D Scan  (Orbit {session.orbit_number})",
        "=" * 55,
        "",
        f"Orbit     : {session.orbit_label}  {('— ' + session.orbit_notes) if session.orbit_notes else ''}",
        f"Lens      : {lens.name}  ({lens.lens_type})  {lens.magnification}x",
        f"Px size   : {eff_um:.3f} µm/pixel at specimen",
        f"Work dist : {D_mm:.1f} mm",
        f"Pan axis  : {pan_tilt}° from vertical toward camera",
        "",
        "RIG GEOMETRY",
        "─────────────",
        "  Camera fixed, specimen rotates on pan-tilt mount.",
        f"  Pan shaft tilted {pan_tilt}° from vertical toward camera.",
        "  Tilt motor sits on pan assembly (axis rotates with pan).",
        "  World origin = specimen centre at start of this orbit.",
        "  All images.txt poses are relative to THIS orbit's specimen position.",
        "",
        "TELECENTRIC LENS NOTES" if is_telecentric else "LENS NOTES",
        "────────────────────────",
        *(["A telecentric lens has orthographic (isometric) projection.",
           "COLMAP uses PINHOLE with f=1e6px as an approximation.",
           "Parallax between orbiting views IS present (specimen rotates).",
           "Depth within each view comes from the focus stack, not stereo.",
           "Use focus-stacked composites (one per orbit position) as input."]
          if is_telecentric else
          ["Use focus-stacked composites (one per orbit position) as input."]),
        "",
        "SINGLE-ORBIT WORKFLOW",
        "──────────────────────",
        "1. Focus-stack each position → one sharp image per stack.",
        "   Tools: Helicon Focus, Zerene Stacker, Affinity Photo, enfuse",
        "2. Rename stacked images to match images.txt:",
        "      stack_001/best_focus.jpg",
        "      stack_002/best_focus.jpg  …",
        "3. Run COLMAP with known poses:",
        "   colmap feature_extractor --database_path db.db --image_path images/",
        "   colmap exhaustive_matcher --database_path db.db",
        "   colmap point_triangulator \\",
        "       --database_path db.db --image_path images/ \\",
        "       --input_path . --output_path sparse/",
        "4. Dense reconstruction (MVS):",
        "   colmap image_undistorter --image_path images/ \\",
        "       --input_path sparse/ --output_path dense/",
        "   colmap patch_match_stereo --workspace_path dense/",
        "   colmap stereo_fusion --workspace_path dense/ \\",
        "       --output_path dense/fused.ply",
        "",
        "MULTI-ORBIT ALIGNMENT (REMOUNTING)",
        "─────────────────────────────────────",
        *(["★ LEGO MOUNT DETECTED — poses can be merged analytically ★",
           f"   LEGO rotation: {session.lego_rotation_deg}° from orbit 1",
           f"   Block: {session.lego_block}  stud pitch: {session.lego_stud_pitch_mm} mm",
           "   See colmap_merged/ in the project folder for pre-computed merged poses.",
           "   The stud grid is also a visual fiducial — leave some studs visible",
           "   around the specimen edges so COLMAP can match them between orbits.",
           ""]
          if session.use_lego_mount else
          ["Problem: after remounting the specimen, its position in world space has",
           "changed by an unknown 6-DOF transform. Each orbit's images.txt has its",
           "OWN world origin, so the point clouds are NOT automatically aligned.",
           "",
           "RECOMMENDED: use a 4×4 LEGO brick as the specimen mount base.",
           "  This gives 0°/90°/180°/270° precision remount positions with",
           "  sub-millimetre accuracy (stud pitch 8.0 mm, tolerance ±0.01 mm).",
           "  The stud grid also provides automatic visual fiducials for COLMAP.",
           "  Enable 'use_lego_mount' in the scan settings.",
           "",
           "OTHER OPTIONS:",
           "  A. Overview images: shoot 12 quick full-scene images at pan=0°,30°…",
           "     before/after each remount. COLMAP matches them to find the transform.",
           "  B. Fiducial dots: stick 3-5 measured markers on the specimen surface.",
           "  C. ICP in CloudCompare: align overlapping regions of separate point clouds.",
           ""]),
        "",
        "GAUSSIAN SPLAT",
        "────────────────",
        "1. Provide one focus-stacked image per orbit position.",
        "2. Run COLMAP (steps above) to get sparse/.",
        "3. nerfstudio:  ns-train splatfacto --data <colmap_folder>",
        "   3DGS:        python train.py -s <colmap_folder>",
        "",
        "SCALE",
        "──────",
        f"  1 pixel ≈ {eff_um:.3f} µm at specimen plane.",
        f"  Focus rail step: {step_um:.1f} µm "
        f"= {step_px:.1f} px of DOF shift per frame.",
    ]
    with open(os.path.join(colmap_dir, "README.txt"), "w") as fh:
        fh.write("\n".join(lines) + "\n")

    return colmap_dir


# ─── LEGO MOUNT REGISTRATION ─────────────────────────────────────────────────

# LEGO stud geometry (Technic standard, unchanged since 1958)
LEGO_STUD_PITCH_MM  = 8.0    # centre-to-centre stud spacing
LEGO_BLOCK_4x4_MM   = 32.0   # 4 studs × 8 mm = 32 mm side length

def lego_inter_orbit_R(rotation_deg: float) -> "np.ndarray":
    """
    Exact rotation matrix from orbit-N world frame to orbit-1 world frame,
    when the specimen LEGO mount was physically rotated by rotation_deg degrees
    (0 / 90 / 180 / 270) around the vertical pan axis.

    Because LEGO bricks snap at discrete angular positions and the stud pitch
    is standardised to 8.0 mm, this transform is known analytically:
        Ry(rotation_deg)

    A 180° remount gives Ry(180°) = diag(−1, 1, −1).

    Usage: to express orbit-N camera poses in orbit-1 world frame:
        R_merged = R_wc_orbitN @ R_inter.T
        t_merged = t_wc_orbitN           (unchanged; camera is physically fixed)
    """
    return _R_rodrigues(np.array([0.0, 1.0, 0.0]),
                        math.radians(rotation_deg))


def export_colmap_project_merge(proj_folder: str,
                                 orbits_data: List[Dict]) -> Optional[str]:
    """
    Build a merged COLMAP input folder (``proj_folder/colmap_merged/``) that
    combines images from ALL orbits into a single world coordinate system.

    Parameters
    ----------
    proj_folder   : project root folder
    orbits_data   : list of dicts, one per orbit, each containing:
        {
          "orb_folder":       str,           # path to orbit subfolder
          "session":          MacroSession,  # session object for this orbit
          "stacks_meta":      List[Dict],    # completed stacks
        }

    Orbit 1 is the reference frame.  For LEGO-mounted orbits, the known
    Ry(lego_rotation_deg) transform is applied analytically.  For free
    remounts (use_lego_mount=False) the transform is unknown; those orbits
    are included with a warning comment in images.txt.

    Returns the colmap_merged/ path, or None if only one orbit present.
    """
    if len(orbits_data) < 2:
        return None

    merge_dir = os.path.join(proj_folder, "colmap_merged")
    os.makedirs(merge_dir, exist_ok=True)

    # Use orbit 1's lens / camera params as the reference
    ref_session = orbits_data[0]["session"]
    lens   = ref_session.lens
    D_mm   = lens.working_distance_mm if lens.working_distance_mm > 0 else 200.0
    eff_um = effective_pixel_um(ref_session)
    f_px   = _telecentric_f_px(D_mm, eff_um)   # calibrated, NOT 1e6
    w, h   = lens.image_width_px, lens.image_height_px

    # ── cameras.txt ──────────────────────────────────────────────────────────
    with open(os.path.join(merge_dir, "cameras.txt"), "w") as fh:
        fh.write("# Camera list: CAMERA_ID MODEL WIDTH HEIGHT PARAMS[]\n")
        fh.write("# Model: PINHOLE (telecentric orthographic approximation)\n")
        fh.write(f"# Calibrated: f = D_mm * 1000 / eff_um\n")
        fh.write(f"#           = {D_mm:.1f} * 1000 / {eff_um:.3f} = {f_px:.0f} px\n")
        fh.write(f"# Effective scale: {eff_um:.4f} µm/px at specimen plane\n")
        fh.write(f"# DO NOT let COLMAP refine focal length (--refine_focal_length 0)\n")
        fh.write(f"# Merged from {len(orbits_data)} orbits\n")
        fh.write(f"1 PINHOLE {w} {h} {f_px:.4f} {f_px:.4f} {w/2:.1f} {h/2:.1f}\n")

    # ── images.txt ───────────────────────────────────────────────────────────
    with open(os.path.join(merge_dir, "images.txt"), "w") as fh:
        fh.write("# Merged COLMAP image list — all orbits in orbit-1 world frame\n")
        fh.write("# Image list: IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME\n")
        fh.write("#             POINTS2D[] as (X Y POINT3D_ID)\n")
        fh.write(f"# {len(orbits_data)} orbits merged\n")

        img_id   = 1
        warnings = []

        for orb_idx, od in enumerate(orbits_data):
            sess       = od["session"]
            stacks     = od["stacks_meta"]
            pan_tilt   = sess.pan_axis_tilt_deg
            D          = lens.working_distance_mm if lens.working_distance_mm > 0 else 200.0

            # Compute the inter-orbit rotation that maps this orbit → orbit 1
            if orb_idx == 0:
                R_inter = np.eye(3)          # orbit 1 is reference
            elif sess.use_lego_mount:
                R_inter = lego_inter_orbit_R(sess.lego_rotation_deg)
            else:
                R_inter = np.eye(3)          # unknown — treat as identity (imprecise)
                warnings.append(
                    f"  Orbit {sess.orbit_number} ({sess.orbit_label}): no LEGO mount; "
                    f"poses assumed aligned (likely WRONG — use LEGO mount or ICP)."
                )

            fh.write(f"\n# ── Orbit {sess.orbit_number}: {sess.orbit_label} "
                     f"({'LEGO ' + str(int(sess.lego_rotation_deg)) + '°' if sess.use_lego_mount else 'free remount'}) ──\n")

            for stack in stacks:
                if not stack.get("completed"):
                    continue
                pan_deg  = float(stack.get("rotation_deg", 0.0))
                tilt_deg = float(stack.get("tilt_deg",     0.0))

                # Pose in this orbit's own world frame
                R_own, t_own = _colmap_pose(pan_deg, tilt_deg, D, pan_tilt)

                # Transform to orbit-1 world frame:
                # x_cam = R_own @ x_world_N + t
                # x_world_N = R_inter.T @ x_world_1  (inter maps N→1)
                # x_cam = R_own @ R_inter.T @ x_world_1 + t
                R_merged = R_own @ R_inter.T
                t_merged = t_own             # [0, 0, D] — camera physically fixed

                qw, qx, qy, qz = _R_to_quat(R_merged)
                stack_id = stack.get("stack_id", f"stack_{img_id:03d}")
                img_name = f"{sess.orbit_label}/{stack_id}/best_focus.jpg"
                fh.write(f"{img_id} {qw:.8f} {qx:.8f} {qy:.8f} {qz:.8f} "
                         f"{t_merged[0]:.4f} {t_merged[1]:.4f} {t_merged[2]:.4f} "
                         f"1 {img_name}\n")
                fh.write("\n")
                img_id += 1

        if warnings:
            fh.write("\n# WARNINGS:\n")
            for w_line in warnings:
                fh.write(f"# {w_line}\n")

    # ── points3D.txt ─────────────────────────────────────────────────────────
    with open(os.path.join(merge_dir, "points3D.txt"), "w") as fh:
        fh.write("# Empty — known-pose workflow\n")

    # ── merge_script.py ──────────────────────────────────────────────────────
    _write_lego_merge_script(merge_dir, proj_folder, orbits_data)

    # ── README.txt ───────────────────────────────────────────────────────────
    lego_orbits = [od for od in orbits_data if od["session"].use_lego_mount]
    lines = [
        "COLMAP Merged Project — All Orbits",
        "=" * 40,
        "",
        f"Orbits     : {len(orbits_data)}",
        f"LEGO-reg'd : {len(lego_orbits)}  "
        f"({'analytically exact' if lego_orbits else 'none — verify alignment'} )",
        f"Total imgs : {img_id - 1}",
        f"World frame: orbit 1 ({orbits_data[0]['session'].orbit_label})",
        "",
        "LEGO MOUNT GEOMETRY",
        "────────────────────",
        f"Stud pitch : {LEGO_STUD_PITCH_MM} mm (LEGO standard — exact)",
        f"4×4 block  : {LEGO_BLOCK_4x4_MM} mm × {LEGO_BLOCK_4x4_MM} mm",
        "Rotations  : 0° / 90° / 180° / 270° around vertical axis",
        "Accuracy   : < 0.01 mm stud-to-stud (injection moulded tolerance)",
        "",
        "IMPORTANT — stud visibility",
        "Leave some studs visible around the specimen edges in every image.",
        "COLMAP will match the stud grid between orbits as visual fiducials,",
        "confirming (or correcting) the analytically-computed transform.",
        "",
        "TELECENTRIC CAMERA MODEL",
        "─────────────────────────",
        "COLMAP has no built-in orthographic/telecentric camera model.",
        "cameras.txt uses PINHOLE with a physically calibrated focal length:",
        "",
        f"  f = D_mm × 1000 / eff_um = {D_mm:.1f} × 1000 / {eff_um:.3f} = {f_px:.0f} px",
        "",
        "Derivation: PINHOLE gives x_px = f * X / Z. At Z = working_distance D,",
        "this matches the telecentric projection x_px = X / eff_size_mm when",
        "f = D_mm * 1000 / eff_um. This is the ONLY physically correct choice.",
        "",
        "★ CRITICAL: lock this focal length — do NOT let COLMAP re-fit it.",
        "  If COLMAP refines f, it will converge to a wrong perspective value.",
        "  Use: --BundleAdjustment.refine_focal_length 0",
        "       --BundleAdjustment.refine_principal_point 0",
        "       --BundleAdjustment.refine_extra_params 0",
        "",
        "USAGE",
        "──────",
        "1. Focus-stack each position → one image per stack",
        "2. Create the images/ folder structure:",
        "   orbit_001/stack_001/best_focus.jpg",
        "   orbit_001/stack_002/best_focus.jpg",
        "   orbit_002/stack_001/best_focus.jpg  …",
        "3. colmap feature_extractor \\",
        "       --database_path db.db --image_path images/ \\",
        "       --ImageReader.camera_model PINHOLE \\",
        "       --ImageReader.single_camera 1 \\",
        f"      --ImageReader.camera_params \"{f_px:.2f},{f_px:.2f},{w/2:.1f},{h/2:.1f}\"",
        "   colmap exhaustive_matcher --database_path db.db",
        "   colmap point_triangulator \\",
        "       --database_path db.db --image_path images/ \\",
        "       --input_path . --output_path sparse/",
        "   colmap bundle_adjuster --input_path sparse/ --output_path sparse/ \\",
        "       --BundleAdjustment.refine_focal_length 0 \\",
        "       --BundleAdjustment.refine_principal_point 0 \\",
        "       --BundleAdjustment.refine_extra_params 0",
        "   (COLMAP verifies poses via feature matching — stud grid helps here)",
        "",
        "   OR: run merge_script.py to auto-run COLMAP end-to-end.",
        "",
        "SCALE",
        "──────",
        f"  1 pixel = {eff_um:.4f} µm at specimen plane",
        f"  LEGO stud pitch {LEGO_STUD_PITCH_MM} mm = "
        f"{LEGO_STUD_PITCH_MM * 1000 / eff_um:.1f} px  ← use to verify scale in COLMAP",
    ]
    with open(os.path.join(merge_dir, "README.txt"), "w") as fh:
        fh.write("\n".join(lines) + "\n")

    return merge_dir


def _write_lego_merge_script(merge_dir: str, proj_folder: str,
                              orbits_data: List[Dict]) -> None:
    """Write a standalone Python script that runs the full COLMAP pipeline."""
    ref = orbits_data[0]["session"]
    lens   = ref.lens
    D_mm   = lens.working_distance_mm if lens.working_distance_mm > 0 else 200.0
    eff_um = effective_pixel_um(ref)
    f_px   = _telecentric_f_px(D_mm, eff_um)
    w, h   = lens.image_width_px, lens.image_height_px
    cam_params = f"{f_px:.2f},{f_px:.2f},{w/2:.1f},{h/2:.1f}"
    orbit_labels = [od["session"].orbit_label for od in orbits_data]
    orbit_lines  = "".join(f"#   {lbl}/stack_NNN/best_focus.jpg\n" for lbl in orbit_labels)
    script = f'''#!/usr/bin/env python3
"""
Auto-generated by PiSlider MacroEngine.
Runs COLMAP with calibrated telecentric camera model on all merged orbits.

Telecentric camera: PINHOLE  f={f_px:.0f} px  ({eff_um:.3f} µm/px at specimen)
  f = D_mm * 1000 / eff_um = {D_mm:.1f} * 1000 / {eff_um:.3f}
  Focal length is FIXED — COLMAP will not refine it.

Requires colmap in PATH.  Run from: {proj_folder}
"""
import subprocess, os

PROJ   = r"{proj_folder}"
MERGE  = r"{merge_dir}"
IMAGES = os.path.join(PROJ, "images_merged")  # place focus-stacked images here
DB     = os.path.join(MERGE, "db.db")
SPARSE = os.path.join(MERGE, "sparse")
DENSE  = os.path.join(MERGE, "dense")

# Image folder structure expected:
# images_merged/
{orbit_lines}
CAM_PARAMS = "{cam_params}"  # f_x,f_y,cx,cy  (PINHOLE, telecentric-calibrated)

def run(cmd):
    print("$", " ".join(cmd))
    subprocess.run(cmd, check=True)

os.makedirs(SPARSE, exist_ok=True)
os.makedirs(DENSE,  exist_ok=True)

# 1. Extract features — supply our calibrated camera params, single shared camera
run(["colmap", "feature_extractor",
     "--database_path",              DB,
     "--image_path",                 IMAGES,
     "--ImageReader.camera_model",   "PINHOLE",
     "--ImageReader.single_camera",  "1",
     "--ImageReader.camera_params",  CAM_PARAMS])

# 2. Match features across all image pairs
run(["colmap", "exhaustive_matcher",
     "--database_path", DB])

# 3. Triangulate 3D points using our known camera poses from images.txt
run(["colmap", "point_triangulator",
     "--database_path", DB,
     "--image_path",    IMAGES,
     "--input_path",    MERGE,    # cameras.txt + images.txt pre-computed by PiSlider
     "--output_path",   SPARSE])

# 4. Bundle adjust with focal length FIXED (telecentric — must not be re-fitted)
run(["colmap", "bundle_adjuster",
     "--input_path",   SPARSE,
     "--output_path",  SPARSE,
     "--BundleAdjustment.refine_focal_length",     "0",
     "--BundleAdjustment.refine_principal_point",  "0",
     "--BundleAdjustment.refine_extra_params",     "0"])

# Optional: dense MVS
dense = input("Run dense MVS? (y/N): ").strip().lower()
if dense == "y":
    run(["colmap", "image_undistorter",
         "--image_path", IMAGES, "--input_path", SPARSE,
         "--output_path", DENSE])
    run(["colmap", "patch_match_stereo", "--workspace_path", DENSE])
    run(["colmap", "stereo_fusion",
         "--workspace_path", DENSE,
         "--output_path", os.path.join(DENSE, "fused.ply")])
    print("Dense point cloud:", os.path.join(DENSE, "fused.ply"))

print("Done. Open", SPARSE, "in COLMAP GUI or use for Gaussian splat.")
'''
    path = os.path.join(merge_dir, "merge_script.py")
    with open(path, "w") as fh:
        fh.write(script)
    try:
        os.chmod(path, 0o755)
    except Exception:
        pass


# ─── MACRO ENGINE ─────────────────────────────────────────────────────────────

class MacroEngine:
    """
    Executes a macro focus-stack / 3D-scan sequence.

    Dependencies are injected so this module stays hardware-agnostic
    for unit testing; in production app.py passes the real objects.

    Parameters
    ----------
    hardware        HardwareController instance
    capture_fn      async callable(frame_id: str, slot: ExposureSlot) -> Optional[str]
    apply_camera_fn async callable(slot: ExposureSlot) -> None
    broadcast_fn    async callable(dict) -> None
    bg_fn           optional async callable(color, kelvin, settle_ms) -> bool
                    sends black/white command to the phone matting display
    """

    def __init__(self, hardware, capture_fn, apply_camera_fn, broadcast_fn,
                 drain_fn=None, bg_fn=None):
        self.hw           = hardware
        self._capture     = capture_fn
        self._apply_cam   = apply_camera_fn
        self._broadcast   = broadcast_fn
        self._drain       = drain_fn   # async callable — flushes pipelined USB downloads
        self._bg          = bg_fn      # async callable — phone bg matte control
        self._stop_event  = asyncio.Event()
        self.is_running   = False

        # Runtime state (also used for resume)
        self._session:     Optional[MacroSession] = None
        self._stacks_meta: List[Dict]             = []
        self._proj_folder: str                    = ""
        self._orb_folder:  str                    = ""

        # Position tracking (absolute steps from home, updated during macro or by jog commands)
        self.rail_pos_steps: int   = 0      # absolute steps from home
        self.pan_pos_deg:    float = 0.0
        self.tilt_pos_deg:   float = 0.0

    # ──────────────────────────────────────────────────────────────────────────
    # Public API
    # ──────────────────────────────────────────────────────────────────────────

    def stop(self):
        self._stop_event.set()

    def get_resume_info(self) -> Optional[Dict]:
        """
        If a partial sequence.json exists at the last orbit folder,
        return enough info for the UI to show a resume prompt.
        """
        if not self._orb_folder:
            return None
        seq_path = os.path.join(self._orb_folder, "sequence.json")
        if not os.path.exists(seq_path):
            return None
        try:
            with open(seq_path) as f:
                doc = json.load(f)
            done  = sum(1 for s in doc.get("stacks", []) if s.get("completed"))
            total = doc.get("rotation", {}).get("num_stacks", 0)
            if done < total:
                return {"done": done, "total": total, "path": seq_path}
        except Exception:
            pass
        return None

    async def run(self, session: MacroSession,
                  resume_from_stack: int = 0) -> None:
        """
        Main entry point.  Call from app.py via asyncio.create_task().

        resume_from_stack: 0 = fresh start, N = skip first N stacks
        """
        self._session    = session
        self._stop_event.clear()
        self.is_running  = True

        # Reset position tracking to home (0°, 0°) at the start of every scan.
        # Motors must be at home before starting — if pan_pos_deg carried over
        # a previous scan's final position, the first delta move would be huge
        # and could drive the pan arm into its physical stops.
        self.pan_pos_deg  = 0.0
        self.tilt_pos_deg = 0.0

        # Build folder tree
        self._proj_folder = project_folder(session)
        self._orb_folder  = orbit_folder(self._proj_folder, session.orbit_label)

        # Verify save path is accessible before proceeding
        try:
            save_path_parent = os.path.dirname(self._proj_folder)
            if not os.path.exists(save_path_parent):
                logger.error(f"Save path does not exist: {save_path_parent}")
                raise FileNotFoundError(f"Save path not found: {save_path_parent}")
            if not os.access(save_path_parent, os.W_OK):
                logger.error(f"No write permission to save path: {save_path_parent}")
                raise PermissionError(f"Cannot write to: {save_path_parent}")

            os.makedirs(self._orb_folder, exist_ok=True)
            logger.info(f"Macro output directory ready: {self._orb_folder}")
        except (OSError, FileNotFoundError, PermissionError) as e:
            logger.error(f"Failed to create output directory: {e}")
            await self._broadcast({"type": "log", "msg": f"⚠ Output directory error: {e}"})
            raise

        # Log macro session parameters
        logger.info("="*70)
        logger.info(f"MACRO SESSION STARTING")
        logger.info(f"  Mode: {session.session_mode} | Scan: {session.scan_type}")
        travel_steps = abs(session.rail_end_steps - session.rail_start_steps)
        travel_mm = travel_steps / STEPS_PER_MM
        start_mm = session.rail_start_steps / STEPS_PER_MM
        end_mm = session.rail_end_steps / STEPS_PER_MM
        logger.info(f"  Rail: {session.rail_start_steps}→{session.rail_end_steps} steps (≈{start_mm:.1f}→{end_mm:.1f}mm, {travel_steps} steps / ≈{travel_mm:.1f}mm travel)")
        logger.info(f"  Images/Stack: {session.images_per_stack} | Stacks: {session.num_stacks}")
        logger.info(f"  Total Frames: {session.num_stacks * session.images_per_stack}")
        logger.info("="*70)

        # Compute full position lists once
        is_grid = session.scan_type == "grid_2d"
        g_pos   = grid_positions(session) if is_grid else []

        # For orbit mode, use orbit_positions() which delegates to
        # geodesic_orbit_positions() when aux_enabled=True, giving even
        # spherical coverage corrected for the physical pan axis tilt.
        if is_grid:
            angles  = [p[0] for p in g_pos]
            aux_pos = [p[1] for p in g_pos]
        else:
            _orb    = orbit_positions(session)   # [(pan, tilt), ...]
            angles  = [p[0] for p in _orb]
            aux_pos = [p[1] for p in _orb]
            # Log first 20 planned positions so we can verify the path order
            logger.info(f"📍 Planned orbit positions ({len(_orb)} stacks):")
            for _i, (_p, _t) in enumerate(_orb[:20]):
                logger.info(f"   [{_i+1:3d}] pan={_p:+8.2f}°  tilt={_t:+8.2f}°")
            if len(_orb) > 20:
                logger.info(f"   ... ({len(_orb)-20} more)")

        # Pre‑compute all keyframes for this session (useful for UI or debugging)
        self._scan_keyframes = generate_scan_keyframes(session)

        n_total = num_stacks_grid(session) if is_grid else session.num_stacks

        # Load existing stacks_meta if resuming, else start fresh
        if resume_from_stack > 0:
            try:
                with open(os.path.join(self._orb_folder, "sequence.json")) as f:
                    existing = json.load(f)
                self._stacks_meta = existing.get("stacks", [])
            except Exception:
                self._stacks_meta = []
        else:
            self._stacks_meta = []

        # Write initial sequence.json and project.json
        write_sequence_json(self._orb_folder, session, self._stacks_meta)
        self._update_project_json(completed=False)

        n_slots = sum(1 for s in session.slots if s.enabled)

        # Broadcast planned stack positions to macro-graph.html BEFORE starting.
        # The graph page listens for 'macro_scan_start' to initialise the 3D sphere
        # with all planned pan/tilt positions so the map is visible immediately.
        # Includes pan_axis_tilt_deg so the JS can render the sphere with
        # hardware-correct kinematics (tilted pan axis).
        if is_grid:
            scan_positions = [
                {"stack": i + 1, "pan_deg": float(p[0]), "tilt_deg": float(p[1])}
                for i, p in enumerate(g_pos)
            ]
        else:
            scan_positions = [
                {"stack": i + 1, "pan_deg": float(angles[i]),
                 "tilt_deg": float(aux_pos[i]) if i < len(aux_pos) else 0.0}
                for i in range(len(angles))
            ]
        await self._broadcast({
            "type":               "macro_scan_start",
            "total_stacks":       n_total,
            "total_frames":       rail_frame_count(session),
            "stack_positions":    scan_positions,
            "pan_axis_tilt_deg":  session.pan_axis_tilt_deg,
            "rail_start_mm":      session.rail_start_steps / STEPS_PER_MM,
            "rail_end_mm":        session.rail_end_steps   / STEPS_PER_MM,
        })

        await self._broadcast({
            "type": "macro_progress",
            "stack": resume_from_stack,
            "total_stacks": n_total,
            "frame": 0,
            "total_frames": rail_frame_count(session),
            "depth_per_image_um": depth_per_image_um(session),
            "msg": (f"Starting grid scan — {session.pan_cols}×{session.tilt_rows} positions × "
                    f"{rail_frame_count(session)} frames × {n_slots} slots"
                    if is_grid else
                    f"Starting macro sequence — {session.num_stacks} stacks × "
                    f"{rail_frame_count(session)} frames × {n_slots} slots")
        })

        try:
            if is_grid:
                await self._run_sequence_grid(session, g_pos, resume_from_stack)
            else:
                await self._run_sequence(session, angles, aux_pos, resume_from_stack)
        finally:
            self.is_running = False
            # Save stop state BEFORE clearing — clearing the event resets is_set() to False,
            # so completed would always appear True if we checked after clear().
            was_stopped = self._stop_event.is_set()
            self._stop_event.clear()
            # Final writes
            write_sequence_json(self._orb_folder, session, self._stacks_meta)
            self._update_project_json(completed=not was_stopped)
            # Export per-orbit COLMAP metadata
            try:
                export_colmap_metadata(self._orb_folder, session, self._stacks_meta)
            except Exception as e:
                logger.warning(f"COLMAP per-orbit export failed (non-fatal): {e}")
            # If orbit_number > 1, try to build the merged project COLMAP folder
            if session.orbit_number > 1:
                try:
                    self._export_merged_colmap(session)
                except Exception as e:
                    logger.warning(f"COLMAP merge export failed (non-fatal): {e}")
            # Safe motor stop
            self.hw.set_tmc_velocity(0, 0)
            self.hw.set_tmc_velocity(1, 0)
            self.hw.set_tmc_velocity(2, 0)
            self.hw.enable_motors(False)
            await self._broadcast({
                "type": "macro_done",
                "interrupted": self._stop_event.is_set(),
                "msg": "Macro sequence complete." if not self._stop_event.is_set()
                       else "Macro sequence stopped by user."
            })

    def _export_merged_colmap(self, current_session: MacroSession) -> None:
        """
        Build proj_folder/colmap_merged/ by reading all completed orbit
        sequence.json files and applying known LEGO inter-orbit transforms.
        Called automatically after each orbit completes when orbit_number > 1.
        """
        proj_path = os.path.join(self._proj_folder, "project.json")
        if not os.path.exists(proj_path):
            return
        try:
            with open(proj_path) as f:
                proj = json.load(f)
        except Exception:
            return

        orbits_data: List[Dict] = []
        for orb_entry in proj.get("orbits", []):
            if not orb_entry.get("completed"):
                continue
            orb_id     = orb_entry.get("orbit_id", "")
            orb_folder = os.path.join(self._proj_folder, orb_entry.get("folder", orb_id))
            seq_path   = os.path.join(orb_folder, "sequence.json")
            if not os.path.exists(seq_path):
                continue
            try:
                with open(seq_path) as f:
                    seq = json.load(f)
            except Exception:
                continue

            stacks = seq.get("stacks", [])

            # Reconstruct a minimal MacroSession from stored metadata
            # (enough for _colmap_pose: pan_axis_tilt_deg, use_lego_mount, etc.)
            lego_rot = orb_entry.get("lego_rotation_deg", 0.0)
            use_lego = orb_entry.get("use_lego_mount", False)
            sess_stub = MacroSession(
                orbit_label      = orb_id,
                orbit_number     = orb_entry.get("orbit_number", 1),
                orbit_notes      = orb_entry.get("orbit_notes", ""),
                pan_axis_tilt_deg= orb_entry.get("pan_axis_tilt_deg",
                                                  current_session.pan_axis_tilt_deg),
                use_lego_mount   = use_lego,
                lego_rotation_deg= lego_rot,
                lens             = current_session.lens,
            )
            orbits_data.append({
                "orb_folder":  orb_folder,
                "session":     sess_stub,
                "stacks_meta": stacks,
            })

        if len(orbits_data) >= 2:
            export_colmap_project_merge(self._proj_folder, orbits_data)
            logger.info(f"COLMAP merged export: {len(orbits_data)} orbits → "
                        f"{self._proj_folder}/colmap_merged/")

    # ──────────────────────────────────────────────────────────────────────────
    # Internal sequence execution
    # ──────────────────────────────────────────────────────────────────────────

    async def _run_sequence(self, session: MacroSession,
                            angles: List[float], aux_pos: List[float],
                            resume_from: int) -> None:

        frames_per_stack = rail_frame_count(session)
        enabled_slots    = [s for s in session.slots if s.enabled]

        for stack_idx in range(resume_from, session.num_stacks):
            if self._stop_event.is_set():
                break

            rot_deg  = angles[stack_idx]
            a_deg    = aux_pos[stack_idx] if session.aux_enabled else None

            await self._broadcast({
                "type":         "macro_progress",
                "stack":        stack_idx + 1,
                "total_stacks": session.num_stacks,
                "frame":        0,
                "total_frames": frames_per_stack,
                "rotation_deg": rot_deg,
                "msg": f"Stack {stack_idx+1}/{session.num_stacks} — "
                       f"rot {rot_deg:+.1f}°"
            })

            # ── 1. Move rotation stage ────────────────────────────────────────
            logger.info(f"📐 Stack {stack_idx+1}/{session.num_stacks}: Moving to pan={rot_deg:.1f}°, tilt={a_deg if a_deg else 0:.1f}°")
            try:
                await self._move_rotation(rot_deg, a_deg, session)
            except RuntimeError as e:
                logger.critical(f"⛔ MACRO HALTED: Soft-limit violation during rotation move")
                logger.critical(f"   Stack: {stack_idx+1}/{session.num_stacks}")
                logger.critical(f"   Error: {e}")
                await self._broadcast({
                    "type": "log",
                    "msg": f"⛔ MACRO HALTED: Soft-limit violation\n{e}"
                })
                self._stop_event.set()
                break

            # ── 2. Move rail to start ─────────────────────────────────────────
            try:
                await self._rail_to(session.rail_start_steps, session, fast=False)
            except RuntimeError as e:
                # Soft limit violation on initial move to stack start
                start_mm = session.rail_start_steps / STEPS_PER_MM
                current_mm = self.rail_pos_steps / STEPS_PER_MM
                logger.critical(f"⛔ MACRO HALTED: Cannot reach stack start position")
                logger.critical(f"   Stack: {stack_idx+1}/{session.num_stacks}")
                logger.critical(f"   Position at collision: {self.rail_pos_steps} steps (≈{current_mm:.2f}mm)")
                logger.critical(f"   Attempted start: {session.rail_start_steps} steps (≈{start_mm:.2f}mm)")
                logger.critical(f"   Valid range: {session.rail_start_steps}–{session.rail_end_steps} steps")
                logger.critical(f"   Error: {e}")

                await self._broadcast({
                    "type": "log",
                    "msg": f"⛔ MACRO HALTED: Cannot reach stack {stack_idx+1} start position\n"
                           f"Current: {self.rail_pos_steps} steps (≈{current_mm:.2f}mm)\n"
                           f"Attempted: {session.rail_start_steps} steps (≈{start_mm:.2f}mm)\n"
                           f"Valid range: {session.rail_start_steps}–{session.rail_end_steps} steps"
                })
                self._stop_event.set()
                raise

            stack_meta = {
                "stack_id":    f"stack_{stack_idx+1:03d}",
                "rotation_deg": rot_deg,
                "tilt_deg":     a_deg if a_deg is not None else 0.0,  # COLMAP export key
                "aux_deg":      a_deg,
                "folder":       os.path.relpath(
                    stack_folder(self._orb_folder, stack_idx, rot_deg,
                                 a_deg if session.aux_enabled else None),
                    self._orb_folder),
                "frame_count":  frames_per_stack,
                "completed":    False,
            }

            # ── 3. Focus stack — timelapse-style execution on the focus rail ──────
            # Macro is exactly like timelapse: a trajectory of evenly-spaced
            # positions is built for the focus rail, then TimelapseTrajectoryPlayer
            # executes it one frame at a time — the same proven motor path.
            #
            # We use np.linspace (not MotionEngine spline) because focus stacking
            # requires EXACTLY equal depth increments.  MotionEngine's clamped cubic
            # spline would cluster frames near the endpoints (S-curve effect).
            rail_start_mm = session.rail_start_steps / STEPS_PER_MM
            rail_end_mm   = session.rail_end_steps   / STEPS_PER_MM
            travel_mm     = abs(rail_end_mm - rail_start_mm)

            # Build evenly-spaced trajectory (pan/tilt held constant during stack)
            traj_rail = np.linspace(rail_start_mm, rail_end_mm, frames_per_stack)
            traj_pan  = np.full(frames_per_stack, self.pan_pos_deg)
            traj_tilt = np.full(frames_per_stack, self.tilt_pos_deg)

            focus_player = TimelapseTrajectoryPlayer(
                hardware=self.hw,
                steps_per_mm=STEPS_PER_MM,           # 800.0 for precision lead-screw rail
                pan_steps_per_deg=PAN_STEPS_PER_DEG,
                tilt_steps_per_deg=TILT_STEPS_PER_DEG,
            )
            focus_player.load(traj_rail, traj_pan, traj_tilt)
            # Sync player start position to engine position (avoids wrong delta on frame 0)
            focus_player.current_mm   = rail_start_mm
            focus_player.current_pan  = self.pan_pos_deg
            focus_player.current_tilt = self.tilt_pos_deg

            # Per-step move duration: 0.25s per mm of travel, minimum 0.3s
            step_mm_each = travel_mm / max(1, frames_per_stack - 1)
            move_duration = max(0.3, step_mm_each * 0.25)

            travel_steps = abs(session.rail_end_steps - session.rail_start_steps)
            logger.info(f"📋 Focus stack {stack_idx+1}/{session.num_stacks}: "
                       f"{frames_per_stack} images, {travel_steps} steps "
                       f"(≈{travel_mm:.2f}mm), {step_mm_each*1000:.1f}µm/step, "
                       f"{move_duration:.2f}s/move")

            frame_global_base = stack_idx * frames_per_stack

            stack_preview_jpgs: list = []   # JPEG previews collected during this stack

            for frame_idx in range(frames_per_stack):
                if self._stop_event.is_set():
                    break

                target_mm    = float(traj_rail[frame_idx])
                target_steps = int(target_mm * STEPS_PER_MM)
                current_mm   = self.rail_pos_steps / STEPS_PER_MM
                frame_num    = frame_global_base + frame_idx + 1

                logger.info(f"📸 Frame {frame_num} ({frame_idx+1}/{frames_per_stack}): "
                           f"{self.rail_pos_steps:+d} → {target_steps:+d} steps "
                           f"(≈{current_mm:.3f} → {target_mm:.3f}mm)")

                # Move to this frame position — same mechanism as timelapse step_to_frame
                await asyncio.to_thread(focus_player.step_to_frame, frame_idx, move_duration)
                self.rail_pos_steps = int(focus_player.current_mm * STEPS_PER_MM)

                # Anti-vibe settle
                await asyncio.sleep(session.vibe_delay_s)

                # ── 4. Fire each enabled exposure slot ────────────────────────
                for slot in enabled_slots:
                    if self._stop_event.is_set():
                        break

                    bg_suffix = f"_{slot.bg_color}" if slot.bg_color else ""
                    frame_id = (f"stack{stack_idx+1:03d}_"
                                f"f{frame_idx+1:04d}_{slot.id}{bg_suffix}")

                    # Set relay states
                    self.hw.set_relay1(slot.relay1)
                    self.hw.set_relay2(slot.relay2)

                    # Settle for relay
                    if slot.relay_settle_ms > 0:
                        await asyncio.sleep(slot.relay_settle_ms / 1000.0)

                    # Apply camera settings for this slot
                    await self._apply_cam(slot)

                    # Build output path and capture
                    slot_dir = slot_folder(
                        stack_folder(self._orb_folder, stack_idx, rot_deg,
                                     a_deg if session.aux_enabled else None),
                        slot
                    )
                    os.makedirs(slot_dir, exist_ok=True)

                    # Optional: command OLED phone background before capture.
                    # Slot bg_color="" means no change (normal lighting).
                    if slot.bg_color and self._bg is not None:
                        await self._bg(slot.bg_color, slot.kelvin, slot.bg_settle_ms)

                    file_path = await self._capture(slot_dir, frame_id, slot)

                    # Collect JPEG preview path for macro-graph flipbook.
                    # For Sony USB the download runs in the background, so the
                    # _preview.jpg for frame N is written when frame N+1 begins
                    # (the pipeline awaits N's download before triggering N+1).
                    # Strategy: check the current frame's preview first (works
                    # for picam / Sony WiFi), then fall back to the most recently
                    # modified _preview.jpg already in slot_dir (catches the
                    # confirmed-previous-frame preview written by USB pipeline).
                    if file_path:
                        jpg = os.path.splitext(file_path)[0] + '_preview.jpg'
                        if os.path.exists(jpg):
                            stack_preview_jpgs.append(jpg)
                        else:
                            # USB pipeline: scan for any confirmed preview in slot_dir
                            _existing = sorted(
                                [os.path.join(slot_dir, f) for f in os.listdir(slot_dir)
                                 if f.endswith('_preview.jpg')],
                                key=os.path.getmtime
                            )
                            if _existing:
                                _latest_confirmed = _existing[-1]
                                if _latest_confirmed not in stack_preview_jpgs:
                                    stack_preview_jpgs.append(_latest_confirmed)

                    # Exposure wait
                    await asyncio.sleep(max(0.02, slot.shutter_s + session.exp_margin_s))

                    # Release relay + post-settle
                    if slot.relay_release_ms > 0:
                        await asyncio.sleep(slot.relay_release_ms / 1000.0)
                    self.hw.set_relay1(False)
                    self.hw.set_relay2(False)

                    # Include the latest preview URL so the graph flipbook
                    # updates frame-by-frame as the scan runs.
                    _latest_jpg = stack_preview_jpgs[-1] if stack_preview_jpgs else None
                    await self._broadcast({
                        "type":         "macro_progress",
                        "stack":        stack_idx + 1,
                        "total_stacks": session.num_stacks,
                        "frame":        frame_idx + 1,
                        "total_frames": frames_per_stack,
                        "slot":         slot.id,
                        "rotation_deg": rot_deg,
                        "pan_deg":      rot_deg,
                        "tilt_deg":     a_deg,
                        "rail_mm":      target_mm,
                        "iso":          slot.iso,
                        "shutter_s":    slot.shutter_s,
                        "preview_url":  f"/macro_img?p={_latest_jpg}" if _latest_jpg else None,
                        "msg": f"Stack {stack_idx+1}/{session.num_stacks}  "
                               f"Frame {frame_idx+1}/{frames_per_stack}  "
                               f"[{slot.label}]"
                    })

            # ── 5. Flush pipelined downloads (sony_usb) ───────────────────────
            # The last frame's download runs in the background; ensure it lands
            # on the Pi before moving to the next stack position.
            if self._drain is not None:
                await self._drain()

            # ── 5b. Collect all _preview.jpg files from this stack's folders ──
            # For USB-pipeline captures, previews are written lazily (during drain
            # or on the next frame's start), so an in-loop check misses the last
            # frame. Scanning the folder after drain guarantees completeness.
            stack_preview_jpgs.clear()
            for slot in enabled_slots:
                s_dir = slot_folder(
                    stack_folder(self._orb_folder, stack_idx, rot_deg,
                                 a_deg if session.aux_enabled else None),
                    slot
                )
                if os.path.isdir(s_dir):
                    for fname in sorted(os.listdir(s_dir)):
                        if fname.endswith('_preview.jpg'):
                            stack_preview_jpgs.append(os.path.join(s_dir, fname))

            # ── 6. Return rail to start at high speed ─────────────────────────
            if not self._stop_event.is_set():
                current_mm = self.rail_pos_steps / STEPS_PER_MM
                start_mm   = session.rail_start_steps / STEPS_PER_MM
                logger.info(f"✓ Stack {stack_idx+1} done — returning focus to start "
                           f"({self.rail_pos_steps:+d} → {session.rail_start_steps:+d} steps) [FAST]")
                await self._rail_to(session.rail_start_steps, session, fast=True)

            # ── 7. Mark stack complete ────────────────────────────────────────
            stack_meta["completed"] = not self._stop_event.is_set()
            # Update or append
            if stack_idx < len(self._stacks_meta):
                self._stacks_meta[stack_idx] = stack_meta
            else:
                self._stacks_meta.append(stack_meta)

            # Incremental save after every stack
            write_sequence_json(self._orb_folder, self._session, self._stacks_meta)

            rail_mm = session.rail_start_steps / STEPS_PER_MM
            logger.info(f"📸 Stack {stack_idx+1} complete — {len(stack_preview_jpgs)} preview(s) collected")
            await self._broadcast({
                "type":         "macro_stack_complete",
                "stack":        stack_idx + 1,
                "total_stacks": session.num_stacks,
                "frame_count":  frames_per_stack,
                "rotation_deg": rot_deg,
                "pan_deg":      rot_deg,
                "tilt_deg":     a_deg,
                "rail_mm":      rail_mm,
                "iso":          session.slots[0].iso if session.slots else 400,
                "shutter_s":    session.slots[0].shutter_s if session.slots else 1/125,
                "completed":    stack_meta["completed"],
                "preview_urls": [f"/macro_img?p={jpg}" for jpg in stack_preview_jpgs],
            })

    async def _run_sequence_grid(self, session: MacroSession,
                                 positions: List[tuple],
                                 resume_from: int) -> None:
        """
        Grid-2D scan: iterate through (pan_deg, tilt_deg) position list,
        running a full focus stack at each grid point.

        Positions are pre-computed by grid_positions() in snake order.
        Both pan AND tilt motors move the SPECIMEN (camera fixed).
        """
        frames_per_stack = rail_frame_count(session)
        n_total          = len(positions)
        enabled_slots    = [s for s in session.slots if s.enabled]

        for stack_idx in range(resume_from, n_total):
            if self._stop_event.is_set():
                break

            pan_deg, tilt_deg = positions[stack_idx]

            await self._broadcast({
                "type":         "macro_progress",
                "stack":        stack_idx + 1,
                "total_stacks": n_total,
                "frame":        0,
                "total_frames": frames_per_stack,
                "rotation_deg": pan_deg,
                "tilt_deg":     tilt_deg,
                "depth_per_image_um": depth_per_image_um(session),
                "msg": f"Stack {stack_idx+1}/{n_total} — "
                       f"pan {pan_deg:+.1f}°  tilt {tilt_deg:+.1f}°"
            })

            # ── 1. Move to grid position (both pan and tilt) ──────────────────
            try:
                await self._move_rotation(pan_deg, tilt_deg, session)
            except RuntimeError as e:
                logger.critical(f"⛔ MACRO HALTED: Soft-limit violation during rotation move")
                logger.critical(f"   Stack: {stack_idx+1}/{n_total}")
                logger.critical(f"   Error: {e}")
                await self._broadcast({
                    "type": "log",
                    "msg": f"⛔ MACRO HALTED: Soft-limit violation\n{e}"
                })
                self._stop_event.set()
                break

            # ── 2. Move rail to start ─────────────────────────────────────────
            await self._rail_to(session.rail_start_steps, session, fast=False)

            stack_meta = {
                "stack_id":     f"stack_{stack_idx+1:03d}",
                "rotation_deg": pan_deg,
                "tilt_deg":     tilt_deg,
                "pan_deg":      pan_deg,
                "folder":       os.path.relpath(
                    stack_folder(self._orb_folder, stack_idx, pan_deg, tilt_deg),
                    self._orb_folder),
                "frame_count":  frames_per_stack,
                "completed":    False,
            }

            # ── 3. Focus stack — timelapse-style execution on the focus rail ──────
            rail_start_mm = session.rail_start_steps / STEPS_PER_MM
            rail_end_mm   = session.rail_end_steps   / STEPS_PER_MM
            travel_mm     = abs(rail_end_mm - rail_start_mm)

            # Evenly-spaced trajectory (pan/tilt held constant during stack)
            traj_rail = np.linspace(rail_start_mm, rail_end_mm, frames_per_stack)
            traj_pan  = np.full(frames_per_stack, self.pan_pos_deg)
            traj_tilt = np.full(frames_per_stack, self.tilt_pos_deg)

            focus_player = TimelapseTrajectoryPlayer(
                hardware=self.hw,
                steps_per_mm=STEPS_PER_MM,
                pan_steps_per_deg=PAN_STEPS_PER_DEG,
                tilt_steps_per_deg=TILT_STEPS_PER_DEG,
            )
            focus_player.load(traj_rail, traj_pan, traj_tilt)
            focus_player.current_mm   = rail_start_mm
            focus_player.current_pan  = self.pan_pos_deg
            focus_player.current_tilt = self.tilt_pos_deg

            step_mm_each  = travel_mm / max(1, frames_per_stack - 1)
            move_duration = max(0.3, step_mm_each * 0.25)

            logger.info(f"📋 Focus stack {stack_idx+1}/{n_total}: "
                       f"{frames_per_stack} images, {travel_mm:.2f}mm travel, "
                       f"{step_mm_each*1000:.1f}µm/step, {move_duration:.2f}s/move")

            for frame_idx in range(frames_per_stack):
                if self._stop_event.is_set():
                    break

                target_mm    = float(traj_rail[frame_idx])
                target_steps = int(target_mm * STEPS_PER_MM)

                logger.info(f"📸 Frame {frame_idx+1}/{frames_per_stack}: "
                           f"{self.rail_pos_steps:+d} → {target_steps:+d} steps "
                           f"(≈{self.rail_pos_steps/STEPS_PER_MM:.3f} → {target_mm:.3f}mm)")

                await asyncio.to_thread(focus_player.step_to_frame, frame_idx, move_duration)
                self.rail_pos_steps = int(focus_player.current_mm * STEPS_PER_MM)

                await asyncio.sleep(session.vibe_delay_s)

                for slot in enabled_slots:
                    if self._stop_event.is_set():
                        break

                    bg_suffix = f"_{slot.bg_color}" if slot.bg_color else ""
                    frame_id = (f"stack{stack_idx+1:03d}_"
                                f"f{frame_idx+1:04d}_{slot.id}{bg_suffix}")

                    self.hw.set_relay1(slot.relay1)
                    self.hw.set_relay2(slot.relay2)
                    if slot.relay_settle_ms > 0:
                        await asyncio.sleep(slot.relay_settle_ms / 1000.0)

                    await self._apply_cam(slot)

                    slot_dir = slot_folder(
                        stack_folder(self._orb_folder, stack_idx, pan_deg, tilt_deg),
                        slot
                    )
                    os.makedirs(slot_dir, exist_ok=True)

                    if slot.bg_color and self._bg is not None:
                        await self._bg(slot.bg_color, slot.kelvin, slot.bg_settle_ms)

                    _gfp = await self._capture(slot_dir, frame_id, slot)

                    # Collect preview jpg for live flipbook updates
                    _gjpg = None
                    if _gfp:
                        _gjpg_path = os.path.splitext(_gfp)[0] + '_preview.jpg'
                        if os.path.exists(_gjpg_path):
                            stack_preview_jpgs.append(_gjpg_path)
                            _gjpg = _gjpg_path

                    await asyncio.sleep(max(0.02, slot.shutter_s + session.exp_margin_s))

                    if slot.relay_release_ms > 0:
                        await asyncio.sleep(slot.relay_release_ms / 1000.0)
                    self.hw.set_relay1(False)
                    self.hw.set_relay2(False)

                    await self._broadcast({
                        "type":         "macro_progress",
                        "stack":        stack_idx + 1,
                        "total_stacks": n_total,
                        "frame":        frame_idx + 1,
                        "total_frames": frames_per_stack,
                        "slot":         slot.id,
                        "rotation_deg": pan_deg,
                        "pan_deg":      pan_deg,
                        "tilt_deg":     tilt_deg,
                        "rail_mm":      target_mm,
                        "iso":          slot.iso,
                        "shutter_s":    slot.shutter_s,
                        "depth_per_image_um": depth_per_image_um(session),
                        "preview_url":  f"/macro_img?p={_gjpg}" if _gjpg else None,
                        "msg": f"Stack {stack_idx+1}/{n_total}  "
                               f"Frame {frame_idx+1}/{frames_per_stack}  "
                               f"[{slot.label}]"
                    })

            # ── 4. Return rail to start ───────────────────────────────────────
            if not self._stop_event.is_set():
                logger.info(f"✓ Stack {stack_idx+1} done — returning focus to start [FAST]")
                await self._rail_to(session.rail_start_steps, session, fast=True)

            # ── 5. Mark stack complete ────────────────────────────────────────
            stack_meta["completed"] = not self._stop_event.is_set()
            if stack_idx < len(self._stacks_meta):
                self._stacks_meta[stack_idx] = stack_meta
            else:
                self._stacks_meta.append(stack_meta)

            write_sequence_json(self._orb_folder, self._session, self._stacks_meta)

            rail_mm = session.rail_start_steps / STEPS_PER_MM
            await self._broadcast({
                "type":         "macro_stack_complete",
                "stack":        stack_idx + 1,
                "total_stacks": n_total,
                "frame_count":  frames_per_stack,
                "rotation_deg": pan_deg,
                "pan_deg":      pan_deg,
                "tilt_deg":     tilt_deg,
                "rail_mm":      rail_mm,
                "iso":          enabled_slots[0].iso if enabled_slots else 400,
                "shutter_s":    enabled_slots[0].shutter_s if enabled_slots else 1/125,
                "completed":    stack_meta["completed"],
            })

    # ──────────────────────────────────────────────────────────────────────────
    # Motor helpers
    # ──────────────────────────────────────────────────────────────────────────

    async def _rail_to(self, target_steps: int, session: MacroSession,
                       fast: bool = False) -> None:
        """Move rail to absolute position (in steps from home).

        Raises RuntimeError if target violates soft limits (hard stop on collision detection).
        """
        delta_steps = target_steps - self.rail_pos_steps
        if delta_steps == 0:
            return

        # Convert to mm for logging/display
        target_mm = target_steps / STEPS_PER_MM
        current_mm = self.rail_pos_steps / STEPS_PER_MM
        delta_mm = delta_steps / STEPS_PER_MM

        # ── CRITICAL: Hard soft-limit check (collision detection) ──
        # If target is outside the focus stack range, HALT immediately
        start_steps = session.rail_start_steps
        end_steps = session.rail_end_steps
        min_steps = min(start_steps, end_steps)
        max_steps = max(start_steps, end_steps)

        if not (min_steps <= target_steps <= max_steps):
            # SOFT COLLISION DETECTED — Halt immediately with detailed diagnostics
            collision_msg = (
                f"⛔ SOFT LIMIT COLLISION DETECTED\n"
                f"  Current position:  {self.rail_pos_steps} steps (≈{current_mm:.2f}mm)\n"
                f"  Attempted move to: {target_steps} steps (≈{target_mm:.2f}mm)\n"
                f"  Valid focus range: {min_steps}–{max_steps} steps "
                f"(≈{min_steps/STEPS_PER_MM:.2f}–{max_steps/STEPS_PER_MM:.2f}mm)\n"
                f"  Delta would be:    {delta_steps:+d} steps (≈{delta_mm:+.2f}mm)\n"
                f"  Session start:     {start_steps} steps (≈{start_steps/STEPS_PER_MM:.2f}mm)\n"
                f"  Session end:       {end_steps} steps (≈{end_steps/STEPS_PER_MM:.2f}mm)"
            )
            logger.critical(collision_msg)
            # Re-raise as error to stop the macro sequence
            raise RuntimeError(f"Focus soft limit violation at position {target_steps} steps: "
                             f"outside valid range {min_steps}–{max_steps} steps")

        # Detailed focus motor logging
        logger.info(f"🔵 FOCUS MOTOR: current={self.rail_pos_steps} steps (≈{current_mm:.2f}mm) "
                   f"→ target={target_steps} steps (≈{target_mm:.2f}mm) "
                   f"(delta={delta_steps:+d} steps, ≈{delta_mm:+.2f}mm)")

        # Motors stay enabled throughout the entire macro sequence.
        # Disabling after each move causes loss of holding torque → drift between exposures.
        # Motors are only disabled at the end of the full run() sequence.
        self.hw.enable_motors(True)

        if fast:
            # Fast return via move_axes_simultaneous.
            # IMPORTANT: duration must be long enough for the GPIO bit-bang loop to
            # actually deliver all steps. move_axes_simultaneous tops out at ~3,000
            # steps/sec (100µs pulse + GPIO overhead per step). Using RAIL_RETURN_MM_S
            # (44.7 mm/s → 35,800 steps/sec) as the duration basis would trigger the
            # 2× safety timeout after ~12s, aborting with 115mm of travel remaining.
            # Use a safe upper bound: 3,000 steps/sec max rate.
            travel_mm = abs(delta_steps) / STEPS_PER_MM
            duration  = max(abs(delta_steps) / 3000.0, 0.5)
            await asyncio.to_thread(
                self.hw.move_axes_simultaneous,
                delta_steps, 0, 0, duration
            )
        else:
            # Precision move via Bresenham stepper with easing-friendly timing.
            # 0.25s per mm gives ~3,200 steps/sec — well within hardware capability.
            duration = max(0.4, abs(delta_steps) / STEPS_PER_MM * 0.25)
            await asyncio.to_thread(
                self.hw.move_axes_simultaneous,
                delta_steps, 0, 0, duration
            )

        # Update internal MacroEngine rail position
        self.rail_pos_steps = target_steps
        # Do NOT disable motors here – they stay enabled for the whole macro run.

        # Do NOT disable motors here — hold torque is required between focus steps
        # to prevent specimen drift. Motors are disabled once at end of run().
        logger.info(f"  ✓ COMPLETE: rail now at {self.rail_pos_steps} steps (≈{target_mm:.2f}mm)")

    async def _move_rotation(self, rot_deg: float, aux_deg: Optional[float],
                             session: MacroSession) -> None:
        """Move pan (and optionally tilt/aux) to target angles."""
        delta_pan  = rot_deg  - self.pan_pos_deg
        delta_aux  = (aux_deg - self.tilt_pos_deg) if aux_deg is not None else 0.0

        pan_steps  = int(delta_pan * PAN_STEPS_PER_DEG)
        tilt_steps = int(delta_aux * TILT_STEPS_PER_DEG)

        if pan_steps == 0 and tilt_steps == 0:
            return

        # Enforce soft limits — STOP the scan rather than silently overtravel
        new_pan  = self.pan_pos_deg + delta_pan
        new_tilt = self.tilt_pos_deg + delta_aux

        pan_lo = min(session.rotation_start_deg, session.rotation_end_deg)
        pan_hi = max(session.rotation_start_deg, session.rotation_end_deg)
        if pan_lo < pan_hi - 5.0:   # only enforce if a real range is configured
            clamped_pan = float(np.clip(new_pan, pan_lo, pan_hi))
            if abs(clamped_pan - new_pan) > 0.5:
                raise RuntimeError(
                    f"Pan soft-limit violation: target {new_pan:.1f}° is outside "
                    f"configured range [{pan_lo:.1f}°, {pan_hi:.1f}°]"
                )
            delta_pan = clamped_pan - self.pan_pos_deg
            new_pan   = clamped_pan
            pan_steps = int(delta_pan * PAN_STEPS_PER_DEG)

        # Note: tilt/aux is NOT clamped here — in orbit helix scans the aux motor
        # rotates the specimen continuously (0°→360°×N turns), so its position
        # legitimately exceeds ±90°.  Hardware limits on the aux axis are enforced
        # by the motor controller itself.  Only the pan (specimen rotation range)
        # needs a soft check at this level.

        # Use easing-friendly timing for smoother ramps (calibration-stable motion)
        # Calculation: base (0.5s min) + angular-distance-proportional with 0.04s per degree
        # This provides smooth acceleration for typical ±30° moves (~1.7s total)
        duration = max(0.5, abs(delta_pan) * 0.04 + abs(delta_aux) * 0.04)

        logger.info(f"🔄 ROTATION: pan {self.pan_pos_deg:+.1f}° → {new_pan:+.1f}°, "
                   f"tilt {self.tilt_pos_deg:+.1f}° → {new_tilt:+.1f}° "
                   f"(eased {duration:.2f}s for calibration stability)")

        self.hw.enable_motors(True)
        await asyncio.to_thread(
            self.hw.move_axes_simultaneous,
            0, pan_steps, tilt_steps, duration
        )
        # Do NOT disable motors here — hold torque keeps pan/tilt at the
        # geodesic position during the full focus stack. Motors are disabled
        # once at the very end of run().

        self.pan_pos_deg  = new_pan
        self.tilt_pos_deg = new_tilt

        await asyncio.sleep(session.vibe_delay_s)

    # ──────────────────────────────────────────────────────────────────────────
    # Project JSON maintenance
    # ──────────────────────────────────────────────────────────────────────────

    def _update_project_json(self, completed: bool) -> None:
        if not self._session or not self._proj_folder:
            return
        done = sum(1 for s in self._stacks_meta if s.get("completed"))
        sess = self._session
        orbit_entry = {
            "orbit_id":   sess.orbit_label,
            "label":      sess.orbit_label,
            "orbit_number": sess.orbit_number,
            "orbit_notes":  sess.orbit_notes,
            "rotation_axis_angle_deg": sess.rotation_axis_angle_deg,
            "rotation_axis_description": sess.rotation_axis_description,
            "folder":     os.path.basename(self._orb_folder),
            "num_stacks": sess.num_stacks,
            "stacks_done": done,
            "completed":  completed,
            # LEGO registration metadata (used by _export_merged_colmap)
            "pan_axis_tilt_deg":  sess.pan_axis_tilt_deg,
            "use_lego_mount":     sess.use_lego_mount,
            "lego_rotation_deg":  sess.lego_rotation_deg,
            "lego_block":         sess.lego_block,
        }

        # Load existing project.json if present, update orbits list
        proj_path = os.path.join(self._proj_folder, "project.json")
        orbits = []
        if os.path.exists(proj_path):
            try:
                with open(proj_path) as f:
                    existing = json.load(f)
                orbits = existing.get("orbits", [])
            except Exception:
                pass

        # Replace or append this orbit's entry
        found = False
        for i, o in enumerate(orbits):
            if o.get("orbit_id") == orbit_entry["orbit_id"]:
                orbits[i] = orbit_entry
                found = True
                break
        if not found:
            orbits.append(orbit_entry)

        write_project_json(self._proj_folder, self._session, orbits)
