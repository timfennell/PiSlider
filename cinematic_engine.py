#!/usr/bin/env python3
"""
cinematic_engine.py — Cinema motion control engine for PiSlider.

Four main classes:
  SoftLimitGuard    — per-axis travel limits with velocity-aware decel ramp.
                      Blocks high-speed movement until both ends calibrated.
  InertiaEngine     — physics simulation: mass + fluid drag model at 50 Hz.
                      Consumes gamepad axis events, outputs TMC2209 velocities.
  ArcTanTracker     — 3D least-squares subject solve from N calibration points.
                      Outputs real-time (pan_deg, tilt_deg) for any slider pos.
  ProgrammedMove    — keyframe store with per-segment timing + cubic spline
                      playback via existing TrajectoryPlayer infrastructure.

MoveLibrary        — named keyframe sequence persistence (~/.pislider_moves.json).
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import time
from dataclasses import dataclass, field, asdict
from typing import Optional, List, Dict, Tuple, Any

import numpy as np

logger = logging.getLogger("PiSlider.Cinematic")

# ─── HARDWARE CONSTANTS ───────────────────────────────────────────────────────
#
# TMC2209 UART address map (MS1/MS2 strap wiring):
#   Addr 0 → Tilt   (All Low)
#   Addr 1 → Pan    (MS1 High)
#   Addr 2 → Slider (MS2 High)
#
ADDR_TILT   = 0
ADDR_PAN    = 1
ADDR_SLIDER = 2

# VACTUAL conversion factors — calibrated from test_uart_optimized.py:
#   Slider: 1875 VACTUAL → 150 mm/s max
#   Pan:     125 VACTUAL →   8 deg/s max
#   Tilt:    185 VACTUAL →   8 deg/s max
# VACTUAL uses the TMC2209 internal hardware clock for smooth, jitter-free steps.
# STEP/DIR (software PWM) is only used for timelapse Bresenham stepping.
#
VACTUAL_PER_MM_S       = 12.500   # = 1875 / 150
VACTUAL_PER_DEG_S_PAN  = 15.625   # = 125 / 8
VACTUAL_PER_DEG_S_TILT = 23.125   # = 185 / 8

# Step conversion factors — 1/8 microstep standalone mode (MS1=MS2=0)
SLIDER_STEPS_PER_MM =  50.000    # 200 × 8 / 32mm
PAN_STEPS_PER_DEG   =  66.667    # 200 × 8 × 15 / 360
TILT_STEPS_PER_DEG  = 133.333    # 200 × 8 × 30 / 360

# Physical speed limits — tuned for camera slider use
# These are the maximum physical velocities commanded by the InertiaEngine.
# Adjust these if motors feel too fast or too slow for your rig.
MAX_SPEED_SLIDER =  80.0   # mm/s  — ~8 cm/s, comfortable cinematic speed
MAX_SPEED_PAN    =  25.0   # deg/s — full-stick = 90° in ~3.6 s
MAX_SPEED_TILT   =  20.0   # deg/s — slightly slower for camera stability

# Deceleration limits (physical units/s²) — used by SoftLimitGuard near end stops
# Must be large enough that braking distance at max speed is manageable.
# At MAX_SPEED_PAN=25 deg/s, MAX_DECEL_PAN=50 → stop distance = 25²/(2×50) = 6.25°
MAX_DECEL_SLIDER =  80.0   # mm/s²
MAX_DECEL_PAN    =  50.0   # deg/s²  (was 5 — braking ramp was 45° overrun before)
MAX_DECEL_TILT   =  40.0   # deg/s²

# Crawl speed (before soft limits calibrated) — 25% of max
CRAWL_SPEED_SLIDER = 20.0   # mm/s
CRAWL_SPEED_PAN    =  3.0   # deg/s
CRAWL_SPEED_TILT   =  3.0   # deg/s

# Nudge speeds — used by d-pad (pan/tilt) and L2/R2 triggers (slider).
# These are direct VACTUAL velocities that bypass InertiaEngine physics entirely.
# Purpose: precise positioning to set soft limits or keyframe points.
# Trigger nudge is proportional (trigger pressure × max nudge speed).
NUDGE_SPEED_PAN    =  3.0   # deg/s — matches crawl; enough for fine positioning
NUDGE_SPEED_TILT   =  3.0   # deg/s
NUDGE_SPEED_SLIDER = 15.0   # mm/s  — slow enough for accurate endpoint setting

# Inertia loop rate
INERTIA_HZ = 50
INERTIA_DT = 1.0 / INERTIA_HZ

MOVES_FILE = os.path.expanduser("~/.pislider_moves.json")


# ─── PATH MATH HELPERS ───────────────────────────────────────────────────────

def _catmull_rom_chain(positions: np.ndarray, tension: float, n_dense: int) -> np.ndarray:
    """
    Catmull-Rom spline through all N control points.

    Uses phantom endpoint repetition for C1 continuity at the ends.
    Tangents are clamped to prevent overshoot when adjacent segment lengths
    are unequal (e.g. keyframes clustered near one end of the path).

    tension controls tangent magnitude:
      0.5  → standard Catmull-Rom (smooth, default)
      0.0  → tangents = 0 → straight lines (degenerate)
      1.0  → loose / more curvature

    Returns dense array of shape (≈n_dense, 3).
    """
    N = len(positions)
    if N < 2:
        return positions.copy()

    # Phantom start/end via endpoint repetition
    pts    = np.vstack([positions[:1], positions, positions[-1:]])   # (N+2, 3)
    n_segs = N - 1
    sps    = n_dense // n_segs   # samples per segment

    result = []
    for i in range(n_segs):
        P0, P1, P2, P3 = pts[i], pts[i+1], pts[i+2], pts[i+3]

        # Raw Catmull-Rom tangents
        T1 = tension * (P2 - P0)   # tangent leaving P1
        T2 = tension * (P3 - P1)   # tangent arriving P2

        # ── Overshoot guard ───────────────────────────────────────────────
        # Standard CR can massively overshoot when segment lengths are unequal
        # (e.g. a keyframe very close to the start followed by one far away).
        # Clamp each tangent so its 3D magnitude does not exceed 3× the chord
        # of the CURRENT segment — the standard monotone cubic sufficient
        # condition.  This preserves smooth curves while bounding overshoot.
        chord = float(np.linalg.norm(P2 - P1))
        if chord > 1e-9:
            t1_mag = float(np.linalg.norm(T1))
            if t1_mag > 3.0 * chord:
                T1 = T1 * (3.0 * chord / t1_mag)
            t2_mag = float(np.linalg.norm(T2))
            if t2_mag > 3.0 * chord:
                T2 = T2 * (3.0 * chord / t2_mag)

        # Include endpoint only on the last segment to avoid duplicates
        n  = sps + (1 if i == n_segs - 1 else 0)
        t  = np.linspace(0.0, 1.0, n)
        t2_arr, t3_arr = t**2, t**3

        # Cubic Hermite basis functions
        h00 =  2*t3_arr - 3*t2_arr + 1
        h10 =    t3_arr - 2*t2_arr + t
        h01 = -2*t3_arr + 3*t2_arr
        h11 =    t3_arr -   t2_arr

        seg = (h00[:, None]*P1 + h10[:, None]*T1 +
               h01[:, None]*P2 + h11[:, None]*T2)
        result.append(seg if i == n_segs - 1 else seg[:-1])

    return np.vstack(result)


def _arc_length_params(dense: np.ndarray) -> np.ndarray:
    """
    Normalized cumulative arc-length for each point in a dense path.
    Returns array ∈ [0, 1] of length len(dense).
    Critical for proper easing: maps actual physical distance rather than
    parameter t, so 'constant speed' easing means constant mm/s, not
    constant t-increment (which would speed up on straight sections).
    """
    diffs = np.diff(dense, axis=0)
    seg_len = np.sqrt((diffs**2).sum(axis=1))
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = cum[-1]
    return cum / total if total > 0.0 else np.linspace(0.0, 1.0, len(dense))


def _easing_curve(name: str, n: int = 1000) -> Tuple[np.ndarray, np.ndarray]:
    """
    Convert a distribution curve name into a normalised time→position mapping.

    The distribution functions act as velocity profiles (higher weight = faster
    motion at that moment).  Cumulative sum converts velocity → position.

    Returns (x, y) where:
      x = normalised input time  [0 … 1]
      y = normalised output arc  [0 … 1]

    Use np.interp(query_t, x, y) to map any time value to path position.
    """
    try:
        from distributions import CURVE_FUNCTIONS, normalize
        fn = CURVE_FUNCTIONS.get(name)
        if fn is None:
            fn = CURVE_FUNCTIONS.get("cycloid") or next(iter(CURVE_FUNCTIONS.values()))
        weights = normalize(fn(n))
        pos = np.concatenate([[0.0], np.cumsum(weights)])
        pos /= pos[-1]
        x = np.linspace(0.0, 1.0, len(pos))
        return x, pos
    except Exception:
        return np.array([0.0, 1.0]), np.array([0.0, 1.0])


# ─── SOFT LIMIT GUARD ────────────────────────────────────────────────────────

class AxisLimit:
    """Per-axis soft limit state and deceleration guard.  All values in physical units."""

    CAL_NONE  = 0   # uncalibrated — crawl only
    CAL_ONE   = 1   # one end calibrated — half speed
    CAL_BOTH  = 2   # both ends calibrated — full speed

    def __init__(self, name: str, max_speed: float, max_decel: float,
                 crawl_speed: float):
        self.name       = name
        self.max_speed  = max_speed    # mm/s or deg/s
        self.max_decel  = max_decel    # mm/s² or deg/s²
        self.crawl_speed = crawl_speed

        self.min_unit: Optional[float] = None   # mm or deg
        self.max_unit: Optional[float] = None
        self.cal_state: int = self.CAL_NONE

    @property
    def speed_limit(self) -> float:
        """Effective max speed given calibration state."""
        if self.cal_state == self.CAL_NONE:
            return self.crawl_speed
        if self.cal_state == self.CAL_ONE:
            return self.max_speed * 0.5
        return self.max_speed

    def set_min(self, unit: float):
        self.min_unit = unit
        self._update_cal()

    def set_max(self, unit: float):
        self.max_unit = unit
        self._update_cal()

    def _update_cal(self):
        both = self.min_unit is not None and self.max_unit is not None
        one  = self.min_unit is not None or  self.max_unit is not None
        self.cal_state = self.CAL_BOTH if both else (self.CAL_ONE if one else self.CAL_NONE)

    def clamp_velocity(self, desired_vel: float, current_pos: float) -> float:
        """Clamp velocity to speed limit and deceleration ramp near soft limits."""
        limit = self.speed_limit
        vel = max(-limit, min(limit, desired_vel))
        if vel != 0 and self.cal_state == self.CAL_BOTH:
            vel = self._apply_ramp(vel, current_pos)
        return vel

    def _apply_ramp(self, vel: float, pos: float) -> float:
        """Reduce velocity if stopping distance exceeds gap to limit."""
        if vel > 0 and self.max_unit is not None:
            gap = self.max_unit - pos
            stop_dist = (vel ** 2) / (2.0 * self.max_decel)
            if stop_dist >= gap > 0:
                vel = min(vel, math.sqrt(max(0, 2.0 * self.max_decel * gap)))
            elif gap <= 0:
                vel = 0.0
        elif vel < 0 and self.min_unit is not None:
            gap = pos - self.min_unit
            stop_dist = (vel ** 2) / (2.0 * self.max_decel)
            if stop_dist >= gap > 0:
                vel = max(vel, -math.sqrt(max(0, 2.0 * self.max_decel * gap)))
            elif gap <= 0:
                vel = 0.0
        return vel


class SoftLimitGuard:
    """Wraps all three axes with AxisLimit instances (physical units)."""

    def __init__(self):
        self.slider = AxisLimit("slider", MAX_SPEED_SLIDER, MAX_DECEL_SLIDER,
                                CRAWL_SPEED_SLIDER)
        self.pan    = AxisLimit("pan",    MAX_SPEED_PAN,    MAX_DECEL_PAN,
                                CRAWL_SPEED_PAN)
        self.tilt   = AxisLimit("tilt",   MAX_SPEED_TILT,   MAX_DECEL_TILT,
                                CRAWL_SPEED_TILT)

    def clamp(self, v_slider: float, v_pan: float, v_tilt: float,
              pos_slider: float, pos_pan: float, pos_tilt: float
              ) -> Tuple[float, float, float]:
        return (
            self.slider.clamp_velocity(v_slider, pos_slider),
            self.pan.clamp_velocity(v_pan,       pos_pan),
            self.tilt.clamp_velocity(v_tilt,     pos_tilt),
        )

    def status(self) -> Dict:
        def _ax(ax: AxisLimit):
            return {
                "cal_state": ax.cal_state,
                "min":       ax.min_unit,
                "max":       ax.max_unit,
                "speed_pct": int(ax.speed_limit / ax.max_speed * 100),
            }
        return {
            "slider": _ax(self.slider),
            "pan":    _ax(self.pan),
            "tilt":   _ax(self.tilt),
        }


# ─── INERTIA ENGINE ───────────────────────────────────────────────────────────

# Preset rigs — (mass, drag, motion_scale)
# motion_scale is a true linear multiplier applied to ALL three axes AFTER the
# power-curve map.  1.0 = full speed.  Reduce for telephoto / crop sensors where
# even small movements produce large frame shifts.
RIG_PRESETS = {
    "responsive": (0.05, 0.95, 1.00),  # Near-direct — tiny ramp, used for positioning
    "light":      (0.15, 0.80, 1.00),  # Snappy cinematic — reaches speed in ~0.2s
    "standard":   (0.40, 0.55, 1.00),  # Smooth cinematic — reaches speed in ~0.7s
    "heavy":      (0.90, 0.30, 1.00),  # Fluid head feel — slow build, long coast
    # Telephoto / live-tracking preset:
    #   • Near-zero mass  → stick moves motor directly, no lag/overshoot
    #   • Low drag        → immediate stop when stick released
    #   • 15% motion_scale → all axes at 15% max speed (~0.75 deg/s pan/tilt,
    #     ~30 mm/s slider) — practical for tracking subjects at 400–1000 mm FL.
    #     User can fine-tune scale live with the Motion Scale slider.
    "tracking":   (0.02, 0.20, 0.15),
    "custom":     None,
}


class InertiaEngine:
    """
    Physics simulation for live camera movement.

    Model:
        acceleration = (stick_force - drag * velocity) / mass
        velocity    += acceleration * dt
        position    += velocity * dt

    stick_force is proportional to stick deflection (non-linear: cube root
    gives fine control near center, full speed at extremes).

    Both mass and drag are user-adjustable. Three presets provided.
    """

    def __init__(self, hardware, guard: SoftLimitGuard, broadcast_fn,
                 slider_axis, pan_axis, tilt_axis):
        self.hw         = hardware
        self.guard      = guard
        self.broadcast  = broadcast_fn
        self._slider    = slider_axis
        self._pan       = pan_axis
        self._tilt      = tilt_axis

        # Physics state (steps/s)
        self._v_slider: float = 0.0
        self._v_pan:    float = 0.0
        self._v_tilt:   float = 0.0

        # Target velocities from gamepad (normalized [-1, 1])
        self._t_slider: float = 0.0
        self._t_pan:    float = 0.0
        self._t_tilt:   float = 0.0

        # Physics params
        self.mass: float = 0.40   # seconds to reach full speed
        self.drag: float = 0.55   # viscosity coefficient

        # Motion scale [0.00025 … 1.0] — global linear multiplier for all axes.
        # Applied AFTER the power-curve map so the label percentage is accurate:
        #   0.15 × full_speed = exactly 15%, not the ~31% you'd get inside the curve.
        # Reduces slider, pan AND tilt together — essential for telephoto / crop
        # sensors where every axis needs proportional attenuation.
        # Still called pan_tilt_scale internally for wire-protocol compatibility.
        self.pan_tilt_scale: float = 1.0

        # Speed multiplier from L1/R1 buttons
        self._speed_mult: float = 1.0
        self._l1_held: bool = False
        self._r1_held: bool = False

        # Guard bypass — True during nudge so user can exceed soft limits to re-set them.
        # NEVER left True permanently; nudge handler resets it after move.
        self._guard_bypass: bool = False

        # Nudge override velocities (physical units: mm/s or deg/s).
        # When non-zero, the tick loop bypasses InertiaEngine physics for that axis
        # and sends this velocity directly to VACTUAL.  Physics state is simultaneously
        # zeroed so releasing the button stops the motor instantly — no coast.
        # Set via set_nudge_pt() / set_nudge_slider(); cleared via clear_nudge_*().
        self._nudge_pan:    float = 0.0
        self._nudge_tilt:   float = 0.0
        self._nudge_slider: float = 0.0

        # Arctan lock
        self.arctan_active: bool = False
        self._tracker: Optional[ArcTanTracker] = None

        self._running: bool = False
        self._task: Optional[asyncio.Task] = None

    def set_preset(self, name: str, apply_pt_scale: bool = True):
        """
        Apply a named preset.  If apply_pt_scale is True (default), also resets
        pan_tilt_scale to the preset's default value.  Pass False when the user
        has manually adjusted the sensitivity slider and wants to keep their value.
        """
        if name in RIG_PRESETS and RIG_PRESETS[name] is not None:
            mass, drag, pt_scale = RIG_PRESETS[name]
            self.mass = mass
            self.drag = drag
            if apply_pt_scale:
                self.pan_tilt_scale = pt_scale
            logger.info(
                f"Inertia preset: {name} "
                f"(mass={self.mass}, drag={self.drag}, pt_scale={self.pan_tilt_scale:.2f})"
            )

    def set_params(self, mass: float, drag: float):
        self.mass = max(0.05, min(3.0, mass))
        self.drag = max(0.05, min(1.50, drag))

    def set_pt_sensitivity(self, scale: float):
        """Set pan/tilt sensitivity scale [0.00025 … 1.0]. Slider stays unaffected."""
        self.pan_tilt_scale = max(0.00025, min(1.0, scale))

    def snap_stop_pt(self):
        """
        Immediately zero pan and tilt velocities and hardware output.
        Called on d-pad release so the camera stops the instant the button is
        released rather than coasting through the inertia model.
        Slider is unaffected.
        """
        self._v_pan  = 0.0
        self._v_tilt = 0.0
        self._t_pan  = 0.0
        self._t_tilt = 0.0
        try:
            self.hw.set_tmc_velocity(ADDR_PAN,  0)
            self.hw.set_tmc_velocity(ADDR_TILT, 0)
        except Exception:
            pass

    def snap_stop_slider(self):
        """
        Immediately zero slider velocity and hardware output.
        Called when both triggers are released simultaneously so the slider
        stops cleanly without coasting.
        """
        self._v_slider = 0.0
        self._t_slider = 0.0
        try:
            self.hw.set_tmc_velocity(ADDR_SLIDER, 0)
        except Exception:
            pass

    # ── Nudge API ─────────────────────────────────────────────────────────────
    # Nudge bypasses InertiaEngine physics entirely: the requested velocity is
    # sent directly to VACTUAL while physics state is kept at zero.  Releasing
    # the nudge button (clear_nudge_*) stops the motor in the same tick —
    # no coast, no ramp-down.  This is essential for accurate soft-limit and
    # keyframe positioning where the user must be able to stop on a dime.

    def set_nudge_pt(self, pan: float, tilt: float):
        """
        Activate pan/tilt nudge at the given physical velocities (deg/s).
        Call with the d-pad direction × NUDGE_SPEED_PAN/TILT.
        """
        self._nudge_pan  = pan
        self._nudge_tilt = tilt

    def clear_nudge_pt(self):
        """Stop pan/tilt nudge.  Motor halts in the next tick (≤ 20 ms)."""
        self._nudge_pan  = 0.0
        self._nudge_tilt = 0.0
        # Zero hardware immediately — don't wait for the next tick
        try:
            self.hw.set_tmc_velocity(ADDR_PAN,  0)
            self.hw.set_tmc_velocity(ADDR_TILT, 0)
        except Exception:
            pass

    def set_nudge_slider(self, speed: float):
        """
        Activate slider nudge at the given physical velocity (mm/s).
        Call with (R2 − L2) × NUDGE_SPEED_SLIDER.
        """
        self._nudge_slider = speed

    def clear_nudge_slider(self):
        """Stop slider nudge.  Motor halts in the next tick (≤ 20 ms)."""
        self._nudge_slider = 0.0
        try:
            self.hw.set_tmc_velocity(ADDR_SLIDER, 0)
        except Exception:
            pass

    def set_target(self, slider: float, pan: float, tilt: float):
        """Called from gamepad event handler with normalized [-1, 1] stick values."""
        self._t_slider = slider
        self._t_pan    = pan
        self._t_tilt   = tilt

    def set_speed_modifier(self, l1: bool, r1: bool):
        self._l1_held = l1
        self._r1_held = r1
        if l1:
            self._speed_mult = 0.25
        elif r1:
            self._speed_mult = 2.0
        else:
            self._speed_mult = 1.0

    def set_arctan_tracker(self, tracker: Optional["ArcTanTracker"]):
        self._tracker = tracker

    def start(self):
        if not self._running:
            self._running = True
            self._task = asyncio.create_task(self._loop())

    def stop(self):
        self._running = False
        if self._task:
            self._task.cancel()
        self.hw.stop_all_axes()

    def instant_stop_pt(self):
        """
        Immediately zero pan+tilt — no coast.

        Called when the UI joystick releases so the motor stops in the current
        20 ms tick rather than coasting for ~1-2 seconds under normal mass/drag.
        Zeros both the physics velocity AND the target so the engine holds 0
        until the stick moves again.  Nudge overrides are left intact.
        """
        self._t_pan  = 0.0
        self._t_tilt = 0.0
        self._v_pan  = 0.0
        self._v_tilt = 0.0
        if self._nudge_pan  == 0.0:
            try: self.hw.set_tmc_velocity(ADDR_PAN,  0)
            except Exception: pass
        if self._nudge_tilt == 0.0:
            try: self.hw.set_tmc_velocity(ADDR_TILT, 0)
            except Exception: pass

    def instant_stop_slider(self):
        """Immediately zero slider — no coast."""
        self._t_slider = 0.0
        self._v_slider = 0.0
        if self._nudge_slider == 0.0:
            try: self.hw.set_tmc_velocity(ADDR_SLIDER, 0)
            except Exception: pass

    async def _loop(self):
        """50 Hz physics + VACTUAL motor command loop."""
        # Motors are already enabled globally — no enable/disable here
        _consecutive_errors = 0
        _MAX_CONSECUTIVE = 10  # ~200 ms of failures before giving up
        try:
            while self._running:
                t0 = asyncio.get_event_loop().time()
                try:
                    self._tick()
                    _consecutive_errors = 0   # reset counter on any successful tick
                except Exception as _tick_err:
                    _consecutive_errors += 1
                    logger.error(
                        f"InertiaEngine _tick error "
                        f"({_consecutive_errors}/{_MAX_CONSECUTIVE}): {_tick_err}",
                        exc_info=True,
                    )
                    # Tolerate transient lgpio / GPIO hiccups (common during rapid
                    # PWM frequency changes at soft limits).  Only abort after
                    # _MAX_CONSECUTIVE failures with no successful tick between them.
                    if _consecutive_errors >= _MAX_CONSECUTIVE:
                        logger.error("InertiaEngine: too many consecutive errors — stopping.")
                        self._running = False
                        break
                elapsed = asyncio.get_event_loop().time() - t0
                await asyncio.sleep(max(0, INERTIA_DT - elapsed))
        finally:
            try:
                self.hw.stop_all_axes()  # VACTUAL=0, hold torque maintained
            except Exception as _stop_err:
                logger.error(f"InertiaEngine stop error: {_stop_err}")

    def _tick(self):
        """One physics integration step.  All velocities in physical units."""
        # Non-linear stick mapping: power curve gives fine control near centre
        def _map(t: float, max_v: float) -> float:
            sign = 1 if t >= 0 else -1
            return sign * (abs(t) ** 0.6) * max_v * self._speed_mult

        # Software deadzone on the normalized stick target.
        # The controller's hardware deadzone (DEADZONE raw counts) filters most drift,
        # but on some 8BitDo firmware in 2.4G mode the stick centre sits 1-3% outside
        # the deadzone, producing a tiny persistent target and a slow motor creep even
        # with the stick released.  Any |target| below 2% of full range is clamped to
        # zero here so the physics has a clean zero to coast toward.
        _DZ = 0.02   # 2% of normalized [-1, 1] range
        t_slider = self._t_slider if abs(self._t_slider) > _DZ else 0.0
        t_pan    = self._t_pan    if abs(self._t_pan)    > _DZ else 0.0
        t_tilt   = self._t_tilt   if abs(self._t_tilt)   > _DZ else 0.0

        # Target velocities in physical units (mm/s, deg/s).
        # motion_scale (pan_tilt_scale) is applied AFTER the power-curve map so it
        # acts as a true linear multiplier on all three axes:
        #   _map(t) × scale  ≠  _map(t × scale)
        # Applying scale inside the curve distorts it — e.g. at 15% scale,
        # _map(t × 0.15) peaks at 0.15^0.6 = 31% of max, not the labelled 15%.
        # Applying outside gives exactly 15% of max at 15% scale, which matches
        # what the UI label promises and what the user's muscle memory expects.
        # All three axes share the same scale so telephoto / crop-sensor users can
        # dial everything down together without the slider thundering at full speed.
        target_slider = _map(t_slider, self.guard.slider.speed_limit) * self.pan_tilt_scale
        target_pan    = _map(t_pan,    self.guard.pan.speed_limit)    * self.pan_tilt_scale
        target_tilt   = _map(t_tilt,   self.guard.tilt.speed_limit)   * self.pan_tilt_scale

        # Physics: dv/dt = (target - drag*v) / mass
        dt = INERTIA_DT
        inv_mass = 1.0 / max(0.01, self.mass)

        self._v_slider += (target_slider - self.drag * self._v_slider) * inv_mass * dt
        self._v_pan    += (target_pan    - self.drag * self._v_pan)    * inv_mass * dt
        self._v_tilt   += (target_tilt   - self.drag * self._v_tilt)   * inv_mass * dt

        # ── Safety: clamp physics state after integration ────────────────────
        # Prevents NaN/Inf from reaching set_tmc_velocity() (int(nan) → ValueError
        # which under the old exception handler killed the engine permanently).
        # Also caps extreme velocities from low-drag / high-mass combinations
        # that can overshoot the speed limits before the guard clamps them.
        _V_CAP_S = MAX_SPEED_SLIDER * 2.0
        _V_CAP_P = MAX_SPEED_PAN    * 2.0
        _V_CAP_T = MAX_SPEED_TILT   * 2.0
        if not math.isfinite(self._v_slider): self._v_slider = 0.0
        if not math.isfinite(self._v_pan):    self._v_pan    = 0.0
        if not math.isfinite(self._v_tilt):   self._v_tilt   = 0.0
        self._v_slider = max(-_V_CAP_S, min(_V_CAP_S, self._v_slider))
        self._v_pan    = max(-_V_CAP_P, min(_V_CAP_P, self._v_pan))
        self._v_tilt   = max(-_V_CAP_T, min(_V_CAP_T, self._v_tilt))

        # Soft limit clamping — done BEFORE position integration so the tracker
        # never drifts past a limit and the physics loop cannot rebuild velocity
        # against a hard wall (which would cause the motor to oscillate at the stop).
        if self._guard_bypass:
            # Nudge mode: bypass guard so user can move beyond limits to re-set them.
            vs, vp, vt = self._v_slider, self._v_pan, self._v_tilt
        else:
            vs, vp, vt = self.guard.clamp(
                self._v_slider, self._v_pan, self._v_tilt,
                self._slider.current_mm, self._pan.current_deg, self._tilt.current_deg
            )

        # ── Nudge override ────────────────────────────────────────────────────
        # D-pad → pan/tilt nudge; L2/R2 → slider nudge.
        # When a nudge velocity is set:
        #   1. Physics state (_v_X) is zeroed → no momentum builds during nudge
        #   2. Output velocity (vs/vp/vt) is replaced with nudge speed
        #   3. Guard clamping is bypassed so the user can cross soft limits to re-set them
        # When nudge is cleared, physics state is already 0 → motor stops instantly.
        if self._nudge_pan != 0.0:
            self._v_pan = 0.0
            vp = self._nudge_pan
        if self._nudge_tilt != 0.0:
            self._v_tilt = 0.0
            vt = self._nudge_tilt
        if self._nudge_slider != 0.0:
            self._v_slider = 0.0
            vs = self._nudge_slider

        # ── Velocity floor — snap to zero below the minimum step rate ───────────
        # The physics equations approach zero asymptotically.  The step/sec value
        # sent to lgpio.tx_pwm() is derived from velocity × a fixed multiplier;
        # below ~4 steps/sec the PWM is already stopped (step pin written low).
        # This explicit floor zeroes both the output velocity AND the internal
        # physics state so no residual drift accumulates between user inputs.
        # Only applied when not nudging (nudge has its own precise stop path).
        #
        # The floors scale with motion_scale (pan_tilt_scale) so they shrink
        # proportionally at low scale settings.  Without this, at scale < ~2%
        # the max achievable velocity falls BELOW the fixed floor and the motor
        # never moves at all — making the rig appear dead in tracking mode.
        # Hard minimum (0.001 / 0.003) prevents rounding to zero at extreme scale.
        _floor_pt = max(0.001, 0.10 * self.pan_tilt_scale)
        _floor_s  = max(0.003, 0.30 * self.pan_tilt_scale)
        if self._nudge_pan    == 0.0 and abs(vp) < _floor_pt: vp = 0.0; self._v_pan    = 0.0
        if self._nudge_tilt   == 0.0 and abs(vt) < _floor_pt: vt = 0.0; self._v_tilt   = 0.0
        if self._nudge_slider == 0.0 and abs(vs) < _floor_s:  vs = 0.0; self._v_slider  = 0.0

        # Integrate position with CLAMPED velocities so tracker stays within limits.
        # Also track steps for focus rail macro mode (convert mm delta to steps)
        delta_mm = vs * dt
        self._slider.current_mm += delta_mm
        self._slider.current_steps += int(delta_mm * self._slider.steps_per_mm)
        self._pan.current_deg   += vp * dt
        self._tilt.current_deg  += vt * dt

        # Arctan override: if locked, recompute pan+tilt from slider position
        if self.arctan_active and self._tracker and self._tracker.is_solved:
            prev_pan  = self._pan.current_deg  - vp * dt
            prev_tilt = self._tilt.current_deg - vt * dt
            pan_t, tilt_t = self._tracker.get_angles(self._slider.current_mm)
            self._pan.current_deg  = pan_t
            self._tilt.current_deg = tilt_t
            vp = (pan_t  - prev_pan)  / dt
            vt = (tilt_t - prev_tilt) / dt

        # Send to TMC2209 VACTUAL (hardware-timed, jitter-free)
        # Negative values spin in reverse; VACTUAL=0 returns to STEP/DIR idle.
        self.hw.set_tmc_velocity(ADDR_SLIDER, int(vs * VACTUAL_PER_MM_S))
        self.hw.set_tmc_velocity(ADDR_PAN,    int(vp * VACTUAL_PER_DEG_S_PAN))
        self.hw.set_tmc_velocity(ADDR_TILT,   int(vt * VACTUAL_PER_DEG_S_TILT))

        # Store clamped velocities so next tick's physics starts from correct state.
        # For nudge axes, _v_X was already zeroed in the nudge override block above;
        # DO NOT overwrite with vs/vp/vt (which hold the nudge velocity) or physics
        # will jump-start from nudge speed the instant the button is released.
        #
        # Feeding the GUARD'S output back (not the raw physics value) means that when
        # the guard hard-zeros an axis at a soft limit, physics restarts from zero next
        # tick rather than from the unconstrained velocity.  This keeps the decel ramp
        # smooth and avoids the step pin toggling start/stop on alternate ticks
        # (which stresses lgpio's waveform thread with high-mass / low-drag settings).
        if self._nudge_slider == 0.0: self._v_slider = vs
        if self._nudge_pan    == 0.0: self._v_pan    = vp
        if self._nudge_tilt   == 0.0: self._v_tilt   = vt

        # Hardware endstop / Hall sensor check (protects against over-travel in live mode).
        # If triggered, zero slider velocity and PWM immediately.
        try:
            if self.hw.read_endstop() or self.hw.read_hall_sensor():
                self._v_slider = 0.0
                self.hw.set_tmc_velocity(ADDR_SLIDER, 0)
        except Exception:
            pass


# ─── ARCTAN TRACKER ──────────────────────────────────────────────────────────

@dataclass
class CalibPoint:
    """One calibration measurement: gantry position + where camera pointed."""
    slider_mm:  float
    pan_deg:    float
    tilt_deg:   float


class ArcTanTracker:
    """
    3D subject position solver using N calibration points.

    The user directly measures (pan_deg, tilt_deg) pointing at the subject
    from each slider position. Rail tilt is irrelevant — it's already encoded
    in the tilt readings. We just need to find the 3D point that best explains
    all the measured ray directions.

    Model:
      Each calibration point gives a unit ray in camera-local space:
        ray = [cos(tilt)*cos(pan), cos(tilt)*sin(pan), sin(tilt)]

      Camera position along rail (1D): x = slider_mm (horizontal component
      only needed for parallax; the ball head stays level so pan/tilt are
      world-space regardless of rail angle).

      We solve for subject (X, Y, Z) in world mm such that for each point i:
        pan_i  = atan2(Y - 0,        X - slider_mm_i)   [horizontal plane]
        tilt_i = atan2(Z,            sqrt((X-s_i)^2 + Y^2))

      This is a straightforward nonlinear least-squares in 3 unknowns.
    """

    WARN_RESIDUAL_DEG = 1.5
    MIN_POINTS = 3

    def __init__(self):
        self.points: List[CalibPoint] = []
        self.subject: Optional[np.ndarray] = None   # [X, Y, Z] world mm
        self.residual_deg: float = 0.0
        self.is_solved: bool = False
        self.warning: str = ""

    def add_point(self, slider_mm: float, pan_deg: float, tilt_deg: float):
        self.points.append(CalibPoint(slider_mm, pan_deg, tilt_deg))
        logger.info(f"ArcTan: added point {len(self.points)}: "
                    f"s={slider_mm:.1f}mm pan={pan_deg:.2f}° tilt={tilt_deg:.2f}°")
        if len(self.points) >= self.MIN_POINTS:
            self.solve()

    def clear_points(self):
        self.points = []
        self.subject = None
        self.is_solved = False
        self.warning = ""

    def solve(self) -> bool:
        """
        Least-squares solve for subject [X, Y, Z].
        No rail tilt needed — pan/tilt measurements are already in world space
        because the ball head keeps the motors level.
        """
        if len(self.points) < self.MIN_POINTS:
            return False

        # Camera X positions (1D along rail horizontal projection)
        cam_xs  = np.array([p.slider_mm  for p in self.points])
        pan_r   = np.array([math.radians(p.pan_deg)  for p in self.points])
        tilt_r  = np.array([math.radians(p.tilt_deg) for p in self.points])

        # Initial guess: subject 500mm ahead of midpoint, same height
        subj = np.array([float(np.mean(cam_xs)) + 500.0, 500.0, 0.0])

        for _ in range(40):
            dx    = subj[0] - cam_xs
            dy    = np.full_like(dx, subj[1])
            dz    = np.full_like(dx, subj[2])
            horiz = np.sqrt(dx**2 + dy**2)

            pred_pan  = np.arctan2(dy, dx)
            pred_tilt = np.arctan2(dz, horiz)

            res_pan  = pan_r  - pred_pan
            res_tilt = tilt_r - pred_tilt

            r2 = dx**2 + dy**2
            r3 = horiz**2 + dz**2

            J = np.zeros((2 * len(self.points), 3))
            J[0::2, 0] = -dy / r2
            J[0::2, 1] =  dx / r2
            J[0::2, 2] =  0
            J[1::2, 0] = -dx * dz / (horiz * r3)
            J[1::2, 1] = -dy * dz / (horiz * r3)
            J[1::2, 2] =  horiz / r3

            residuals       = np.empty(2 * len(self.points))
            residuals[0::2] = res_pan
            residuals[1::2] = res_tilt

            try:
                delta, _, _, _ = np.linalg.lstsq(J, residuals, rcond=None)
            except np.linalg.LinAlgError:
                break
            subj += delta
            if np.linalg.norm(delta) < 1e-4:
                break

        # RMS residual
        dx    = subj[0] - cam_xs
        dy    = np.full_like(dx, subj[1])
        dz    = np.full_like(dx, subj[2])
        horiz = np.sqrt(dx**2 + dy**2)
        pred_pan  = np.degrees(np.arctan2(dy, dx))
        pred_tilt = np.degrees(np.arctan2(dz, horiz))
        meas_pan  = np.array([p.pan_deg  for p in self.points])
        meas_tilt = np.array([p.tilt_deg for p in self.points])
        rms = float(np.sqrt(np.mean(
            (pred_pan - meas_pan)**2 + (pred_tilt - meas_tilt)**2
        )))

        self.subject      = subj
        self.residual_deg = rms
        self.is_solved    = True

        slider_span = max(p.slider_mm for p in self.points) - min(p.slider_mm for p in self.points)

        self.warning = ""
        if rms > self.WARN_RESIDUAL_DEG:
            self.warning = (f"High residual ({rms:.2f}°) — re-aim more carefully "
                            f"or add more calibration points.")
        elif slider_span < 50:
            self.warning = ("Points too close together — spread them further "
                            "along the rail for accuracy.")

        logger.info(f"ArcTan solved: subject={subj.round(1)}, RMS={rms:.3f}° "
                    f"warning='{self.warning}'")
        return True

    def get_angles(self, slider_mm: float) -> Tuple[float, float]:
        """Return (pan_deg, tilt_deg) for a given slider position."""
        if not self.is_solved or self.subject is None:
            return 0.0, 0.0
        sx, sy, sz = self.subject
        dx    = sx - slider_mm
        dy    = sy
        dz    = sz
        horiz = math.sqrt(dx**2 + dy**2)
        pan   = math.degrees(math.atan2(dy, dx))
        tilt  = math.degrees(math.atan2(dz, horiz))
        return pan, tilt


# ─── PROGRAMMED MOVE ─────────────────────────────────────────────────────────

@dataclass
class Keyframe:
    slider_mm: float
    pan_deg:   float
    tilt_deg:  float
    duration_s: float = 3.0          # time to reach NEXT keyframe
    easing:     str   = "gaussian"   # from distributions.CURVE_FUNCTIONS


class ProgrammedMove:
    """
    Manages a list of keyframes and plays them back via TrajectoryPlayer.

    Segment model: each keyframe specifies the duration and easing to reach
    the NEXT keyframe. The last keyframe's duration/easing are ignored.

    Origin system: positions stored as ABSOLUTE (mm/deg). At playback,
    they are offset by the session origin so the move plays relative to
    wherever the user parked the rig.
    """

    def __init__(self, hardware, guard: SoftLimitGuard,
                 slider_axis, pan_axis, tilt_axis,
                 broadcast_fn, arctan_tracker: Optional[ArcTanTracker] = None):
        self.hw         = hardware
        self.guard      = guard
        self._slider    = slider_axis
        self._pan       = pan_axis
        self._tilt      = tilt_axis
        self.broadcast  = broadcast_fn
        self.tracker    = arctan_tracker

        self.keyframes: List[Keyframe] = []
        self.preroll_s: float = 3.0
        self.loop: bool = False

        # Origin offset (set by user "Set Origin at Start" action)
        self.origin_slider: float = 0.0
        self.origin_pan:    float = 0.0
        self.origin_tilt:   float = 0.0

        # Reference point — an easy-to-reproduce physical spot (e.g. rail centre,
        # camera at horizon) stored in design space so it's portable between
        # deployments.  None until the user explicitly saves it.
        # reference_* = ref_physical - origin  (set while physically at reference)
        # To deploy from reference: new_origin = current_physical - reference_*
        self.reference_slider: Optional[float] = None
        self.reference_pan:    Optional[float] = None
        self.reference_tilt:   Optional[float] = None

        # ── Path planning ─────────────────────────────────────────────────────
        # path_mode: "linear" | "catmull_rom"
        #   linear      — straight lines between keyframes, per-segment easing
        #   catmull_rom — smooth continuous curve through all keyframes, same easing system
        self.path_mode: str = "linear"

        # Catmull-Rom tangent scale.
        #   0.5 = standard CR (natural, smooth)
        #   0.0 = near-linear (tight to keyframes)
        #   1.0 = loose (tangents doubled, slight overshoot on curves)
        self.catmull_tension: float = 0.5

        # Global easing curve — default for all segments.
        # Per-segment kf.easing overrides this for individual segments.
        # Cycloid default: brachistochrone profile, more natural than sinusoidal.
        self.global_easing: str = "cycloid"

        self._running: bool = False
        self._stop_event    = asyncio.Event()

    # ── Path-planning API ─────────────────────────────────────────────────────

    def set_path_mode(self, mode: str):
        """'linear' or 'catmull_rom'."""
        if mode in ("linear", "catmull_rom"):
            self.path_mode = mode

    def set_catmull_tension(self, tension: float):
        self.catmull_tension = max(0.0, min(1.5, tension))

    def set_global_easing(self, curve: str):
        """Set the default easing curve for all segments."""
        self.global_easing = curve

    def apply_global_easing_to_all(self):
        """Reset every segment's easing to the current global curve."""
        for kf in self.keyframes:
            kf.easing = self.global_easing

    def scale_total_duration(self, new_total_s: float) -> float:
        """
        Rescale every segment's duration_s proportionally so the sum equals
        new_total_s.  The relative pacing between segments is preserved — a
        segment that was 40 % of the move stays 40 % of the new total.

        Returns the actual new total (may differ slightly due to per-segment
        floor of 0.1 s preventing any segment collapsing to zero).
        """
        segs = self.keyframes[:-1]   # last keyframe has no outgoing segment
        if len(segs) == 0:
            return 0.0
        current = sum(max(0.1, kf.duration_s) for kf in segs)
        if current <= 0.0:
            return 0.0
        scale = max(0.0, new_total_s) / current
        for kf in segs:
            kf.duration_s = round(max(0.1, kf.duration_s * scale), 2)
        return sum(kf.duration_s for kf in segs)

    def total_duration_s(self) -> float:
        """Sum of all segment durations (excludes last keyframe)."""
        if len(self.keyframes) < 2:
            return 0.0
        return sum(max(0.1, kf.duration_s) for kf in self.keyframes[:-1])

    def set_segment_pct(self, seg_index: int, pct: float):
        """
        Set segment[seg_index] to pct% of total duration.

        The immediately following segment absorbs the difference so the total
        duration stays constant.  If the following segment would drop below its
        0.1 s floor the change is clamped so neither segment collapses.

        The last segment (index n-1) is intentionally read-only from this
        method — it auto-fills as whatever percentage remains after the user
        works through the earlier segments.
        """
        segs  = self.keyframes[:-1]   # all keyframes except the final endpoint
        n     = len(segs)
        if n < 2 or seg_index < 0 or seg_index >= n - 1:
            return  # nothing to do (last segment can't be pushed forward)

        MIN_DUR = 0.1   # seconds floor per segment

        total   = sum(max(MIN_DUR, kf.duration_s) for kf in segs)
        new_dur = max(MIN_DUR, total * float(pct) / 100.0)
        old_dur = max(MIN_DUR, segs[seg_index].duration_s)
        delta   = new_dur - old_dur

        # Next segment absorbs the difference
        nxt     = segs[seg_index + 1]
        new_nxt = max(MIN_DUR, nxt.duration_s - delta)

        # If next hit its floor, clamp this segment too
        actual_delta = nxt.duration_s - new_nxt
        segs[seg_index].duration_s = round(old_dur + actual_delta, 2)
        nxt.duration_s             = round(new_nxt, 2)

    def add_keyframe(self, slider_mm: float, pan_deg: float, tilt_deg: float,
                     duration_s: float = None, easing: Optional[str] = None) -> int:
        """
        Add a keyframe at the end of the sequence.

        duration_s defaults to the current average segment duration so adding
        a new keyframe keeps all proportions equal rather than throwing off
        the pacing.  Explicit value overrides this.
        """
        if duration_s is None:
            segs = self.keyframes[:-1] if self.keyframes else []
            if segs:
                duration_s = sum(max(0.1, k.duration_s) for k in segs) / len(segs)
            else:
                duration_s = 3.0
        kf = Keyframe(slider_mm, pan_deg, tilt_deg,
                      duration_s,
                      easing if easing is not None else self.global_easing)
        self.keyframes.append(kf)
        return len(self.keyframes) - 1

    def generate_unified_trajectory(
        self,
        n_points: int,
        origin_slider: float = 0.0,
        origin_pan:    float = 0.0,
        origin_tilt:   float = 0.0,
        for_timelapse: bool  = False,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate (traj_s, traj_p, traj_t) arrays of length *n_points*.

        Both modes use kf.duration_s proportions to distribute points across
        segments — this controls pacing (how much time / how many frames each
        segment gets relative to the others).

        for_timelapse=False (cinema) — per-segment easing (kf.easing) respected;
            output is 60 fps velocity commands streamed in real-time.

        for_timelapse=True — global_easing applied uniformly within each segment
            (per-segment overrides ignored for simplicity); output is N discrete
            capture positions, one per timelapse frame.  duration_s proportions
            still control how many frames are allocated to each segment, so a
            segment with duration_s=6 gets twice as many frames as one with
            duration_s=3, producing slower apparent motion in that part of the
            final timelapse video.

        Origin offsets are applied so the move plays relative to wherever the
        user parked the rig when they set the reference position.
        """
        kfs = self.keyframes
        N   = len(kfs)

        if N == 0:
            return (np.zeros(n_points), np.zeros(n_points), np.zeros(n_points))
        if N == 1:
            s = kfs[0].slider_mm + origin_slider
            p = kfs[0].pan_deg   + origin_pan
            t = kfs[0].tilt_deg  + origin_tilt
            return (np.full(n_points, s), np.full(n_points, p), np.full(n_points, t))

        # Build position matrix (N × 3) with origin offsets
        positions = np.array(
            [[kf.slider_mm + origin_slider,
              kf.pan_deg   + origin_pan,
              kf.tilt_deg  + origin_tilt]
             for kf in kfs],
            dtype=float,
        )

        # ── 1. Spatial path (dense samples) ──────────────────────────────────
        N_DENSE = max(10_000, n_points * 20)

        if self.path_mode == "catmull_rom":
            dense = _catmull_rom_chain(positions, self.catmull_tension, N_DENSE)
        else:
            # Linear segments — each evenly sampled, endpoints joined
            seg_n  = N_DENSE // (N - 1)
            parts  = []
            for i in range(N - 1):
                t_seg  = np.linspace(0.0, 1.0,
                                     seg_n + (1 if i == N - 2 else 0))
                seg    = positions[i] + t_seg[:, None] * (positions[i+1] - positions[i])
                parts.append(seg if i == N - 2 else seg[:-1])
            dense = np.vstack(parts)

        # ── 2. Arc-length parameterisation ───────────────────────────────────
        arc_s = _arc_length_params(dense)   # monotone [0 … 1] for each dense point

        # Keyframe i sits at index round(i/(N-1) * (len(dense)-1)) in the dense path
        kf_arc = np.array([
            arc_s[min(round(i / (N - 1) * (len(dense) - 1)), len(dense) - 1)]
            for i in range(N)
        ])
        kf_arc[0]  = 0.0
        kf_arc[-1] = 1.0

        # ── 3. Time → arc mapping ─────────────────────────────────────────────
        # Both cinema and timelapse use duration_s proportions.
        # Cinema respects per-segment kf.easing; timelapse uses global_easing
        # uniformly so the pacing curve is predictable across all segments.
        total_dur = sum(max(0.1, kf.duration_s) for kf in kfs[:-1])
        if total_dur <= 0.0:
            total_dur = float(N - 1) * 3.0

        N_GRID   = max(n_points * 50, 50_000)
        arc_grid = np.zeros(N_GRID)
        t_cursor = 0.0

        for i, kf in enumerate(kfs[:-1]):
            seg_frac  = max(0.1, kf.duration_s) / total_dur
            t_end     = t_cursor + seg_frac
            arc_start = kf_arc[i]
            arc_end   = kf_arc[i + 1]

            # Easing curve: timelapse always uses global_easing so all segments
            # feel consistent in the final video; cinema respects per-segment overrides.
            curve_name = self.global_easing if for_timelapse else kf.easing
            e_x, e_y  = _easing_curve(curve_name)

            i_start = round(t_cursor * N_GRID)
            i_end   = min(round(t_end * N_GRID), N_GRID)
            n_fill  = max(i_end - i_start, 2)

            local_t   = np.linspace(0.0, 1.0, n_fill)
            local_arc = np.interp(local_t, e_x, e_y)
            arc_grid[i_start:i_end] = arc_start + local_arc * (arc_end - arc_start)
            t_cursor = t_end

        arc_grid[-1] = 1.0
        query_t    = np.linspace(0.0, 1.0, n_points)
        grid_t     = np.linspace(0.0, 1.0, N_GRID)
        target_arc = np.interp(query_t, grid_t, arc_grid)

        # ── 4. Sample dense path at target arc positions ──────────────────────
        result = np.column_stack([
            np.interp(target_arc, arc_s, dense[:, j]) for j in range(3)
        ])
        return result[:, 0], result[:, 1], result[:, 2]

    def compute_min_duration(self) -> Tuple[float, str]:
        """
        Minimum safe total duration for cinematic playback without exceeding
        hardware speed limits on any axis.

        Returns (min_seconds, description_of_limiting_axis).
        A return of 0.0 means the path is stationary or has < 2 keyframes.
        """
        if len(self.keyframes) < 2:
            return 0.0, ""

        N  = 2000
        ts, tp, tt = self.generate_unified_trajectory(N)

        # Each normalised step corresponds to total_duration/N seconds.
        # velocity = Δposition * N / total_duration  ≤  max_speed
        # → total_duration ≥  Δposition * N / max_speed
        max_ds = float(np.max(np.abs(np.diff(ts))))
        max_dp = float(np.max(np.abs(np.diff(tp))))
        max_dt = float(np.max(np.abs(np.diff(tt))))

        min_s  = max_ds * N / MAX_SPEED_SLIDER if max_ds > 0 else 0.0
        min_p  = max_dp * N / MAX_SPEED_PAN    if max_dp > 0 else 0.0
        min_t  = max_dt * N / MAX_SPEED_TILT   if max_dt > 0 else 0.0
        min_dur = max(min_s, min_p, min_t)

        if min_dur <= 0.0:
            return 0.0, ""

        if min_dur == min_s:
            desc = f"slider ({max_ds * N / min_dur:.1f} mm/s peak)"
        elif min_dur == min_p:
            desc = f"pan ({max_dp * N / min_dur:.1f}°/s peak)"
        else:
            desc = f"tilt ({max_dt * N / min_dur:.1f}°/s peak)"

        return round(min_dur, 2), desc

    def set_origin(self, slider_mm: float, pan_deg: float, tilt_deg: float):
        self.origin_slider = slider_mm
        self.origin_pan    = pan_deg
        self.origin_tilt   = tilt_deg
        logger.info(f"Origin set: s={slider_mm:.2f}mm p={pan_deg:.2f}° t={tilt_deg:.2f}°")

    def save_reference_point(self, current_mm: float, current_pan: float,
                             current_tilt: float):
        """Record the current physical position as the reproducible reference
        point, expressed in design space so it survives redeployment.

        Call this while physically standing at your easy-to-find spot
        (e.g. gantry at rail centre, camera at horizon), *after* set_origin
        has already been called so we know where design-space zero lives.
        """
        self.reference_slider = current_mm  - self.origin_slider
        self.reference_pan    = current_pan  - self.origin_pan
        self.reference_tilt   = current_tilt - self.origin_tilt
        logger.info(
            f"Reference point saved: design=("
            f"s={self.reference_slider:.2f}mm, "
            f"p={self.reference_pan:.2f}°, "
            f"t={self.reference_tilt:.2f}°)"
        )

    def apply_reference_point(self, current_mm: float, current_pan: float,
                              current_tilt: float) -> bool:
        """Compute and set the correct origin from the reference position.

        Call this when the rig is physically parked at the saved reference
        spot.  After this, 'return_to_start' will move accurately to
        keyframe[0] even across power cycles and re-deployments.

        Returns True on success, False if no reference has been saved.
        """
        if self.reference_slider is None:
            logger.warning("apply_reference_point: no reference point saved for this move")
            return False
        self.set_origin(
            current_mm  - self.reference_slider,
            current_pan  - self.reference_pan,
            current_tilt - self.reference_tilt,
        )
        logger.info(
            f"Origin set from reference: physical=("
            f"s={current_mm:.2f}mm, p={current_pan:.2f}°, t={current_tilt:.2f}°) "
            f"→ origin=("
            f"s={self.origin_slider:.2f}mm, "
            f"p={self.origin_pan:.2f}°, "
            f"t={self.origin_tilt:.2f}°)"
        )
        return True


    def update_keyframe(self, index: int, **kwargs):
        if 0 <= index < len(self.keyframes):
            for k, v in kwargs.items():
                setattr(self.keyframes[index], k, v)

    def remove_keyframe(self, index: int):
        if 0 <= index < len(self.keyframes):
            self.keyframes.pop(index)

    def reverse_keyframes(self):
        """Flip keyframe order so the move plays from end to start."""
        if len(self.keyframes) < 2:
            return
        positions = [(kf.slider_mm, kf.pan_deg, kf.tilt_deg) for kf in self.keyframes]
        durations = [kf.duration_s for kf in self.keyframes[:-1]]
        positions.reverse()
        durations.reverse()
        for i, kf in enumerate(self.keyframes):
            kf.slider_mm = positions[i][0]
            kf.pan_deg   = positions[i][1]
            kf.tilt_deg  = positions[i][2]
            if i < len(self.keyframes) - 1:
                kf.duration_s = durations[i]

    def clear_keyframes(self):
        self.keyframes = []

    def stop(self):
        self._stop_event.set()

    async def return_to_start(self, speed_fraction: float = 0.5):
        """Move rig to first keyframe (world position) at reduced speed.

        Bypasses soft-limit guard because keyframe positions are always
        valid targets — the guard's decel ramp can otherwise zero out
        velocity before the rig reaches a keyframe sitting at a limit
        boundary.
        """
        if not self.keyframes:
            return
        kf0 = self.keyframes[0]
        target_s = kf0.slider_mm + self.origin_slider
        target_p = kf0.pan_deg   + self.origin_pan
        target_t = kf0.tilt_deg  + self.origin_tilt

        await self.broadcast({"type": "cinematic_status",
                               "msg": "Returning to start position…"})
        await self._move_to(target_s, target_p, target_t,
                            duration_s=max(2.0, kf0.duration_s * 0.5),
                            bypass_guard=True)

    async def play(self, skip_first_return: bool = False):
        """Execute the programmed move once (or loop until stopped)."""
        if len(self.keyframes) < 2:
            await self.broadcast({"type": "cinematic_status",
                                   "msg": "Need at least 2 keyframes to play."})
            return

        self._stop_event.clear()
        self._running = True

        try:
            take = 0
            while not self._stop_event.is_set():
                take += 1

                # Always move to first keyframe before each take so the
                # trajectory starts from the correct position.
                if not (take == 1 and skip_first_return):
                    await self.return_to_start()
                if self._stop_event.is_set():
                    break

                await self.broadcast({"type": "cinematic_status",
                                       "msg": f"Pre-roll {self.preroll_s:.1f}s… (Take {take})"})

                # Pre-roll delay
                for _ in range(int(self.preroll_s * 10)):
                    if self._stop_event.is_set():
                        break
                    await asyncio.sleep(0.1)

                if self._stop_event.is_set():
                    break

                await self.broadcast({"type": "cinematic_play_start", "take": take})
                await self._execute_move()

                if not self.loop or self._stop_event.is_set():
                    break

        finally:
            self._running = False
            self.hw.stop_all_axes()
            await self.broadcast({"type": "cinematic_play_done"})

    async def _execute_move(self):
        """
        Execute the full programmed move using the unified trajectory generator.

        The trajectory is pre-computed in the asyncio event loop, then the
        motor update loop runs in a dedicated thread via asyncio.to_thread().
        This isolates UART timing from asyncio event loop contention (Sony USB
        polling, WebSocket messages, etc.) which was causing jitter at 60 Hz.
        """
        FPS = 60

        total_dur = sum(max(0.1, kf.duration_s) for kf in self.keyframes[:-1])
        n_frames  = max(2, int(total_dur * FPS))

        # ── Pre-compute full trajectory before handing off to motor thread ────
        traj_s, traj_p, traj_t = self.generate_unified_trajectory(
            n_frames,
            self.origin_slider,
            self.origin_pan,
            self.origin_tilt,
            for_timelapse=False,
        )

        # ── ArcTan subject-tracking override (compute while still in loop) ───
        if self.tracker and self.tracker.is_solved:
            for j in range(n_frames):
                traj_p[j], traj_t[j] = self.tracker.get_angles(traj_s[j])

        self.hw.enable_motors(True)
        loop = asyncio.get_running_loop()

        # ── Motor loop runs in a dedicated thread — no asyncio interference ──
        def _motor_loop():
            dt      = 1.0 / FPS
            start_t = time.monotonic()

            for frame in range(n_frames - 1):
                if self._stop_event.is_set():
                    break

                delta_s = float(traj_s[frame + 1] - traj_s[frame])
                delta_p = float(traj_p[frame + 1] - traj_p[frame])
                delta_t = float(traj_t[frame + 1] - traj_t[frame])

                v_s = delta_s / dt
                v_p = delta_p / dt
                v_t = delta_t / dt

                # Track steps for focus rail (convert mm delta to steps)
                self._slider.current_steps += int(delta_s * self._slider.steps_per_mm)
                self._slider.current_mm = float(traj_s[frame + 1])
                self._pan.current_deg   = float(traj_p[frame + 1])
                self._tilt.current_deg  = float(traj_t[frame + 1])

                v_s, v_p, v_t = self.guard.clamp(
                    v_s, v_p, v_t,
                    self._slider.current_mm,
                    self._pan.current_deg,
                    self._tilt.current_deg,
                )

                self.hw.set_tmc_velocity(ADDR_SLIDER, int(v_s * VACTUAL_PER_MM_S))
                self.hw.set_tmc_velocity(ADDR_PAN,    int(v_p * VACTUAL_PER_DEG_S_PAN))
                self.hw.set_tmc_velocity(ADDR_TILT,   int(v_t * VACTUAL_PER_DEG_S_TILT))

                # Non-blocking progress broadcast back to event loop
                if frame % 30 == 0:
                    asyncio.run_coroutine_threadsafe(
                        self.broadcast({
                            "type":     "cinematic_progress",
                            "segment":  0,
                            "segments": 1,
                            "progress": round(frame / (n_frames - 1), 3),
                            "pos_s":    round(self._slider.current_mm, 2),
                            "pos_p":    round(self._pan.current_deg, 2),
                            "pos_t":    round(self._tilt.current_deg, 2),
                        }),
                        loop,
                    )

                # Precision timing with monotonic clock — no asyncio involvement
                next_t = start_t + (frame + 1) * dt
                remaining = next_t - time.monotonic()
                if remaining > 0.001:
                    time.sleep(remaining - 0.001)
                while time.monotonic() < next_t:
                    pass  # sub-millisecond spin

            self.hw.stop_all_axes()

        await asyncio.to_thread(_motor_loop)

    async def _move_to(self, target_s: float, target_p: float, target_t: float,
                       duration_s: float = 3.0, bypass_guard: bool = False):
        """Simple smooth move to absolute position.

        When bypass_guard is True the soft-limit decel ramp is skipped so
        the move can reach targets sitting at limit boundaries (e.g.
        keyframe 0 parked at a soft limit).  The smootherstep profile
        already decelerates to zero at the endpoint so this is safe.
        """
        FPS   = 60
        n     = max(2, int(duration_s * FPS))
        dt    = 1.0 / FPS

        cur_s = self._slider.current_mm
        cur_p = self._pan.current_deg
        cur_t = self._tilt.current_deg

        # Smootherstep for the return move
        def _ss(x):
            t = max(0.0, min(1.0, x))
            return t * t * t * (t * (6 * t - 15) + 10)

        self.hw.enable_motors(True)
        start_t = asyncio.get_event_loop().time()

        for frame in range(n - 1):
            if self._stop_event.is_set():
                break
            frac = _ss(frame / (n - 1))
            next_frac = _ss((frame + 1) / (n - 1))

            s_nxt = cur_s + next_frac * (target_s - cur_s)
            p_nxt = cur_p + next_frac * (target_p - cur_p)
            t_nxt = cur_t + next_frac * (target_t - cur_t)
            s_now = cur_s + frac      * (target_s - cur_s)
            p_now = cur_p + frac      * (target_p - cur_p)
            t_now = cur_t + frac      * (target_t - cur_t)

            v_s = (s_nxt - s_now) / dt
            v_p = (p_nxt - p_now) / dt
            v_t = (t_nxt - t_now) / dt

            if bypass_guard:
                v_s_c, v_p_c, v_t_c = v_s, v_p, v_t
            else:
                v_s_c, v_p_c, v_t_c = self.guard.clamp(v_s, v_p, v_t, s_nxt, p_nxt, t_nxt)

            self.hw.set_tmc_velocity(ADDR_SLIDER, int(v_s_c * VACTUAL_PER_MM_S))
            self.hw.set_tmc_velocity(ADDR_PAN,    int(v_p_c * VACTUAL_PER_DEG_S_PAN))
            self.hw.set_tmc_velocity(ADDR_TILT,   int(v_t_c * VACTUAL_PER_DEG_S_TILT))

            if bypass_guard:
                # Track steps for focus rail (calculate delta from current position)
                delta_s = s_nxt - self._slider.current_mm
                self._slider.current_steps += int(delta_s * self._slider.steps_per_mm)
                self._slider.current_mm = s_nxt
                self._pan.current_deg   = p_nxt
                self._tilt.current_deg  = t_nxt
            else:
                # Track steps for focus rail (convert velocity delta to steps)
                delta_s = v_s_c * dt
                self._slider.current_steps += int(delta_s * self._slider.steps_per_mm)
                self._slider.current_mm += delta_s
                self._pan.current_deg   += v_p_c * dt
                self._tilt.current_deg  += v_t_c * dt

            next_t = start_t + (frame + 1) * dt
            sleep  = next_t - asyncio.get_event_loop().time() - 0.001
            if sleep > 0:
                await asyncio.sleep(sleep)

        self.hw.stop_all_axes()
        # Do NOT disable motors — InertiaEngine takes over immediately after this
        # move completes, and it needs the EN pin to stay active (LOW).


# ─── MOVE LIBRARY ─────────────────────────────────────────────────────────────

@dataclass
class SavedMove:
    name:        str
    created:     str
    notes:       str
    total_duration_s: float
    extents:     Dict        # slider_range_mm, pan_range_deg, tilt_range_deg
    keyframes:   List[Dict]  # serialised Keyframe dicts
    # Optional reproducible reference point stored in design space.
    # None for moves saved before this feature was added (backward-compatible).
    reference:   Optional[Dict] = None   # {slider_mm, pan_deg, tilt_deg}


class MoveLibrary:
    """
    Named keyframe sequence persistence.

    Moves are stored absolute (mm/deg). At playback, the session origin
    is applied as an offset so the move is always relative to the user's
    parked home position.
    """

    def __init__(self, path: str = MOVES_FILE):
        self._path = path
        self._moves: Dict[str, SavedMove] = {}
        self._load()

    def _load(self):
        if os.path.exists(self._path):
            try:
                with open(self._path) as f:
                    raw = json.load(f)
                for name, data in raw.items():
                    self._moves[name] = SavedMove(**data)
                logger.info(f"MoveLibrary: loaded {len(self._moves)} moves from {self._path}")
            except Exception as e:
                logger.warning(f"MoveLibrary load failed: {e}")

    def _save(self):
        try:
            with open(self._path, "w") as f:
                json.dump({n: asdict(m) for n, m in self._moves.items()}, f, indent=2)
        except Exception as e:
            logger.error(f"MoveLibrary save failed: {e}")

    def save_move(self, name: str, keyframes: List[Keyframe], notes: str = "",
                  reference: Optional[Dict] = None) -> SavedMove:
        import datetime
        if not keyframes:
            raise ValueError("Cannot save empty keyframe list.")

        total_dur = sum(kf.duration_s for kf in keyframes[:-1])
        s_vals = [kf.slider_mm for kf in keyframes]
        p_vals = [kf.pan_deg   for kf in keyframes]
        t_vals = [kf.tilt_deg  for kf in keyframes]

        move = SavedMove(
            name     = name,
            created  = datetime.datetime.now().isoformat(),
            notes    = notes,
            total_duration_s = round(total_dur, 2),
            extents  = {
                "slider_min_mm":  round(min(s_vals), 2),
                "slider_max_mm":  round(max(s_vals), 2),
                "pan_min_deg":    round(min(p_vals), 2),
                "pan_max_deg":    round(max(p_vals), 2),
                "tilt_min_deg":   round(min(t_vals), 2),
                "tilt_max_deg":   round(max(t_vals), 2),
            },
            keyframes = [asdict(kf) for kf in keyframes],
            reference = reference,
        )
        self._moves[name] = move
        self._save()
        logger.info(f"MoveLibrary: saved '{name}' ({len(keyframes)} keyframes, "
                    f"{total_dur:.1f}s, ref={'yes' if reference else 'none'})")
        return move

    def load_move(self, name: str):
        """Return (keyframes, reference_dict_or_None)."""
        if name not in self._moves:
            raise KeyError(f"Move '{name}' not found.")
        m = self._moves[name]
        return [Keyframe(**kf) for kf in m.keyframes], m.reference

    def delete_move(self, name: str):
        if name in self._moves:
            del self._moves[name]
            self._save()

    def list_moves(self) -> List[Dict]:
        return [
            {
                "name":     m.name,
                "created":  m.created,
                "notes":    m.notes,
                "duration": m.total_duration_s,
                "extents":  m.extents,
                "keyframes": len(m.keyframes),
            }
            for m in self._moves.values()
        ]

    def rename_move(self, old_name: str, new_name: str):
        if old_name not in self._moves:
            raise KeyError(f"Move '{old_name}' not found.")
        move = self._moves.pop(old_name)
        move.name = new_name
        self._moves[new_name] = move
        self._save()
