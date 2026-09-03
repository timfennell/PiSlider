#!/usr/bin/env python3
"""
holygrail.py — Holy Grail exposure brain for PiSlider  v3.0

Architecture
────────────
Three layers blend together each frame, with weights that shift
dynamically based on phase, tracker confidence, and conditions:

  1. ASTRONOMICAL MODEL  (_compute_astro)
     Sun/moon positions -> phase classification -> EV/Kelvin/interval
     priors. Includes ambient moonlight model (full moon +2.5 stops vs
     new moon), disc-in-frame geometry, and look-ahead anticipation
     (begins pre-adjusting EV before sun/moon crosses into frame).

  2. DNG CAPTURE TRACKER  (AdaptiveEVTracker + push_capture_ev)
     After every saved frame, app.py reads luminance from the thumbnail
     and calls push_capture_ev(). Builds a rolling window of 20 frames.
     Fits weighted linear regression -> slope (stops/frame) + R².
     Works at night - reads from the capture, not the preview.

  3. DYNAMIC BLEND WEIGHT  (_blend_weight)
     Pixel vs astro trust shifts each frame:
     - Deep stable night  -> 15% pixel / 85% astro
     - Active transition  -> 50/50
     - Day with clouds    -> 75% pixel / 25% astro
     - High R² (clean trend) -> more pixel weight
     - Moon rising/setting   -> more astro weight
"""

from __future__ import annotations

import math
import time
import datetime
import logging
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any, Tuple

import numpy as np

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

from zoneinfo import ZoneInfo
from astral import LocationInfo
from astral.sun import elevation as sun_elevation, azimuth as sun_azimuth
from astral.moon import elevation as moon_elevation, azimuth as moon_azimuth
from astral.moon import phase as moon_phase

logger = logging.getLogger("PiSlider.HG")


# ─────────────────────────────────────────────────────────────────────────────
# HGSettings
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class HGSettings:
    enabled: bool = True

    # Location & time
    lat: float = 49.8951
    lon: float = -97.1384
    tz:  str   = "America/Winnipeg"

    start_dt:        Optional[datetime.datetime] = None
    interval_sec:    float = 5.0
    frames:          int   = 1000
    vibration_delay: float = 1.0
    exposure_margin: float = 0.2

    # Camera geometry
    cam_az:  float = 180.0
    cam_alt: float = 0.0
    hfov:    float = 60.0
    vfov:    float = 40.0

    # ── EV targets per phase ─────────────────────────────────────────────────
    ev_day:      float = 13.0
    ev_golden:   float = 10.0
    ev_twilight: float =  6.0
    ev_night:    float =  3.0

    # Kelvin targets per phase
    kelvin_day:      int = 5500
    kelvin_golden:   int = 4200
    kelvin_twilight: int = 4800
    kelvin_night:    int = 3800

    # Per-phase intervals (s)
    interval_day:      float = 5.0
    interval_golden:   float = 7.0
    interval_twilight: float = 10.0
    interval_night:    float = 20.0

    # Aperture & ISO
    aperture_day:      float = 5.6
    aperture_night:    float = 2.8
    iso_min:        int   = 100
    iso_max:        int   = 3200
    iso_max_night:  int   = 6400

    # ISO-invariant camera support (e.g. Sony A7III dual-native ISO).
    # Cameras with a dual-gain architecture have two invariant stages separated
    # by a "native high ISO" transition point. ISO values *between* iso_min and
    # iso_native_high are a dead zone: they have less DR than iso_min AND worse
    # read-noise than iso_native_high — so we skip them entirely.
    # Sony A7III: stage 1 = ISO 100–500, stage 2 = ISO 640–51200.
    # Set to 0 (or None) to use classic 1/3-stop stepping with no jump.
    iso_native_high: int = 640

    # ── Anchor exposure ───────────────────────────────────────────────────────
    # Set once by calibration shot. ALL meter shots use these exact settings
    # so every measurement is directly comparable — no compensation math needed.
    anchor_shutter_s: Optional[float] = None
    anchor_iso:       Optional[int]   = None
    anchor_ev:        Optional[float] = None

    # Shutter limits
    # IMX477 hardware maximum is ~670s. These are soft caps — the real
    # constraint is the frame interval (shutter can't exceed interval minus
    # vibration_delay + exposure_margin). Set these to match your intended
    # night interval. e.g. 25s interval → set shutter_max_night ~23s.
    # The interval floor in _compute_params auto-extends the interval if
    # needed, so setting this higher than your interval just means the
    # interval stretches to accommodate the exposure.
    shutter_max_night:     float = 25.0   # set to ~interval_night - 2s
    shutter_max_twilight:  float = 20.0
    night_prefer_low_iso:  bool  = True
    continuous_shutter:    bool  = False

    # ── Histogram targets (0–255 luminance in meter JPEG) ────────────────────
    # User-facing controls for the look of the timelapse.
    # The system steers toward these using clean anchor-exposure meter shots.

    # Highlight protection: if this fraction of pixels exceeds clip_level,
    # pull exposure down regardless of trend. Prevents blown daylight skies.
    highlight_clip_level:    int   = 245    # pixel value = blown
    highlight_clip_limit:    float = 0.005  # 0.5% blown pixels = pull down

    # Midtone target: where we want the P50 of the histogram.
    # Night: push brighter to gather more light and show stars.
    # Lightroom/LRTimelapse normalises brightness in post anyway.
    midtone_target_day:   int   = 110
    midtone_target_night: int   = 80
    midtone_percentile:   float = 0.50  # steer this percentile toward target

    # Shadow floor: if too many pixels are crushed to black, boost exposure.
    shadow_floor_level: int   = 18     # below this = crushed black
    shadow_floor_limit: float = 0.40   # >40% crushed → boost needed

    # ── Per-phase agility (max stops/frame the output is allowed to change) ──
    # This is the primary "butter" control. Low = smooth. High = responsive.
    # Transitions (golden, twilight) get more agility. Stable phases get less.
    agility_day:      float = 0.008   # stable sun — almost no movement
    agility_golden:   float = 0.035   # golden/sunset — fast ramp allowed
    agility_twilight: float = 0.030   # still ramping, slightly slower
    agility_night:    float = 0.020   # stable darkness — tiny corrections only

    # Extra multiplier near the horizon (±15°) for moon rise/set events.
    horizon_agility_boost: float = 1.8

    # ── Tracker tuning ────────────────────────────────────────────────────────
    adaptive_weight:      float = 0.75
    ev_max_delta_flat:    float = 0.04
    ev_max_delta_fast:    float = 0.35
    kelvin_max_delta:     int   = 60
    anomaly_threshold_ev: float = 1.5
    tracker_window:       int   = 20
    tracker_warmup:       int   = 5
    slope_ma_window:      int   = 12   # frames to average for slope_ma
    # Exponential recency decay for regression weights.
    # Newest frame = 1.0; each step back is multiplied by this factor.
    # At 0.92 with a 20-frame window: oldest sample carries 0.92^19 ≈ 0.20 weight.
    # This lets old data fade out naturally at phase transitions — no flush needed.
    # Range: 0.80 (aggressive fade) → 1.0 (flat / equal weights).
    tracker_recency_decay: float = 0.92
    # How strongly ev_smooth drifts toward the blended drift target per frame.
    # At the default 0.030 a 1-stop gap contributes 0.030 stops of pull before
    # hitting the ±max_step/2 cap.  Raise for faster reality-anchoring;
    # lower for more inertia during cloudy / variable conditions.
    drift_pull_strength:  float = 0.030

    # ── Celestial disc tuning ─────────────────────────────────────────────────
    disc_lookahead_min:   float = 10.0
    sun_weight:           float = 1.0
    moon_weight:          float = 0.4
    moon_phase_weight:    float = 1.0
    moonlight_ev_max:     float = 4.5


# ─────────────────────────────────────────────────────────────────────────────
# MeterShot — result of a clean anchor-exposure meter capture
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class MeterShot:
    """
    Luminance histogram extracted from a JPEG captured at fixed anchor
    settings (anchor_shutter_s, anchor_iso). Because settings never change,
    every MeterShot is directly comparable — no compensation math needed.

    Fields that matter to the tracker:
      meter_ev          — EV computed from P50 luminance (the reference signal)
      highlight_fraction — fraction of pixels at or above clip_level
      shadow_fraction   — fraction of pixels at or below floor_level
      midtone_p50       — raw P50 luminance (0–255)
      condition         — 'clear' | 'hazy' | 'overcast' from histogram variance
      kelvin            — colour temperature (day only; astro ramp at night)
    """
    timestamp:          float
    meter_ev:           float   # EV at anchor exposure — scene luminance signal
    midtone_p50:        int     # P50 luminance (0–255)
    highlight_fraction: float   # fraction of pixels >= clip_level
    shadow_fraction:    float   # fraction of pixels <= floor_level
    hist_std:           float   # stddev of luminance — sky variance / cloud indicator
    kelvin:             float
    condition:          str     # 'clear' | 'hazy' | 'overcast'
    is_anomaly:         bool  = False
    weight:             float = 1.0


# ─────────────────────────────────────────────────────────────────────────────
# CaptureMeasurement — kept for push_capture_ev compatibility
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class CaptureMeasurement:
    timestamp:    float
    pixel_ev:     float
    kelvin:       float
    sky_fraction: float
    condition:    str
    is_anomaly:   bool  = False
    weight:       float = 1.0


# ─────────────────────────────────────────────────────────────────────────────
# SkyMeasurement  (preview / day use)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class SkyMeasurement:
    timestamp:    float
    ev:           float
    rg_ratio:     float
    bg_ratio:     float
    lum_mean:     float
    sky_fraction: float
    condition:    str
    source:       str   = 'preview'
    is_anomaly:   bool  = False
    weight:       float = 1.0


# ─────────────────────────────────────────────────────────────────────────────
# SkyAnalyser
# ─────────────────────────────────────────────────────────────────────────────

class SkyAnalyser:
    _SKY_HSV_RANGES = [
        ((90,  20,  60), (130, 255, 255)),
        ((0,   0,  130), (180,  55, 230)),
        ((5,   40,  80), ( 35, 255, 255)),
        ((95,  10,  40), (140,  80, 200)),
    ]

    def __init__(self):
        self._prev_mask: Optional[np.ndarray] = None

    def analyse(
        self,
        frame_rgb: np.ndarray,
        cam_alt:   float = 0.0,
        sun_az:    float = 0.0,
        sun_alt:   float = 0.0,
        cam_az:    float = 0.0,
        hfov:      float = 60.0,
        vfov:      float = 40.0,
        moon_az:   float = 0.0,
        moon_alt:  float = 0.0,
        camera_ev: Optional[float] = None,
    ) -> Optional[SkyMeasurement]:
        if not _HAS_CV2 or frame_rgb is None:
            return None
        h, w = frame_rgb.shape[:2]
        if cam_alt < -25.0:
            return None

        hsv  = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
        mask = np.zeros((h, w), dtype=np.uint8)
        for lo, hi in self._SKY_HSV_RANGES:
            mask |= cv2.inRange(hsv,
                                np.array(lo, dtype=np.uint8),
                                np.array(hi, dtype=np.uint8))

        sky_top = max(1, int(h * 0.65))
        pos_mask = np.zeros((h, w), dtype=np.uint8)
        pos_mask[:sky_top, :] = 255
        mask &= pos_mask

        if self._prev_mask is not None and self._prev_mask.shape == mask.shape:
            mask = cv2.addWeighted(mask, 0.7, self._prev_mask, 0.3, 0).astype(np.uint8)
        self._prev_mask = mask.copy()

        sky_pixels   = frame_rgb[mask > 127]
        sky_fraction = float(np.sum(mask > 127)) / (h * w)
        if sky_fraction < 0.02 or len(sky_pixels) < 50:
            return None

        lum_arr = (0.2126 * sky_pixels[:, 0].astype(float) +
                   0.7152 * sky_pixels[:, 1].astype(float) +
                   0.0722 * sky_pixels[:, 2].astype(float))
        thresh   = np.percentile(lum_arr, 90)
        good     = lum_arr[lum_arr <= thresh]
        if len(good) == 0:
            good = lum_arr
        lum_mean = float(np.mean(good))
        lum_safe = max(lum_mean, 1.0)
        ev = math.log2((lum_safe / 255.0) ** 2.2 / 0.18) + 12.0

        rg = float(np.mean(sky_pixels[:, 0])) / max(float(np.mean(sky_pixels[:, 1])), 1.0)
        bg = float(np.mean(sky_pixels[:, 2])) / max(float(np.mean(sky_pixels[:, 1])), 1.0)

        hsv_sky  = hsv[mask > 127]
        sat_mean = float(np.mean(hsv_sky[:, 1])) if len(hsv_sky) > 0 else 0
        if sat_mean > 60:
            condition = 'clear'
        elif sat_mean > 25:
            condition = 'hazy'
        else:
            condition = 'overcast'

        weight = 1.0
        if camera_ev is not None and abs(ev - camera_ev) > 2.0:
            weight = 0.3

        return SkyMeasurement(
            timestamp    = time.time(),
            ev           = ev,
            rg_ratio     = rg,
            bg_ratio     = bg,
            lum_mean     = lum_mean,
            sky_fraction = sky_fraction,
            condition    = condition,
            source       = 'preview',
            weight       = weight,
        )


# ─────────────────────────────────────────────────────────────────────────────
# AdaptiveEVTracker
# ─────────────────────────────────────────────────────────────────────────────

class AdaptiveEVTracker:
    """
    Rolling window of meter-shot EV measurements.
    Each measurement comes from a dedicated anchor-exposure capture, so
    the EV values are directly comparable — no compensation math, no
    "infected" samples from exposure changes.

    Weighted linear regression gives slope = rate of EV change (stops/frame).
    slope_ma is the moving average of recent slope estimates — only sustained
    trends affect the output. A single cloudy frame cannot dominate.

    Highlight and shadow fractions from the histogram allow hard-limit
    overrides: if highlights are clipping, pull down regardless of trend.
    """

    def __init__(self, window_size: int = 20, warmup: int = 5,
                 slope_ma_window: int = 12, recency_decay: float = 0.92):
        self.window_size     = window_size
        self.warmup          = warmup
        self.recency_decay   = recency_decay
        self._window: deque  = deque(maxlen=window_size)
        self._last_ev:     Optional[float] = None
        self._last_kelvin: Optional[float] = None
        self.ev_slope:     float = 0.0
        self.kelvin_slope: float = 0.0
        self.r_squared:    float = 0.0
        self.condition:    str   = 'unknown'
        self.n_frames:     int   = 0
        self._slope_history: deque[float] = deque(maxlen=slope_ma_window)
        self.slope_ma:       float = 0.0
        # meas_ev / meas_kelvin: what the SCENE is doing (from regression).
        # _last_ev / _last_kelvin: what we TOLD the camera (output state).
        # These are deliberately separate — output only moves by max_step.
        self.meas_ev:        Optional[float] = None
        self.meas_kelvin:    Optional[float] = None
        # Latest histogram stats (from most recent MeterShot)
        self.highlight_fraction: float = 0.0
        self.shadow_fraction:    float = 0.0
        self.midtone_p50:        int   = 128
        self.hist_std:           float = 0.0
        # Staleness tracking — wall-clock time of most recent push_meter_shot call.
        # Used by _compute_params to fall back to astro-only drift when pixel
        # data is missing (write error, camera busy, USB glitch).
        self._last_meter_time:   float = 0.0

    @property
    def is_warm(self) -> bool:
        return len(self._window) >= self.warmup

    def push_meter_shot(self, m: MeterShot) -> None:
        """
        Primary feedback path. Called after every anchor-exposure meter shot.
        m.meter_ev is already at a fixed reference — no normalization needed.
        """
        self._last_meter_time = m.timestamp   # freshness tracking
        # Store histogram stats for highlight/shadow override logic
        self.highlight_fraction = m.highlight_fraction
        self.shadow_fraction    = m.shadow_fraction
        self.midtone_p50        = m.midtone_p50
        self.hist_std           = m.hist_std

        # Anomaly detection on the clean meter EV stream
        if len(self._window) >= 3:
            recent_evs = [x.meter_ev for x in self._window]
            median_ev  = float(np.median(recent_evs))
            if abs(m.meter_ev - median_ev) > 1.0:
                m.is_anomaly = True
                m.weight     = 0.15
                # Two consecutive anomalies in the same direction = real event
                if len(self._window) >= 2:
                    last = self._window[-1]
                    if (getattr(last, 'is_anomaly', False) and
                            (m.meter_ev - median_ev) * (last.meter_ev - median_ev) > 0):
                        m.weight = 1.0

        self._window.append(m)
        self.n_frames += 1
        self.condition = m.condition
        self._refit_meter()

    def push_capture(self, m: CaptureMeasurement) -> None:
        """Legacy path — converts CaptureMeasurement to minimal MeterShot."""
        ms = MeterShot(
            timestamp          = m.timestamp,
            meter_ev           = m.pixel_ev,
            midtone_p50        = 128,
            highlight_fraction = 0.0,
            shadow_fraction    = 0.0,
            hist_std           = 20.0,
            kelvin             = m.kelvin,
            condition          = m.condition,
            weight             = m.weight,
        )
        self.push_meter_shot(ms)

    def seed(self, ev: float, kelvin: float) -> None:
        self._last_ev     = ev
        self._last_kelvin = kelvin

    def current_ev(self) -> Optional[float]:
        return self._last_ev

    def current_kelvin(self) -> Optional[float]:
        return self._last_kelvin

    def predict_ev(self, seconds_ahead: float = 1.0) -> Optional[float]:
        """Predict EV seconds_ahead from now. ev_slope is stops/second."""
        if not self.is_warm or self._last_ev is None:
            return None
        return self._last_ev + self.ev_slope * seconds_ahead

    def predict_kelvin(self, seconds_ahead: float = 1.0) -> Optional[float]:
        """Predict Kelvin seconds_ahead from now. kelvin_slope is K/second."""
        if not self.is_warm or self._last_kelvin is None:
            return None
        return self._last_kelvin + self.kelvin_slope * seconds_ahead

    def smooth_ev(
        self, ev_target: float,
        max_flat: float = 0.12, max_fast: float = 0.35,
    ) -> float:
        if self._last_ev is None:
            self._last_ev = ev_target
            return ev_target
        diff = ev_target - self._last_ev
        if abs(diff) > 1.5:
            # Emergency recovery — large drift, allow up to max_fast per frame
            max_delta = max_fast
        else:
            # Scale with slope, capped at 0.5 stops/frame
            capped_slope = min(abs(self.ev_slope), 0.5)
            slope_factor = min(1.0, capped_slope * 10.0)
            # Also scale with r_squared: low confidence = tighter limit
            # This kills transient dips (person, bird, cloud edge) that don't
            # produce a sustained trend. r_squared near 0 = noisy/unsustained.
            confidence_factor = max(0.1, self.r_squared)
            max_delta = max_flat * confidence_factor + (max_fast - max_flat) * slope_factor * confidence_factor
            max_delta = max(max_flat * 0.3, max_delta)   # floor: always allow tiny creep
        diff = max(-max_delta, min(max_delta, diff))
        self._last_ev += diff
        return self._last_ev

    def smooth_kelvin(self, kelvin_target: float, max_delta: int = 60) -> int:
        if self._last_kelvin is None:
            self._last_kelvin = float(kelvin_target)
            return int(kelvin_target)
        diff = kelvin_target - self._last_kelvin
        diff = max(-max_delta, min(max_delta, diff))
        self._last_kelvin += diff
        return int(self._last_kelvin)

    def get_status(self) -> Dict[str, Any]:
        meter_age = round(time.time() - self._last_meter_time, 1) if self._last_meter_time > 0 else None
        return {
            "warm":        self.is_warm,
            "n_frames":    self.n_frames,
            "window_used": len(self._window),
            "ev_slope":    round(self.ev_slope, 4),
            "kelvin_slope":round(self.kelvin_slope, 2),
            "r_squared":   round(self.r_squared, 3),
            "condition":   self.condition,
            "last_ev":     round(self._last_ev, 3) if self._last_ev is not None else None,
            "last_kelvin": int(self._last_kelvin) if self._last_kelvin is not None else None,
            "meter_age_s": meter_age,
        }

    def _refit_meter(self) -> None:
        """Refit regression over clean meter_ev measurements."""
        meas = list(self._window)
        if len(meas) < 2:
            self.ev_slope = self.kelvin_slope = self.r_squared = 0.0
            return

        n   = len(meas)
        # Use wall-clock timestamps as x-axis so slope is in stops/second.
        # frame_index was never incremented during normal operation, making
        # the denominator of the regression always zero and slope always 0.
        xs  = np.array([m.timestamp for m in meas], dtype=float)
        xs -= xs[0]
        evs = np.array([m.meter_ev for m in meas])
        ws  = np.array([m.weight   for m in meas])
        ks  = np.array([m.kelvin   for m in meas])

        # Exponential recency weighting: newest frame = 1.0, each step back
        # multiplied by recency_decay. Combined with per-sample anomaly weights
        # so a stale outlier gets both the age penalty and the anomaly penalty.
        # This lets old data fade out naturally at phase transitions without
        # any need for a window flush.
        # Shape: [decay^(n-1), decay^(n-2), ..., decay^1, decay^0]
        recency = self.recency_decay ** np.arange(n - 1, -1, -1)
        ws = ws * recency

        self.ev_slope     = _weighted_slope(xs, evs, ws)
        self.kelvin_slope = _weighted_slope(xs, ks,  ws)

        # Anchor the fitted line on the WEIGHTED CENTROID, not on evs[0].
        # evs[0] is the oldest sample and therefore carries the *smallest*
        # recency weight (decay^(n-1) ≈ 0.20 at n=20), so using it as the
        # intercept let the least trustworthy point define the line's
        # position: a single stale outlier shifted meas_ev by >1.2 stops and
        # collapsed r² to 0, which in turn starved pixel_w via r2_mod.
        x_bar = float(np.average(xs, weights=ws))
        y_bar = float(np.average(evs, weights=ws))

        if len(xs) >= 3:
            ev_pred = y_bar + self.ev_slope * (xs - x_bar)
            ss_res  = float(np.sum(ws * (evs - ev_pred) ** 2))
            ss_tot  = float(np.sum(ws * (evs - y_bar) ** 2))
            self.r_squared = max(0.0, min(1.0, 1.0 - ss_res / max(ss_tot, 1e-9)))
        else:
            self.r_squared = 0.3

        # meas_ev = regression fitted value at latest frame.
        # This is the MEASUREMENT trend — what the scene is doing.
        # It is NOT written to _last_ev because _last_ev is the OUTPUT
        # (what we last told the camera). The output only moves by max_step
        # in _compute_params. Separating these two is the key to smooth output.
        if len(meas) >= 2:
            self.meas_ev = float(y_bar + self.ev_slope * (xs[-1] - x_bar))
        else:
            self.meas_ev = float(evs[-1])

        # Slope moving average — the stable trend rate.
        # Transient blips produce one out-of-family slope estimate that gets
        # averaged away across slope_ma_window frames. Only a sustained real
        # trend shifts slope_ma enough to drive meaningful output movement.
        self._slope_history.append(self.ev_slope)
        self.slope_ma = float(np.mean(self._slope_history))

        n    = min(3, len(meas))
        rw   = ws[-n:]
        wsum = float(np.sum(rw))
        if wsum > 0:
            self.meas_kelvin = float(np.average(ks[-n:], weights=rw))


# ─────────────────────────────────────────────────────────────────────────────
# HolyGrailController
# ─────────────────────────────────────────────────────────────────────────────

class HolyGrailController:

    def __init__(self, settings: Optional[HGSettings] = None):
        self.settings  = settings or HGSettings()
        self._tzinfo   = self._make_tzinfo(self.settings.tz)
        self._location = self._make_location()
        self._analyser = SkyAnalyser()
        self._tracker  = AdaptiveEVTracker(
            window_size    = self.settings.tracker_window,
            warmup         = self.settings.tracker_warmup,
            slope_ma_window= self.settings.slope_ma_window,
            recency_decay  = self.settings.tracker_recency_decay,
        )
        self._last_phase:    str            = 'unknown'
        self._last_aperture: Optional[float] = None   # for per-frame rate limiting
        self._prev_iso:      Optional[int]   = None   # dead-zone ISO hysteresis

    # ── Configuration ─────────────────────────────────────────────────────────

    def set_settings(self, settings) -> None:
        import dataclasses as _dc
        if isinstance(settings, dict):
            valid = {f.name for f in _dc.fields(HGSettings)}
            clean = {k: v for k, v in settings.items() if k in valid}
            self.settings = HGSettings(**clean)
        else:
            self.settings = settings
        self._tzinfo   = self._make_tzinfo(self.settings.tz)
        self._location = self._make_location()
        self._tracker  = AdaptiveEVTracker(
            window_size    = self.settings.tracker_window,
            warmup         = self.settings.tracker_warmup,
            slope_ma_window= self.settings.slope_ma_window,
            recency_decay  = self.settings.tracker_recency_decay,
        )
        self._last_phase    = 'unknown'
        self._last_aperture = None

    def get_settings_dict(self) -> Dict[str, Any]:
        import dataclasses as _dc
        return _dc.asdict(self.settings)

    # ── Capture feedback (primary closed-loop input) ──────────────────────────

    def push_capture_ev(
        self,
        pixel_ev:     float,
        kelvin:       float,
        sky_fraction: float = 0.5,
        condition:    str   = 'unknown',
    ) -> None:
        """
        Push EV measured from a real saved DNG/thumbnail.
        Primary closed-loop feedback. Works day and night.

        pixel_ev must use same formula as anchor_ev:
            ev = log2((lum/255)^2.2 / 0.18) + 12
        where lum is mean luminance of non-blown pixels.
        """
        m = CaptureMeasurement(
            timestamp    = time.time(),
            pixel_ev     = pixel_ev,
            kelvin       = kelvin,
            sky_fraction = sky_fraction,
            condition    = condition,
        )
        self._tracker.push_capture(m)
        logger.debug(
            f"HG capture push: ev={pixel_ev:.3f} "
            f"K={kelvin:.0f} slope={self._tracker.ev_slope:.4f} "
            f"R2={self._tracker.r_squared:.2f}"
        )

    # ── Meter shot feedback (primary clean-signal path) ──────────────────────

    def push_meter_shot(
        self,
        jpeg_rgb: np.ndarray,
        sun_alt:  Optional[float] = None,
        camera_ev_offset: float = 0.0,
    ) -> Optional[Dict[str, Any]]:
        """
        PRIMARY feedback path. Call after every dedicated anchor-exposure
        meter JPEG. Because the camera is always at anchor_shutter_s /
        anchor_iso, every call is directly comparable — no math compensation.

        For Sony USB (variable-exposure captures): pass camera_ev_offset =
        log2(shutter_s / anchor_shutter_s) + log2(iso / anchor_iso) so that
        meter_ev is normalized to the anchor-equivalent scene brightness scale.

        jpeg_rgb: uint8 RGB array decoded from the meter JPEG.
        Returns the MeterShot dict for logging, or None on failure.
        """
        s = self.settings
        if jpeg_rgb is None or jpeg_rgb.size == 0:
            return None

        try:
            h, w   = jpeg_rgb.shape[:2]
            lum    = (0.2126 * jpeg_rgb[:, :, 0].astype(float)
                    + 0.7152 * jpeg_rgb[:, :, 1].astype(float)
                    + 0.0722 * jpeg_rgb[:, :, 2].astype(float))
            lum_flat = lum.flatten()

            # ── Histogram stats ──────────────────────────────────────────────
            midtone_p50        = int(np.percentile(lum_flat, s.midtone_percentile * 100))
            highlight_fraction = float(np.mean(lum_flat >= s.highlight_clip_level))
            shadow_fraction    = float(np.mean(lum_flat <= s.shadow_floor_level))
            hist_std           = float(np.std(lum_flat))

            # ── meter_ev from median luminance ───────────────────────────────
            # We use P50 (not mean) so a few bright street-lights at night
            # don't dominate the measurement.
            lum_safe      = max(float(midtone_p50), 1.0)
            raw_meter_ev  = math.log2(max((lum_safe / 255.0) ** 2.2, 1e-9) / 0.18) + 12.0
            # Normalize to anchor-scale for cameras that don't use fixed meter shots
            meter_ev      = raw_meter_ev - camera_ev_offset

            # ── Condition from sky region (upper half), not full frame ───────
            # Full-image std is unreliable: dark foreground + bright sky gives
            # high contrast regardless of cloud cover.  The upper half of the
            # JPEG is almost always sky, giving a signal that's actually about
            # atmospheric conditions.
            lum_sky   = lum[:h // 2, :].flatten()
            sky_std   = float(np.std(lum_sky)) if lum_sky.size > 0 else hist_std
            condition = ('clear' if sky_std > 45 else
                         'hazy'  if sky_std > 18 else 'overcast')

            # ── Kelvin — measured through civil twilight, astro below -12° ───
            # The original hard cutoff at sun_alt=0 left the entire blue hour
            # (0° to -12°) on an astro estimate.  RGB ratios are still valid
            # signal during civil/nautical twilight; we blend them smoothly
            # toward the astro prior as the sky darkens below the horizon.
            if sun_alt is None:
                from astral.sun import elevation as _se
                sun_alt = _se(self._location.observer,
                              datetime.datetime.now(self._tzinfo))

            if sun_alt >= -12.0:
                r_mean     = float(np.mean(jpeg_rgb[:, :, 0]))
                g_mean     = max(float(np.mean(jpeg_rgb[:, :, 1])), 1.0)
                b_mean     = float(np.mean(jpeg_rgb[:, :, 2]))
                rg         = r_mean / g_mean
                bg         = b_mean / g_mean
                kelvin_rgb  = float(max(2500, min(10000,
                    5500 - (rg - 1.0) * 2000 + (bg - 1.0) * 1600)))
                kelvin_astro = float(self._kelvin_for_phase(sun_alt))
                # trust=1.0 at sun_alt=0°, trust=0.0 at sun_alt=-12°
                trust  = max(0.0, min(1.0, (sun_alt + 12.0) / 12.0))
                kelvin = trust * kelvin_rgb + (1.0 - trust) * kelvin_astro
            else:
                kelvin = float(self._kelvin_for_phase(sun_alt))

            ms = MeterShot(
                timestamp          = time.time(),
                meter_ev           = meter_ev,
                midtone_p50        = midtone_p50,
                highlight_fraction = highlight_fraction,
                shadow_fraction    = shadow_fraction,
                hist_std           = hist_std,
                kelvin             = kelvin,
                condition          = condition,
            )
            self._tracker.push_meter_shot(ms)
            logger.debug(
                f"MeterShot ev={meter_ev:.3f} "
                f"p50={midtone_p50} hl={highlight_fraction:.3f} "
                f"shadow={shadow_fraction:.3f} cond={condition} K={kelvin:.0f}"
            )
            return {
                "meter_ev":           round(meter_ev, 3),     # normalized to anchor scale
                "raw_meter_ev":       round(raw_meter_ev, 3), # actual pixel EV of the capture
                "midtone_p50":        midtone_p50,
                "highlight_fraction": round(highlight_fraction, 4),
                "shadow_fraction":    round(shadow_fraction, 4),
                "hist_std":           round(hist_std, 1),
                "condition":          condition,
                "kelvin":             int(kelvin),
            }
        except Exception as e:
            logger.warning(f"push_meter_shot failed: {e}")
            return None

    # ── Preview feedback (daylight only, supplementary) ───────────────────────

    def push_preview_frame(
        self,
        frame_rgb:  np.ndarray,
        camera_ev:  Optional[float] = None,
    ) -> Optional[SkyMeasurement]:
        """Preview metering — day only (sun > -6 deg). Night = None immediately."""
        s   = self.settings
        now = datetime.datetime.now(self._tzinfo)
        obs = self._location.observer
        sun_alt = sun_elevation(obs, now)
        if sun_alt < -6.0:
            return None

        sun_az   = sun_azimuth(obs, now)
        moon_alt = moon_elevation(obs, now)
        moon_az  = moon_azimuth(obs, now)

        m = self._analyser.analyse(
            frame_rgb,
            cam_alt=s.cam_alt, sun_az=sun_az, sun_alt=sun_alt,
            cam_az=s.cam_az, hfov=s.hfov, vfov=s.vfov,
            moon_az=moon_az, moon_alt=moon_alt, camera_ev=camera_ev,
        )
        if m is not None:
            self.push_capture_ev(
                pixel_ev     = m.ev,
                kelvin       = float(_rg_bg_to_kelvin(m.rg_ratio, m.bg_ratio, m.lum_mean)),
                sky_fraction = m.sky_fraction,
                condition    = m.condition,
            )
            # Preview is less reliable than a real DNG — reduce weight
            if self._tracker._window:
                self._tracker._window[-1].weight *= 0.4
        return m

    def seed_from_calibration(self, ev: float, kelvin: int) -> None:
        self._tracker.seed(ev, float(kelvin))
        logger.info(f"HG tracker seeded: EV={ev:.2f} K={kelvin}")

    # ── Main API ──────────────────────────────────────────────────────────────

    def get_next_shot_parameters(
        self,
        now: Optional[datetime.datetime] = None,
    ) -> Dict[str, Any]:
        if not self.settings.enabled:
            return {
                "mode": "manual", "iso": self.settings.iso_min,
                "shutter": "1/125", "shutter_s": 1/125,
                "kelvin": self.settings.kelvin_day,
                "interval": self.settings.interval_sec,
            }
        if now is None:
            now = datetime.datetime.now(self._tzinfo)
        else:
            now = self._ensure_tz(now)
        return self._compute_params(now)

    # ── Core per-frame computation ────────────────────────────────────────────

    def _compute_params(self, now: datetime.datetime) -> Dict[str, Any]:
        s   = self.settings
        obs = self._location.observer

        # 1. Astronomical model
        sun_alt      = sun_elevation(obs, now)
        sun_az       = sun_azimuth(obs, now)
        moon_alt     = moon_elevation(obs, now)
        moon_az      = moon_azimuth(obs, now)
        moon_ph_days = moon_phase(now)
        moon_ph      = max(0.0, min(1.0, moon_ph_days / 29.53))

        phase         = _phase_for_alt(sun_alt)
        astro_ev      = self._ev_for_phase(sun_alt)
        astro_kelvin  = self._kelvin_for_phase(sun_alt)
        interval_base = self._interval_for_phase(sun_alt)

        # Log phase transitions for diagnostics. No flush needed — the tracker
        # window is capped at 20 frames (~6 minutes of data at night intervals),
        # so stale data from a previous phase naturally ages out frame by frame.
        # Flushing forces a cold start that causes exposure jumps at boundaries;
        # the rolling regression handles gradual transitions correctly on its own.
        if phase != self._last_phase and self._last_phase != 'unknown':
            logger.info(f"HG phase transition {self._last_phase}→{phase}")
        self._last_phase = phase

        # 2. Ambient moonlight EV contribution
        # Full moon at zenith raises scene brightness by ~4–5 stops vs new moon.
        # moonlight_ev_max default 4.5 — applied in full (no damping multiplier)
        # so "Dark Sky" presets automatically shift toward moonlit exposures as
        # the moon rises during an overnight timelapse.
        moonlight_ev = 0.0
        if phase == 'night' and moon_alt > 0.0:
            # Raised-cosine (Hann) formula: matches the actual illuminated-disc
            # fraction.  0.5*(1-cos(2π·ph)) = 0 at new moon (ph=0 or 1),
            # = 1.0 at full moon (ph=0.5).  The old sin^0.5 approximation
            # was too flat near full moon, overstating gibbous-moon brightness.
            phase_factor = 0.5 * (1.0 - math.cos(2.0 * math.pi * moon_ph))
            alt_factor   = math.sin(math.radians(max(0.0, moon_alt)))
            moonlight_ev = s.moonlight_ev_max * phase_factor * alt_factor
            if moonlight_ev > 0.3:
                astro_ev += moonlight_ev

        # 3. Disc-in-frame geometric offset
        disc_ev_offset = self._disc_ev_offset(
            sun_az, sun_alt, moon_az, moon_alt, moon_ph
        )

        # 4. Look-ahead anticipatory ramp
        disc_entry = self.next_disc_entry()
        disc_anticipation_ev = 0.0
        if disc_entry:
            for body, sign in [("sun", -s.sun_weight),
                                ("moon", -s.moon_weight * moon_ph)]:
                if body in disc_entry:
                    mins_away = disc_entry[body]["minutes"]
                    if mins_away < s.disc_lookahead_min:
                        ramp = _smootherstep(1.0 - mins_away / s.disc_lookahead_min)
                        disc_anticipation_ev += sign * ramp

        total_astro_ev = astro_ev + disc_ev_offset + disc_anticipation_ev

        # 5. Dynamic blend weights
        pixel_w, astro_w = self._blend_weight(sun_alt, moon_alt, moon_ph)

        # 6. Tracker predictions — pass interval_base so predict_ev/kelvin
        # return the expected change over one frame (slope is stops/second).
        tracker_ev     = self._tracker.predict_ev(seconds_ahead=interval_base)
        tracker_kelvin = self._tracker.predict_kelvin(seconds_ahead=interval_base)

        # 7. Blend
        if tracker_ev is not None:
            blended_ev = total_astro_ev * astro_w + tracker_ev * pixel_w
            # Kelvin: only blend pixel Kelvin during daylight (sun > 0).
            if sun_alt >= 0 and tracker_kelvin is not None:
                kelvin_pixel_w = pixel_w * min(1.0, sun_alt / 10.0)
                blended_kelvin = astro_kelvin * (1.0 - kelvin_pixel_w) + tracker_kelvin * kelvin_pixel_w
            else:
                blended_kelvin = float(astro_kelvin)
        else:
            blended_ev     = total_astro_ev
            blended_kelvin = float(astro_kelvin)
            pixel_w        = 0.0
            astro_w        = 1.0

        # ── 7b. Phase-variable agility ─────────────────────────────────────────
        #
        # max_step is the maximum EV change per frame. It is phase-dependent:
        # large during golden/twilight (fast ramp needed), small at stable day
        # or deep night (butter smooth). Additionally scaled up near the
        # horizon (±15°) for moon rise/set and civil-twilight events.
        #
        # This is the primary "butter" control. The slope-driven output is then
        # clamped to max_step, so no single frame can make a large jump.
        # Interpolate agility across the phase pair, the way ev/kelvin/interval
        # already do.  Taking it from the hard _phase_for_alt step meant that at
        # sun_alt = -6 exactly — the start of nautical twilight, the fastest
        # light change of the whole day — agility dropped from agility_twilight
        # to agility_night ("stable darkness — tiny corrections only") while the
        # interval simultaneously doubled.
        _ag = {'day':      s.agility_day,
               'golden':   s.agility_golden,
               'twilight': s.agility_twilight,
               'night':    s.agility_night}
        _p0, _p1, _tt = _phase_pair(sun_alt)
        _a0 = _ag.get(_p0, s.agility_golden)
        _a1 = _ag.get(_p1, s.agility_golden)
        phase_agility = _a0 if _p0 == _p1 else _a0 + (_a1 - _a0) * _tt

        # Horizon boost: near sunrise/sunset OR moonrise/moonset, allow more
        # agility so we don't lag behind rapid lighting changes.
        # Moon boost is weighted by phase — full moon crossing the horizon
        # causes a much larger light swing than a crescent.
        sun_horizon_factor  = max(0.0, min(1.0, (15.0 - abs(sun_alt))  / 15.0))
        moon_horizon_factor = max(0.0, min(1.0, (15.0 - abs(moon_alt)) / 15.0)) * moon_ph
        horizon_factor = max(sun_horizon_factor, moon_horizon_factor)
        max_step = phase_agility * (1.0 + (s.horizon_agility_boost - 1.0) * horizon_factor)

        # ── 8. EV output path ─────────────────────────────────────────────────
        anchor_set   = (s.anchor_ev is not None and
                        s.anchor_shutter_s is not None and
                        s.anchor_iso is not None)
        tracker_warm = self._tracker.is_warm
        last_ev      = self._tracker._last_ev

        highlight_override = False
        shadow_override    = False
        drift_gap          = 0.0   # brightness error (stops, pixel-EV domain)

        if anchor_set and not tracker_warm:
            # Cold start — no regression yet (either very first frames of the
            # sequence, or just after a night-boundary tracker flush).
            if self._tracker._last_ev is not None:
                # We already have a tracked position (e.g. just after a flush):
                # hold there rather than snapping back to anchor_ev, which may
                # be many stops stale by the time of a night→twilight transition.
                ev_smooth = self._tracker._last_ev
            else:
                # Truly cold (very first frames of the sequence):
                # anchor_ev is the best reference we have.
                ev_smooth = s.anchor_ev
                self._tracker._last_ev = s.anchor_ev

        elif tracker_warm and last_ev is not None:
            # ── Slope-driven output (primary path) ────────────────────────────
            #
            # KEY DESIGN: slope_ma is the rate of change of the SCENE
            # (from meter shot measurements). We apply a fraction of it
            # each frame, clamped to max_step. The output (ev_smooth) is
            # a moving average of previous outputs, not a jump to the
            # current measurement. This is what makes it "butter smooth".
            #
            # Think of it like a ship's rudder: the slope_ma tells us
            # which direction the light is going; max_step limits how
            # fast we turn the wheel. We never jerk to match the reading —
            # we steer toward it gradually.

            # ── Staleness guard (#3) ──────────────────────────────────────────
            # If the last meter shot is older than 2× the capture interval,
            # the pixel signal is stale (camera paused, network drop, etc.).
            # Force pixel_w→0 so we fall back to astro-only drift instead of
            # extrapolating a possibly very old slope measurement further.
            meter_age_s = (time.time() - self._tracker._last_meter_time
                           if self._tracker._last_meter_time > 0 else None)
            if meter_age_s is not None and meter_age_s > 2.0 * interval_base:
                pixel_w_live = 0.0
                astro_w_live = 1.0
            else:
                pixel_w_live = pixel_w
                astro_w_live = astro_w

            # ── Brightness error (proportional term) ──────────────────────────
            # drift_gap is the number of stops ev_smooth must move to land the
            # capture on its midtone setpoint.  Both terms are pixel EV (log2 of
            # image luminance), so the difference is scale-consistent — unlike
            # total_astro_ev, which is a camera-EV constant and was what pinned
            # the output at ev_night and produced black night frames.
            #
            # The two output modes measure different things and need different
            # algebra:
            #
            #  * No anchor: meter shots are the captures themselves with
            #    camera_ev_offset = 0 (app.py:2301), so meas_ev IS the delivered
            #    image brightness.  This is a true closed loop, and since one
            #    stop of ev_smooth changes the delivered brightness by one stop
            #    the required move is just the error itself.
            #
            #  * Anchor set: every meter shot is either taken at the fixed
            #    anchor exposure (app.py:3176) or normalized back to it by
            #    camera_ev_offset (app.py:2310).  meas_ev is therefore SCENE
            #    brightness on the anchor scale and does not respond to what we
            #    commanded — open loop.  Solve it directly instead:
            #        delivered = meas_ev - (ev_smooth - anchor_ev) = target
            #    so the command we want is anchor_ev + meas_ev - target.
            meas_ev = self._tracker.meas_ev
            if meas_ev is not None and pixel_w_live > 0.0:
                _target_px = self._midtone_target_ev(sun_alt)
                if anchor_set:
                    drift_gap = (s.anchor_ev + meas_ev - _target_px) - last_ev
                else:
                    drift_gap = meas_ev - _target_px
            else:
                drift_gap = 0.0

            # ── Error-adaptive agility ────────────────────────────────────────
            # The fixed per-phase agility cannot follow nautical twilight.
            # Measured at Winnipeg in September: between -6 and -12 deg the sky
            # falls ~0.081 stops/frame, while agility_night (0.020) with the
            # horizon boost allows ~0.026 — so the loop fell behind by ~8 stops
            # and the rate limit then held it there.  Scaling the allowance by
            # the size of the brightness error fixes the lag without touching
            # steady-state smoothness: once converged the boost is ~1.0.
            err_boost    = 1.0 + min(4.0, abs(drift_gap) / 0.5)
            max_step_eff = max_step * err_boost

            # Apply staleness to slope step as well.
            # NOTE: slope_ma is the rate of change of the brightness ERROR, so
            # it is the derivative term of the loop. It is deliberately NOT
            # scaled by pixel_w (that weight expresses trust in the absolute
            # level, not in the rate); scaling it there throttled the ramp to
            # 15% of the required rate at night.
            slope_gate = 0.0 if pixel_w_live <= 0.0 else 1.0
            slope_step = max(-max_step_eff, min(max_step_eff,
                             self._tracker.slope_ma * interval_base * slope_gate))

            # ── Drift pull (#4 + remove ×10 bug) ─────────────────────────────
            # Drift pull steers ev_smooth toward reality so it stays anchored
            # even during flat/stable scenes. Capped at max_step/2 so a 1-stop
            # gap closes in ~20 frames at golden without causing visible jumps.
            #
            # Drift target (#4): instead of pulling toward raw meas_ev, pull
            # toward a blend of meas_ev and total_astro_ev weighted by the
            # same pixel_w/astro_w blend weights. During deep night (astro_w≈1)
            # the target is almost entirely the astro model, which is stable.
            # During day/golden (pixel_w≈1) it's mostly the measured scene EV.
            # This prevents over-correction when pixel measurements are noisy.
            #
            # IMPORTANT: drift_pull is intentionally exempt from the highlight/
            # shadow brake below. If the brake reduced drift_pull, ev_smooth
            # could get stuck multiple stops above reality during golden hour
            # (western horizon glow triggers the brake throughout the whole
            # sunset, leaving the camera 4+ stops underexposed by twilight).
            if drift_gap != 0.0:
                # ── Emergency recovery (#1) ───────────────────────────────────
                # A brightness error over 1.5 stops (cloud bank, missed capture
                # burst, a phase ramp we fell behind on) widens the pull cap so
                # we recover in ~10 frames instead of staying mis-exposed.
                emergency = abs(drift_gap) > 1.5
                pull_cap = max_step_eff * (1.5 if emergency else 0.5)
                if emergency:
                    logger.info(
                        f"HG emergency recovery: meas_ev={meas_ev:.2f} "
                        f"target={self._midtone_target_ev(sun_alt):.2f} "
                        f"gap={drift_gap:+.2f}")

                # NOTE: drift_pull_strength is used directly (no ×10 multiplier).
                # The historical bug applied ×10 here, making a setting of 0.003
                # behave as 0.030. The default was corrected to 0.030 so existing
                # behaviour is preserved while the formula is now correct.
                drift_pull = max(-pull_cap,
                                 min(pull_cap,
                                     drift_gap * s.drift_pull_strength))
            else:
                # No usable measurement (cold or stale): hold position and let
                # the slope term dead-reckon. Pulling toward total_astro_ev here
                # is what produced the black frames — it is a camera-EV constant
                # and cannot be compared against a pixel-EV output.
                drift_pull = 0.0

            # ── Slope-only interim (before braking) ───────────────────────────
            # Compute the slope-driven position first, then apply the brake only
            # to that component. drift_pull is added afterward, unconditionally.
            # This prevents a bright horizon glow from also braking the
            # reality-correction pull, which caused multi-stop lag in real tests.
            ev_slope_only = last_ev + slope_step

            # ── Hard histogram overrides (slope component only) ───────────────
            # These act as BRAKES on the slope movement, not as reversals.
            # They do NOT touch drift_pull — drift_pull always runs at full strength.

            # 1. Highlight protection: brake the DOWNWARD slope when highlights clip.
            #    During golden/twilight a bright horizon glow commonly clips a small
            #    fraction of pixels. We slow the slope descent; we do NOT push it up.
            if self._tracker.highlight_fraction > s.highlight_clip_limit:
                excess = self._tracker.highlight_fraction - s.highlight_clip_limit
                # brake_factor 0..0.75 (was 0.9) — max 75% slowdown so the slope
                # still makes forward progress even when the horizon is very bright.
                brake_factor = min(0.75, excess * 15.0)
                if ev_slope_only < last_ev:           # slope moving down
                    downward = last_ev - ev_slope_only
                    ev_slope_only = last_ev - downward * (1.0 - brake_factor)
                highlight_override = True
                logger.debug(
                    f"HG highlight brake: hl={self._tracker.highlight_fraction:.3f}"
                    f" brake={brake_factor:.2f} slope_pos={ev_slope_only:.3f}")

            # 2. Shadow boost: brake the UPWARD slope when too many pixels are crushed.
            elif (self._tracker.shadow_fraction > s.shadow_floor_limit
                  and phase in ('night', 'twilight')):
                excess = self._tracker.shadow_fraction - s.shadow_floor_limit
                brake_factor = min(0.75, excess * 3.0)
                if ev_slope_only > last_ev:           # slope moving up
                    upward = ev_slope_only - last_ev
                    ev_slope_only = last_ev + upward * (1.0 - brake_factor)
                shadow_override = True
                logger.debug(
                    f"HG shadow brake: shadow={self._tracker.shadow_fraction:.3f}"
                    f" brake={brake_factor:.2f} slope_pos={ev_slope_only:.3f}")

            # Combine: braked slope + unchecked drift pull
            ev_smooth = ev_slope_only + drift_pull

            # Write ev_smooth back as the new output state.
            # CRITICAL: this is what next frame's slope_step builds on.
            # Without this write-back, every frame starts from the same
            # _last_ev and the smoothing does nothing.
            self._tracker._last_ev = ev_smooth

        else:
            # Fully cold and no anchor — follow astro model with rate limiting.
            #
            # NIGHT COLD-START SPECIAL CASE:
            # ev_night = 3.0 (and ev_twilight = 6.0) are calibrated for the
            # anchor-delta system (pixel EV scale). In the no-anchor fallback
            # the formula is camera EV: t = N²/2^(ev+log2(iso/100)).
            # ev=3.0 gives only 1s at ISO100 f/2.8 — far too short for night.
            #
            # Correct fix: compute the EV that produces shutter_max_night at
            # iso_max_night directly from hardware limits. This is the darkest
            # achievable exposure, which is the right starting point for an
            # overcast or fully dark night. The anti-windup ceiling will hold
            # it there; the tracker corrects down if the scene is brighter.
            _cold_target = total_astro_ev
            if phase in ('night', 'twilight'):
                # ev_night/ev_twilight (3.0 / 6.0) are calibrated for the anchor
                # pixel-EV path. In the no-anchor fallback the formula is camera
                # EV, and those values produce <2s at ISO100 — far too short.
                # Compute the EV that fills the shutter to the phase maximum so
                # the system starts dark and the tracker/anti-windup correct up.
                _ap_n  = _snap_1_3_aperture(s.aperture_night)
                if phase == 'night':
                    _t_max   = s.shutter_max_night
                    _iso_max = s.iso_max_night
                else:  # twilight
                    _t_max   = s.shutter_max_twilight
                    _iso_max = s.iso_max_night   # twilight also uses night ISO cap
                _ev_phase_floor = (math.log2(_ap_n ** 2 / _t_max)
                                   - math.log2(_iso_max / 100.0))
                # Take the darker (lower EV = longer exposure) of astro vs floor.
                _cold_target = min(total_astro_ev, _ev_phase_floor)
                logger.debug(
                    f"HG {phase} cold-start: ev_floor={_ev_phase_floor:.2f} "
                    f"astro={total_astro_ev:.2f} cold_target={_cold_target:.2f}")

            ev_smooth = self._tracker.smooth_ev(
                _cold_target,
                max_flat = s.ev_max_delta_flat,
                max_fast = s.ev_max_delta_fast,
            )

        # ── Kelvin output ─────────────────────────────────────────────────────
        k_min = {'day': 4000, 'golden': 3800, 'twilight': 3500, 'night': 3000}.get(phase, 3500)
        k_max = {'day': 7500, 'golden': 6500, 'twilight': 5500, 'night': 5000}.get(phase, 6000)
        blended_kelvin = max(k_min, min(k_max, blended_kelvin))
        if self._tracker._last_kelvin is not None:
            self._tracker._last_kelvin = max(k_min, min(k_max, self._tracker._last_kelvin))
        kelvin_smooth = self._tracker.smooth_kelvin(blended_kelvin, s.kelvin_max_delta)

        # 9. EV -> ISO + shutter + aperture -> Output 1/3 stop snapped
        aperture_target = self._step_aperture(self._aperture_for_phase(sun_alt))
        iso, shutter_s, aperture, sidecar_error_ev = self._ev_to_exposure(ev_smooth, phase, aperture_target)

        # ── Anti-windup: prevent ev_smooth from drifting past hardware limits ──
        #
        # WHY NOT use sidecar_error_ev directly:
        #   anchor_ev / ev_smooth are on the PIXEL EV scale (log2(lum/0.18)+12).
        #   actual_ev from _ev_to_exposure() is on the CAMERA EV scale
        #   (log2(N²/t) - log2(ISO/100)).  These two scales differ by ~1–2 stops
        #   depending on scene luminance, so `sidecar_error_ev = actual_ev - ev_smooth`
        #   is a cross-scale comparison that fires on every single frame during cold
        #   start — driving a runaway cascade toward maximum shutter + ISO.
        #
        # CORRECT approach: detect whether the camera actually hit a hardware
        # ceiling (both shutter and ISO simultaneously pegged at their limits).
        # That unambiguously means the requested EV is unreachable regardless of
        # scale.  Pull ev_smooth toward the blended/astro prediction to keep it
        # anchored to reality while the tracker catches up.
        _iso_max_now = s.iso_max_night if phase in ('twilight', 'night') else s.iso_max
        # These MUST match the slo/shi used inside _ev_to_exposure() for each phase
        _shi_now     = (s.shutter_max_night     if phase == 'night'
                        else s.shutter_max_twilight if phase == 'twilight'
                        else 1.0)          # day and golden both use shi=1.0
        _slo_now     = (1/500 if phase in ('night', 'twilight')
                        else 1/8000)       # day and golden both use slo=1/8000

        # Upper ceiling: camera can't give more light (ev_smooth drifting too dark)
        _at_upper = (shutter_s >= _shi_now * 0.98
                     and iso   >= int(_iso_max_now * 0.98))
        # Lower floor: camera can't give less light (ev_smooth drifting too bright)
        _at_lower = (shutter_s <= _slo_now * 1.02
                     and iso   <= int(s.iso_min   * 1.02))

        if _at_upper or _at_lower:
            # ── Integrator freeze (clamping anti-windup) ─────────────────────
            # The camera is against a hardware limit. Moving the command
            # further in that direction cannot change a single pixel, so any
            # such movement is pure windup: freeze at last_ev.
            #
            # This branch used to only skip a nudge, leaving drift_pull and the
            # slope term free to keep integrating. Measured in simulate_hg.py:
            # ev_smooth reached -332 stops across one night against a sky
            # darker than the camera can record, and +152 stops across one day
            # with the shutter pegged at 1/8000. Either way the command had to
            # travel all of that back before the exposure moved at all, so the
            # following sunrise/sunset came out black.
            #
            # The direction of travel is the whole test — deliberately not the
            # sign of drift_gap, which is 0 when there is no usable measurement
            # and would let the slope term integrate unchecked. Freezing at
            # last_ev leaves the loop ready to reverse the instant the light
            # comes back.
            if last_ev is not None:
                if _at_upper and ev_smooth < last_ev:      # no more light available
                    ev_smooth = last_ev
                elif _at_lower and ev_smooth > last_ev:    # no less light available
                    ev_smooth = last_ev
                self._tracker._last_ev = ev_smooth
                logger.debug(
                    f"HG anti-windup: {'ceiling' if _at_upper else 'floor'} hit "
                    f"— frozen at ev_smooth={ev_smooth:.3f} (gap={drift_gap:+.2f})")

        # 10. Interval floor + adaptive shortening (#6)
        required = shutter_s + s.vibration_delay + s.exposure_margin

        # Interval adaptation (#6): when the scene is changing rapidly
        # (|slope_ma| large) suggest a shorter capture interval so the tracker
        # gets fresh data before ev_smooth falls too far behind.
        # Target: no more than 0.25 stops of change per frame in the final
        # output.  Clamp to [required, interval_base] — we never go shorter
        # than hardware requires, and never longer than the phase default.
        _abs_slope = abs(self._tracker.slope_ma) if self._tracker.is_warm else 0.0
        if _abs_slope > 1e-4:
            _slope_interval = 0.25 / _abs_slope   # seconds for 0.25-stop change
            interval = max(required, min(interval_base, _slope_interval))
        else:
            interval = max(interval_base, required)

        condition = self._tracker.condition if self._tracker.is_warm else 'prior'

        # Staleness info for telemetry (computed above inside warm path;
        # provide a safe fallback if the cold path ran instead).
        _meter_age_telem = (round(time.time() - self._tracker._last_meter_time, 1)
                            if self._tracker._last_meter_time > 0 else None)

        return {
            "mode":               "holygrail",
            "phase":              phase,
            "condition":          condition,
            "sun_alt":            round(sun_alt, 2),
            "sun_az":             round(sun_az, 2),
            "moon_alt":           round(moon_alt, 2),
            "moon_az":            round(moon_az, 2),
            "moon_phase":         round(moon_ph, 3),
            "moonlight_ev":       round(moonlight_ev, 3),
            "ev_astro":           round(total_astro_ev, 3),
            "ev_disc_offset":     round(disc_ev_offset, 3),
            "ev_anticipation":    round(disc_anticipation_ev, 3),
            "ev_tracker":         round(tracker_ev, 3) if tracker_ev is not None else None,
            "ev_blended":         round(blended_ev, 3),
            "ev_final":           round(ev_smooth, 3),
            "ev_target":          round(total_astro_ev, 3),
            "ev_offset":          round(disc_ev_offset, 3),
            "pixel_weight":       round(pixel_w, 2),
            "astro_weight":       round(astro_w, 2),
            "tracker_warm":       self._tracker.is_warm,
            "ev_slope":           round(self._tracker.ev_slope, 4),
            "slope_ma":           round(self._tracker.slope_ma, 4),
            "r_squared":          round(self._tracker.r_squared, 3),
            "max_step":           round(max_step, 4),
            "highlight_override": highlight_override,
            "shadow_override":    shadow_override,
            "highlight_fraction": round(self._tracker.highlight_fraction, 4),
            "shadow_fraction":    round(self._tracker.shadow_fraction, 4),
            "midtone_p50":        self._tracker.midtone_p50,
            "hist_std":           round(self._tracker.hist_std, 1),
            "iso":                iso,
            "aperture":           aperture,
            "shutter":            _format_shutter(shutter_s),
            "shutter_s":          shutter_s,
            "ev_sidecar_error":   round(sidecar_error_ev, 3),
            "kelvin":             kelvin_smooth,
            "interval":           interval,
            "interval_base":      round(interval_base, 1),
            "meter_age_s":        _meter_age_telem,
            "disc_entry":         disc_entry,
        }

    # ── Dynamic blend weight ──────────────────────────────────────────────────

    def _blend_weight(
        self, sun_alt: float, moon_alt: float, moon_ph: float,
    ) -> Tuple[float, float]:
        if not self._tracker.is_warm:
            return 0.0, 1.0

        # Base pixel weight from phase
        if sun_alt > 10:
            base_pixel = 0.75
        elif sun_alt > 0:
            t = (10 - sun_alt) / 10.0
            base_pixel = 0.75 - t * 0.25
        elif sun_alt > -6:
            t = (-sun_alt) / 6.0
            base_pixel = 0.50 - t * 0.20
        elif sun_alt > -12:
            t = (-6 - sun_alt) / 6.0
            base_pixel = 0.30 - t * 0.15
        else:
            base_pixel = 0.15

        # R² modulation: confident trend -> more pixel weight
        r2     = self._tracker.r_squared
        r2_mod = max(-0.15, min(0.20, (r2 - 0.3) / 0.7 * 0.20))

        # Condition modulation
        cond_mod = {'clear': +0.05, 'overcast': -0.10}.get(
            self._tracker.condition, 0.0)

        # Moon near horizon -> rely more on astro
        moon_mod = 0.0
        if -5 < moon_alt < 15 and moon_ph > 0.3:
            moon_mod = -0.08

        pixel_w = max(0.05, min(0.90, base_pixel + r2_mod + cond_mod + moon_mod))
        return pixel_w, 1.0 - pixel_w

    # ── Celestial forecast ────────────────────────────────────────────────────

    def get_celestial_forecast(
        self,
        minutes_ahead: float = 60.0,
        step_min:      float = 1.0,
    ) -> List[Dict[str, Any]]:
        now = datetime.datetime.now(self._tzinfo)
        obs = self._location.observer
        s   = self.settings
        results = []
        for i in range(int(minutes_ahead / step_min)):
            dt       = now + datetime.timedelta(minutes=i * step_min)
            sun_alt  = sun_elevation(obs, dt);  sun_az  = sun_azimuth(obs, dt)
            moon_alt = moon_elevation(obs, dt); moon_az = moon_azimuth(obs, dt)
            sun_in,  sun_dist  = _is_in_frame(sun_az,  sun_alt,
                                               s.cam_az, s.cam_alt, s.hfov, s.vfov)
            moon_in, moon_dist = _is_in_frame(moon_az, moon_alt,
                                               s.cam_az, s.cam_alt, s.hfov, s.vfov)
            results.append({
                "minutes_from_now": i * step_min,
                "sun_alt":     round(sun_alt, 2),  "sun_az":   round(sun_az, 2),
                "sun_in_frame": sun_in,             "sun_dist": round(sun_dist, 3),
                "moon_alt":    round(moon_alt, 2),  "moon_az":  round(moon_az, 2),
                "moon_in_frame": moon_in,           "moon_dist":round(moon_dist, 3),
            })
        return results

    def next_disc_entry(self) -> Dict[str, Any]:
        forecast = self.get_celestial_forecast(
            minutes_ahead=self.settings.disc_lookahead_min, step_min=0.5)
        result = {}
        for entry in forecast:
            if entry["sun_in_frame"]  and "sun"  not in result:
                result["sun"]  = {"minutes": entry["minutes_from_now"],
                                   "alt": entry["sun_alt"],  "az": entry["sun_az"]}
            if entry["moon_in_frame"] and "moon" not in result:
                result["moon"] = {"minutes": entry["minutes_from_now"],
                                   "alt": entry["moon_alt"], "az": entry["moon_az"]}
            if "sun" in result and "moon" in result:
                break
        return result

    def get_tracker_status(self) -> Dict[str, Any]:
        return self._tracker.get_status()

    # ── Pure astro (no tracker) ───────────────────────────────────────────────

    def _compute_astro(self, dt: datetime.datetime) -> Dict[str, Any]:
        s   = self.settings
        obs = self._location.observer
        sun_alt  = sun_elevation(obs, dt);  sun_az  = sun_azimuth(obs, dt)
        moon_alt = moon_elevation(obs, dt); moon_az = moon_azimuth(obs, dt)
        moon_ph  = max(0.0, min(1.0, moon_phase(dt) / 29.53))
        phase         = _phase_for_alt(sun_alt)
        astro_ev      = self._ev_for_phase(sun_alt)
        astro_kelvin  = self._kelvin_for_phase(sun_alt)
        interval_base = self._interval_for_phase(sun_alt)
        disc_offset   = self._disc_ev_offset(sun_az, sun_alt, moon_az, moon_alt, moon_ph)
        ev_target     = astro_ev + disc_offset
        aperture_target = self._aperture_for_phase(sun_alt)
        iso, shutter_s, aperture, sidecar_error_ev = self._ev_to_exposure(ev_target, phase, aperture_target)
        required = shutter_s + s.vibration_delay + s.exposure_margin
        return {
            "mode": "holygrail", "phase": phase,
            "sun_alt": round(sun_alt, 2), "sun_az": round(sun_az, 2),
            "moon_alt": round(moon_alt, 2), "moon_az": round(moon_az, 2),
            "moon_phase": round(moon_ph, 3),
            "ev_target": round(ev_target, 3), "ev_final": round(ev_target, 3),
            "ev_offset": round(disc_offset, 3), "ev_sidecar_error": round(sidecar_error_ev, 3),
            "iso": iso, "aperture": aperture, "shutter": _format_shutter(shutter_s), "shutter_s": shutter_s,
            "kelvin": astro_kelvin, "interval": max(interval_base, required),
        }

    # ── Aperture smoothing ────────────────────────────────────────────────────

    def _step_aperture(self, target_raw: float) -> float:
        """
        Rate-limit aperture changes: max one 1/3-stop step per frame.

        The S-curve in _aperture_for_phase already makes changes very slow
        (hours across the full transition), but snapping to 1/3-stop values
        can cause oscillation at boundaries if sun altitude jitters.
        Hysteresis: only step when the snapped target has been at the new
        value for at least one frame (i.e. the new snapped value differs from
        the current, and we commit to it immediately — one step per call).
        """
        snapped = _snap_1_3_aperture(target_raw)
        if self._last_aperture is None:
            self._last_aperture = snapped
            return snapped
        if snapped == self._last_aperture:
            return self._last_aperture
        # Limit to one 1/3-stop step per frame toward the target
        cur_stops = math.log2(max(self._last_aperture, 0.1))
        tgt_stops = math.log2(max(snapped, 0.1))
        # Round step to nearest 1/3-stop unit
        raw_step   = tgt_stops - cur_stops
        clamped    = max(-1/3, min(1/3, raw_step))
        new_stops  = cur_stops + round(clamped * 3) / 3
        new_ap     = _snap_1_3_aperture(2.0 ** new_stops)
        self._last_aperture = new_ap
        return new_ap

    # ── Phase blending ────────────────────────────────────────────────────────

    def _ev_for_phase(self, sun_alt: float) -> float:
        s = self.settings
        p0, p1, t = _phase_pair(sun_alt)
        m = {"day": s.ev_day, "golden": s.ev_golden,
             "twilight": s.ev_twilight, "night": s.ev_night}
        e0, e1 = m.get(p0, s.ev_day), m.get(p1, s.ev_day)
        return e0 if p0 == p1 else e0 + (e1 - e0) * t

    def _midtone_target_ev(self, sun_alt: float) -> float:
        """Desired image brightness for this sun altitude, on the pixel-EV scale.

        This is the setpoint the exposure loop steers toward.  midtone_target_day
        / midtone_target_night were declared but never read, which left the loop
        with no brightness reference at all: it steered ev_smooth toward
        total_astro_ev (ev_night = 3.0), a *camera*-EV constant, while its only
        feedback (meas_ev) is *pixel* EV.  At night those scales differ by ~8
        stops, so the output parked on ev_night and every frame came out black.
        """
        s = self.settings
        t = _smootherstep((0.0 - sun_alt) / 12.0)   # 0 at sunset, 1 at -12 deg
        p50 = (s.midtone_target_day
               + (s.midtone_target_night - s.midtone_target_day) * t)
        return _p50_to_pixel_ev(p50)

    def _kelvin_for_phase(self, sun_alt: float) -> int:
        s = self.settings
        p0, p1, t = _phase_pair(sun_alt)
        m = {"day": s.kelvin_day, "golden": s.kelvin_golden,
             "twilight": s.kelvin_twilight, "night": s.kelvin_night}
        k0, k1 = m.get(p0, s.kelvin_day), m.get(p1, s.kelvin_day)
        return int(k0) if p0 == p1 else int(k0 + (k1 - k0) * t)

    def _interval_for_phase(self, sun_alt: float) -> float:
        s = self.settings
        p0, p1, t = _phase_pair(sun_alt)
        m = {"day": s.interval_day, "golden": s.interval_golden,
             "twilight": s.interval_twilight, "night": s.interval_night}
        i0, i1 = m.get(p0, s.interval_day), m.get(p1, s.interval_day)
        return float(i0 if p0 == p1 else i0 + (i1 - i0) * t)

    def _aperture_for_phase(self, sun_alt: float) -> float:
        s = self.settings
        if sun_alt > 10.0:
            return s.aperture_day
        if sun_alt < -12.0:
            return s.aperture_night
        
        # Smoothly interpolate across the entire golden + twilight arc (10 to -12)
        total_span = 22.0
        t = (10.0 - sun_alt) / total_span
        # We can apply _smootherstep to the 0->1 progression to keep it S-curved
        t_smooth = _smootherstep(t)
        
        return s.aperture_day + (s.aperture_night - s.aperture_day) * t_smooth

    def _disc_ev_offset(
        self, sun_az: float, sun_alt: float,
        moon_az: float, moon_alt: float, moon_ph: float,
    ) -> float:
        s = self.settings
        offset = 0.0
        sun_in,  sd = _is_in_frame(sun_az,  sun_alt,  s.cam_az, s.cam_alt, s.hfov, s.vfov)
        moon_in, md = _is_in_frame(moon_az, moon_alt, s.cam_az, s.cam_alt, s.hfov, s.vfov)
        if sun_in:
            offset += -s.sun_weight * (1.0 - sd)
        if moon_in:
            offset += -0.3 * s.moon_weight * moon_ph * s.moon_phase_weight * (1.0 - md)
        return offset

    def _ev_to_exposure(self, ev: float, phase: str, aperture_target: float) -> Tuple[int, float, float, float]:
        s = self.settings
        aperture_snapped = _snap_1_3_aperture(aperture_target)

        def _snap_s(t_val):
            if s.continuous_shutter: return t_val
            return _snap_1_3_shutter(t_val)

        if (s.anchor_shutter_s is not None
                and s.anchor_iso is not None
                and s.anchor_ev is not None):
            # Compensate ev_delta for aperture change since calibration.
            # Calibration was at aperture_day; current aperture may be wider
            # at night. Each stop of aperture opening = 1 stop more light =
            # shutter needs to be shorter by the same amount.
            ap_anchor  = _snap_1_3_aperture(s.aperture_day)
            ap_ev_comp = 2.0 * math.log2(max(ap_anchor, 0.1) /
                                          max(aperture_snapped, 0.1))
            ev_delta = (ev - s.anchor_ev) + ap_ev_comp

            if phase == 'night':
                slo, shi = 1/500, s.shutter_max_night
            elif phase == 'twilight':
                slo, shi = 1/500, s.shutter_max_twilight
            else:
                slo, shi = 1/8000, 1.0
            iso_max = s.iso_max_night if phase in ('twilight', 'night') else s.iso_max
            new_s = _snap_s(s.anchor_shutter_s / (2 ** ev_delta))
            if slo <= new_s <= shi:
                new_iso = _snap_1_3_iso(s.anchor_iso)
                self._prev_iso = new_iso
            else:
                new_s   = _snap_s(max(slo, min(shi, new_s)))
                s_ev    = math.log2(s.anchor_shutter_s / new_s)
                remain  = ev_delta - s_ev
                ideal_iso = s.anchor_iso / (2 ** remain)

                # Best achievable exposure for a candidate ISO, given that the
                # shutter is already against a limit.
                def _err_for(iso_c):
                    want_t = s.anchor_shutter_s / (
                        2 ** (ev_delta - math.log2(s.anchor_iso / max(iso_c, 1e-1))))
                    got_t  = _snap_s(max(slo, min(shi, want_t)))
                    got_ev = (math.log2(s.anchor_shutter_s / max(got_t, 1e-9))
                              + math.log2(s.anchor_iso / max(iso_c, 1e-1)))
                    return abs(got_ev - ev_delta), got_t

                dz_iso = _snap_1_3_iso(max(s.iso_min, min(iso_max, ideal_iso)))
                # Skip the dual-gain dead zone: jump from iso_min to iso_native_high
                # rather than stepping through intermediate ISOs that have worse DR
                # than iso_min AND worse noise than iso_native_high.
                if s.iso_native_high and s.iso_min < dz_iso < s.iso_native_high:
                    # Both edges of the dead zone are legal; take the closer one
                    # rather than always rounding up, so we don't overshoot by
                    # the full 2.7-stop step when iso_min was the better fit.
                    dz_iso = min((s.iso_min, s.iso_native_high),
                                 key=lambda c: _err_for(c)[0])
                new_iso, new_s = dz_iso, _err_for(dz_iso)[1]

                # ── Dead-zone escape ─────────────────────────────────────────
                # iso_min -> iso_native_high is a single 2.7-stop step (100->640
                # on the A7III). With the shutter pegged at its ceiling there is
                # then no achievable exposure near the target: the loop sees a
                # large error whichever edge it picks, corrects, overshoots, and
                # limit-cycles — a 2.7-stop flicker every few frames. The dead
                # zone is a noise/DR optimisation; visible flicker is far worse,
                # so when neither edge lands within 0.75 stop we fall back to
                # the full 1/3-stop ladder.
                if _err_for(new_iso)[0] > 0.75:
                    esc_iso = _snap_1_3_iso(max(s.iso_min, min(iso_max, ideal_iso)))
                    if _err_for(esc_iso)[0] < _err_for(new_iso)[0] - 0.1:
                        new_iso, new_s = esc_iso, _err_for(esc_iso)[1]

                # Hysteresis: keep the ISO we are already on unless switching
                # buys more than half a stop of accuracy, so small wander near a
                # boundary does not toggle it back and forth.
                prev_iso = self._prev_iso
                if prev_iso and prev_iso != new_iso and s.iso_min <= prev_iso <= iso_max:
                    e_new,  _      = _err_for(new_iso)
                    e_prev, t_prev = _err_for(prev_iso)
                    if e_prev - e_new < 0.5:
                        new_iso, new_s = prev_iso, t_prev
                self._prev_iso = new_iso

            # Sidecar error: actual stops of compensation vs ideal (continuous) delta.
            # Positive = camera gave more exposure than requested (overexposed).
            # Negative = hardware ceiling prevented full compensation (underexposed).
            # This is on the same EV scale as ev_delta — no cross-scale issue.
            shutter_ev_actual = math.log2(s.anchor_shutter_s / max(new_s,   1e-9))
            iso_ev_actual     = math.log2(s.anchor_iso        / max(new_iso, 1e-1))
            sidecar_error_ev  = (shutter_ev_actual + iso_ev_actual) - ev_delta
            return new_iso, new_s, aperture_snapped, sidecar_error_ev

        # Fallback aperture-based path (no anchor calibration set)
        iso_max  = s.iso_max_night  if phase in ('twilight', 'night') else s.iso_max

        if phase == 'night':
            slo, shi = 1/500, s.shutter_max_night
            # ── Shutter priority for night sky ────────────────────────────────
            # Apertures are often fixed on manual lenses. Long shutter gathers
            # more photons with less read-noise penalty than high ISO.
            # Strategy: find the LOWEST ISO where the required shutter still
            # fits under the ceiling (shi). That gives the longest possible
            # shutter at the cleanest ISO.
            # If even the highest ISO needs shutter > shi, peg both at their
            # maximums and let the anti-windup clamp handle the EV error.
            _iso_candidates = [100, 640, 800, 1000, 1250, 1600, 2000, 2500, 3200,
                               4000, 5000, 6400, 8000, 12800]
            if not s.iso_native_high:
                # Classic camera: allow the intermediate ISOs too
                _iso_candidates = [100, 125, 160, 200, 250, 320, 400, 500,
                                   640, 800, 1000, 1250, 1600, 2000, 2500, 3200,
                                   4000, 5000, 6400, 8000, 12800]
            iso_ord = [x for x in _iso_candidates
                       if s.iso_min <= x <= iso_max] or [s.iso_min]

            def _t(iso): return (aperture_snapped**2) / (2**(ev + math.log2(iso/100.0)))

            chosen_iso, chosen_t = iso_ord[-1], _snap_s(shi)   # fallback: max ISO, max shutter
            for iso in iso_ord:
                t = _snap_s(_t(iso))
                if t <= shi:          # shutter fits — this is the lowest ISO that works
                    chosen_iso = iso
                    chosen_t   = max(slo, t)
                    break
                # t > shi: scene too dark for this ISO at max shutter — try next ISO up

            actual_ev = math.log2((aperture_snapped**2) / chosen_t) - math.log2(chosen_iso / 100.0)
            return chosen_iso, chosen_t, aperture_snapped, actual_ev - ev

        elif phase == 'twilight':
            slo, shi = 1/500, s.shutter_max_twilight
            if s.iso_native_high:
                iso_ord = [1600, 3200, 800, 640, 100]
            else:
                iso_ord = [1600, 3200, 800, 400, 200, 100]
        else:
            slo, shi = 1/8000, 1.0
            if s.iso_native_high:
                iso_ord = [100, 640, 800, 1600, 3200]
            else:
                iso_ord = [100, 200, 400, 800, 1600, 3200]

        iso_ord = [x for x in iso_ord if s.iso_min <= x <= iso_max] or [s.iso_min]

        def _t(iso): return (aperture_snapped**2) / (2**(ev + math.log2(iso/100.0)))

        for iso in iso_ord:
            t = _snap_s(_t(iso))
            if slo <= t <= shi:
                actual_ev = math.log2((aperture_snapped**2) / t) - math.log2(iso / 100.0)
                return iso, t, aperture_snapped, actual_ev - ev

        # Fallback for twilight/day: target middle of shutter range
        mid = math.sqrt(slo * shi)
        best_iso, best_err = iso_ord[0], float("inf")
        for iso in iso_ord:
            err = abs(math.log(max(_t(iso),1e-9)) - math.log(mid))
            if err < best_err:
                best_err, best_iso = err, iso

        best_t = _snap_s(max(slo, min(_t(best_iso), shi)))
        actual_ev = math.log2((aperture_snapped**2) / best_t) - math.log2(best_iso / 100.0)
        return best_iso, best_t, aperture_snapped, actual_ev - ev

    # ── Location helpers ──────────────────────────────────────────────────────

    def _make_location(self) -> LocationInfo:
        s = self.settings
        return LocationInfo(name="UserLocation", region="",
                            timezone=s.tz, latitude=s.lat, longitude=s.lon)

    @staticmethod
    def _make_tzinfo(tz_name: str) -> ZoneInfo:
        try:    return ZoneInfo(tz_name)
        except: return ZoneInfo("UTC")

    def _ensure_tz(self, dt: datetime.datetime) -> datetime.datetime:
        return dt.replace(tzinfo=self._tzinfo) if dt.tzinfo is None else dt.astimezone(self._tzinfo)


# ─────────────────────────────────────────────────────────────────────────────
# Pure helper functions
# ─────────────────────────────────────────────────────────────────────────────

def _wrap_angle(a: float) -> float:
    return (a + 180) % 360 - 180

def _snap_1_3_iso(iso: float) -> int:
    standard_isos = [
        50, 64, 80, 100, 125, 160, 200, 250, 320, 400, 500, 640,
        800, 1000, 1250, 1600, 2000, 2500, 3200, 4000, 5000, 6400,
        8000, 10000, 12800, 16000, 20000, 25600, 32000, 40000, 51200
    ]
    return min(standard_isos, key=lambda x: abs(math.log2(x) - math.log2(max(1.0, iso))))

def _snap_1_3_shutter(t: float) -> float:
    stops = math.log2(max(1e-9, t))
    snapped_stops = round(stops * 3.0) / 3.0
    return 2.0 ** snapped_stops

def _snap_1_3_aperture(f: float) -> float:
    stops2 = math.log2(max(1.0, f)) * 2.0
    snapped_stops2 = round(stops2 * 3.0) / 3.0
    return 2.0 ** (snapped_stops2 / 2.0)

def _p50_to_pixel_ev(p50: float) -> float:
    """Convert a target P50 luminance (0-255) to the same pixel-EV scale that
    push_meter_shot() produces for meter_ev.  Keeping the setpoint and the
    measurement in one scale is what makes the brightness loop closable."""
    lum = max(float(p50), 1.0) / 255.0
    return math.log2(max(lum ** 2.2, 1e-9) / 0.18) + 12.0


def _smootherstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * t * (t * (6 * t - 15) + 10)

def _phase_for_alt(sun_alt: float) -> str:
    if sun_alt > 10:  return "day"
    if sun_alt >  0:  return "golden"
    if sun_alt > -6:  return "twilight"
    return "night"

def _phase_pair(sun_alt: float) -> Tuple[str, str, float]:
    if sun_alt > 10:           return ("day",      "day",      0.0)
    if 0   < sun_alt <= 10:    return ("day",      "golden",   _smootherstep((10 - sun_alt) / 10.0))
    if -6  < sun_alt <= 0:     return ("golden",   "twilight", _smootherstep(-sun_alt / 6.0))
    if -12 < sun_alt <= -6:    return ("twilight", "night",    _smootherstep((-6 - sun_alt) / 6.0))
    return ("night", "night", 0.0)

def _is_in_frame(
    obj_az: float, obj_alt: float,
    cam_az: float, cam_alt: float,
    hfov: float, vfov: float,
) -> Tuple[bool, float]:
    d_az  = _wrap_angle(obj_az - cam_az)
    d_alt = obj_alt - cam_alt
    if abs(d_az) <= hfov/2 and abs(d_alt) <= vfov/2:
        nx   = abs(d_az)  / (hfov/2) if hfov > 0 else 0.0
        ny   = abs(d_alt) / (vfov/2) if vfov > 0 else 0.0
        return True, min(1.0, math.sqrt((nx*nx + ny*ny) / 2.0))
    return False, 1.0

def _format_shutter(t: float) -> str:
    if t >= 1.0: return f"{round(t,3)}s"
    return f"1/{max(1, int(round(1.0/t)))}"

def _rg_bg_to_kelvin(rg: float, bg: float, luminance: float = 128.0) -> int:
    kelvin_raw = 5500 - (rg - 1.0) * 2000 + (bg - 1.0) * 1600
    trust  = max(0.0, min(1.0, (luminance - 10.0) / 50.0))
    kelvin = trust * kelvin_raw + (1.0 - trust) * 5000.0
    return int(max(2500, min(10000, kelvin)))

def _weighted_slope(t: np.ndarray, y: np.ndarray, w: np.ndarray) -> float:
    if len(t) < 2: return 0.0
    w_sum = float(np.sum(w))
    if w_sum == 0: return 0.0
    t_mean = float(np.average(t, weights=w))
    y_mean = float(np.average(y, weights=w))
    num = float(np.sum(w * (t - t_mean) * (y - y_mean)))
    den = float(np.sum(w * (t - t_mean) ** 2))
    return num / den if abs(den) > 1e-10 else 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Self-test
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import random
    hg = HolyGrailController()
    print("Cold:", hg.get_next_shot_parameters())
    random.seed(42)
    base_ev = 6.0
    for i in range(25):
        ev   = base_ev + i * 0.05 + random.gauss(0, 0.08)
        kelv = 4800.0 + random.gauss(0, 50)
        hg.push_capture_ev(ev, kelv, i, sky_fraction=0.4, condition="clear")
    p = hg.get_next_shot_parameters()
    print(f"Warm: EV={p['ev_final']:.3f} shutter={p['shutter']} "
          f"ISO={p['iso']} K={p['kelvin']} "
          f"pixel_w={p['pixel_weight']} astro_w={p['astro_weight']} "
          f"slope={p['ev_slope']:.4f} R2={p['r_squared']:.3f}")
