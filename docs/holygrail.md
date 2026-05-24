# Holy Grail Mode — Technical Deep Dive

Holy Grail timelapse refers to shooting through an entire day-to-night or night-to-day transition, maintaining smooth exposure throughout. It's considered the hardest challenge in timelapse photography because the scene brightness changes by up to **14 stops** (from bright daylight to deep night), and any step-change in exposure is immediately visible as a flash in the final video.

The PiSlider Holy Grail system (`holygrail.py`) solves this with a three-layer architecture that combines an astronomical prediction model, pixel-based feedback, and a dynamic blending system. The result is smooth, artifact-free exposure tracking through all lighting phases.

---

## Architecture Overview

```
┌────────────────────────────────────────────────────────┐
│                  HolyGrailController                   │
│                                                        │
│  ┌─────────────────┐  ┌──────────────────────────┐    │
│  │  ASTRONOMICAL   │  │    DNG CAPTURE TRACKER   │    │
│  │     MODEL       │  │   (AdaptiveEVTracker)    │    │
│  │                 │  │                          │    │
│  │ sun/moon pos    │  │ rolling 20-frame window  │    │
│  │ phase → EV/K    │  │ weighted linear regress. │    │
│  │ moonlight model │  │ slope (stops/frame) + R² │    │
│  │ disc-in-frame   │  │                          │    │
│  │ look-ahead ramp │  │ → meas_ev, slope_ma      │    │
│  └────────┬────────┘  └────────────┬─────────────┘    │
│           │                        │                   │
│           │    DYNAMIC BLEND       │                   │
│           │    WEIGHT FUNCTION     │                   │
│           │  ┌─────────────────┐   │                   │
│           └──┤  astro_w (0-1)  ├───┘                   │
│              │  pixel_w (0-1)  │                       │
│              └────────┬────────┘                       │
│                       │                               │
│              blended_ev = astro_ev × astro_w          │
│                       + tracker_ev × pixel_w          │
│                                                        │
│              ↓ agility clamp (max_step/frame)          │
│              ↓ histogram brakes (highlight/shadow)     │
│              ↓ drift pull (reality anchor)             │
│                                                        │
│              → shutter / ISO / Kelvin output           │
└────────────────────────────────────────────────────────┘
```

Each frame, the system computes a target EV by blending the astronomical model with the pixel tracker, then passes it through a rate limiter (agility) and histogram-based safety brakes before converting it to actual camera settings.

---

## Layer 1: Astronomical Model

**Code:** `_compute_astro()`, `_ev_for_phase()`, `_kelvin_for_phase()`

The astronomical model uses the [astral](https://astral.readthedocs.io/) library to compute real-time sun and moon positions from GPS coordinates, date, and time.

### Phase Classification

The sun altitude determines which lighting phase is active:

| Sun Altitude | Phase | EV target | Kelvin | Interval |
|-------------|-------|-----------|--------|----------|
| > 6° | `day` | 13.0 | 5500K | 5s |
| -6° to 6° | `golden` | 10.0 | 5200K | 7s |
| -12° to -6° | `twilight` | 6.0 | 4800K | 10s |
| < -12° | `night` | 3.0 | 3800K | 20s |

These are the *prior* values — what the system expects without any measured feedback. The tracker then adjusts from here.

Transitions between phases use **cubic smoothstep interpolation** (`_phase_pair()`), so the EV and Kelvin targets blend smoothly across the horizon crossing. There's no step-change at the phase boundary.

### Moonlight Model

At night, a rising moon dramatically brightens the scene. The model accounts for this:

```python
moonlight_ev = moonlight_ev_max × sin(moon_phase × π)^0.5 × sin(moon_alt)
```

- `moon_phase` — 0 to 1 over the 29.53-day lunar cycle (0 = new moon, 0.5 = full moon)
- `moon_alt` — moon altitude above horizon (0° = rising/setting, 90° = zenith)
- `moonlight_ev_max` — default 4.5 stops (full moon at zenith vs. new moon)

A full moon rising during an overnight shoot adds up to 4.5 EV to the astro model automatically, preventing the camera from over-exposing into the moonlit sky.

### Disc-in-Frame Detection

When the sun or moon is within the camera's field of view, it creates a direct light source that doesn't follow the ambient model. The `_disc_ev_offset()` function computes whether the disc is geometrically inside the frame based on:
- Camera azimuth and altitude (`cam_az`, `cam_alt`)
- Camera horizontal and vertical FOV (`hfov`, `vfov`)
- Sun/moon position

If the sun is directly in frame, the model applies a negative EV offset (pull down exposure) to protect highlights.

### Look-Ahead Anticipation

Before the sun or moon enters the frame, the system begins pre-adjusting EV up to `disc_lookahead_min` minutes in advance using a smoothstep ramp:

```python
ramp = smoothstep(1.0 - minutes_away / lookahead_min)
disc_anticipation_ev += sign × ramp
```

This prevents the sudden lurch that occurs if you wait until the disc is already in frame before compensating.

---

## Layer 2: DNG Capture Tracker (AdaptiveEVTracker)

**Code:** `AdaptiveEVTracker`, `push_meter_shot()`, `_refit_meter()`

The tracker is the closed-loop feedback system. After every captured DNG, the app reads luminance from the thumbnail and feeds the measurement back into the tracker. The tracker maintains a rolling window of the last 20 measurements and fits a weighted linear regression to detect the *rate* of EV change.

### Anchor Exposure System

A critical design choice: all meter shots are taken at a **fixed anchor exposure** (`anchor_shutter_s`, `anchor_iso`). Because the settings never change for meter shots, every EV reading is directly comparable — no normalization or compensation math is needed.

The system takes a "meter shot" at anchor exposure every interval, separate from the actual timelapse frame. This gives a clean, unbiased measurement of scene luminance.

Without anchor exposure, the measurement would be contaminated by the very exposure change you're trying to control — a chicken-and-egg problem. With anchor exposure, the measurement is always an independent observation of reality.

### Weighted Linear Regression

For each update, the tracker fits:

```
EV = intercept + slope × time
```

over the rolling 20-frame window, using two weight factors:

1. **Anomaly weight** — frames that deviate >1 stop from the window median get weight 0.15. Two consecutive anomalies in the same direction get full weight (confirming a real event).

2. **Recency decay** — the newest frame has weight 1.0; each step back in time is multiplied by `recency_decay` (default 0.92). At 20 frames, the oldest sample has weight 0.92^19 ≈ 0.20.

```python
recency = recency_decay ** np.arange(n-1, -1, -1)
weights = anomaly_weights × recency
slope = weighted_slope(timestamps, evs, weights)
```

Recency decay is key to handling **phase transitions** gracefully. When sunset begins, the new fast-changing samples quickly dominate the regression, pushing the old stable-day samples out of influence. There's no hard flush or reset — the old data just fades away.

### R² Confidence

The regression also outputs R² (coefficient of determination). A high R² means the EV values fit the linear trend well — reliable, predictable change. A low R² means scatter — clouds, birds, transient shadows.

R² is used to gate how strongly the tracker output drives the exposure:
- High R² → tracker confident → more pixel weight allowed
- Low R² → noisy scene → stick closer to the astro model

### slope_ma (Moving Average of Slope)

`slope_ma` is a moving average of recent slope estimates over `slope_ma_window` frames (default 12). This filters out single-frame slope spikes. A brief cloud shadow can make the instantaneous slope look huge; `slope_ma` ignores it if the trend doesn't sustain.

---

## Layer 3: Dynamic Blend Weight

**Code:** `_blend_weight()`

The blend weight decides how much to trust the pixel tracker vs. the astronomical model at any given moment:

```python
blended_ev = total_astro_ev × astro_w + tracker_ev × pixel_w
# where pixel_w + astro_w = 1.0
```

The weights shift based on phase and conditions:

| Condition | pixel_w | astro_w | Reason |
|-----------|---------|---------|--------|
| Deep stable night | 0.15 | 0.85 | Dark scenes are noisy; astro is more reliable |
| Active transition (golden/twilight) | 0.50 | 0.50 | Both sources needed equally |
| Day with clouds | 0.75 | 0.25 | Pixel tracks clouds; astro can't |
| High R² (clean linear trend) | +bonus | -bonus | Tracker is confident |
| Moon near horizon | -penalty | +penalty | Moonrise/set requires astro precision |

The function computes a `base_pixel_w` from the sun altitude (linear ramp from night→day), then adjusts for sky variance (cloud indicator), tracker confidence (R²), and celestial events near the horizon.

---

## EV Output Path

After blending, the target EV goes through several more steps before becoming camera settings:

### Phase-Variable Agility (Rate Limiter)

`max_step` is the maximum EV change allowed per frame. It's phase-dependent:

| Phase | Agility | max_step |
|-------|---------|---------|
| `day` | 0.008 | ~0.008 EV/frame |
| `golden` | 0.035 | ~0.035 EV/frame |
| `twilight` | 0.030 | ~0.030 EV/frame |
| `night` | 0.020 | ~0.020 EV/frame |

Lower = smoother, but slower to respond. Higher = more responsive, but risks visible steps.

Near the horizon (sun or moon within ±15°), `max_step` is boosted by `horizon_agility_boost` (default 1.8×) to handle the rapid lighting changes at sunrise/sunset without lagging.

This is the primary "butter" control. Think of it as a **rate limiter on the rudder** — we know which direction the light is going; `max_step` controls how fast we turn.

### Slope-Driven Movement

Each frame, the exposure moves by the slope component:

```python
slope_step = clamp(slope_ma × interval_sec × pixel_w, -max_step, max_step)
ev_smooth = last_ev + slope_step
```

Note: `slope_ma` is in stops/second; multiplying by `interval_sec` converts to stops/frame so max_step can clamp it correctly.

### Drift Pull (Reality Anchor)

Even during flat, stable scenes where `slope_ma ≈ 0`, the output could drift away from measured reality over time. A small `drift_pull` steers `ev_smooth` toward `meas_ev` each frame:

```python
drift_pull = clamp(drift_gap × drift_pull_strength × 10, -max_step/2, max_step/2)
ev_smooth = ev_slope_result + drift_pull
```

The drift pull is capped at `max_step/2` and is intentionally **exempt from histogram brakes** (see below). This ensures the camera can always correct toward reality even when a bright horizon is slowing the slope component.

### Histogram Brakes

Two safety overrides act as **brakes** on the slope component (not reversals):

**1. Highlight Protection**
If `highlight_fraction` (fraction of pixels ≥ 245) exceeds `highlight_clip_limit` (0.5%), the downward slope is slowed:

```python
brake_factor = min(0.75, excess × 15.0)
if slope_moving_down:
    ev_slope = last_ev - (downward × (1 - brake_factor))
```

This prevents the system from chasing a bright horizon glow down into blown highlights. The camera holds exposure higher until the highlights clear.

**2. Shadow Boost**
If `shadow_fraction` (fraction of pixels ≤ 18) exceeds 40% during night/twilight, the upward slope is slowed — the system doesn't push exposure up aggressively when the scene is already mostly dark pixels (deep night sky correctly looks dark).

---

## Camera Settings Conversion

**Code:** `_ev_to_exposure()`

After the EV output is computed, it must be converted to actual shutter/ISO/aperture values. The system follows a priority order:

1. **Aperture** is phase-variable: `aperture_day` (f/5.6) → `aperture_night` (f/2.8), transitioning through twilight. Wider aperture at night lets in more light.

2. **Shutter speed** is adjusted first (ISO held constant). If shutter would exceed `shutter_max_night`, it's capped.

3. **ISO** is adjusted to make up remaining EV. It's clamped between `iso_min` and `iso_max_night`.

4. **Night prefer-low-ISO mode** — the system prefers to extend shutter before raising ISO, which produces cleaner images. ISO only rises when shutter is already at its cap.

All values are snapped to **1/3-stop increments** (matching standard camera menu steps), using lookup tables for standard shutter speeds, ISO, and aperture values.

---

## Calibration and Cold Start

Before starting a Holy Grail sequence, take a **calibration shot** at known settings:

```python
controller.seed_from_calibration(ev=measured_ev, kelvin=3800)
```

This seeds the tracker's `_last_ev` and `_last_kelvin` so the system knows where to start. Without a seed, the system guesses from the astronomical model, which may be several stops off depending on local conditions.

On the first few frames (before `tracker_warmup` = 5 frames), the system operates in **cold start mode**: it holds the seeded EV and only begins slope-tracking after the regression has enough data.

---

## Settings Reference

All parameters are in `HGSettings`. Key ones:

| Setting | Default | Description |
|---------|---------|-------------|
| `lat`, `lon` | 49.89, -97.14 | Location (Winnipeg default) |
| `tz` | `America/Winnipeg` | Timezone |
| `ev_day` / `ev_night` | 13.0 / 3.0 | Per-phase EV targets |
| `kelvin_day` / `kelvin_night` | 5500K / 3800K | Per-phase Kelvin targets |
| `interval_day` / `interval_night` | 5s / 20s | Per-phase capture interval |
| `agility_golden` | 0.035 | Max EV change/frame at golden hour |
| `agility_night` | 0.020 | Max EV change/frame at night |
| `tracker_window` | 20 | Rolling measurement window size |
| `tracker_warmup` | 5 | Frames before regression is trusted |
| `tracker_recency_decay` | 0.92 | Exponential fade for old samples |
| `drift_pull_strength` | 0.003 | How strongly output tracks meas_ev |
| `highlight_clip_level` | 245 | Pixel value = "blown" |
| `highlight_clip_limit` | 0.005 | Max fraction of blown pixels |
| `shutter_max_night` | 25.0s | Max shutter at night |
| `iso_max_night` | 3200 | Max ISO at night |
| `moonlight_ev_max` | 4.5 | Full-moon EV contribution at zenith |

---

## Sky Analyser

**Code:** `SkyAnalyser`

A supplementary system for daylight metering from the live preview stream. It uses HSV color segmentation to isolate sky pixels (top 65% of frame, blue/white hue ranges), then measures:

- **Luminance mean** of sky pixels → EV estimate
- **R/G and B/G ratios** → Kelvin estimate
- **HSV saturation** → `condition` (clear / hazy / overcast)

Sky measurements are used in `push_preview_frame()` as a supplementary input alongside the primary anchor-exposure meter shots. At night (sun below -6°), the sky analyser returns `None` immediately — night luminance from a preview frame is too noisy to be useful.

---

## Phase Transitions

The Holy Grail's greatest challenge is handling phase transitions smoothly. Several mechanisms work together:

1. **Smoothstep interpolation** between phase EV/Kelvin targets (no step change at boundary)
2. **Recency decay** in regression (old data from previous phase fades naturally)
3. **No window flush** on phase transition (flushes cause cold starts that create exposure jumps)
4. **Drift pull** keeps output anchored to measured reality through the transition
5. **Agility boost** at horizon events allows fast response to rapid changes

The result: a sunset transition that takes 45 minutes to unfold produces no visible flicker or stepping in the final video.

---

## Practical Tips

- **Set location precisely** — latitude/longitude affects sun elevation by minutes. Use a GPS app if unsure.
- **Shoot toward the direction of the sun** for the most dramatic golden hour light. Set `cam_az` to match your camera's pointing direction.
- **Use anchor exposure** — take the calibration shot before sunset begins. The system works much better with a real seed than a pure astro cold start.
- **Set `shutter_max_night`** to roughly `interval_night - 2s` (e.g., 23s for a 25s night interval). This leaves time for read-out and prevents the camera from being occupied when the next frame is due.
- **Night ISO** — `iso_max_night = 3200` is conservative. Modern sensors can go to 6400 with acceptable noise. Raise this for darker conditions or faster intervals.
- **`agility_golden`** is the key smoothness control. Lower it (0.020) for extremely butter-smooth transitions; raise it (0.050) if the system is lagging behind rapid sunset changes.
- **Watch the graph** — the `graph.html` view shows EV targets, measured EV, blend weights, and slopes in real time. If the tracker is fighting the astro model, the blend weight chart will show it.
