#!/usr/bin/env python3
"""
app.py — PiSlider Master Orchestrator v2.4

Changes in v2.4:
  - STATE PERSISTENCE: session.json saved on every meaningful change.
    New clients reconnecting get full state via 'init' packet.
    Browser can close/reopen freely; timelapse continues on Pi.
  - HG CALIBRATION SHOT: sequence now starts with a single AE/AWB-ON
    exposure to measure ambient EV, which seeds _smooth_ev so the HG
    engine starts in the right ballpark instead of ramping from cold.
  - MOTION DETECTION TRIGGER: 'picam_motion' trigger mode uses
    background subtraction on the preview stream to fire the shutter
    when pixel change in a user-defined ROI exceeds a threshold.
    Two variants: picam_motion_only (like aux_only) and
    picam_motion_hybrid (like aux_hybrid — fires at deadline if quiet).
  - REMOVED generate_hg_plan: plan runs automatically; no explicit user
    action needed. simulate_plan() still called internally at start.
  - REMOVED build_sky_map: placeholder removed from command table;
    kept as stub returning a clear "not wired" message.
  - SEQUENCE PROGRESS: broadcasts estimated_end_time, estimated_frames,
    current_interval so UI can show live HG interval and end-time.
  - Stop button becomes Reset when idle (handled client-side).
  - HG mode locks exposure engine controls (UI-side only; backend already
    overrides them each frame).
"""

import asyncio
import json
import os
import socket as _socket
import time
import math
import datetime
import logging
import signal
import atexit
import subprocess
import shutil
import requests
import cv2
import numpy as np
from pathlib import Path
from typing import Optional
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Query
from fastapi.responses import StreamingResponse, HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from hardware import HardwareController
from holygrail import HolyGrailController, HGSettings
from motion_engine import MotionEngine
from slider import TrajectoryPlayer, LinearAxis, RotationAxis
from distributions import CURVE_FUNCTIONS, normalize
from macro_engine import (MacroEngine, MacroSession, ExposureSlot, LensProfile,
                          rail_frame_count, total_image_count, estimated_storage_gb,
                          depth_per_image_um, num_stacks_grid, effective_pixel_um,
                          compute_geodesic_grid, stereo_multiplier,
                          generate_scan_positions)
from cinematic_engine import (SoftLimitGuard, InertiaEngine, ArcTanTracker,
                               ProgrammedMove, MoveLibrary, Keyframe, RIG_PRESETS,
                               NUDGE_SPEED_PAN, NUDGE_SPEED_TILT, NUDGE_SPEED_SLIDER,
                               ADDR_PAN, ADDR_TILT, ADDR_SLIDER,
                               VACTUAL_PER_DEG_S_PAN, VACTUAL_PER_DEG_S_TILT,
                               VACTUAL_PER_MM_S)
from gamepad import GamepadReader, GamepadEvent
from neopixel_status import leds as status_leds

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("PiSlider")

# ── Persistent rotating log file ──────────────────────────────────────────────
# Logs survive browser disconnects and server restarts.
# Location: ~/.pislider.log  (always on the Pi's local filesystem, not the T7)
# Keeps last 2 MB × 3 files = 6 MB of history.
import logging.handlers as _lh
_LOG_FILE = Path.home() / ".pislider.log"
_fh = _lh.RotatingFileHandler(
    _LOG_FILE, maxBytes=2 * 1024 * 1024, backupCount=3, encoding="utf-8"
)
_fh.setLevel(logging.DEBUG)
_fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s"))
logging.getLogger().addHandler(_fh)   # attach to root so uvicorn logs go there too

# Suppress high-frequency uvicorn access log entries for polling endpoints
class _SuppressPollingRoutes(logging.Filter):
    _MUTED = ("loupe_crop", "mjpeg_feed", "latest_frame", "disk_info",
              "video_feed", "latest_frame")
    def filter(self, record):
        msg = record.getMessage()
        return not any(r in msg for r in self._MUTED)

logging.getLogger("uvicorn.access").addFilter(_SuppressPollingRoutes())
_fh_suppress = _SuppressPollingRoutes()
_fh.addFilter(_fh_suppress)   # keep the log file clean too

app    = FastAPI()
hw     = HardwareController(gpio_chip_index=4)   # Pi 5: main GPIO is gpiochip4

# Sony shutter limit — camera preset list tops out at 30s.
# Any exposure beyond this requires bulb mode (startBulbShooting / stopBulbShooting).
SONY_BULB_THRESHOLD = 30.0   # seconds

# Focus rail stepper: 200 steps/rev × 8 microsteps ÷ 2mm pitch = 800 steps/mm
STEPS_PER_MM = 800.0  # Must match hardware.py and macro_engine.py

# Enable motors permanently — hold torque maintained by driver standstill current.
# Motors are only disabled on clean shutdown in cleanup().
hw.enable_motors(True)

hg     = HolyGrailController()
engine = MotionEngine()
# 1/8 microstep: slider=50 steps/mm, pan=66.667 steps/°, tilt=133.333 steps/°
player = TrajectoryPlayer(hw, steps_per_mm=50.0, pan_steps_per_deg=66.667, tilt_steps_per_deg=133.333)

slider_axis = LinearAxis(hw, addr=2, steps_per_mm=50.0)
pan_axis    = RotationAxis(hw, addr=1, steps_per_deg=66.667)
tilt_axis   = RotationAxis(hw, addr=0, steps_per_deg=133.333)

# Start LED status thread
status_leds.start()
status_leds.set_mode("startup")

# ─── CLEAN SHUTDOWN ───────────────────────────────────────────────────────────
# Ensure GPIO handles are always released — prevents "GPIO busy" on next start.
def _shutdown_cleanup():
    """Called on any exit: normal, Ctrl+C, kill, or os.execv restart."""
    try:
        status_leds.stop()
    except Exception:
        pass
    try:
        hw.cleanup()
    except Exception:
        pass

atexit.register(_shutdown_cleanup)

def _signal_handler(sig, frame):
    """Handle SIGTERM/SIGINT so atexit fires cleanly."""
    _shutdown_cleanup()
    raise SystemExit(0)

signal.signal(signal.SIGTERM, _signal_handler)
signal.signal(signal.SIGINT,  _signal_handler)

# ─── MACRO ENGINE INSTANCE ────────────────────────────────────────────────────
_macro_task: Optional[asyncio.Task] = None

# ─── CINEMATIC ENGINE INSTANCES ───────────────────────────────────────────────
_soft_guard    = SoftLimitGuard()
_arctan        = ArcTanTracker()
_move_library  = MoveLibrary()
_inertia: Optional[InertiaEngine] = None
_prog_move: Optional[ProgrammedMove] = None
_prog_task: Optional[asyncio.Task] = None
_gamepad_queue: asyncio.Queue = asyncio.Queue(maxsize=64)
_gamepad_reader: Optional[GamepadReader] = None
_gamepad_task: Optional[asyncio.Task] = None
_cinematic_mode: str = "live"   # "live" | "programmed"
_cinematic_live_active: bool = False   # True only while cinematic_live_start is active
_pending_play: dict = {}   # holds context while awaiting path-limit-warning confirmation
_thumb_retry_queue: list  = []   # frame_ids that fired a shutter but have no thumb yet
# Joystick input is allowed when is_running is False OR when _cinematic_live_active
# is True (live cinematic is specifically joystick-driven).  Real automated sequences
# (timelapse, programmed move) leave _cinematic_live_active=False so joystick is
# blocked during unattended runs.

# Video recording state
_recording: bool = False
_record_start_time: Optional[float] = None
_video_output_path: Optional[str] = None

PREVIEW_CONFIG_43  = {
    "size":   (1280, 960),   # full IMX477 4:3 sensor area, downscaled from 4056×3040
    "format": "RGB888"
}
PREVIEW_CONFIG_169 = {
    "size":   (1280, 720),   # 16:9 cinematic — slight sensor crop at top/bottom
    "format": "RGB888"
}
PREVIEW_CONFIG     = PREVIEW_CONFIG_43   # default
STREAM_SIZE_43     = (640, 480)
STREAM_SIZE_169    = (640, 360)
STREAM_SIZE        = STREAM_SIZE_43      # default

SESSION_FILE = Path.home() / ".pislider_session.json"

_last_frame:  Optional[np.ndarray] = None
_last_capture_time: float = 0.0   # timestamp of last still capture (for ISP settle guard)
_latest_shot: Optional[bytes]      = None

# Sony liveview shared state — one background thread maintains the connection;
# multiple /video_feed consumers read from this buffer (same pattern as PiCam).
_sony_last_frame: Optional[bytes] = None
_sony_liveview_running: bool      = False
_sony_liveview_thread: Optional[object] = None   # threading.Thread
# Sony USB tether liveview — same _sony_last_frame buffer, separate worker
_sony_usb_liveview_running: bool       = False
_sony_usb_liveview_thread: Optional[object] = None
_SONY_USB_PREVIEW_TMP = "/tmp/sony_usb_preview.jpg"
# Set True when timelapse_worker pauses liveview so the finally block can
# restart it regardless of whether the sequence ended normally or crashed.
_timelapse_paused_liveview: bool  = False
_cinematic_paused_liveview: bool  = False  # stopped for Record+Run; restart after

# Motion detection state — optical flow based
_motion_triggered: bool             = False
_motion_prev_gray: Optional[np.ndarray] = None   # previous frame for LK flow
_motion_consec_hits: int            = 0           # consecutive trigger frames

# ─── CAMERA INIT ──────────────────────────────────────────────────────────────
try:
    from picamera2 import Picamera2
    picam = Picamera2()
    # Use full IMX477 sensor area (4056×3040) downscaled to 1280×960 preview.
    # transform= default keeps full sensor FOV — no ISP centre crop.
    cfg = picam.create_video_configuration(
        main=PREVIEW_CONFIG_43,
        raw={"size": picam.camera_properties["PixelArraySize"]},
    )
    picam.configure(cfg)
    picam.start()
    _HAS_PICAM = True
    logger.info("PiCamera2 (IMX477) started — full sensor FOV at 1280×960.")
except Exception as e:
    logger.warning(f"PiCamera2 full config failed: {type(e).__name__}: {e}")
    # Fallback: try simple preview config (no raw stream)
    try:
        from picamera2 import Picamera2
        picam = Picamera2()
        picam.configure(picam.create_preview_configuration(main=PREVIEW_CONFIG_43))
        picam.start()
        _HAS_PICAM = True
        logger.info("PiCamera2 (IMX477) started — standard preview (full sensor FOV unavailable).")
    except Exception as e2:
        logger.error(f"PiCamera2 both configs failed. Full: {type(e).__name__}. Preview: {type(e2).__name__}: {e2}")
        logger.error("Camera troubleshooting: (1) Check raspi-config camera is enabled, (2) Try 'libcamera-hello' from terminal")
        picam = None
        _HAS_PICAM = False

# ─── CAMERA RECOVERY ──────────────────────────────────────────────────────────
def _restart_picam() -> bool:
    """
    Attempt to recover from a camera frontend timeout or ISP error.
    Stops the camera, waits briefly, restarts, and immediately re-locks
    HG exposure controls so the preview doesn't revert to auto.
    Returns True if recovery succeeded.
    """
    global picam, _HAS_PICAM
    logger.warning("Camera recovery: attempting stop → restart…")
    try:
        if picam:
            try:
                picam.stop()
            except Exception:
                pass
            time.sleep(1.5)
            active_mode = state.get("active_mode", "timelapse")
            cfg_main = PREVIEW_CONFIG_169 if active_mode == "cinematic" else PREVIEW_CONFIG_43
            try:
                picam.configure(picam.create_video_configuration(
                    main=cfg_main,
                    raw={"size": picam.camera_properties["PixelArraySize"]},
                ))
            except Exception:
                picam.configure(picam.create_preview_configuration(main=cfg_main))
            picam.start()
            time.sleep(0.5)

            # Re-lock HG controls immediately after restart
            if hg.settings.enabled:
                _reapply_hg_after_capture()

            logger.info("Camera recovery: restart successful.")
            return True
    except Exception as e:
        logger.error(f"Camera recovery failed: {e}")
    return False


_DEFAULTS = {
    "active_camera":   "picam",
    "active_mode":     "timelapse",  # timelapse | cinematic | macro
    "camera_orientation": "landscape",  # landscape | portrait_cw | portrait_ccw | inverted
    "cine_fps":           24,           # cinematic recording frame rate: 24|25|30|60
    "save_path":       str(Path.home() / "Pictures/PiSlider"),
    "sony_ssid":       "Sony_A7III_WiFi",
    "sony_ip":         "",   # discovered dynamically from wlan1 gateway — never hardcode
    "is_running":      False,
    "current_frame":   0,
    "total_frames":    300,
    "manual_interval": 5.0,  # interval used when HG is disabled
    "vibe_delay":      1.0,
    "exp_margin":      0.2,
    "tl_preroll_s":    0.0,  # seconds to hold at start before motion begins (timelapse)
    "picam_ae":        True,
    "picam_awb":       True,
    "picam_shutter_s": 1/125,
    "picam_iso":       400,
    "picam_kelvin":    5500,
    "pan_min":  -90.0,
    "pan_max":   90.0,
    "tilt_min": -30.0,
    "tilt_max":  30.0,
    "slider_inverted": False,
    "pan_inverted":    False,
    "tilt_inverted":   False,
    "trigger_mode":  "normal",
    "aux_triggered": False,
    "origin_az":   0.0,
    "origin_tilt": 0.0,
    # Motion detection ROI: fraction of frame [x1,y1,x2,y2] 0.0–1.0
    "motion_roi":        [0.25, 0.25, 0.75, 0.75],
    "motion_threshold":  2000,   # frame-diff: total blob area in px² to trigger (tune 500–20000)
    "motion_warmup_frames": 10,  # frames before triggering begins
    "motion_cooldown":   2.0,    # min seconds between triggers
}

# ─── SESSION HISTORY (for graph tab) ─────────────────────────────────────────
from collections import deque
_session_history: deque = deque(maxlen=2000)   # last 2000 frames in memory
_SESSION_HISTORY_FILE = Path.home() / ".pislider_graph_history.json"
_timelapse_run_id: str = ""   # unique ID per run — used by graph page to bust thumb cache
_seq_wall_start:  float = 0.0 # wall-clock time when the sequence's phase=0 fired

def _load_session_history():
    """Load persisted graph history from disk on startup."""
    try:
        if _SESSION_HISTORY_FILE.exists():
            data = json.loads(_SESSION_HISTORY_FILE.read_text())
            for frame in data.get("frames", []):
                _session_history.append(frame)
            logger.info(f"Graph history loaded: {len(_session_history)} frames")
    except Exception as e:
        logger.warning(f"Graph history load failed: {e}")

def _save_session_history():
    """Persist graph history to disk (called every 10 frames)."""
    try:
        _SESSION_HISTORY_FILE.write_text(
            json.dumps({"frames": list(_session_history)}, separators=(',', ':'))
        )
    except Exception as e:
        logger.warning(f"Graph history save failed: {e}")

# ─── STATE PERSISTENCE ────────────────────────────────────────────────────────
def _load_session() -> dict:
    s = dict(_DEFAULTS)
    s["stop_event"]    = asyncio.Event()
    s["aux_triggered"] = False
    if SESSION_FILE.exists():
        try:
            saved = json.loads(SESSION_FILE.read_text())
            safe_keys = [k for k in _DEFAULTS if k not in ("stop_event", "aux_triggered")]
            for k in safe_keys:
                if k in saved:
                    s[k] = saved[k]
            # ALWAYS reset run state on startup — asyncio tasks don't survive restarts.
            # If the server was killed mid-sequence, is_running would be stuck True.
            if s.get("is_running"):
                s["_was_interrupted"] = True   # tells the client to show a warning
            s["is_running"] = False
            # Restore HG settings — strip any unknown fields from old sessions
            if "hg_settings" in saved:
                try:
                    import dataclasses as _dc
                    valid_keys = {f.name for f in _dc.fields(HGSettings)}
                    clean = {k: v for k, v in saved["hg_settings"].items() if k in valid_keys}
                    clean["continuous_shutter"] = (s.get("active_camera", "picam") == "picam")
                    hg.set_settings(HGSettings(**clean))
                except Exception as e:
                    logger.warning(f"Session HG restore failed: {e}")
            # Restore axis positions
            if "pan_deg"    in saved: pan_axis.current_deg    = saved["pan_deg"]
            if "tilt_deg"   in saved: tilt_axis.current_deg   = saved["tilt_deg"]
            if "slider_mm"  in saved: slider_axis.current_mm  = saved["slider_mm"]

            # Initialize soft limits guard ONLY from explicitly saved values —
            # not from _DEFAULTS (e.g. pan_min=-90 is a clamp default, not a
            # user-calibrated cinematic soft limit).  Using 'saved' (the raw file
            # dict) instead of 's' (merged with _DEFAULTS) ensures limits only
            # activate when the user intentionally set them.
            for axis in ("slider", "pan", "tilt"):
                guard_ax = getattr(_soft_guard, axis)
                min_val = saved.get(f"{axis}_min")   # only from file, not defaults
                max_val = saved.get(f"{axis}_max")
                if min_val is not None: guard_ax.min_unit = float(min_val)
                if max_val is not None: guard_ax.max_unit = float(max_val)
                guard_ax._update_cal()

            logger.info(f"Session restored from {SESSION_FILE}")
        except Exception as e:
            logger.warning(f"Session load failed: {e}")
    _load_session_history()
    return s


def save_session():
    """Persist serialisable state to disk so browser reloads and reconnects work."""
    try:
        import dataclasses
        saveable = {k: v for k, v in state.items()
                    if k not in ("stop_event", "aux_triggered") and isinstance(v, (str, int, float, bool, list, dict))}
        saveable["hg_settings"] = {
            k: (v if not isinstance(v, datetime.datetime) else v.isoformat())
            for k, v in dataclasses.asdict(hg.settings).items()
            if k != "start_dt"
        }
        saveable["pan_deg"]    = pan_axis.current_deg
        saveable["tilt_deg"]   = tilt_axis.current_deg
        saveable["slider_mm"]  = slider_axis.current_mm
        # Path planning settings (survive browser reload)
        if _prog_move:
            saveable["path_mode"]       = _prog_move.path_mode
            saveable["global_easing"]   = _prog_move.global_easing
            saveable["catmull_tension"] = _prog_move.catmull_tension
        SESSION_FILE.write_text(json.dumps(saveable, indent=2))
    except Exception as e:
        logger.error(f"save_session: {e}")
        if "No space left" in str(e):
            # Non-blocking broadcast — don't await inside sync function
            import threading
            def _alert():
                import asyncio as _asyncio
                loop = _asyncio.get_event_loop()
                if loop.is_running():
                    loop.call_soon_threadsafe(
                        loop.create_task,
                        broadcast({"type": "disk_full",
                                   "msg": "⛔ DISK FULL — sequence halted. Free space on destination drive."})
                    )
            threading.Thread(target=_alert, daemon=True).start()


def reset_session():
    """Wipe persisted session and reload defaults."""
    if SESSION_FILE.exists():
        try:
            SESSION_FILE.unlink()
        except Exception as e:
            logger.error(f"Permission denied to delete SESSION_FILE: {e}")
    for k, v in _DEFAULTS.items():
        if k not in ("stop_event", "aux_triggered"):
            state[k] = v
    hg.set_settings(HGSettings(continuous_shutter=(state.get("active_camera", "picam") == "picam")))
    pan_axis.current_deg   = 0.0
    tilt_axis.current_deg  = 0.0
    slider_axis.current_mm = 0.0
    slider_axis.current_steps = 0  # ← Reset step counter when resetting session
    engine.clear_keyframes()
    logger.info("Session reset to defaults.")


def _keyframes_to_list() -> list:
    """Serialise ProgrammedMove keyframes for the WebSocket client."""
    if not _prog_move:
        return []
    return [
        {
            "index":      i,
            "slider_mm":  round(kf.slider_mm, 3),
            "pan_deg":    round(kf.pan_deg,   3),
            "tilt_deg":   round(kf.tilt_deg,  3),
            "duration_s": kf.duration_s,
            "easing":     kf.easing,
        }
        for i, kf in enumerate(_prog_move.keyframes)
    ]


def sync_all_keyframes():
    """
    Synchronises points across all modes:
    1. Cinematic (ProgrammedMove) -> Timelapse (MotionEngine)
    2. Cinematic (ProgrammedMove) -> Macro (State variables)
    """
    if not _prog_move:
        return

    # 1. Sync Cinematic -> Timelapse
    engine.clear_keyframes()
    for kf in _prog_move.keyframes:
        engine.add_keyframe(kf.slider_mm, kf.pan_deg, kf.tilt_deg)

    # 2. Sync Cinematic -> Macro endpoints (convert mm to steps)
    # IMPORTANT: kf.slider_mm is in belt-drive mm (slider_axis.current_mm units, 50 steps/mm).
    # Use slider_axis.steps_per_mm (50.0) NOT STEPS_PER_MM (800.0) for the conversion.
    # belt_mm × 50 = actual motor steps = same unit as slider_axis.current_steps.
    if len(_prog_move.keyframes) >= 1:
        kf0 = _prog_move.keyframes[0]
        state["macro_rail_start_steps"] = int(kf0.slider_mm * slider_axis.steps_per_mm)
        state["macro_rotation_start_deg"] = kf0.pan_deg
        state["macro_aux_start_deg"] = kf0.tilt_deg

    if len(_prog_move.keyframes) >= 2:
        kfn = _prog_move.keyframes[-1]
        state["macro_rail_end_steps"] = int(kfn.slider_mm * slider_axis.steps_per_mm)
        state["macro_rotation_end_deg"] = kfn.pan_deg
        state["macro_aux_end_deg"] = kfn.tilt_deg

    save_session()
    # Invalidate timelapse trajectory cache
    if hasattr(timelapse_worker, '_traj_cache'):
        del timelapse_worker._traj_cache


async def broadcast_points():
    """Notify all clients of the updated state of points/keyframes."""
    await broadcast({"type": "cinematic_keyframes", "keyframes": _keyframes_to_list(),
                     "reversed": bool(state.get("_move_reversed", False))})
    await broadcast({"type": "status", "nodes": len(engine.keyframes)})
    # Convert steps back to mm for UI display
    rail_start_steps = state.get("macro_rail_start_steps", 0)
    rail_end_steps = state.get("macro_rail_end_steps", 0)
    await broadcast({
        "type": "macro_points_updated",
        "rail_start_steps": rail_start_steps,
        "rail_end_steps": rail_end_steps,
        "rail_start_mm": rail_start_steps / STEPS_PER_MM,
        "rail_end_mm": rail_end_steps / STEPS_PER_MM,
        "pan_start":  state.get("macro_rotation_start_deg", 0.0),
        "pan_end":    state.get("macro_rotation_end_deg", 0.0)
    })


def _build_macro_session(msg: dict) -> "MacroSession":
    """
    Construct a MacroSession from a WebSocket macro_start message.
    Falls back to persisted state values where fields are absent.
    """
    # Slots
    raw_slots = msg.get("slots", [{}])
    slots = []
    for i, rs in enumerate(raw_slots):
        slots.append(ExposureSlot(
            id               = rs.get("id",    f"slot_{chr(65+i)}"),
            label            = rs.get("label", f"slot {i+1}"),
            enabled          = bool(rs.get("enabled", True)),
            relay1           = bool(rs.get("relay1",  False)),
            relay2           = bool(rs.get("relay2",  False)),
            relay_settle_ms  = int(rs.get("relay_settle_ms",  0)),
            relay_release_ms = int(rs.get("relay_release_ms", 0)),
            iso              = int(rs.get("iso",     400)),
            shutter_s        = float(rs.get("shutter_s", 1/125)),
            kelvin           = int(rs.get("kelvin",  5500)),
            ae               = bool(rs.get("ae",  False)),
            awb              = bool(rs.get("awb", False)),
        ))

    # Lens profile
    lp = msg.get("lens_profile", state.get("macro_lens_profile", {}))
    lens = LensProfile(
        name                = lp.get("name",                "unknown"),
        lens_type           = lp.get("lens_type",           "macro"),
        magnification       = float(lp.get("magnification", 1.0)),
        working_distance_mm = float(lp.get("working_distance_mm", 0.0)),
        sensor_pixel_um     = float(lp.get("sensor_pixel_um",     3.92)),
        image_width_px      = int(lp.get("image_width_px",   6000)),
        image_height_px     = int(lp.get("image_height_px",  4000)),
        notes               = lp.get("notes",               ""),
    )

    # Use absolute step tracking from home position.
    # Prefer the step values already stored in state by macro_set_rail_start/end handlers
    # (which are set when user positions the rail and presses Set Start/End).
    # Fall back to mm→steps conversion only if step values haven't been set yet.
    images = int(msg.get("images_per_stack", 9))

    stored_start_steps = state.get("macro_rail_start_steps")
    stored_end_steps   = state.get("macro_rail_end_steps")

    if stored_start_steps is not None and stored_end_steps is not None:
        # Use already-precise step values from hardware (set via gamepad nudge)
        rail_start_steps = int(stored_start_steps)
        rail_end_steps   = int(stored_end_steps)
        rail_start_mm    = rail_start_steps / STEPS_PER_MM
        rail_end_mm      = rail_end_steps   / STEPS_PER_MM
        logger.info(f"Rail macro: using stored steps {rail_start_steps}→{rail_end_steps} "
                   f"({rail_start_mm:.3f}→{rail_end_mm:.3f} mm)")
    else:
        # Fall back: convert mm from message/state to steps
        rail_start_mm    = float(msg.get("rail_start_mm", state.get("macro_rail_start_mm", 0.0)))
        rail_end_mm      = float(msg.get("rail_end_mm",   state.get("macro_rail_end_mm",   5.0)))
        rail_start_steps = int(rail_start_mm * STEPS_PER_MM)
        rail_end_steps   = int(rail_end_mm   * STEPS_PER_MM)
        logger.warning(f"Rail macro: no stored steps — converting from mm: "
                      f"{rail_start_mm:.3f}→{rail_end_mm:.3f} mm → "
                      f"{rail_start_steps}→{rail_end_steps} steps")

    # Calculate step increment for this stack sequence
    travel_steps = abs(rail_end_steps - rail_start_steps)
    if images > 1:
        step_increment = travel_steps // (images - 1)  # Integer division for precise steps
        logger.info(f"Rail macro: {rail_start_mm:.1f}→{rail_end_mm:.1f}mm → "
                   f"{rail_start_steps}→{rail_end_steps} steps, {step_increment} steps per image ({images} images)")
    else:
        step_increment = 0
        logger.warning(f"Rail macro: Only {images} image(s) — no stepping needed")

    return MacroSession(
        project_name         = msg.get("project_name",    "macro_project"),
        orbit_label          = msg.get("orbit_label",     "orbit_001"),
        session_mode         = msg.get("session_mode",    "scan"),
        # Scan geometry
        scan_type            = msg.get("scan_type",        "orbit"),
        pan_cols             = int(msg.get("pan_cols",      4)),
        tilt_rows            = int(msg.get("tilt_rows",     3)),
        grid_snake           = bool(msg.get("grid_snake",   True)),
        pan_axis_tilt_deg    = float(msg.get("pan_axis_tilt_deg", 90.0)),
        orbit_number         = int(msg.get("orbit_number",   1)),
        orbit_notes          = msg.get("orbit_notes",        ""),
        use_lego_mount       = bool(msg.get("use_lego_mount",  False)),
        lego_rotation_deg    = float(msg.get("lego_rotation_deg", 0.0)),
        lego_block           = msg.get("lego_block",           "4x4"),
        lego_rotation_axis_offset_mm = float(msg.get("lego_rotation_axis_offset_mm", 0.0)),
        # Rail (focus depth stack) — absolute steps from home position
        rail_start_steps     = rail_start_steps,
        rail_end_steps       = rail_end_steps,
        images_per_stack     = images,
        step_increment_steps = step_increment,
        # Rotation / orbit
        rotation_mode        = msg.get("rotation_mode",           "full"),
        rotation_start_deg   = float(msg.get("rotation_start_deg",
                                state.get("macro_rotation_start_deg", 0.0))),
        rotation_end_deg     = float(msg.get("rotation_end_deg",
                                state.get("macro_rotation_end_deg", 360.0))),
        num_stacks           = int(msg.get("num_stacks",           36)),
        rotation_easing      = msg.get("rotation_easing",         "even"),
        rotation_axis_angle_deg = float(msg.get("rotation_axis_angle_deg", 90.0)),
        rotation_axis_description = msg.get("rotation_axis_description",   "vertical"),
        # Aux / tilt axis (geodesic 2D grid)
        aux_enabled          = bool(msg.get("aux_enabled",
                                    msg.get("scan_type") == "grid_2d")),
        aux_label            = msg.get("aux_label",                "tilt"),
        aux_start_deg        = float(msg.get("aux_start_deg",
                                state.get("macro_aux_start_deg", 0.0))),
        aux_end_deg          = float(msg.get("aux_end_deg",
                                state.get("macro_aux_end_deg", 0.0))),
        aux_easing           = msg.get("aux_easing",               "even"),
        # Stereo 3D capture (for VR and 3D video)
        stereo_enabled       = bool(msg.get("stereo_enabled",      False)),
        stereo_offset_deg    = float(msg.get("stereo_offset_deg",  3.0)),
        # Timing
        vibe_delay_s         = float(msg.get("vibe_delay_s",       0.5)),
        exp_margin_s         = float(msg.get("exp_margin_s",       0.2)),
        active_camera        = state.get("active_camera",          "picam"),
        lens                 = lens,
        save_path            = msg.get("save_path", state.get("save_path",
                                       str(Path.home() / "Pictures/PiSlider"))),
        slots                = slots,
    )


state = _load_session()


# ─── AUX GPIO INTERRUPT ───────────────────────────────────────────────────────
def _setup_aux_trigger():
    try:
        import lgpio
        def _cb(chip, gpio, level, tick):
            if level == 0:
                state["aux_triggered"] = True
                logger.info("AUX: GPIO trigger fired.")
        lgpio.gpio_claim_alert(hw.gpio_chip, 13, lgpio.FALLING_EDGE)
        lgpio.callback(hw.gpio_chip, 13, lgpio.FALLING_EDGE, _cb)
    except Exception as e:
        logger.warning(f"AUX GPIO setup skipped: {e}")

_setup_aux_trigger()


# ─── PICAM SETTINGS ───────────────────────────────────────────────────────────
def apply_picam_settings():
    if not _HAS_PICAM or not picam:
        return
    try:
        controls = {"AeEnable": state["picam_ae"], "AwbEnable": state["picam_awb"]}
        if not state["picam_ae"]:
            controls["ExposureTime"] = int(state["picam_shutter_s"] * 1_000_000)
            controls["AnalogueGain"] = state["picam_iso"] / 100.0
        if not state["picam_awb"]:
            controls["ColourTemperature"] = int(state["picam_kelvin"])
        picam.set_controls(controls)
    except Exception as e:
        logger.error(f"apply_picam_settings: {e}")


_last_hg_params: Optional[dict] = None   # cache for post-capture re-application

def apply_picam_from_hg(params: dict):
    """
    Push HG-computed exposure to picamera2.
    Caches params so _reapply_hg_after_capture() can immediately restore
    manual control after switch_mode_and_capture_file resets AE/AWB to auto.
    """
    global _last_hg_params
    if not _HAS_PICAM or not picam:
        return
    try:
        # Apply preview-adjusted controls (gain-boosted for long-shutter nights so
        # the live view stays bright even when capture shutter is 25s+).
        # The DNG capture uses the FULL shutter/ISO via _capture_controls_from_hg()
        # passed directly to switch_mode_and_capture_file() in capture_picam().
        picam.set_controls(_preview_controls_from_hg(params))
        _last_hg_params = params
    except Exception as e:
        logger.error(f"apply_picam_from_hg: {e}")


# Maximum preview exposure — picamera2 preview stream can't go slower than
# this without dropping to <1fps and becoming useless as a live view.
# When HG requests longer shutters, we boost AnalogueGain instead so the
# preview approximately matches the brightness of the actual captured frames.
_PREVIEW_MAX_SHUTTER_S = 0.25   # 4fps minimum preview rate


def _preview_controls_from_hg(params: dict) -> dict:
    """
    Compute picam controls for the PREVIEW stream when HG is active.

    The preview stream cannot run at shutter speeds > ~0.25s (it would drop
    below 4fps and feel broken as a live view). When HG requests a longer
    exposure (e.g. 1s at night), we cap the preview shutter and boost gain
    proportionally so the live view brightness approximately matches the
    actual DNG captures. This is display-only — the stills always use the
    full HG shutter/ISO.
    """
    shutter_s = params["shutter_s"]
    iso       = params["iso"]

    if shutter_s <= _PREVIEW_MAX_SHUTTER_S:
        # Short enough — preview matches stills exactly
        preview_shutter = shutter_s
        preview_gain    = iso / 100.0
    else:
        # Boost gain to compensate for capped preview shutter
        ratio           = shutter_s / _PREVIEW_MAX_SHUTTER_S
        preview_shutter = _PREVIEW_MAX_SHUTTER_S
        preview_gain    = min((iso / 100.0) * ratio, 64.0)   # cap at ~ISO 6400 equiv

    return {
        "AeEnable":          False,
        "AwbEnable":         False,
        "ExposureTime":      int(preview_shutter * 1_000_000),
        "AnalogueGain":      preview_gain,
        "ColourTemperature": int(params["kelvin"]),
    }


def _reapply_hg_after_capture():
    """
    Re-apply the last HG controls immediately after switch_mode_and_capture_file
    returns.  picamera2 resets AeEnable/AwbEnable to True on every reconfigure,
    so without this the preview runs in full auto for up to one full interval
    before the next apply_picam_from_hg call.  Called inside capture_picam()
    on the thread-pool thread, right after the DNG is saved.
    """
    if not _HAS_PICAM or not picam or not _last_hg_params:
        return
    try:
        picam.set_controls(_preview_controls_from_hg(_last_hg_params))
    except Exception as e:
        logger.warning(f"_reapply_hg_after_capture: {e}")




# ─── HG CALIBRATION SHOT ──────────────────────────────────────────────────────
def _capture_cal_frame(dest_path: str) -> Optional[str]:
    """
    Capture a single calibration DNG to dest_path.
    Uses the current picam HG settings (already applied before this call).
    Returns saved path or None on failure. Does NOT update _latest_shot or
    push to HG tracker — the caller handles that.
    """
    if not _HAS_PICAM or not picam:
        return None
    try:
        still_cfg = picam.create_still_configuration(
            main={"size": (4056, 3040), "format": "RGB888"},
            raw={}
        )
        picam.switch_mode_and_capture_file(still_cfg, dest_path, name="raw")
        if hg.settings.enabled:
            _reapply_hg_after_capture()
        return dest_path if os.path.exists(dest_path) else None
    except Exception as e:
        logger.error(f"Cal frame capture failed: {e}")
        return None


def _load_cal_frame_rgb(path: str) -> Optional[object]:
    """
    Load a DNG cal frame as an RGB numpy array for the HG sky analyser.
    Uses PIL — works for DNG (TIFF) and JPEG. Returns None on failure.
    """
    try:
        from PIL import Image
        import numpy as np
        with Image.open(path) as img:
            rgb = img.convert("RGB")
            # Downsample for speed — sky analyser doesn't need full 12MP
            rgb = rgb.resize((1280, 960), Image.LANCZOS)
            return np.array(rgb)
    except Exception as e:
        logger.debug(f"Cal frame load failed: {e}")
        return None



def hg_calibration_shot() -> Optional[float]:
    """
    Take an AE/AWB-ON exposure to measure ambient EV, then immediately
    lock the camera back to manual control using the computed values.

    KEY FIX: waits for picam2 AE to genuinely converge before reading
    metadata.  3 frames at 30fps = 100ms — not enough for a bright snow
    scene where AE starts slow and needs 1-2 seconds to settle.
    We poll until ExposureTime stabilises (< 5% change over 3 consecutive
    frames) or we time out at 3 seconds.

    NIGHT COLD-START: when sun_alt < -6°, AE cannot properly expose a dark
    scene (preview shutter too short — typically 0.25s max at preview fps).
    The AE result would be a nearly-black image that anchors ev_smooth at
    the wrong value (producing 1s/ISO100 forever instead of 25s/ISO6400).
    In this case we skip AE entirely and leave anchor_ev=None so the
    no-anchor night path in holygrail._compute_params() computes the correct
    starting exposure from shutter_max_night + iso_max_night.
    """
    global _last_hg_params
    if not _HAS_PICAM or not picam:
        return None
    try:
        # ── Night cold-start: skip AE, no anchor ─────────────────────────────
        try:
            from astral.sun import elevation as _sun_el_pre
            _sun_alt_pre = _sun_el_pre(hg._location.observer,
                                       datetime.datetime.now(hg._tzinfo))
        except Exception:
            _sun_alt_pre = 0.0

        if _sun_alt_pre < -6.0:
            # AE is unreliable at night — preview frame rate limits shutter to
            # ~0.25s, producing a nearly-black image and a wrong anchor_ev.
            # Leave anchor_ev=None so holygrail uses the no-anchor night path
            # which computes ev_smooth from shutter_max_night + iso_max_night.
            logger.info(
                "HG Cal: night cold-start (sun_alt=%.1f°) — "
                "AE skipped. No-anchor path will use hardware night limits.",
                _sun_alt_pre
            )
            hg.settings.anchor_shutter_s = None
            hg.settings.anchor_iso       = None
            hg.settings.anchor_ev        = None
            # Seed kelvin only (no ev) — _last_ev stays None so
            # smooth_ev() on the first frame initialises correctly.
            kelvin_night = float(hg._kelvin_for_phase(_sun_alt_pre))
            hg._tracker._last_kelvin = kelvin_night
            # Lock camera to reasonable starting point (short preview shutter,
            # high ISO) so the preview stream is usable while calibration runs.
            try:
                picam.set_controls({
                    "AeEnable":    False,
                    "AwbEnable":   False,
                    "ExposureTime": 250_000,        # 0.25s — fast preview
                    "AnalogueGain": hg.settings.iso_max_night / 100.0,
                    "ColourTemperature": int(kelvin_night),
                })
            except Exception as _e:
                logger.debug(f"HG Cal night lock preview: {_e}")
            return None   # sentinel: no anchor EV; no-anchor path handles it

        picam.set_controls({"AeEnable": True, "AwbEnable": True})

        # ── Wait for AE to converge ───────────────────────────────────────────
        # Poll metadata until ExposureTime is stable for 3 consecutive frames
        # or we've waited 3 seconds.  This handles bright snow / dark scenes
        # where the sensor starts far from the correct exposure.
        prev_exp_us  = None
        stable_count = 0
        deadline     = time.time() + 3.0
        frame        = None
        metadata     = None

        while time.time() < deadline:
            frame    = picam.capture_array()
            metadata = picam.capture_metadata()
            exp_us   = metadata.get("ExposureTime", 0)

            if prev_exp_us is not None and prev_exp_us > 0:
                ratio = abs(exp_us - prev_exp_us) / prev_exp_us
                if ratio < 0.05:   # < 5% change = stable
                    stable_count += 1
                    if stable_count >= 3:
                        break
                else:
                    stable_count = 0
            prev_exp_us = exp_us

        exp_us    = metadata.get("ExposureTime", 1_000_000 // 125)
        gain      = metadata.get("AnalogueGain", 1.0)
        shutter_s = exp_us / 1_000_000
        iso_equiv = int(gain * 100)

        # ── EV from pixels, NOT from aperture math ────────────────────────────
        # anchor_ev MUST be on the same scale as the tracker's pixel-based EV.
        # The tracker computes: ev = log2(lum_linear / 0.18) + 12
        # where lum_linear = (lum_8bit/255)^2.2
        #
        # Using aperture in the EV formula (log2(N²/t) - log2(ISO/100)) gives
        # a "camera EV" that differs from pixel EV by 2*log2(aperture) stops.
        # For unknown apertures (PiCam fixed lens, manual glass) this creates
        # a systematic offset that makes the delta system over- or under-expose
        # by a fixed amount on every single frame.
        #
        # Solution: compute anchor_ev the same way _meter() does — from the
        # actual pixel luminance of the converged AE frame.
        import numpy as _np
        frame_arr = _np.array(frame, dtype=float)
        if frame_arr.ndim == 3:
            lum_arr = (0.2126 * frame_arr[:,:,0]
                     + 0.7152 * frame_arr[:,:,1]
                     + 0.0722 * frame_arr[:,:,2])
        else:
            lum_arr = frame_arr
        # Exclude blown highlights (top 10%) — snow scenes clip easily
        hi = float(_np.percentile(lum_arr, 90))
        lo = float(_np.percentile(lum_arr,  5))
        valid = (lum_arr >= lo) & (lum_arr <= hi) & (lum_arr > 0)
        lum_mean_cal = float(_np.mean(lum_arr[valid])) if _np.any(valid) else 128.0
        lum_linear   = (max(lum_mean_cal, 1.0) / 255.0) ** 2.2
        ev_measured  = math.log2(max(lum_linear, 1e-6) / 0.18) + 12.0

        logger.info(
            f"HG Cal: AE settled SS={shutter_s:.4f}s ISO={iso_equiv} "
            f"lum={lum_mean_cal:.1f} → pixel-EV={ev_measured:.2f} "
            f"(aperture NOT used — pixel scale matches tracker)"
        )

        # ── Kelvin seed for calibration ───────────────────────────────────────
        # At night, pixel-derived Kelvin from the preview frame is polluted by
        # artificial light (streetlamps, sodium, LED) and can't be trusted.
        # Seed directly from the user's configured kelvin_night target instead,
        # so WB starts at the right value from frame 1 rather than slowly
        # crawling down from a wrong 5500K+ reading over many frames.
        try:
            from astral.sun import elevation as _sun_el_cal
            _sun_alt_cal = _sun_el_cal(hg._location.observer,
                                       datetime.datetime.now(hg._tzinfo))
        except Exception:
            _sun_alt_cal = 0.0

        if _sun_alt_cal < 0:
            # Night/twilight — use configured target directly
            kelvin_measured = float(hg._kelvin_for_phase(_sun_alt_cal))
        else:
            # Day/golden — pixel ratios are reliable
            from holygrail import SkyAnalyser, _rg_bg_to_kelvin
            analyser = SkyAnalyser()
            m = analyser.analyse(frame, cam_alt=hg.settings.cam_alt)
            kelvin_measured = (_rg_bg_to_kelvin(m.rg_ratio, m.bg_ratio, m.lum_mean)
                               if m else float(hg.settings.kelvin_day))

        logger.info(
            f"HG Cal shot: SS={shutter_s:.4f}s ISO={iso_equiv} "
            f"pixel-EV={ev_measured:.2f} K={kelvin_measured}"
        )

        hg.seed_from_calibration(ev_measured, kelvin_measured)
        hg.push_capture_frame(frame)

        # ── Set anchor exposure — this is the source of truth ─────────────────
        # From this point _ev_to_exposure works in delta space relative to
        # this frame. Aperture is never used again — unknown lenses work correctly.
        hg.settings.anchor_shutter_s = shutter_s
        hg.settings.anchor_iso       = iso_equiv
        hg.settings.anchor_ev        = ev_measured
        logger.info(
            f"HG anchor set: {shutter_s:.4f}s ISO{iso_equiv} EV{ev_measured:.2f} — "
            f"aperture-free delta mode active."
        )

        # ── Immediately lock camera to manual with calibrated values ──────────
        lock_params = {
            "shutter_s": shutter_s,
            "iso":       iso_equiv,
            "kelvin":    kelvin_measured,
        }
        try:
            picam.set_controls({
                "AeEnable":          False,
                "AwbEnable":         False,
                "ExposureTime":      int(shutter_s * 1_000_000),
                "AnalogueGain":      iso_equiv / 100.0,
                "ColourTemperature": kelvin_measured,
            })
            _last_hg_params = lock_params
            logger.info("HG Cal: camera locked to manual control.")
        except Exception as e:
            logger.warning(f"HG Cal: could not lock controls: {e}")

        return ev_measured

    except Exception as e:
        logger.error(f"hg_calibration_shot: {e}")
        return None


# ─── XMP SIDECAR ──────────────────────────────────────────────────────────────
def _read_aperture_from_exif(path: str) -> Optional[float]:
    """
    Read FNumber from EXIF of a DNG, ARW, or JPEG file.
    DNG and ARW are both TIFF-based — PIL handles them directly.

    Returns f-stop as float (e.g. 2.8, 5.6) or None if not present.
    None means: manual/unknown lens, stay in anchor-delta mode.
    """
    try:
        from PIL import Image
        with Image.open(path) as img:
            exif = img.getexif()
            if not exif:
                return None
            # Tag 33437 = FNumber (preferred — direct f-stop rational)
            fnumber = exif.get(33437)
            if fnumber is not None:
                v = float(fnumber)
                if v > 0:
                    logger.info(f"EXIF FNumber from {os.path.basename(path)}: f/{v}")
                    return v
            # Fallback: tag 37378 = ApertureValue (APEX stops)
            # f-stop = 2^(ApertureValue/2)
            apex = exif.get(37378)
            if apex is not None:
                v = float(apex)
                fstop = round(2 ** (v / 2), 1)
                if fstop > 0:
                    logger.info(
                        f"EXIF ApertureValue {v:.2f} APEX → f/{fstop} "
                        f"from {os.path.basename(path)}"
                    )
                    return fstop
    except Exception as e:
        logger.debug(f"EXIF aperture read failed for {path}: {e}")
    return None



# Sensor dimensions (width × height in mm) per camera type
_SENSOR_DIMS: dict[str, tuple[float, float]] = {
    "sony":     (35.6, 23.8),   # Sony A7III full-frame
    "sony_s2":  (35.6, 23.8),
    "sony_usb": (35.6, 23.8),
    "picam":    (6.287, 4.712), # IMX477 High Quality Camera
}
_SENSOR_DIMS_DEFAULT = (35.6, 23.8)


def _compute_fov(focal_mm: float, camera: str, orientation: str) -> tuple[float, float]:
    """
    Compute HFOV and VFOV in degrees from focal length and sensor dimensions.
    Returns (hfov_deg, vfov_deg) already swapped for portrait orientation.
    """
    import math as _m
    w_mm, h_mm = _SENSOR_DIMS.get(camera, _SENSOR_DIMS_DEFAULT)
    hfov = 2 * _m.degrees(_m.atan(w_mm / (2 * focal_mm)))
    vfov = 2 * _m.degrees(_m.atan(h_mm / (2 * focal_mm)))
    # Portrait: sensor rotated 90° — the long edge is now vertical
    if orientation in ("portrait_cw", "portrait_ccw"):
        hfov, vfov = vfov, hfov
    return round(hfov, 1), round(vfov, 1)


def _read_focal_from_exif(path: str) -> tuple[Optional[float], Optional[str]]:
    """
    Read FocalLength (mm) and LensModel from a RAW/DNG/ARW/JPEG via ExifTool.
    Returns (focal_mm, lens_model) — either value may be None if not present.

    Works for:
    - Native electronic lenses: focal length from EXIF FocalLength tag
    - Manual lenses with Sony IBIS focal length set: Sony stores the in-body
      focal length setting in standard EXIF FocalLength, ExifTool reads it correctly
    - PiCam DNGs: focal length from embedded camera module data
    """
    try:
        result = subprocess.run(
            ["exiftool", "-FocalLength", "-LensModel", "-json", path],
            capture_output=True, text=True, timeout=8
        )
        if result.returncode != 0:
            return None, None
        data = json.loads(result.stdout)
        if not data:
            return None, None
        entry      = data[0]
        lens_model = entry.get("LensModel") or None
        fl_raw     = entry.get("FocalLength")   # e.g. "85.0 mm" or 85.0
        focal_mm   = None
        if fl_raw is not None:
            try:
                # ExifTool may return "85.0 mm" (string) or 85.0 (number)
                focal_mm = float(str(fl_raw).replace("mm", "").strip())
                if focal_mm <= 0:
                    focal_mm = None
            except (ValueError, TypeError):
                focal_mm = None
        return focal_mm, lens_model
    except FileNotFoundError:
        logger.debug("exiftool not found — focal length auto-detect unavailable")
        return None, None
    except Exception as e:
        logger.debug(f"Focal length EXIF read failed for {path}: {e}")
        return None, None



def write_sidecar(dng_path: str, params: dict):
    import math as _math
    base, _ = os.path.splitext(dng_path)
    xmp_path = base + ".xmp"
    world_az = (state.get("origin_az", 0.0) + pan_axis.current_deg) % 360

    # ── crs:Exposure2012 — Lightroom exposure correction in stops ─────────────
    # Combines two sources of error:
    #
    # 1. HG quantisation error (ev_sidecar_error): the HolyGrail requested
    #    ev_final but the camera's discrete ISO/shutter steps couldn't hit it
    #    exactly. ev_sidecar_error = actual_camera_ev - ev_final (from
    #    _ev_to_exposure). We invert it: if camera gave +0.1 EV too much,
    #    pull back −0.1 in Lightroom.
    #
    # 2. Bulb timing error (_bulb_ev_error): WiFi jitter means the shutter
    #    may have stayed open slightly longer than intended.
    #    _bulb_ev_error = log2(actual_s / intended_s), positive = overexposed.
    #    We invert it to pull back in post.
    #
    # Combined: crs_exposure = −(ev_sidecar_error + bulb_ev_error)
    ev_sidecar_error = float(params.get("ev_sidecar_error", 0.0))
    bulb_ev_error    = float(params.get("_bulb_ev_error", 0.0))
    crs_exposure     = round(-(ev_sidecar_error + bulb_ev_error), 4)

    # Metadata tags for the bulb capture
    bulb_flag   = "true" if abs(bulb_ev_error) > 0.0 else "false"
    bulb_err_s  = f"{bulb_ev_error:+.4f}"

    xmp = f"""<?xpacket begin='' id='W5M0MpCehiHzreSzNTczkc9d'?>
<x:xmpmeta xmlns:x='adobe:ns:meta/'>
  <rdf:RDF xmlns:rdf='http://www.w3.org/1999/02/22-rdf-syntax-ns#'>
    <rdf:Description rdf:about=''
      xmlns:ps='http://ns.pislider.io/1.0/'
      xmlns:crs='http://ns.adobe.com/camera-raw-settings/1.0/'>
      <ps:HG_Mode>{params.get('mode','manual')}</ps:HG_Mode>
      <ps:HG_Phase>{params.get('phase','')}</ps:HG_Phase>
      <ps:HG_EV_Target>{params.get('ev_target',0):.3f}</ps:HG_EV_Target>
      <ps:HG_EV_Final>{params.get('ev_final',0):.3f}</ps:HG_EV_Final>
      <ps:HG_ISO>{params.get('iso',0)}</ps:HG_ISO>
      <ps:HG_Shutter>{params.get('shutter','')}</ps:HG_Shutter>
      <ps:HG_Kelvin>{params.get('kelvin',0)}</ps:HG_Kelvin>
      <ps:HG_EV_SidecarError>{ev_sidecar_error:.4f}</ps:HG_EV_SidecarError>
      <ps:Bulb_Mode>{bulb_flag}</ps:Bulb_Mode>
      <ps:Bulb_EV_Error>{bulb_err_s}</ps:Bulb_EV_Error>
      <ps:Sun_Alt>{params.get('sun_alt',0):.4f}</ps:Sun_Alt>
      <ps:Sun_Az>{params.get('sun_az',0):.4f}</ps:Sun_Az>
      <ps:Moon_Alt>{params.get('moon_alt',0):.4f}</ps:Moon_Alt>
      <ps:Moon_Az>{params.get('moon_az',0):.4f}</ps:Moon_Az>
      <ps:Moon_Phase>{params.get('moon_phase',0):.4f}</ps:Moon_Phase>
      <ps:Rig_Pan_Deg>{pan_axis.current_deg:.3f}</ps:Rig_Pan_Deg>
      <ps:Rig_Tilt_Deg>{tilt_axis.current_deg:.3f}</ps:Rig_Tilt_Deg>
      <ps:Rig_Slider_MM>{slider_axis.current_mm:.3f}</ps:Rig_Slider_MM>
      <ps:Rig_World_Az>{world_az:.3f}</ps:Rig_World_Az>
      <crs:WhiteBalance>Custom</crs:WhiteBalance>
      <crs:Temperature>{params.get('kelvin',5500)}</crs:Temperature>
      <crs:Exposure2012>{crs_exposure}</crs:Exposure2012>
    </rdf:Description>
  </rdf:RDF>
</x:xmpmeta>
<?xpacket end='w'?>"""
    try:
        with open(xmp_path, "w") as f:
            f.write(xmp)
    except Exception as e:
        logger.error(f"XMP: {e}")


# ─── SONY AUTO-DISCOVERY ──────────────────────────────────────────────────────

SONY_IFACE = "wlan1"   # USB Realtek dongle — dedicated Sony camera interface.
# wlan0 is the internal Broadcom chip used as a hotspot for browser control.
# These two must never be swapped: connecting the Sony on wlan0 would drop
# every browser client; connecting the browser on wlan1 would leave the
# Sony unreachable.

def _wlan1_lock():
    """
    Tell NetworkManager not to auto-connect wlan1 to any profile.

    Call this immediately after joining the Sony camera network so NM won't
    roam to a stronger known network (home WiFi, Starlink, etc.) mid-shoot.
    Safe to call even if wlan1 is already locked.
    """
    try:
        subprocess.run(
            ["nmcli", "device", "set", SONY_IFACE, "autoconnect", "no"],
            capture_output=True, timeout=5)
        logger.info(f"{SONY_IFACE} autoconnect LOCKED — NM will not roam away from Sony")
    except Exception as exc:
        logger.warning(f"_wlan1_lock: {exc}")


def _wlan1_unlock():
    """
    Re-enable NetworkManager auto-connect on wlan1.

    Call this when the Sony session ends so NM resumes normal management
    (e.g. reconnects to Starlink/home WiFi on the next shoot setup).
    """
    try:
        subprocess.run(
            ["nmcli", "device", "set", SONY_IFACE, "autoconnect", "yes"],
            capture_output=True, timeout=5)
        logger.info(f"{SONY_IFACE} autoconnect UNLOCKED — NM resumes normal management")
    except Exception as exc:
        logger.warning(f"_wlan1_unlock: {exc}")


def _check_sony_wlan1() -> dict:
    """
    Check whether wlan1 (USB Realtek dongle) is connected to a Sony camera.

    wlan1 is the dedicated Sony interface. wlan0 is reserved for the Pi
    hotspot that serves the browser control UI — we never touch it here.

    Returns dict: connected (bool), ip (str), ssid (str), iface (str), error (str).
    """
    result = {"connected": False, "ip": "", "ssid": "", "iface": "", "error": ""}
    try:
        # Step 1: check wlan1 specifically
        dev_info = subprocess.run(
            ["nmcli", "-t", "-f", "DEVICE,STATE,CONNECTION", "dev"],
            capture_output=True, text=True, timeout=5)

        sony_iface = ""
        sony_ssid  = ""

        for line in dev_info.stdout.splitlines():
            parts = line.split(":")
            if len(parts) < 3:
                continue
            iface, state_str, ssid = parts[0], parts[1], parts[2]
            if iface != SONY_IFACE:          # only wlan1 — never touch wlan0
                continue
            if state_str != "connected":
                result["error"] = (
                    f"{SONY_IFACE} (Realtek dongle) is not connected. "
                    f"Connect it to the camera's WiFi Direct network (DIRECT-…) first. "
                    f"Use: nmcli device wifi connect \"DIRECT-XXXX\" ifname {SONY_IFACE}")
                return result
            ssid_upper = ssid.upper()
            is_sony = (ssid.startswith("DIRECT-")
                       or "ILCE"  in ssid_upper
                       or "SONY"  in ssid_upper
                       or "DSC-"  in ssid_upper)
            if is_sony:
                sony_iface = iface
                sony_ssid  = ssid
            else:
                result["error"] = (
                    f"{SONY_IFACE} is connected to '{ssid}' — not a Sony camera network. "
                    f"Connect it to the camera's WiFi Direct network (DIRECT-…). "
                    f"Use: nmcli device wifi connect \"DIRECT-XXXX\" ifname {SONY_IFACE}")
                return result
            break

        if not sony_iface:
            result["error"] = (
                f"{SONY_IFACE} (Realtek dongle) not found. "
                f"Check that the USB WiFi adapter is plugged in.")
            return result

        result["ssid"]  = sony_ssid
        result["iface"] = sony_iface

        # Step 2: discover camera IP from the gateway on that interface
        routes = subprocess.run(
            ["ip", "route", "show", "dev", sony_iface],
            capture_output=True, text=True, timeout=5)
        camera_ip = ""
        for line in routes.stdout.splitlines():
            if "default via" in line:
                parts = line.split()
                camera_ip = parts[parts.index("via") + 1]
                break
        # Fallback to previously stored IP if route table has no default gateway
        if not camera_ip:
            camera_ip = state.get("sony_ip", "")

        if not camera_ip:
            result["error"] = (
                f"{SONY_IFACE} is connected to {sony_ssid} "
                f"but no default gateway found — DHCP may not have finished. "
                f"Wait a few seconds and tap Detect again.")
            return result

        # Persist IP as soon as we have it (confirmed Sony SSID)
        state["sony_ip"] = camera_ip
        result["ip"] = camera_ip

        import socket as _sock
        import time   as _time

        # Step 3a: SSDP M-SEARCH — pokes the camera's UPnP service so it wakes
        # its HTTP API before we try to connect.  Sony cameras often won't respond
        # on port 8080 until they've received at least one SSDP discovery probe.
        try:
            _pi_ip = ""
            _addrs = subprocess.run(
                ["ip", "-4", "-o", "addr", "show", sony_iface],
                capture_output=True, text=True, timeout=3)
            for _l in _addrs.stdout.splitlines():
                _tok = _l.split()
                if len(_tok) >= 4:
                    _pi_ip = _tok[3].split("/")[0]
                    break

            _ssdp_msg = (
                "M-SEARCH * HTTP/1.1\r\n"
                "HOST: 239.255.255.250:1900\r\n"
                'MAN: "ssdp:discover"\r\n'
                "MX: 3\r\n"
                "ST: urn:schemas-sony-com:service:ScalarWebAPI:1\r\n"
                "\r\n"
            ).encode()
            _ssdp_sock = _sock.socket(_sock.AF_INET, _sock.SOCK_DGRAM)
            _ssdp_sock.settimeout(3)
            _ssdp_sock.setsockopt(_sock.SOL_SOCKET, _sock.SO_REUSEADDR, 1)
            if _pi_ip:
                _ssdp_sock.bind((_pi_ip, 0))
            _ssdp_sock.sendto(_ssdp_msg, ("239.255.255.250", 1900))
            # Read any response — we don't need to parse it, just let the camera
            # know someone is here.  Timeout is expected if camera ignores SSDP.
            try:
                _ssdp_resp, _ = _ssdp_sock.recvfrom(2048)
                logger.debug(f"Sony SSDP response: {_ssdp_resp[:200]}")
            except _sock.timeout:
                pass
            _ssdp_sock.close()
        except Exception as _ssdp_err:
            logger.debug(f"Sony SSDP probe skipped: {_ssdp_err}")

        # Step 3b: probe known Sony ports with retries + backoff.
        # Port 8080  = Sony HTTP Camera Remote API ("Control with Smartphone" / Smart Remote).
        # Port 10000 = Alternative HTTP port used on some Sony models / firmware versions.
        # Port 15740 = PTP/IP for gphoto2 — only when "PC Remote" is also ON.
        SONY_HTTP_PORTS = (8080, 10000)
        RETRIES = 4
        RETRY_DELAY = 1.5   # seconds between retries

        def _port_open(port, t=2):
            try:
                c = _sock.create_connection((camera_ip, port), timeout=t)
                c.close()
                return True
            except Exception:
                return False

        http_ok  = False
        http_port = 8080
        ptp_ok   = False

        for _attempt in range(RETRIES):
            for _p in SONY_HTTP_PORTS:
                if _port_open(_p):
                    http_ok   = True
                    http_port = _p
                    break
            ptp_ok = _port_open(15740)
            if http_ok or ptp_ok:
                break
            if _attempt < RETRIES - 1:
                logger.debug(
                    f"Sony port probe attempt {_attempt+1}/{RETRIES} — "
                    f"no response yet at {camera_ip}, retrying…")
                _time.sleep(RETRY_DELAY)

        # If still nothing, do a quick broad scan to report what IS open
        open_ports: list = []
        if not http_ok and not ptp_ok:
            SCAN_PORTS = (80, 443, 2345, 8080, 8443, 10000, 15740, 49152)
            open_ports = [p for p in SCAN_PORTS if _port_open(p, t=1)]

        # Camera is "connected" if EITHER protocol is reachable.
        # http_mode=True means HTTP API works; ptp_mode=True means gphoto2 capture works.
        result["connected"]  = http_ok or ptp_ok
        result["http_mode"]  = http_ok
        result["http_port"]  = http_port if http_ok else 8080
        result["ptp_mode"]   = ptp_ok

        if http_ok and not ptp_ok:
            # HTTP API is active but PC Remote / PTP not enabled — that's fine,
            # we'll use the HTTP capture path.
            result["error"] = ""
        elif not http_ok and not ptp_ok:
            result["connected"] = False
            hint = ""
            if open_ports:
                hint = f"  (Open ports found: {open_ports} — camera may use a non-standard port)"
            else:
                hint = (
                    "  The camera is in 'Control with Smartphone' mode — this is correct. "
                    "The HTTP API (port 8080) may need a moment to start after WiFi joins. "
                    "Try tapping Detect again in 5–10 s.  "
                    "If it keeps failing, check: Menu → Network → Ctrl w/ Smartphone → On, "
                    "and make sure the camera screen shows the connection waiting icon (not sleeping)."
                )
            result["error"] = (
                f"{sony_iface} WiFi connected ({sony_ssid}) "
                f"but camera not responding on ports 8080/10000 (HTTP) or 15740 (PTP/IP) "
                f"at {camera_ip} after {RETRIES} attempts.{hint}")

    except Exception as e:
        result["error"] = str(e)

    return result


# ─── STREAMS ──────────────────────────────────────────────────────────────────
def _sony_api(method: str, params=None) -> dict:
    """Call Sony Camera Remote API. Returns parsed JSON dict."""
    ip   = state.get("sony_ip", "")
    port = state.get("sony_http_port", 8080)
    if not ip:
        raise RuntimeError("Sony IP not set")
    resp = requests.post(
        f"http://{ip}:{port}/sony/camera",
        json={"method": method, "params": params or [], "id": 1, "version": "1.0"},
        timeout=8)
    return resp.json()


def _sony_api_system(method: str, params=None) -> dict:
    """Call Sony system service endpoint (setCurrentTime, etc.)."""
    ip   = state.get("sony_ip", "")
    port = state.get("sony_http_port", 8080)
    if not ip:
        raise RuntimeError("Sony IP not set")
    resp = requests.post(
        f"http://{ip}:{port}/sony/system",
        json={"method": method, "params": params or [], "id": 1, "version": "1.0"},
        timeout=8)
    return resp.json()


def _sony_set_current_time():
    """Sync Sony camera clock to Pi's current local time.

    Accurate to ±1 second (Sony API dateTime field has no sub-second
    precision).  Good enough to correlate MP4 creation_time metadata
    with Pi timestamps for clip-to-sidecar matching.  Flash sync gives
    the per-frame accuracy; this just keeps the coarse clock in sync.
    """
    import datetime
    now    = datetime.datetime.now(datetime.timezone.utc).astimezone()
    dt_str = now.strftime("%Y-%m-%dT%H:%M:%S")
    tz_min = int(now.utcoffset().total_seconds() / 60)
    try:
        res = _sony_api_system("setCurrentTime", [{
            "dateTime":             dt_str,
            "timeZoneOffsetMinute": tz_min,
            "dstOffsetMinute":      0,
        }])
        logger.info(f"Sony setCurrentTime → {dt_str} (tz={tz_min:+d}min): {res}")
    except Exception as e:
        logger.warning(f"Sony setCurrentTime failed (non-fatal): {e}")


def _float_to_sony_shutter(shutter_s: float) -> str:
    """Convert a float shutter speed in seconds to Sony string format."""
    _sec_steps = [1, 2, 3, 4, 5, 6, 8, 10, 13, 15, 20, 25, 30]
    _sub_steps = [2, 3, 4, 5, 6, 8, 10, 13, 15, 20, 25, 30, 40, 50, 60,
                  80, 100, 125, 160, 200, 250, 320, 400, 500, 640, 800,
                  1000, 1250, 1600, 2000, 2500, 3200, 4000]
    if shutter_s >= 30:
        return '30"'
    if shutter_s >= 1:
        nearest = min(_sec_steps, key=lambda n: abs(n - shutter_s))
        return f'{nearest}"'
    # sub-second: find nearest 1/N denominator
    denom = 1.0 / shutter_s
    nearest = min(_sub_steps, key=lambda n: abs(n - denom))
    return f"1/{nearest}"


def _float_to_gphoto2_shutter(shutter_s: float) -> str:
    """Convert float shutter speed to gphoto2 string (e.g. '1/125', '2"').
    Sony A7III gphoto2 uses N" for >= 1s and 1/N for sub-second."""
    _sec_steps = [1, 2, 3, 4, 5, 6, 8, 10, 13, 15, 20, 25, 30]
    _sub_steps = [2, 3, 4, 5, 6, 8, 10, 13, 15, 20, 25, 30, 40, 50, 60,
                  80, 100, 125, 160, 200, 250, 320, 400, 500, 640, 800,
                  1000, 1250, 1600, 2000, 2500, 3200, 4000, 5000, 6400, 8000]
    if shutter_s >= 30:
        return '30"'
    if shutter_s >= 1:
        nearest = min(_sec_steps, key=lambda n: abs(n - shutter_s))
        return f'{nearest}"'
    denom = 1.0 / shutter_s
    nearest = min(_sub_steps, key=lambda n: abs(n - denom))
    return f"1/{nearest}"


def detect_sony_usb() -> dict:
    """Run gphoto2 --auto-detect and return first Sony camera found.
    Returns {"found": bool, "model": str, "port": str}."""
    try:
        res = subprocess.run(
            ["gphoto2", "--auto-detect"],
            capture_output=True, text=True, timeout=10
        )
        for line in res.stdout.splitlines():
            lower = line.lower()
            if "sony" in lower and ("usb:" in lower or "usb:" in line):
                # Lines look like:  "Sony Corporation A7M3     usb:001,005"
                parts = line.split()
                port  = parts[-1] if parts else ""
                model = " ".join(parts[:-1]).strip()
                return {"found": True, "model": model, "port": port}
        return {"found": False, "model": "", "port": ""}
    except FileNotFoundError:
        return {"found": False, "model": "", "port": "gphoto2 not installed"}
    except Exception as e:
        logger.warning(f"detect_sony_usb: {e}")
        return {"found": False, "model": "", "port": ""}


_sony_last_ae: bool | None = None  # track last AE mode to avoid redundant setExposureMode calls


def _sony_api_checked(method: str, params=None) -> dict:
    """Call _sony_api and raise RuntimeError if the response contains an error."""
    res = _sony_api(method, params)
    if "error" in res:
        code, msg = res["error"][0], res["error"][1] if len(res["error"]) > 1 else ""
        raise RuntimeError(f"{method} → Sony error {code}: {msg}")
    return res


def set_sony_settings(ae: bool, awb: bool, shutter_s: float, iso: int, kelvin: int):
    """Push exposure settings to Sony camera via Camera Remote API."""
    global _sony_last_ae
    errors = []
    try:
        # Only send setExposureMode when it actually changes — sending it on every
        # slider move can briefly lock the camera and block subsequent API calls.
        if ae != _sony_last_ae:
            try:
                # Pick the manual mode string from what the camera actually supports.
                # Sony cameras report their supported strings via getSupportedExposureMode;
                # common variants are "Manual" and "Manual Exposure".
                if ae:
                    exp_mode = "Program Auto"
                else:
                    supported = state.get("_sony_supported_exposure_modes", [])
                    if "Manual Exposure" in supported:
                        exp_mode = "Manual Exposure"
                    else:
                        exp_mode = "Manual"  # most Sony WiFi cameras use this
                _sony_api_checked("setExposureMode", [exp_mode])
                _sony_last_ae = ae
            except Exception as exc:
                errors.append(str(exc))

        if not ae:
            ss_str = _float_to_sony_shutter(shutter_s)
            try:
                _sony_api_checked("setShutterSpeed", [ss_str])
            except Exception as exc:
                errors.append(str(exc))
            try:
                _sony_api_checked("setIsoSpeedRate", [str(iso)])
            except Exception as exc:
                errors.append(str(exc))

        if awb:
            try:
                _sony_api_checked("setWhiteBalance",
                                  [{"mode": "Auto WB", "colorTemperature": -1}])
            except Exception as exc:
                errors.append(str(exc))
        else:
            try:
                _sony_api_checked("setWhiteBalance",
                                  [{"mode": "Color Temperature",
                                    "colorTemperature": round(kelvin / 100) * 100}])
            except Exception as exc:
                errors.append(str(exc))
    except Exception as exc:
        errors.append(str(exc))

    if errors:
        for e in errors:
            logger.warning(f"set_sony_settings: {e}")
        return errors  # caller may broadcast these
    return []


def set_sony_settings_usb(ae: bool, awb: bool, shutter_s: float, iso: int, kelvin: int):
    """Push exposure settings to Sony camera via gphoto2 USB tethering.
    All --set-config calls are batched into one gphoto2 invocation to
    minimise USB re-enumeration overhead."""
    try:
        args = ["gphoto2"]
        if ae:
            args += ["--set-config", "expprogram=P"]
        else:
            args += [
                "--set-config", "expprogram=M",
                "--set-config", f"shutterspeed={_float_to_gphoto2_shutter(shutter_s)}",
                "--set-config", f"iso={iso}",
            ]
        if awb:
            args += ["--set-config", "whitebalance=Automatic"]
        else:
            # Sony A7III gphoto2: WB "CT" = Color Temperature mode
            args += [
                "--set-config", "whitebalance=CT",
                "--set-config", f"colortemperature={kelvin}",
            ]
        result = subprocess.run(args, capture_output=True, text=True, timeout=10)
        if result.returncode != 0:
            logger.warning(f"set_sony_settings_usb: {result.stderr.strip()}")
    except Exception as exc:
        logger.warning(f"set_sony_settings_usb: {exc}")


def _sony_liveview_worker():
    """
    Background thread — holds ONE persistent connection to the Sony liveview
    stream and updates _sony_last_frame / _latest_shot for every received JPEG.

    Architecture: Sony cameras only allow a single liveview consumer at a time.
    Running this as a thread means multiple /video_feed browser tabs all read
    from the same shared _sony_last_frame buffer (same pattern as PiCam's
    _last_frame).

    Uses a buffer-based parser with requests.iter_content so that:
      • HTTP/1.1 chunked-transfer decoding is handled by urllib3 transparently
      • Partial frames accumulate in `buf` until complete before being parsed
      • Sync loss is recovered byte-by-byte (not 8-bytes at a time)
      • Unknown Sony packet types are properly skipped (not silently ignored
        while the stream pointer sits in the middle of their payload)
    """
    import time as _time
    global _sony_last_frame, _sony_liveview_running, _latest_shot

    RECONNECT_DELAY  = 4.0
    MAX_FRAME_BYTES  = 2 * 1024 * 1024   # 2 MB sanity cap — bail if exceeded

    # ── Inner frame parser ────────────────────────────────────────────────────
    # Works on a running bytes buffer; returns the remaining (unparsed) buffer
    # plus any new JPEG bytes found.  Returns None for the buffer on a fatal
    # framing error (triggers reconnect).
    def _parse_buf(buf, frame_count_ref):
        last_jpeg  = None
        frames_new = 0

        while True:
            # ── Find 0xFF sync byte ──────────────────────────────────────────
            if not buf:
                break
            if buf[0] != 0xFF:
                idx = buf.find(b'\xFF')
                if idx < 0:
                    logger.warning("Sony liveview: no 0xFF in buffer — discarding %d bytes", len(buf))
                    buf = b""
                    break
                if idx > 0:
                    logger.warning("Sony liveview: resync — skipping %d stale bytes", idx)
                buf = buf[idx:]

            if len(buf) < 8:
                break   # need full common header

            ptype = buf[1]

            # ── JPEG payload (type 0x01) ─────────────────────────────────────
            if ptype == 0x01:
                if len(buf) < 136:          # 8 common + 128 payload header
                    break
                # Sony binary protocol: payload size is uint24 BE at payload
                # header bytes 4–6 (stream offsets 12–14). Padding size is 1
                # byte at offset 15.  The next 120 bytes (16–135) are reserved.
                # Sources: tonytonyjan/sonycam PACKET.md, micolous/gst-plugins-
                # sonyalpha FRAMING_FORMAT.md + sonyalphademux.c
                # GST_READ_UINT24_BE(data + 12) confirms the 3-byte read.
                if buf[8:12] != b'\x24\x35\x68\x79':
                    logger.warning("Sony liveview: bad magic %s — resyncing", buf[8:12].hex())
                    idx = buf.find(b'\xFF', 1)
                    if idx < 0:
                        buf = b""
                        break
                    buf = buf[idx:]
                    continue
                size = int.from_bytes(buf[12:15], "big")   # uint24 BE — payload hdr bytes 4-6
                pad  = buf[15]                              # uint8   — payload hdr byte 7
                if size > MAX_FRAME_BYTES:
                    logger.error("Sony liveview: insane JPEG size %d — reconnecting", size)
                    return None, frames_new, last_jpeg      # signal reconnect
                total = 136 + size + pad
                if len(buf) < total:
                    break   # incomplete frame — wait for more data
                last_jpeg   = buf[136:136 + size]
                buf         = buf[total:]
                frames_new += 1
                fc = frame_count_ref[0] + frames_new
                if fc <= 5:
                    logger.info("Sony liveview: frame #%d received (%d bytes)", fc, size)
                elif fc % 50 == 0:
                    logger.info("Sony liveview: %d frames received", fc)

            # ── Frame-info packet (type 0x02) ────────────────────────────────
            elif ptype == 0x02:
                if len(buf) < 16:           # 8 common + 8 payload header
                    break
                size = int.from_bytes(buf[8:12], "big")    # payload hdr bytes 0-3
                pad  = buf[12]                              # payload hdr byte 4
                if size > MAX_FRAME_BYTES:
                    logger.error("Sony liveview: insane frame-info size %d — reconnecting", size)
                    return None, frames_new, last_jpeg
                total = 16 + size + pad
                if len(buf) < total:
                    break
                buf = buf[total:]           # discard frame-info, keep parsing

            # ── Unknown packet type — resync past current byte ───────────────
            else:
                logger.warning("Sony liveview: unknown ptype 0x%02X — resyncing", ptype)
                idx = buf.find(b'\xFF', 1)  # search from byte 1 onward
                if idx < 0:
                    buf = b""
                    break
                buf = buf[idx:]

        return buf, frames_new, last_jpeg

    # ── Outer reconnect loop ──────────────────────────────────────────────────
    while _sony_liveview_running:
        r = None
        try:
            # 1. Ensure camera is in rec mode
            try:
                _sony_api("startRecMode")
            except Exception:
                pass   # already in rec mode — ignore

            # 1b. Log supported exposure modes on first connect — helps diagnose
            #     setExposureMode failures if the camera rejects our mode string.
            if not state.get("_sony_exposure_modes_logged"):
                try:
                    modes_res = _sony_api("getSupportedExposureMode", [])
                    modes = (modes_res.get("result") or [[]])[0]
                    logger.info("Sony supported exposure modes: %s", modes)
                    state["_sony_exposure_modes_logged"] = True
                    state["_sony_supported_exposure_modes"] = modes
                except Exception:
                    pass

            # 2. Get liveview URL (A7III does not support startLiveviewWithSize)
            url = None
            try:
                res = _sony_api("startLiveview", [])
                if res.get("error"):
                    logger.warning("Sony liveview: startLiveview error: %s", res["error"])
                url = (res.get("result") or [None])[0]
                if url:
                    logger.info("Sony liveview: connecting to %s", url)
                else:
                    logger.warning("Sony liveview: no URL in response: %s", res)
            except Exception as exc:
                logger.warning("Sony liveview: startLiveview exception: %s", exc)

            if not url:
                _time.sleep(RECONNECT_DELAY)
                continue

            # 3. Open streaming HTTP connection
            # requests+urllib3 handles HTTP/1.1 chunked-transfer decoding so that
            # iter_content yields clean Sony binary payload bytes.
            r = requests.get(url, stream=True, timeout=(10, 30))
            logger.info("Sony liveview: HTTP %d (content-type: %s)",
                        r.status_code, r.headers.get("content-type", "?"))
            r.raise_for_status()

            # 4. Parse Sony binary frames from the stream
            buf          = b""
            frame_count  = [0]   # mutable so _parse_buf can read it

            for chunk in r.iter_content(chunk_size=8192):
                if not _sony_liveview_running:
                    break
                if not chunk:
                    continue
                buf += chunk

                result = _parse_buf(buf, frame_count)
                if result[0] is None:          # fatal framing error
                    raise RuntimeError("Liveview frame framing error — reconnecting")

                buf, frames_new, last_jpeg = result
                if last_jpeg:
                    _sony_last_frame  = last_jpeg
                    _latest_shot      = last_jpeg
                    frame_count[0]   += frames_new

        except Exception as exc:
            logger.warning("Sony liveview: %s: %s — reconnecting in %.0fs",
                           type(exc).__name__, exc, RECONNECT_DELAY)
        finally:
            if r is not None:
                try:
                    r.close()
                except Exception:
                    pass

        if _sony_liveview_running:
            _time.sleep(RECONNECT_DELAY)

    logger.info("Sony liveview worker: stopped")


def _start_sony_liveview():
    """Start the background Sony liveview thread (idempotent)."""
    import threading as _th
    global _sony_liveview_running, _sony_liveview_thread
    if _sony_liveview_running:
        return
    _sony_liveview_running = True
    _sony_liveview_thread  = _th.Thread(target=_sony_liveview_worker,
                                         name="SonyLiveview", daemon=True)
    _sony_liveview_thread.start()
    logger.info("Sony liveview worker: started")


def _stop_sony_liveview():
    """Signal the background Sony liveview thread to exit."""
    global _sony_liveview_running, _sony_last_frame
    _sony_liveview_running = False
    _sony_last_frame       = None
    logger.info("Sony liveview worker: stop requested")


def _sony_usb_liveview_worker():
    """
    Background thread — captures Sony preview frames via gphoto2 USB and
    feeds _sony_last_frame at ~2-3fps for the MJPEG /video_feed endpoint.

    gphoto2 requires exclusive USB device access per-call, so this worker
    MUST be stopped before any timelapse capture command runs (same pattern
    as the WiFi liveview worker).  _timelapse_worker_inner handles that via
    the _timelapse_paused_liveview flag.
    """
    global _sony_last_frame, _sony_usb_liveview_running
    import time as _t
    logger.info("Sony USB liveview worker started")
    _first_frame = True
    # gphoto2 saves --capture-preview to ./capture_preview_XXXX.jpg in cwd.
    # Run from /tmp so the file is always findable regardless of --filename support.
    _PREVIEW_CWD  = "/tmp"
    _PREVIEW_FILE = "/tmp/capture_preview.jpg"   # gphoto2 default on this build

    while _sony_usb_liveview_running:
        try:
            # Remove stale file so we can detect a fresh write
            try:
                os.unlink(_PREVIEW_FILE)
            except FileNotFoundError:
                pass

            result = subprocess.run(
                ["gphoto2",
                 "--capture-preview",
                 "--filename", _PREVIEW_FILE,
                 "--force-overwrite"],
                capture_output=True, timeout=8,
                cwd=_PREVIEW_CWD
            )

            jpeg = b""

            # Primary: read the file gphoto2 saved (filename varies by build)
            for candidate in (_PREVIEW_FILE,
                              "/tmp/capture_preview_0001.jpg",
                              "/tmp/capture_preview_0000.jpg"):
                try:
                    if os.path.getsize(candidate) > 500:
                        with open(candidate, "rb") as fh:
                            jpeg = fh.read()
                        break
                except OSError:
                    pass

            # Fallback: some gphoto2 builds write JPEG to stdout
            if not jpeg and len(result.stdout) > 500:
                jpeg = result.stdout

            if jpeg:
                _sony_last_frame = jpeg
                if _first_frame:
                    logger.info(
                        f"Sony USB liveview: first frame OK "
                        f"({len(jpeg):,} bytes, rc={result.returncode})")
                    _first_frame = False
                # Successful frame — pace to ~3 fps
                _t.sleep(0.3)
            else:
                stderr_txt = result.stderr.decode(errors='replace').strip()
                logger.warning(
                    f"Sony USB preview: no frame "
                    f"(rc={result.returncode}) stderr={stderr_txt!r}")
                _t.sleep(0.5)

        except subprocess.TimeoutExpired:
            logger.warning("Sony USB preview: gphoto2 timed out (8s)")
            _t.sleep(0.5)
        except Exception as e:
            logger.warning(f"Sony USB preview: {e}")
            _t.sleep(0.5)
    logger.info("Sony USB liveview worker stopped")


def _start_sony_usb_liveview():
    """Start the USB preview worker thread if not already running."""
    global _sony_usb_liveview_running, _sony_usb_liveview_thread
    if _sony_usb_liveview_running:
        return
    _sony_usb_liveview_running = True
    _sony_usb_liveview_thread = _th.Thread(
        target=_sony_usb_liveview_worker, name="SonyUsbLiveview", daemon=True)
    _sony_usb_liveview_thread.start()


def _stop_sony_usb_liveview():
    """Signal the USB liveview worker to exit and clear the frame buffer."""
    global _sony_usb_liveview_running, _sony_last_frame
    _sony_usb_liveview_running = False
    _sony_last_frame = None
    logger.info("Sony USB liveview worker: stop requested")


def _restart_cinematic_liveview():
    """Restart whichever liveview was paused for a cinematic programmed move."""
    global _cinematic_paused_liveview
    if not _cinematic_paused_liveview:
        return
    _cinematic_paused_liveview = False
    cam = state.get("active_camera", "")
    if cam == "sony_usb":
        _start_sony_usb_liveview()
        logger.info("Sony USB liveview restarted after cinematic move")
    elif cam == "sony":
        # Re-enable the WiFi liveview flag — the worker loop restarts itself
        global _sony_liveview_running
        if not _sony_liveview_running:
            _sony_liveview_running = True
            import threading as _threading
            global _sony_liveview_thread
            _sony_liveview_thread = _threading.Thread(
                target=_sony_liveview_worker, name="SonyLiveview", daemon=True)
            _sony_liveview_thread.start()
            logger.info("Sony WiFi liveview restarted after cinematic move")


async def get_sony_liveview():
    """
    Async generator — yields MJPEG frames from the shared _sony_last_frame
    buffer maintained by _sony_liveview_worker().  Multiple browser tabs can
    consume this simultaneously without opening extra camera connections.
    """
    enc = [cv2.IMWRITE_JPEG_QUALITY, 70]
    _connecting_frame = None   # cached placeholder while waiting for first frame

    while True:
        frame = _sony_last_frame
        if frame:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                   + frame + b'\r\n')
        else:
            if _connecting_frame is None:
                img  = np.zeros((360, 640, 3), dtype=np.uint8)
                is_usb = (state.get("preview_camera",
                                    state.get("active_camera")) == "sony_usb")
                if is_usb:
                    if _sony_usb_liveview_running:
                        label = "SONY USB: LOADING PREVIEW..."
                        xpos  = 80
                    else:
                        label = "SONY USB: CLICK LIVEVIEW BUTTON"
                        xpos  = 30
                else:
                    label = "SONY: CONNECTING..."
                    xpos  = 130
                cv2.putText(img, label, (xpos, 180),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 200, 200), 2)
                _, buf = cv2.imencode('.jpg', img, enc)
                _connecting_frame = buf.tobytes()
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                   + _connecting_frame + b'\r\n')
        await asyncio.sleep(0.05)   # ~20 fps (actual rate limited by USB preview worker)


def meter_sony_from_shot() -> None:
    """
    Push the most recently captured Sony JPEG into the HG adaptive tracker.

    Uses _latest_shot (set by _generate_sony_thumb after each timelapse
    capture) rather than the liveview stream.  Liveview is stopped during
    Sony timelapse sequences because the A7III only allows ONE HTTP consumer
    at a time, and the capture/exposure API calls already saturate that slot.
    The captured JPEG is a much more reliable EV source — it IS the actual
    frame being exposed.

    Uses push_meter_shot() directly (not push_preview_frame) so that:
      • Night captures are processed — push_preview_frame returns None at night
      • Full histogram stats are extracted (P50, highlight/shadow fractions)
        so the highlight brake and shadow boost actually fire
      • The 0.4 preview weight penalty is not applied to real captures
    """
    if not hg.settings.enabled:
        return
    try:
        jpeg_bytes = _latest_shot
        if not jpeg_bytes:
            return

        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is not None:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_index = state.get("current_frame", 0)
            result = hg.push_meter_shot(frame_rgb, frame_index=frame_index)
            if result:
                logger.debug(
                    f"Sony shot meter: EV={result['meter_ev']:.2f} "
                    f"p50={result['midtone_p50']} "
                    f"hl={result['highlight_fraction']:.3f} "
                    f"cond={result['condition']}")
    except Exception as e:
        logger.debug(f"Sony shot meter: {e}")


async def get_picam_liveview():
    global _last_frame
    enc = [cv2.IMWRITE_JPEG_QUALITY, 60]
    _hg_meter_interval = 2.0   # seconds between sky measurements
    _hg_last_meter     = 0.0

    while True:
        if _HAS_PICAM and picam:
            try:
                frame = await asyncio.to_thread(picam.capture_array)
                _last_frame = frame
                active_mode = state.get("active_mode", "timelapse")
                sz = STREAM_SIZE_169 if active_mode == "cinematic" else STREAM_SIZE_43
                small = cv2.resize(frame, sz, interpolation=cv2.INTER_LINEAR)

                # Apply camera orientation transform
                orient = state.get("camera_orientation", "landscape")
                if orient == "portrait_cw":
                    small = cv2.rotate(small, cv2.ROTATE_90_CLOCKWISE)
                elif orient == "portrait_ccw":
                    small = cv2.rotate(small, cv2.ROTATE_90_COUNTERCLOCKWISE)
                elif orient == "inverted":
                    small = cv2.rotate(small, cv2.ROTATE_180)

                _, buf = cv2.imencode('.jpg', small, enc)
                yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n'

                # ── HG sky metering (timelapse mode only, every 2s) ────────────
                now_t = time.time()
                # Skip metering until ISP settles after a still capture.
                # The settle time scales with shutter speed — at night a 4s
                # exposure means the mode-switch takes longer to recover from.
                # Minimum 3s, or 2× the last shutter time, whichever is longer.
                last_shutter = _last_hg_params.get("shutter_s", 0.0) if _last_hg_params else 0.0
                settle_needed = max(3.0, last_shutter * 2.0)
                isp_settled = (now_t - _last_capture_time) > settle_needed
                if (active_mode == "timelapse"
                        and hg.settings.enabled
                        and state.get("is_running", False)
                        and isp_settled
                        and now_t - _hg_last_meter >= _hg_meter_interval):
                    _hg_last_meter = now_t
                    # Fire-and-forget on thread pool — but only if previous
                    # meter task has completed (backpressure guard).
                    # We keep a reference and check done() before submitting.
                    loop = asyncio.get_running_loop()
                    if not hasattr(get_picam_liveview, '_meter_future') or \
                            get_picam_liveview._meter_future.done():
                        # Compute current camera EV so the analyser can
                        # down-weight measurements from blown-out frames.
                        cam_ev = None
                        if _last_hg_params:
                            try:
                                p = _last_hg_params
                                cam_ev = (math.log2((hg.settings.aperture_day ** 2)
                                          / p["shutter_s"])
                                          - math.log2(p["iso"] / 100.0))
                            except Exception:
                                cam_ev = None
                        get_picam_liveview._meter_future = loop.run_in_executor(
                            None, hg.push_preview_frame, frame.copy(), cam_ev
                        )

            except Exception as e:
                logger.error(f"Liveview: {e}")
                await asyncio.sleep(0.5)
        else:
            err = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(err, "PI CAMERA OFFLINE", (140, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
            _, buf = cv2.imencode('.jpg', err, enc)
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n'
        await asyncio.sleep(0.05)   # ~20fps preview


@app.get("/video_feed")
async def video_feed():
    # preview_camera can be overridden independently of capture camera
    preview_cam = state.get("preview_camera", state["active_camera"])
    if preview_cam in ("sony", "sony_usb"):
        # Both WiFi and USB tether feed _sony_last_frame; get_sony_liveview()
        # serves that buffer with an appropriate placeholder while it loads.
        return StreamingResponse(get_sony_liveview(),
            media_type="multipart/x-mixed-replace; boundary=frame")
    return StreamingResponse(get_picam_liveview(),
        media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/latest_frame")
async def latest_frame():
    global _latest_shot
    if _latest_shot:
        return StreamingResponse(iter([_latest_shot]), media_type="image/jpeg",
            headers={"Cache-Control": "no-cache, no-store"})
    blank = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.putText(blank, "WAITING FOR FIRST FRAME", (80, 180),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (60,60,60), 1)
    _, buf = cv2.imencode('.jpg', blank)
    return StreamingResponse(iter([buf.tobytes()]), media_type="image/jpeg")

@app.get("/browse")
async def browse_dir(path: str = Query(default=str(Path.home() / "Pictures"))):
    try:
        p = Path(path).resolve()
        if not p.is_dir():
            return JSONResponse({"error": "Not a directory"}, status_code=400)
        entries = []
        if p.parent != p:
            entries.append({"name": "..", "path": str(p.parent), "type": "dir"})
        for child in sorted(p.iterdir()):
            entries.append({"name": child.name, "path": str(child),
                            "type": "dir" if child.is_dir() else "file"})
        # Include disk usage for this path's filesystem
        usage = shutil.disk_usage(str(p))
        return JSONResponse({"path": str(p), "entries": entries,
                             "disk_free": usage.free, "disk_total": usage.total})
    except PermissionError:
        return JSONResponse({"error": "Permission denied"}, status_code=403)


@app.get("/disk_info")
async def disk_info():
    """Return free/total bytes for the current save path's filesystem."""
    try:
        save = state.get("save_path", str(Path.home() / "Pictures"))
        # Fall back to / if save path doesn't exist yet
        check = save if os.path.exists(save) else "/"
        usage = shutil.disk_usage(check)
        return JSONResponse({"path": save, "free": usage.free, "total": usage.total,
                             "used": usage.used})
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)


@app.get("/loupe_crop")
async def loupe_crop(cx: float = 0.5, cy: float = 0.5, r: float = 0.15):
    """
    Return a high-quality JPEG crop from the current live preview frame.
    cx, cy are fractions in the *displayed* (possibly rotated) frame.
    Works for both PiCam (_last_frame numpy array) and Sony (_sony_last_frame
    JPEG bytes); source chosen by preview_camera state.
    """
    def _blank(msg: str = "NO FEED"):
        blank = np.zeros((320, 320, 3), dtype=np.uint8)
        cv2.putText(blank, msg, (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (60, 60, 60), 1)
        _, buf = cv2.imencode('.jpg', blank, [cv2.IMWRITE_JPEG_QUALITY, 90])
        return StreamingResponse(iter([buf.tobytes()]), media_type="image/jpeg",
                                 headers={"Cache-Control": "no-cache, no-store"})

    preview_cam = state.get("preview_camera", state.get("active_camera", "picam"))

    # ── Sony path (WiFi or USB — both feed _sony_last_frame) ─────────────────
    if preview_cam in ("sony", "sony_usb"):
        jpeg_bytes = _sony_last_frame
        if not jpeg_bytes:
            label = "SONY USB — NO FEED" if preview_cam == "sony_usb" else "SONY — NO FEED"
            return _blank(label)
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return _blank("SONY — DECODE ERR")
        # Apply same orientation transform as PiCam — Sony can be mounted at
        # any angle (portrait, inverted, etc.) just like the PiCam.
        orient = state.get("camera_orientation", "landscape")
        if orient == "portrait_cw":
            cx, cy = cy, 1.0 - cx
        elif orient == "portrait_ccw":
            cx, cy = 1.0 - cy, cx
        elif orient == "inverted":
            cx, cy = 1.0 - cx, 1.0 - cy
        h, w = frame.shape[:2]
        rp = max(20, int(r * w))
        px = int(cx * w)
        py = int(cy * h)
        x1 = max(0, px - rp);  x2 = min(w, px + rp)
        y1 = max(0, py - rp);  y2 = min(h, py + rp)
        crop = frame[y1:y2, x1:x2]
        if orient == "portrait_cw":
            crop = cv2.rotate(crop, cv2.ROTATE_90_CLOCKWISE)
        elif orient == "portrait_ccw":
            crop = cv2.rotate(crop, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif orient == "inverted":
            crop = cv2.rotate(crop, cv2.ROTATE_180)
        out = cv2.resize(crop, (320, 320), interpolation=cv2.INTER_LANCZOS4)
        _, buf = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 92])
        return StreamingResponse(iter([buf.tobytes()]), media_type="image/jpeg",
                                 headers={"Cache-Control": "no-cache, no-store"})

    # ── PiCam path ────────────────────────────────────────────────────────────
    frame = _last_frame
    if frame is None or not _HAS_PICAM:
        return _blank()

    # Transform (cx, cy) from displayed-frame coords back to raw-frame coords
    orient = state.get("camera_orientation", "landscape")
    if orient == "portrait_cw":
        # display: rotated 90° CW → raw: cx_raw = cy_disp, cy_raw = 1 - cx_disp
        cx, cy = cy, 1.0 - cx
    elif orient == "portrait_ccw":
        # display: rotated 90° CCW → raw: cx_raw = 1 - cy_disp, cy_raw = cx_disp
        cx, cy = 1.0 - cy, cx
    elif orient == "inverted":
        cx, cy = 1.0 - cx, 1.0 - cy

    h, w = frame.shape[:2]
    rp = max(20, int(r * w))
    px = int(cx * w)
    py = int(cy * h)
    x1 = max(0, px - rp);  x2 = min(w, px + rp)
    y1 = max(0, py - rp);  y2 = min(h, py + rp)
    crop = frame[y1:y2, x1:x2]

    # Re-apply the same rotation so the loupe image appears upright
    if orient == "portrait_cw":
        crop = cv2.rotate(crop, cv2.ROTATE_90_CLOCKWISE)
    elif orient == "portrait_ccw":
        crop = cv2.rotate(crop, cv2.ROTATE_90_COUNTERCLOCKWISE)
    elif orient == "inverted":
        crop = cv2.rotate(crop, cv2.ROTATE_180)

    out = cv2.resize(crop, (320, 320), interpolation=cv2.INTER_LANCZOS4)
    _, buf = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 92])
    return StreamingResponse(iter([buf.tobytes()]), media_type="image/jpeg",
                             headers={"Cache-Control": "no-cache, no-store"})


# ─── CAPTURE ──────────────────────────────────────────────────────────────────
def capture_sony(frame_id: str, shutter_s: float = 0.0) -> Optional[str]:
    """
    Capture one Sony frame.

    Two transport modes — chosen by which port responds:

      HTTP  (port 8080, Sony Camera Remote API / Smart Remote / DIRECT-* WiFi):
          Normal:  actTakePicture  — up to 30 s shutter preset.
          Bulb:    startBulbShooting / stopBulbShooting — unlimited duration.

      PTP/IP (port 15740, gphoto2 / PC Remote):
          Normal:  --capture-image-and-download
          Bulb:    --bulb N  — gphoto2 holds the shutter for N integer seconds.

    When shutter_s > SONY_BULB_THRESHOLD (30 s) the bulb path is chosen
    automatically. The Pi times the actual exposure and stores the EV error
    from WiFi jitter in state["_bulb_ev_error"] so write_sidecar() can write
    a crs:Exposure2012 correction. Normal (non-bulb) captures store 0.0.
    """
    import math as _math
    import time as _time
    import socket as _s
    import requests as _req

    state["_bulb_ev_error"] = 0.0   # reset every call
    ip        = state["sony_ip"]
    http_port = state.get("sony_http_port", 8080)   # discovered during connect; default 8080
    api       = f"http://{ip}:{http_port}/sony/camera"

    def _port_open(port: int) -> bool:
        try:
            c = _s.create_connection((ip, port), timeout=2); c.close(); return True
        except Exception:
            return False

    # ── helpers ───────────────────────────────────────────────────────────────
    def _record_bulb_ev_error(intended: float, actual: float) -> None:
        """Compute and store EV error from timing jitter. +EV = overexposed."""
        if intended > 0 and actual > 0:
            err = _math.log2(actual / intended)
            state["_bulb_ev_error"] = err
            logger.info(
                f"Sony bulb timing: intended={intended:.2f}s actual≈{actual:.2f}s "
                f"ev_error={err:+.3f} stops"
            )

    # ── HTTP normal capture ───────────────────────────────────────────────────
    def _ensure_rec_mode() -> bool:
        """Ensure camera is in rec mode AND still-photo shoot mode.
        Called before every actTakePicture to guard against camera standby
        or a previous setShootMode("movie") call that wasn't fully reversed."""
        try:
            rec = _req.post(api,
                json={"method": "startRecMode", "params": [], "id": 1, "version": "1.0"},
                timeout=8)
            j = rec.json()
            if "result" not in j:
                err = j.get("error", [None])[0]
                if err != 7:   # 7 = already in rec mode — that's fine
                    logger.warning(f"startRecMode returned error {err}")
        except Exception as e:
            logger.warning(f"startRecMode failed: {e}")

        # Ensure shoot mode is "still" — setShootMode("movie") in the video
        # recorder path may not have been reversed if recording was aborted.
        # This call is harmless if already in still mode (returns result:[0]).
        try:
            _req.post(api,
                json={"method": "setShootMode", "params": ["still"],
                      "id": 1, "version": "1.0"},
                timeout=5)
        except Exception:
            pass
        return True

    def _http_normal() -> Optional[str]:
        try:
            # Ensure rec mode is active — camera may have timed out or reset.
            # Error 7 ("Illegal Request") means already in rec mode — that's fine.
            _ensure_rec_mode()

            resp = _req.post(api,
                json={"method": "actTakePicture", "params": [], "id": 1, "version": "1.0"},
                timeout=15)
            data = resp.json()
            if "error" in data:
                err_code = data["error"][0] if data["error"] else "?"
                err_msg  = data["error"][1] if len(data["error"]) > 1 else ""
                human = {
                    3:     "IllegalArgument — check shutter/ISO values",
                    7:     "Camera not in shooting state — check mode dial",
                    40402: "actTakePicture not available — camera may be in video/review mode",
                    41501: "Camera busy — another operation in progress",
                    41503: "Camera not ready — startRecMode may have failed",
                }.get(err_code, err_msg or f"error {err_code}")
                err_str = f"Sony actTakePicture error {err_code}: {human}"
                logger.error(err_str)
                state["_capture_error"] = err_str
                return None
            urls     = data.get("result", [[]])[0] if data.get("result") else []
            arw_url  = next((u for u in urls if ".ARW" in u.upper()), None)
            jpeg_url = next((u for u in urls if any(e in u.upper()
                             for e in (".JPG", ".JPEG"))), None)
            dl_url   = arw_url or jpeg_url
            ext      = ".ARW" if arw_url else ".JPG"
            dest     = os.path.join(state["save_path"], f"FRAME_{frame_id}{ext}")

            # ── Card filename tracking ────────────────────────────────────────
            # Sony card files are named DSC01954, DSC01955 … (prefix + counter).
            # We learn the number once then just increment — no per-frame API
            # call needed.
            #
            # arw_url (when present) gives the real card name directly.
            # jpeg_url is a postview temp (pict260502_1721310000.jpg) whose
            # basename does NOT match the card RAW — never use it for naming.
            # If arw_url is absent, bootstrap from getContentList once, then
            # count up from there.
            import re as _re
            _seq = state.get("_sony_card_seq")   # {"prefix","digits","next_num"}

            # If arw_url is present it IS the ground truth — sync counter to it.
            if arw_url:
                _arw_base = os.path.splitext(os.path.basename(arw_url))[0]
                _m = _re.match(r'^([A-Za-z_]+)(\d+)$', _arw_base)
                if _m:
                    _digits = len(_m.group(2))
                    _max    = 10 ** _digits
                    state["_sony_card_seq"] = {
                        "prefix":   _m.group(1),
                        "digits":   _digits,
                        "next_num": (int(_m.group(2)) + 1) % _max,
                    }
                    state["_last_sony_basename"] = _arw_base
                    logger.debug(f"Sony card basename (arw_url): {_arw_base}")
                _seq = None   # already handled above

            elif _seq:
                # Counter already bootstrapped — just use next number.
                # Sony wraps DSC99999 → DSC00000 (modulo 10^digits).
                _max = 10 ** _seq["digits"]
                _card_base = f"{_seq['prefix']}{_seq['next_num']:0{_seq['digits']}d}"
                _seq["next_num"] = (_seq["next_num"] + 1) % _max
                state["_last_sony_basename"] = _card_base
                logger.debug(f"Sony card basename (seq): {_card_base}")

            else:
                # Neither arw_url nor a known counter — bootstrap once via
                # getContentList so we can count up from the correct file.
                try:
                    cl_resp = _req.post(api,
                        json={"method": "getContentList",
                              "params": [{"uri": "storage:memoryCard1",
                                          "startIndex": 0, "maxResults": 1,
                                          "type": ["still"], "view": "date"}],
                              "id": 1, "version": "1.3"},
                        timeout=5)
                    cl_items = (cl_resp.json().get("result", [[]])[0]
                                if cl_resp.json().get("result") else [])
                    cl_url = (cl_items[0].get("content", {})
                                         .get("original", [{}])[0]
                                         .get("url", "")) if cl_items else ""
                    if cl_url:
                        _cl_base = os.path.splitext(os.path.basename(cl_url))[0]
                        _m = _re.match(r'^([A-Za-z_]+)(\d+)$', _cl_base)
                        if _m:
                            _digits = len(_m.group(2))
                            _max    = 10 ** _digits
                            state["_sony_card_seq"] = {
                                "prefix":   _m.group(1),
                                "digits":   _digits,
                                "next_num": (int(_m.group(2)) + 1) % _max,
                            }
                            state["_last_sony_basename"] = _cl_base
                            logger.info(f"Sony card seq bootstrapped: {_cl_base}")
                except Exception as _cle:
                    logger.debug(f"Sony getContentList bootstrap failed: {_cle}")

            if dl_url:
                r = _req.get(dl_url, timeout=60, stream=True)
                with open(dest, "wb") as fh:
                    for chunk in r.iter_content(chunk_size=65536):
                        fh.write(chunk)
                logger.info(f"Sony HTTP capture → {dest}")
            else:
                logger.info("Sony HTTP capture: shutter triggered, file on camera card")
                dest = None

            # ── Thumbnail: always download the JPEG postview ───────────────
            # If ARW was primary, fetch the JPEG separately (it's small, fast).
            # If JPEG was primary, read it back from disk.
            # Either way, generate THUMB_{frame_id}.jpg and update _latest_shot.
            if jpeg_url and arw_url:
                # ARW downloaded above — now fetch JPEG postview for thumb
                try:
                    jr = _req.get(jpeg_url, timeout=20)
                    if jr.status_code == 200:
                        _generate_sony_thumb(jr.content, frame_id)
                except Exception as te:
                    logger.warning(f"Sony JPEG postview download failed: {te}")
            elif dest and ext == ".JPG":
                # JPEG was the only file — re-read it for thumb generation
                try:
                    with open(dest, "rb") as fh:
                        _generate_sony_thumb(fh.read(), frame_id)
                except Exception as te:
                    logger.warning(f"Sony JPEG thumb from file failed: {te}")

            return dest or ""
        except Exception as e:
            err = f"Sony HTTP capture failed: {e}"
            logger.error(err)
            state["_capture_error"] = err
            return None

    # ── HTTP bulb capture ─────────────────────────────────────────────────────
    def _http_bulb() -> Optional[str]:
        try:
            # Confirm startBulbShooting is available (requires M mode + BULB shutter)
            avail_resp = _req.post(api,
                json={"method": "getAvailableApiList", "params": [], "id": 1, "version": "1.0"},
                timeout=5)
            available = avail_resp.json().get("result", [[]])[0] \
                        if avail_resp.json().get("result") else []
            if "startBulbShooting" not in available:
                err_str = ("Sony bulb: startBulbShooting not available — "
                           "set camera mode dial to M and shutter to BULB")
                logger.warning(err_str)
                state["_capture_error"] = err_str
                return None

            # Set shutter to BULB (camera may already be there; harmless if so)
            _req.post(api,
                json={"method": "setShutterSpeed", "params": ["BULB"],
                      "id": 1, "version": "1.0"},
                timeout=5)

            # Open shutter — record time AFTER ack (camera opens on receipt)
            start_resp = _req.post(api,
                json={"method": "startBulbShooting", "params": [], "id": 1, "version": "1.0"},
                timeout=10)
            if start_resp.json().get("error"):
                logger.error(f"Sony startBulbShooting: {start_resp.json()['error']}")
                return None
            t_open = _time.monotonic()

            # Sleep for intended duration minus estimated one-way WiFi latency so
            # the stop command arrives at the camera right on time.
            # Any residual error is measured and written to the sidecar.
            ONE_WAY_LATENCY = 0.15   # seconds — conservative WiFi estimate
            _time.sleep(max(0.1, shutter_s - ONE_WAY_LATENCY))

            # Close shutter — bracket timestamps to estimate actual close time
            t_stop_sent = _time.monotonic()
            _req.post(api,
                json={"method": "stopBulbShooting", "params": [], "id": 1, "version": "1.0"},
                timeout=10)
            t_stop_ack = _time.monotonic()

            # Best estimate: camera received stop at midpoint of round-trip
            rtt     = t_stop_ack - t_stop_sent
            t_close = t_stop_sent + rtt / 2.0
            actual  = t_close - t_open

            _record_bulb_ev_error(shutter_s, actual)
            logger.info(f"Sony HTTP bulb: {shutter_s:.1f}s intended, WiFi RTT {rtt*1000:.0f}ms")

            # ── Thumbnail: poll awaitTakePicture to get postview JPEG URL ──
            # Camera needs a moment to write the file before the URL is ready.
            # If the URL isn't available within the poll window the frame_id is
            # queued in _thumb_retry_queue so the timelapse worker can try again
            # from a downloaded file during subsequent inter-shot waits.
            _got_bulb_thumb = False
            try:
                for _ in range(8):   # up to ~4s of polling
                    _time.sleep(0.5)
                    ev_resp = _req.post(api,
                        json={"method": "awaitTakePicture", "params": [],
                              "id": 1, "version": "1.0"},
                        timeout=6)
                    ev_data = ev_resp.json()
                    if ev_data.get("error"):
                        break
                    urls = ev_data.get("result", [[]])[0] if ev_data.get("result") else []
                    jpeg_url = next((u for u in urls if any(e in u.upper()
                                    for e in (".JPG", ".JPEG"))), None)
                    if jpeg_url:
                        jr = _req.get(jpeg_url, timeout=20)
                        if jr.status_code == 200:
                            _generate_sony_thumb(jr.content, frame_id)
                            _got_bulb_thumb = True
                        break
            except Exception as te:
                logger.warning(f"Sony bulb postview fetch failed: {te}")

            if not _got_bulb_thumb and frame_id not in _thumb_retry_queue:
                _thumb_retry_queue.append(frame_id)

            return ""   # file saved to camera card

        except Exception as e:
            logger.error(f"Sony HTTP bulb failed: {e}")
            # Safety: attempt to close shutter if it was left open
            try:
                _req.post(api,
                    json={"method": "stopBulbShooting", "params": [], "id": 1, "version": "1.0"},
                    timeout=5)
            except Exception:
                pass
            return None

    # ── PTP/IP normal capture ─────────────────────────────────────────────────
    def _ptp_normal() -> Optional[str]:
        dest = os.path.join(state["save_path"], f"FRAME_{frame_id}.ARW")
        try:
            subprocess.run(
                ["gphoto2", "--port", f"ptpip:{ip}",
                 "--capture-image-and-download", "--filename", dest],
                check=True, timeout=30)
            _extract_arw_thumb(dest, frame_id)
            return dest
        except Exception as e:
            logger.error(f"Sony PTP capture: {e}")
            return None

    # ── PTP/IP bulb capture ───────────────────────────────────────────────────
    def _ptp_bulb() -> Optional[str]:
        dest = os.path.join(state["save_path"], f"FRAME_{frame_id}.ARW")
        bulb_secs = max(1, int(round(shutter_s)))   # gphoto2 --bulb takes integer seconds
        try:
            t_start = _time.monotonic()
            subprocess.run(
                ["gphoto2", "--port", f"ptpip:{ip}",
                 "--set-config", "shutterspeed=bulb",
                 "--bulb", str(bulb_secs),
                 "--capture-image-and-download", "--filename", dest],
                check=True, timeout=bulb_secs + 30)
            actual = _time.monotonic() - t_start
            _record_bulb_ev_error(shutter_s, actual)
            _extract_arw_thumb(dest, frame_id)
            return dest
        except Exception as e:
            logger.error(f"Sony PTP bulb: {e}")
            return None

    # ── Route: pick transport × mode ─────────────────────────────────────────
    use_bulb = shutter_s > SONY_BULB_THRESHOLD

    if _port_open(http_port):
        return _http_bulb() if use_bulb else _http_normal()
    if _port_open(15740):
        return _ptp_bulb()  if use_bulb else _ptp_normal()

    err = (f"Sony camera unreachable at {ip} "
           f"(port {http_port} and PTP 15740 both closed). "
           f"Check camera WiFi / Smart Remote is enabled.")
    logger.error(err)
    state["_capture_error"] = err
    return None



def _save_thumb(save_path: str, frame_id: str, frame_rgb: "np.ndarray") -> None:
    """Save a 640×480 JPEG thumbnail for the graph timelapse player."""
    try:
        thumb_dir = os.path.join(save_path, "thumbs")
        os.makedirs(thumb_dir, exist_ok=True)
        thumb = cv2.resize(frame_rgb, (640, 480), interpolation=cv2.INTER_LINEAR)
        thumb_path = os.path.join(thumb_dir, f"THUMB_{frame_id}.jpg")
        cv2.imwrite(thumb_path, thumb, [cv2.IMWRITE_JPEG_QUALITY, 75])
    except Exception as e:
        logger.warning(f"Thumb save failed (frame {frame_id}): {e}")


def _generate_sony_thumb(jpeg_bytes: bytes, frame_id: str) -> None:
    """
    Decode a Sony JPEG postview into a thumbnail + _latest_shot.
    Called from all four Sony capture paths (HTTP normal/bulb, PTP normal/bulb).
    Thread-safe: only writes Python object references (GIL-protected).
    On failure the frame_id is added to _thumb_retry_queue so the
    timelapse worker can attempt recovery during subsequent inter-shot waits.

    Also extracts FocalLength + LensModel from EXIF while bytes are in memory
    (avoids a separate ExifTool call later for lens detection).
    """
    global _latest_shot
    if not jpeg_bytes:
        if frame_id and frame_id not in _thumb_retry_queue:
            _thumb_retry_queue.append(frame_id)
        return

    # ── Extract focal length + lens model from JPEG EXIF in-memory ────────────
    # Sony WiFi postview JPEGs carry full EXIF — grab focal data while we have
    # the raw bytes, before cv2 decodes and discards the metadata.
    # FocalLength (0x920A) and LensModel (0xA434) live in the ExifIFD sub-IFD
    # (tag 0x8769), NOT in IFD0. PIL's getexif() returns IFD0 only, so we must
    # use get_ifd(0x8769) to reach them.
    if not state.get("_sony_focal_mm"):   # only read once per session
        try:
            import io as _io
            from PIL import Image as _PILImage
            with _PILImage.open(_io.BytesIO(jpeg_bytes)) as _img:
                _exif = _img.getexif()
                _exif_ifd = _exif.get_ifd(0x8769)  # ExifIFD sub-IFD
                _fl = _exif_ifd.get(0x920A) or _exif.get(0x920A)   # FocalLength
                if _fl:
                    try:
                        _fl_f = float(_fl)
                        if _fl_f > 0:
                            state["_sony_focal_mm"] = _fl_f
                    except (TypeError, ValueError):
                        pass
                _lm_bytes = _exif_ifd.get(0xA434) or _exif.get(0xA434)  # LensModel
                if _lm_bytes:
                    state["_sony_lens_model"] = (
                        _lm_bytes.decode("utf-8", errors="ignore").strip()
                        if isinstance(_lm_bytes, bytes) else str(_lm_bytes).strip()
                    )
        except Exception:
            pass   # EXIF read failure is non-fatal

        # Fallback: if PIL didn't find focal length, try exiftool on the raw bytes
        if not state.get("_sony_focal_mm"):
            try:
                import tempfile as _tf, subprocess as _sp, json as _json
                with _tf.NamedTemporaryFile(suffix=".jpg", delete=False) as _tmp:
                    _tmp.write(jpeg_bytes)
                    _tmp_path = _tmp.name
                _et = _sp.run(["exiftool", "-FocalLength", "-LensModel", "-json", _tmp_path],
                              capture_output=True, text=True, timeout=5)
                os.unlink(_tmp_path)
                if _et.returncode == 0 and _et.stdout:
                    _et_data = _json.loads(_et.stdout)
                    if _et_data:
                        _et_fl  = _et_data[0].get("FocalLength")
                        _et_lm  = _et_data[0].get("LensModel")
                        if _et_fl is not None:
                            try:
                                _fl_f = float(str(_et_fl).replace("mm","").strip())
                                if _fl_f > 0:
                                    state["_sony_focal_mm"] = _fl_f
                            except (ValueError, TypeError):
                                pass
                        if _et_lm and not state.get("_sony_lens_model"):
                            state["_sony_lens_model"] = str(_et_lm).strip()
            except Exception:
                pass

    try:
        _latest_shot = jpeg_bytes
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        frame_bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame_bgr is not None:
            _save_thumb(state["save_path"], frame_id, frame_bgr)
        else:
            raise ValueError("cv2.imdecode returned None — corrupt JPEG?")
    except Exception as e:
        logger.warning(f"Sony thumb generation failed (frame {frame_id}): {e}")
        if frame_id and frame_id not in _thumb_retry_queue:
            _thumb_retry_queue.append(frame_id)


def _extract_arw_thumb(arw_path: str, frame_id: str) -> None:
    """
    Extract the embedded JPEG preview from a Sony ARW file using exiftool
    and pass it to _generate_sony_thumb().  Used by PTP capture paths where
    no separate JPEG URL is available from the API.
    On failure the frame_id is added to _thumb_retry_queue.
    """
    try:
        result = subprocess.run(
            ["exiftool", "-b", "-PreviewImage", arw_path],
            capture_output=True, timeout=15)
        if result.returncode == 0 and result.stdout:
            _generate_sony_thumb(result.stdout, frame_id)
        else:
            logger.warning(f"exiftool preview extract failed for {arw_path}")
            if frame_id and frame_id not in _thumb_retry_queue:
                _thumb_retry_queue.append(frame_id)
    except FileNotFoundError:
        logger.warning("exiftool not found — install with: sudo apt install libimage-exiftool-perl")
        if frame_id and frame_id not in _thumb_retry_queue:
            _thumb_retry_queue.append(frame_id)
    except Exception as e:
        logger.warning(f"ARW thumb extract failed: {e}")
        if frame_id and frame_id not in _thumb_retry_queue:
            _thumb_retry_queue.append(frame_id)


def _attempt_thumb_recovery(frame_id: str) -> bool:
    """
    Try to generate a missing thumbnail from whatever file is already on disk.
    Checks for FRAME_{frame_id}.ARW (exiftool), then .JPG (direct decode).
    Returns True if a thumb was successfully written, False otherwise.
    Camera-card-only captures (HTTP bulb, no local file) will always return False.
    """
    save_path = state.get("save_path", "")
    if not save_path:
        return False
    # Skip if thumb already exists
    thumb_path = os.path.join(save_path, "thumbs", f"THUMB_{frame_id}.jpg")
    if os.path.exists(thumb_path):
        return True

    arw = os.path.join(save_path, f"FRAME_{frame_id}.ARW")
    jpg = os.path.join(save_path, f"FRAME_{frame_id}.JPG")

    if os.path.exists(arw):
        # ARW on disk — extract embedded JPEG preview via exiftool
        try:
            result = subprocess.run(
                ["exiftool", "-b", "-PreviewImage", arw],
                capture_output=True, timeout=15)
            if result.returncode == 0 and result.stdout:
                import numpy as _np
                arr = _np.frombuffer(result.stdout, dtype=_np.uint8)
                frame_bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame_bgr is not None:
                    _save_thumb(save_path, frame_id, frame_bgr)
                    return True
        except Exception as e:
            logger.debug(f"Thumb recovery (ARW) frame {frame_id}: {e}")
        return False

    if os.path.exists(jpg):
        try:
            with open(jpg, "rb") as fh:
                data = fh.read()
            import numpy as _np
            arr = _np.frombuffer(data, dtype=_np.uint8)
            frame_bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame_bgr is not None:
                _save_thumb(save_path, frame_id, frame_bgr)
                return True
        except Exception as e:
            logger.debug(f"Thumb recovery (JPG) frame {frame_id}: {e}")
        return False

    return False   # file not on Pi (still on camera card)


def _take_meter_shot(frame_index: int) -> Optional[dict]:
    """
    Capture a dedicated metering image at the fixed anchor exposure
    (anchor_shutter_s, anchor_iso). Because the camera settings never change
    between meter shots, every reading is directly comparable — no
    exposure-compensation math required, no feedback oscillation possible.

    The meter JPEG is temporary; only the histogram data is kept.
    Returns the meter result dict from push_meter_shot(), or None on failure.
    """
    if not hg.settings.enabled:
        return None
    if not picam:
        return None
    if (hg.settings.anchor_shutter_s is None or
            hg.settings.anchor_iso is None):
        return None

    try:
        # 1. Switch camera to fixed anchor settings
        anchor_controls = {
            "AeEnable":       False,
            "AwbEnable":      False,
            "ExposureTime":   max(1, int(hg.settings.anchor_shutter_s * 1_000_000)),
            "AnalogueGain":   max(1.0, hg.settings.anchor_iso / 100.0),
            "ColourGains":    (1.0, 1.0),  # neutral WB for meter shot
        }
        picam.set_controls(anchor_controls)

        # Brief settle — one preview frame to let the ISP apply the controls
        import time as _t; _t.sleep(0.25)

        # 2. Capture a JPEG preview frame (not a full DNG — fast, no disk writes)
        frame_array = picam.capture_array()
        rgb = cv2.cvtColor(frame_array, cv2.COLOR_BGR2RGB) if frame_array is not None else None

        # 3. Immediately re-apply HG controls so the creative capture isn't
        #    affected by the meter shot's fixed settings
        if _last_hg_params:
            _reapply_hg_after_capture()

        if rgb is None or rgb.size == 0:
            return None

        # 4. Push histogram to HolyGrail tracker
        from astral.sun import elevation as _se
        sun_alt = _se(hg._location.observer,
                      datetime.datetime.now(hg._tzinfo))

        result = hg.push_meter_shot(rgb, frame_index=frame_index, sun_alt=sun_alt)
        if result:
            logger.info(
                f"Meter shot frame={frame_index}: "
                f"ev={result['meter_ev']:.3f} p50={result['midtone_p50']} "
                f"hl={result['highlight_fraction']:.3f} "
                f"shadow={result['shadow_fraction']:.3f} "
                f"cond={result['condition']} K={result['kelvin']}"
            )
        return result

    except Exception as e:
        logger.warning(f"_take_meter_shot frame={frame_index}: {e}")
        # Always try to restore HG controls even if meter shot failed
        try:
            if _last_hg_params:
                _reapply_hg_after_capture()
        except Exception:
            pass
        return None


def _capture_controls_from_hg(params: dict) -> dict:
    """
    Build picamera2 controls for the actual DNG still capture.
    Unlike _preview_controls_from_hg(), this uses the FULL shutter_s with no cap —
    the IMX477 can natively do up to ~670s, so any HG-requested duration works.
    """
    shutter_s = params["shutter_s"]
    iso       = params["iso"]
    return {
        "AeEnable":          False,
        "AwbEnable":         False,
        "ExposureTime":      int(shutter_s * 1_000_000),   # full duration, no cap
        "AnalogueGain":      iso / 100.0,
        "ColourTemperature": int(params["kelvin"]),
    }


def capture_picam(frame_id: str) -> Optional[str]:
    """
    Capture a DNG still. On camera timeout/ISP error, attempts one recovery
    restart before giving up so the sequence can continue.

    When HolyGrail is active the DNG is captured with the full HG-requested
    ExposureTime (no preview cap). _preview_controls_from_hg() boosts preview
    gain to keep the live view bright, but the actual RAW always uses the real
    shutter/ISO so that night sky exposures of 20–60s work correctly.
    """
    global _latest_shot
    dest = os.path.join(state["save_path"], f"FRAME_{frame_id}.dng")
    if not picam:
        return None

    # Build the full (uncapped) capture controls from cached HG params.
    # This is what actually gets written to the DNG — critical for long night
    # exposures where preview shutter is capped at 0.25s but capture needs 25s+.
    capture_controls = None
    if hg.settings.enabled and _last_hg_params:
        capture_controls = _capture_controls_from_hg(_last_hg_params)

    for attempt in range(2):   # try once, recover, try once more
        try:
            # Apply the FULL capture controls right before the still switch.
            # This uses set_controls() (not the `controls=` kwarg of
            # switch_mode_and_capture_file, which is only available in newer
            # picamera2 versions and would raise TypeError on older installs).
            # picamera2 carries set_controls() settings into the still mode
            # when AeEnable=False, so the DNG gets the full HG shutter/ISO.
            if capture_controls:
                picam.set_controls(capture_controls)

            still_cfg = picam.create_still_configuration(
                main={"size": (4056, 3040), "format": "RGB888"},
                raw={}
            )
            picam.switch_mode_and_capture_file(still_cfg, dest, name="raw")

            # ── CRITICAL: re-lock HG controls immediately ─────────────────────
            # switch_mode_and_capture_file reconfigures the camera twice
            # (still → preview), resetting AeEnable/AwbEnable to True each time.
            # Re-applying here means the preview is back under manual control
            # within milliseconds rather than waiting up to one full interval.
            if hg.settings.enabled:
                _reapply_hg_after_capture()

            try:
                preview_frame = picam.capture_array()
                _, buf = cv2.imencode('.jpg', preview_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                _latest_shot = buf.tobytes()
                # Save thumbnail for graph timelapse player
                _save_thumb(state["save_path"], frame_id, preview_frame)

                # HG meter shot runs in step 8 (motor move window) instead,
                # so it is guaranteed to fire on every trigger mode.
                # Nothing to do here.
            except Exception:
                pass

            if os.path.exists(dest):
                logger.info(f"PiCam DNG saved: {dest}")
                return dest
            else:
                err_msg = f"PiCam capture ran but file not found on disk: {dest}"
                logger.error(err_msg)
                state["_capture_error"] = err_msg
                return None

        except Exception as e:
            err_msg = f"PiCam capture attempt {attempt+1} failed: {e}"
            logger.error(err_msg)
            state["_capture_error"] = err_msg   # surfaced to UI by timelapse loop
            if attempt == 0:
                # First failure — try to recover the camera and retry
                recovered = _restart_picam()
                if not recovered:
                    state["_capture_error"] = "Camera recovery failed — check hardware."
                    logger.error("Camera recovery failed — skipping frame.")
                    return None
                logger.info("Camera recovered — retrying capture…")
            # Second failure — give up on this frame, sequence continues
    return None


# ─── MACRO CAPTURE HELPERS ────────────────────────────────────────────────────

async def macro_capture(slot_dir: str, frame_id: str, slot: "ExposureSlot") -> Optional[str]:
    """
    Capture one frame for a macro slot.
    Saves to slot_dir/frame_id.dng (or .ARW for Sony).
    Returns the saved file path or None on failure.

    All camera modes write:
      • The raw file (DNG / ARW)
      • An XMP sidecar with rig position metadata
      • A _preview.jpg alongside the raw for the macro-graph flipbook
    """
    global _latest_shot
    cam = state.get("active_camera", "picam")

    def _save_preview(path: str):
        """Write _preview.jpg alongside a raw file using the current _latest_shot."""
        if _latest_shot:
            jpg = os.path.splitext(path)[0] + '_preview.jpg'
            try:
                with open(jpg, 'wb') as _f:
                    _f.write(_latest_shot)
            except Exception as e:
                logger.warning(f"macro_capture: preview save failed: {e}")

    if cam == "picam":
        dest = os.path.join(slot_dir, f"{frame_id}.dng")
        if not picam:
            return None
        try:
            still_cfg = picam.create_still_configuration(
                main={"size": (4056, 3040), "format": "RGB888"},
                raw={}
            )
            picam.switch_mode_and_capture_file(still_cfg, dest, name="raw")
            try:
                preview_frame = picam.capture_array()
                _, buf = cv2.imencode('.jpg', preview_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                _latest_shot = buf.tobytes()
            except Exception:
                pass
            _write_macro_sidecar(dest, slot)
            _save_preview(dest)
            logger.info(f"Macro DNG: {dest}")
            return dest if os.path.exists(dest) else None
        except Exception as e:
            logger.error(f"macro_capture picam: {e}")
            return None

    elif cam == "sony":
        # Sony WiFi — gphoto2 PTP/IP transfers ARW directly to dest
        dest = os.path.join(slot_dir, f"{frame_id}.ARW")
        result = await asyncio.to_thread(capture_sony_to, dest)
        if result:
            _write_macro_sidecar(result, slot)
            await asyncio.to_thread(_extract_arw_thumb, result, frame_id)  # → _latest_shot
            _save_preview(result)
        return result

    elif cam == "sony_usb":
        # Sony USB — gphoto2 USB transfers ARW directly to dest inside slot_dir
        dest = os.path.join(slot_dir, f"{frame_id}.ARW")
        result = await asyncio.to_thread(capture_sony_usb_to, dest, slot.shutter_s)
        if result:
            _write_macro_sidecar(result, slot)
            # _extract_arw_thumb already called inside capture_sony_usb_to → _latest_shot set
            _save_preview(result)
        return result

    else:
        # S2 / aux shutter trigger — no file returned
        await asyncio.to_thread(hw.trigger_camera, 0.2)
        return None


def capture_sony_to(dest: str) -> Optional[str]:
    """Capture Sony ARW via WiFi PTP/IP (gphoto2) to an explicit path (for macro mode)."""
    try:
        subprocess.run(
            ["gphoto2", "--port", f"ptpip:{state['sony_ip']}",
             "--capture-image-and-download", "--filename", dest,
             "--force-overwrite"],
            check=True, timeout=45)
        if os.path.exists(dest):
            logger.info(f"Sony WiFi capture → {dest}")
            return dest
        logger.error(f"Sony WiFi capture: file not found at {dest}")
        return None
    except Exception as e:
        logger.error(f"Sony WiFi capture to dest: {e}")
        return None


def capture_sony_usb_to(dest: str, shutter_s: float = 0.0) -> Optional[str]:
    """
    Capture Sony ARW via USB tethering (gphoto2) to an explicit destination path.
    Used by macro_capture() so each frame lands in the correct stack/slot folder
    with the correct PiSlider filename rather than a generic FRAME_xxx.ARW in save_path.

    Mirrors capture_sony_usb() but takes an explicit dest instead of building one
    from save_path.  Calls _extract_arw_thumb() on success to populate _latest_shot.
    """
    import time as _time
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    use_bulb = shutter_s > SONY_BULB_THRESHOLD
    try:
        if use_bulb:
            bulb_secs = max(1, int(round(shutter_s)))
            t_start   = _time.monotonic()
            subprocess.run(
                ["gphoto2",
                 "--set-config", "expprogram=M",
                 "--set-config", "shutterspeed=bulb",
                 "--bulb", str(bulb_secs),
                 "--capture-image-and-download",
                 "--filename", dest, "--force-overwrite"],
                check=True, capture_output=True, text=True,
                timeout=bulb_secs + 30)
            actual = _time.monotonic() - t_start
            if shutter_s > 0 and actual > 0:
                state["_bulb_ev_error"] = math.log2(actual / shutter_s)
        else:
            subprocess.run(
                ["gphoto2",
                 "--capture-image-and-download",
                 "--filename", dest, "--force-overwrite"],
                check=True, capture_output=True, text=True,
                timeout=max(45, int(shutter_s) + 15))
        if os.path.exists(dest):
            frame_id = os.path.splitext(os.path.basename(dest))[0]
            _extract_arw_thumb(dest, frame_id)   # → _latest_shot
            logger.info(f"Sony USB macro capture → {dest}")
            return dest
        logger.error(f"Sony USB macro capture: file not found at {dest}")
        return None
    except Exception as e:
        logger.error(f"capture_sony_usb_to {dest}: {e}")
        return None


def capture_sony_usb(frame_id: str, shutter_s: float = 0.0) -> Optional[str]:
    """
    Capture one Sony frame via USB tethering (gphoto2) and download to save_path.
    Uses --bulb for exposures longer than SONY_BULB_THRESHOLD (30 s).
    Returns the local ARW path on success, None on failure.
    """
    import time as _time
    state["_bulb_ev_error"] = 0.0
    save_path = state["save_path"]
    os.makedirs(save_path, exist_ok=True)
    dest     = os.path.join(save_path, f"FRAME_{frame_id}.ARW")
    use_bulb = shutter_s > SONY_BULB_THRESHOLD
    try:
        if use_bulb:
            bulb_secs = max(1, int(round(shutter_s)))
            t_start   = _time.monotonic()
            subprocess.run(
                ["gphoto2",
                 "--set-config", "expprogram=M",
                 "--set-config", "shutterspeed=bulb",
                 "--bulb", str(bulb_secs),
                 "--capture-image-and-download",
                 "--filename", dest, "--force-overwrite"],
                check=True, capture_output=True, text=True,
                timeout=bulb_secs + 30)
            actual = _time.monotonic() - t_start
            if shutter_s > 0 and actual > 0:
                state["_bulb_ev_error"] = math.log2(actual / shutter_s)
        else:
            subprocess.run(
                ["gphoto2",
                 "--capture-image-and-download",
                 "--filename", dest, "--force-overwrite"],
                check=True, capture_output=True, text=True,
                timeout=max(30, int(shutter_s) + 15))
        if os.path.exists(dest):
            _extract_arw_thumb(dest, frame_id)   # sets _latest_shot for HG metering
            logger.info(f"Sony USB capture → {dest}")
            return dest
        logger.error(f"Sony USB capture: file not found at {dest}")
        return None
    except Exception as e:
        logger.error(f"capture_sony_usb frame {frame_id}: {e}")
        return None


def apply_sony_usb_from_hg(params: dict):
    """Push HG-computed exposure to Sony via gphoto2 USB before each frame."""
    global _last_hg_params
    try:
        set_sony_settings_usb(
            ae        = False,
            awb       = False,   # HG always manages WB explicitly
            shutter_s = params["shutter_s"],
            iso       = int(params["iso"]),
            kelvin    = int(params.get("kelvin", 5500)),
        )
        _last_hg_params = params
    except Exception as e:
        logger.error(f"apply_sony_usb_from_hg: {e}")


def apply_sony_from_hg(params: dict):
    """Push HG-computed exposure to Sony via WiFi Camera Remote API before each frame."""
    global _last_hg_params
    try:
        errs = set_sony_settings(
            ae        = False,
            awb       = False,
            shutter_s = params["shutter_s"],
            iso       = int(params["iso"]),
            kelvin    = int(params.get("kelvin", 5500)),
        )
        if errs:
            logger.error(f"apply_sony_from_hg: {'; '.join(errs)}")
        else:
            _last_hg_params = params
    except Exception as e:
        logger.error(f"apply_sony_from_hg: {e}")


def hg_calibration_shot_usb() -> Optional[float]:
    """
    AE calibration shot for Sony USB mode.
    Sets camera to Program Auto, captures one frame, reads EXIF SS/ISO,
    computes pixel-EV from the embedded JPEG preview, seeds HG tracker,
    then locks camera back to manual with the measured values.
    Returns pixel-EV on success, None for night cold-start.
    """
    global _last_hg_params

    # Night cold-start check — same -6° threshold as picam path
    try:
        from astral.sun import elevation as _sun_el
        _sun_alt = _sun_el(hg._location.observer, datetime.datetime.now(hg._tzinfo))
    except Exception:
        _sun_alt = 0.0

    if _sun_alt < -6.0:
        hg.settings.anchor_shutter_s = None
        hg.settings.anchor_iso       = None
        hg.settings.anchor_ev        = None
        kelvin_night = float(hg._kelvin_for_phase(_sun_alt))
        hg._tracker._last_kelvin = kelvin_night
        logger.info("USB HG Cal: night cold-start — AE skipped, no-anchor night path active.")
        return None

    # Switch camera to Program Auto + AWB for AE measurement
    try:
        subprocess.run(
            ["gphoto2", "--set-config", "expprogram=P",
             "--set-config", "whitebalance=Automatic"],
            capture_output=True, text=True, timeout=10)
    except Exception as e:
        logger.warning(f"USB HG cal: AE mode set failed: {e}")

    # Capture AE frame to a temporary calibration directory
    cal_dir  = os.path.join(state["save_path"], "hg_cal")
    os.makedirs(cal_dir, exist_ok=True)
    cal_path = os.path.join(cal_dir, "cal_ae.ARW")
    try:
        subprocess.run(
            ["gphoto2",
             "--capture-image-and-download",
             "--filename", cal_path, "--force-overwrite"],
            check=True, capture_output=True, text=True, timeout=30)
    except Exception as e:
        logger.error(f"USB HG cal: capture failed: {e}")
        return None

    # Read EXIF SS + ISO + FocalLength + LensModel via exiftool (single pass)
    shutter_s  = 1.0 / 125
    iso_equiv  = 400
    focal_mm   = None
    lens_model = None
    try:
        res = subprocess.run(
            ["exiftool", "-ShutterSpeed", "-ISO",
             "-FocalLength", "-LensModel", "-json", cal_path],
            capture_output=True, text=True, timeout=15)
        if res.returncode == 0 and res.stdout.strip().startswith("["):
            data  = json.loads(res.stdout)[0]
            ss_raw = str(data.get("ShutterSpeed", "1/125"))
            if "/" in ss_raw:
                num, den = ss_raw.split("/")
                shutter_s = float(num) / float(den)
            else:
                shutter_s = float(ss_raw) if ss_raw else 1 / 125
            iso_equiv = int(data.get("ISO", 400))
            # Focal length — present for native lenses and manual lenses
            # with Sony IBIS focal length set in-body
            fl_raw = data.get("FocalLength")
            if fl_raw is not None:
                try:
                    focal_mm = float(str(fl_raw).replace("mm", "").strip())
                    if focal_mm <= 0:
                        focal_mm = None
                except (ValueError, TypeError):
                    focal_mm = None
            lens_model = data.get("LensModel") or None
            if lens_model in ("----", ""):
                lens_model = None
    except Exception as e:
        logger.warning(f"USB HG cal: EXIF parse failed: {e}")

    # Broadcast lens data if focal length was detected
    if focal_mm:
        cam  = state.get("active_camera", "sony_usb")
        ori  = state.get("camera_orientation", "landscape")
        hfov, vfov = _compute_fov(focal_mm, cam, ori)
        label = f"{lens_model} — " if lens_model else ""
        logger.info(f"USB HG cal: lens {label}{focal_mm:.0f}mm → HFOV={hfov}° VFOV={vfov}°")
        # Broadcast — asyncio.run_coroutine_threadsafe not available in sync context;
        # store in state for the async layer to pick up and send
        state["_pending_lens_info"] = {
            "type":       "lens_info",
            "focal_mm":   focal_mm,
            "lens_model": lens_model or "",
            "hfov":       hfov,
            "vfov":       vfov,
            "source":     "hg_calibration_usb",
        }

    # Compute pixel-EV from embedded JPEG preview (same method as picam path)
    ev_measured = None
    try:
        prev_res = subprocess.run(
            ["exiftool", "-b", "-PreviewImage", cal_path],
            capture_output=True, timeout=15)
        if prev_res.returncode == 0 and prev_res.stdout:
            arr       = np.frombuffer(prev_res.stdout, dtype=np.uint8)
            frame_bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame_bgr is not None:
                f_arr    = frame_bgr.astype(float)
                # OpenCV is BGR — use correct channel weights
                lum_arr  = (0.2126 * f_arr[:, :, 2]
                          + 0.7152 * f_arr[:, :, 1]
                          + 0.0722 * f_arr[:, :, 0])
                hi    = float(np.percentile(lum_arr, 90))
                lo    = float(np.percentile(lum_arr,  5))
                valid = (lum_arr >= lo) & (lum_arr <= hi) & (lum_arr > 0)
                lum_mean   = (float(np.mean(lum_arr[valid]))
                              if np.any(valid) else 128.0)
                lum_linear = (max(lum_mean, 1.0) / 255.0) ** 2.2
                ev_measured = math.log2(max(lum_linear, 1e-6) / 0.18) + 12.0
    except Exception as e:
        logger.warning(f"USB HG cal: pixel-EV compute failed: {e}")

    if ev_measured is None:
        # Fallback: derive EV from EXIF (less accurate than pixel-EV but usable)
        ev_measured = (math.log2(max(1.0 / max(shutter_s, 1e-6), 1e-6))
                     + math.log2(max(iso_equiv / 100.0, 1e-6)))
        logger.info(f"USB HG cal: pixel-EV unavailable, using EXIF EV≈{ev_measured:.2f}")

    # Seed Kelvin — use configured day target as the best available estimate
    kelvin_measured = float(getattr(hg.settings, "kelvin_day", 5500))

    logger.info(
        f"USB HG Cal: SS={shutter_s:.4f}s ISO={iso_equiv} "
        f"pixel-EV={ev_measured:.2f} K≈{kelvin_measured:.0f}")

    hg.seed_from_calibration(ev_measured, kelvin_measured)
    hg.settings.anchor_shutter_s = shutter_s
    hg.settings.anchor_iso       = iso_equiv
    hg.settings.anchor_ev        = ev_measured

    # Lock camera back to manual with calibrated values
    set_sony_settings_usb(
        ae=False, awb=False,
        shutter_s=shutter_s, iso=iso_equiv, kelvin=int(kelvin_measured))
    _last_hg_params = {"shutter_s": shutter_s, "iso": iso_equiv,
                       "kelvin": kelvin_measured, "phase": "day"}
    logger.info("USB HG Cal: camera locked to manual control.")
    return ev_measured


def _write_macro_sidecar(file_path: str, slot: "ExposureSlot"):
    """XMP sidecar with macro rig position and slot metadata."""
    xmp_path = file_path.replace(".dng", ".xmp").replace(".ARW", ".xmp")
    xmp = f"""<?xpacket begin='' id='W5M0MpCehiHzreSzNTczkc9d'?>
<x:xmpmeta xmlns:x='adobe:ns:meta/'>
  <rdf:RDF xmlns:rdf='http://www.w3.org/1999/02/22-rdf-syntax-ns#'>
    <rdf:Description rdf:about=''
      xmlns:ps='http://ns.pislider.io/1.0/'>
      <ps:Macro_Slot>{slot.id}</ps:Macro_Slot>
      <ps:Macro_SlotLabel>{slot.label}</ps:Macro_SlotLabel>
      <ps:Macro_Relay1>{slot.relay1}</ps:Macro_Relay1>
      <ps:Macro_Relay2>{slot.relay2}</ps:Macro_Relay2>
      <ps:Macro_ISO>{slot.iso}</ps:Macro_ISO>
      <ps:Macro_Shutter>{slot.shutter_s:.6f}</ps:Macro_Shutter>
      <ps:Macro_Kelvin>{slot.kelvin}</ps:Macro_Kelvin>
      <ps:Rig_Rail_MM>{slider_axis.current_mm:.4f}</ps:Rig_Rail_MM>
      <ps:Rig_Rotation_Deg>{pan_axis.current_deg:.4f}</ps:Rig_Rotation_Deg>
      <ps:Rig_Aux_Deg>{tilt_axis.current_deg:.4f}</ps:Rig_Aux_Deg>
    </rdf:Description>
  </rdf:RDF>
</x:xmpmeta>
<?xpacket end='w'?>"""
    try:
        with open(xmp_path, "w") as f:
            f.write(xmp)
    except Exception as e:
        logger.error(f"Macro XMP: {e}")


async def macro_apply_camera(slot: "ExposureSlot"):
    """Push a slot's camera settings before each macro capture.

    picam   — sets ExposureTime / AnalogueGain / ColourTemperature controls directly.
    sony_usb — pushes ISO / shutter / kelvin via gphoto2 before the capture command.
    sony    — no pre-push needed; camera retains its last manual settings via WiFi.
    """
    cam = state.get("active_camera", "picam")

    if cam == "picam":
        if not _HAS_PICAM or not picam:
            return
        try:
            controls = {
                "AeEnable":  slot.ae,
                "AwbEnable": slot.awb,
            }
            if not slot.ae:
                controls["ExposureTime"] = int(slot.shutter_s * 1_000_000)
                controls["AnalogueGain"] = slot.iso / 100.0
            if not slot.awb:
                controls["ColourTemperature"] = int(slot.kelvin)
            await asyncio.to_thread(picam.set_controls, controls)
            # Let camera settle for 2 frames at the new settings
            await asyncio.to_thread(picam.capture_array)
            await asyncio.to_thread(picam.capture_array)
        except Exception as e:
            logger.error(f"macro_apply_camera picam: {e}")

    elif cam == "sony_usb":
        try:
            await asyncio.to_thread(
                set_sony_settings_usb,
                slot.ae, slot.awb, slot.shutter_s, slot.iso, slot.kelvin)
            logger.info(f"macro_apply_camera sony_usb: ISO={slot.iso} "
                        f"shutter={slot.shutter_s:.4f}s kelvin={slot.kelvin}")
        except Exception as e:
            logger.error(f"macro_apply_camera sony_usb: {e}")

    elif cam == "sony":
        # Sony WiFi: settings are managed manually on the camera body.
        # Log the intended values for reference only.
        logger.info(f"macro_apply_camera sony (manual): ISO={slot.iso} "
                    f"shutter={slot.shutter_s:.4f}s kelvin={slot.kelvin} "
                    f"— set these on the camera before starting the stack.")
# Frame-differencing approach:
#   1. Convert ROI crop to grayscale, apply Gaussian blur to kill sensor noise
#   2. Absolute difference between current and previous frame
#   3. Threshold the diff image → binary mask of changed pixels
#   4. Morphological close to merge nearby blobs (fills gaps in a car body)
#   5. Find contours, sum area of contours above min_contour_area
#   6. Trigger if total changed area (px²) exceeds user threshold
#   7. Temporal debounce: N consecutive trigger frames required
#
# Why this beats Lucas-Kanade for this use case:
#   - Works in any lighting, any texture (LK needs trackable corners)
#   - No feature seeding needed — instant response to any change
#   - Contour area maps directly to "how much of the ROI changed" — intuitive to tune
#   - Morphological close prevents a car being ignored because its body is smooth

_motion_prev_gray: Optional[np.ndarray] = None
_motion_consec_hits: int = 0
_MOTION_CONSEC_REQUIRED = 1      # trigger immediately on first frame
_MOTION_MIN_CONTOUR_PX  = 150    # ignore tiny noise blobs below this area (px²)
_MOTION_BLUR_K          = 5      # Gaussian blur kernel — higher = less noise sensitivity

# Background Pi camera frame capture task for motion detection.
# Only started when Sony is the capture camera and picam_motion_* trigger is
# active — in that scenario /video_feed serves Sony (or nothing), so the
# MJPEG generator never runs and _last_frame goes stale.
_picam_bg_capture_task: Optional[asyncio.Task] = None


async def _picam_bg_capture_loop():
    """
    Capture Pi camera frames at ~20 fps and update _last_frame.

    Runs as a background task during timelapse when Sony is the capture camera
    and a picam_motion_* trigger mode is active.  The motion_detection_loop()
    reads _last_frame; without this task the buffer would never refresh and
    motion would never fire.

    The Pi camera stays in auto-exposure (whatever controls were last applied)
    so it remains sensitive to scene changes — good for motion detection.
    """
    global _last_frame
    logger.info("PiCam background capture loop started (motion detection feed).")
    while True:
        if _HAS_PICAM and picam:
            try:
                frame = await asyncio.to_thread(picam.capture_array)
                _last_frame = frame
            except Exception as _bg_err:
                logger.debug(f"PiCam bg capture: {_bg_err}")
        await asyncio.sleep(0.05)   # ~20 fps — keeps pace with motion_detection_loop
    # (loop exits only when the task is cancelled)


def _check_motion_in_roi(frame: np.ndarray) -> bool:
    """
    Frame-differencing motion detector.

    Returns True when:
    - Total area of changed-pixel blobs in the ROI exceeds the user threshold
    - This persists for _MOTION_CONSEC_REQUIRED consecutive frames

    motion_threshold in state is stored as px² directly (e.g. 2000 = 2000 px² of change).
    A typical car crossing a 50%-wide ROI at 640×480 occupies ~8000–20000 px² depending on
    distance — so default of 2000 is a conservative trigger well above noise.
    """
    global _motion_prev_gray, _motion_consec_hits

    h, w = frame.shape[:2]
    roi  = state["motion_roi"]
    x1, y1 = int(roi[0]*w), int(roi[1]*h)
    x2, y2 = int(roi[2]*w), int(roi[3]*h)
    if x2 <= x1 or y2 <= y1:
        return False

    crop = frame[y1:y2, x1:x2]
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY) if len(crop.shape) == 3 else crop.copy()
    gray = cv2.GaussianBlur(gray, (_MOTION_BLUR_K, _MOTION_BLUR_K), 0)

    if _motion_prev_gray is None or _motion_prev_gray.shape != gray.shape:
        _motion_prev_gray = gray
        _motion_consec_hits = 0
        return False

    # Absolute difference
    diff = cv2.absdiff(_motion_prev_gray, gray)
    _motion_prev_gray = gray

    # Threshold — pixels that changed by more than ~15/255
    _, thresh = cv2.threshold(diff, 15, 255, cv2.THRESH_BINARY)

    # Morphological close: merge horizontally separated parts of a single vehicle
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 5))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    # Find contours and filter for horizontally dominant shapes (cars/bikes)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    changed_area = 0
    for c in contours:
        if cv2.contourArea(c) >= _MOTION_MIN_CONTOUR_PX:
            _, _, w, h = cv2.boundingRect(c)
            # Require the blob to be wider than it is tall (horizontal movement)
            if w > h * 1.1:
                changed_area += cv2.contourArea(c)

    threshold_px2 = state.get("motion_threshold", 2000)
    triggered = changed_area >= threshold_px2

    if triggered:
        _motion_consec_hits += 1
    else:
        _motion_consec_hits = 0

    return _motion_consec_hits >= _MOTION_CONSEC_REQUIRED


async def motion_detection_loop():
    """
    Background task: continuously check _last_frame for motion using frame differencing.
    Runs at ~20fps to ensure lowest latency for fast horizontal objects.
    """
    # Motion Loop Variables
    global _motion_triggered, _motion_prev_gray, _motion_consec_hits
    _motion_prev_gray   = None
    _motion_consec_hits = 0
    warmup              = state.get("motion_warmup_frames", 10)
    last_trigger_time   = 0.0
    count               = 0
    logger.info("Motion detection loop started (frame differencing).")
    while state["is_running"]:
        tmode = state["trigger_mode"]
        if not tmode.startswith("picam_motion"):
            await asyncio.sleep(0.05)
            continue
        frame = _last_frame
        if frame is not None:
            count += 1
            if count > warmup and not _motion_triggered:
                if _check_motion_in_roi(frame):
                    cooldown = state.get("motion_cooldown", 2.0)
                    now      = time.time()
                    if now - last_trigger_time >= cooldown:
                        _motion_triggered = True
                        last_trigger_time = now
                        logger.info("MOTION: frame-diff trigger fired.")
            else:
                _check_motion_in_roi(frame)   # keep prev_gray current during warmup
        await asyncio.sleep(0.05)   # ~20fps for immediate response
    logger.info("Motion detection loop ended.")


# ─── SOFT LIMITS ──────────────────────────────────────────────────────────────
def clamp_pan(v):  return max(state["pan_min"],  min(state["pan_max"],  v))
def clamp_tilt(v): return max(state["tilt_min"], min(state["tilt_max"], v))

def _check_path_vs_limits(traj_s, traj_p, traj_t):
    """
    Check trajectory arrays against current soft limits.
    Only checks axes with both ends calibrated.
    Returns a list of violation dicts (empty = clear to run).
    Each dict: {axis, end, needed, current, expand_by, unit}
    """
    violations = []
    TOL = 0.5   # ignore sub-half-unit floating-point overshoot
    checks = [
        ("Slider", "min", float(np.min(traj_s)), _soft_guard.slider.min_unit, "mm"),
        ("Slider", "max", float(np.max(traj_s)), _soft_guard.slider.max_unit, "mm"),
        ("Pan",    "min", float(np.min(traj_p)), _soft_guard.pan.min_unit,    "°"),
        ("Pan",    "max", float(np.max(traj_p)), _soft_guard.pan.max_unit,    "°"),
        ("Tilt",   "min", float(np.min(traj_t)), _soft_guard.tilt.min_unit,   "°"),
        ("Tilt",   "max", float(np.max(traj_t)), _soft_guard.tilt.max_unit,   "°"),
    ]
    for axis, end, path_val, limit_val, unit in checks:
        if limit_val is None:
            continue   # uncalibrated — no boundary to check
        if end == "min" and path_val < limit_val - TOL:
            violations.append({
                "axis": axis, "end": end,
                "needed":    round(path_val, 1),
                "current":   round(limit_val, 1),
                "expand_by": round(limit_val - path_val, 1),
                "unit": unit,
            })
        elif end == "max" and path_val > limit_val + TOL:
            violations.append({
                "axis": axis, "end": end,
                "needed":    round(path_val, 1),
                "current":   round(limit_val, 1),
                "expand_by": round(path_val - limit_val, 1),
                "unit": unit,
            })
    return violations

def _expand_limits_to_fit(traj_s, traj_p, traj_t):
    """
    Widen soft limits so the full trajectory fits with a small safety margin.
    Only expands axes that already have a limit set (never creates new ones).
    """
    S_MARGIN = 2.0   # mm extra beyond path extremes
    D_MARGIN = 1.0   # degree extra beyond path extremes
    for guard_ax, vals, margin in [
        (_soft_guard.slider, traj_s, S_MARGIN),
        (_soft_guard.pan,    traj_p, D_MARGIN),
        (_soft_guard.tilt,   traj_t, D_MARGIN),
    ]:
        v_min = float(np.min(vals))
        v_max = float(np.max(vals))
        if guard_ax.min_unit is not None:
            guard_ax.min_unit = min(guard_ax.min_unit, v_min - margin)
            guard_ax._update_cal()
        if guard_ax.max_unit is not None:
            guard_ax.max_unit = max(guard_ax.max_unit, v_max + margin)
            guard_ax._update_cal()
    # Sync state mirrors
    state["pan_min"]  = _soft_guard.pan.min_unit
    state["pan_max"]  = _soft_guard.pan.max_unit
    state["tilt_min"] = _soft_guard.tilt.min_unit
    state["tilt_max"] = _soft_guard.tilt.max_unit


# ─── SEQUENCE PROGRESS ESTIMATES ──────────────────────────────────────────────
def _estimate_progress(current_frame: int, current_interval: float) -> dict:
    """
    Return estimated total_frames (if time-based) or estimated end-time
    (if frame-count based), plus current interval.
    """
    remaining = state["total_frames"] - current_frame
    secs_left = remaining * current_interval
    est_end   = datetime.datetime.now() + datetime.timedelta(seconds=secs_left)
    return {
        "current_interval": current_interval,
        "estimated_end":    est_end.strftime("%H:%M:%S"),
        "estimated_end_ts": est_end.isoformat(),
        "secs_remaining":   int(secs_left),
    }


# ─── TIMELAPSE WORKER ─────────────────────────────────────────────────────────
async def timelapse_worker(base_interval: float):
    global _latest_shot, _motion_triggered
    try:
        await _timelapse_worker_inner(base_interval)
    except asyncio.CancelledError:
        reason = f"Cancelled at frame {state['current_frame']} of {state['total_frames']}."
        logger.info(f"Timelapse worker cancelled: {reason}")
        state["_stop_reason"] = reason
        _save_session_history()
        save_session()
    except Exception as e:
        import traceback as _tb
        reason = (f"⛔ CRASHED at frame {state['current_frame']} of {state['total_frames']}: "
                  f"{type(e).__name__}: {e}")
        logger.error(f"Timelapse worker crashed: {e}", exc_info=True)
        state["is_running"]  = False
        state["stop_event"].clear()
        state["_stop_reason"] = reason   # persisted — visible to reconnecting browser
        _save_session_history()          # flush graph data so graph is complete on reconnect
        save_session()                   # persist stop_reason through server restart
        status_leds.set_error()
        await broadcast({"type": "run_state",  "running": False})
        await broadcast({"type": "stop_reason","msg": reason})
        await broadcast({"type": "log",        "msg": reason})
        # Also write crash detail to a dedicated file next to the frames so
        # you always have it even if the server restarts before you reconnect.
        try:
            crash_path = os.path.join(state["save_path"], "CRASH_REPORT.txt")
            with open(crash_path, "w") as _cf:
                _cf.write(f"{reason}\n\n")
                _cf.write(_tb.format_exc())
            logger.info(f"Crash report written to {crash_path}")
        except Exception:
            pass
    finally:
        # Always restart InertiaEngine after timelapse (normal end, crash, or cancel).
        # This is the authoritative restart point — it runs AFTER move_axes_simultaneous
        # has finished, so there is no STEP-pin conflict between Bresenham gpio_write
        # and InertiaEngine tx_pwm.
        if _inertia:
            _inertia.set_target(0, 0, 0)   # zero stale targets before restart
            _inertia.set_preset("responsive")
            if not _inertia._running:
                _inertia.start()
        # Notify clients that movement control is live again (important after E-stop
        # where the stop handler deliberately deferred the InertiaEngine restart).
        asyncio.create_task(broadcast({"type": "control_resumed"}))
        # Restart liveview if the sequence paused it — runs even on crash/cancel
        if _timelapse_paused_liveview and state.get("active_camera") == "sony":
            _start_sony_liveview()
            logger.info("Sony WiFi liveview restarted after timelapse sequence")
        elif _timelapse_paused_liveview and state.get("active_camera") == "sony_usb":
            _start_sony_usb_liveview()
            logger.info("Sony USB liveview restarted after timelapse sequence")
        # Stop the background Pi camera capture task if it was started
        if _picam_bg_capture_task and not _picam_bg_capture_task.done():
            _picam_bg_capture_task.cancel()
            _picam_bg_capture_task = None
            logger.info("PiCam background capture stopped.")
        # Write motion sidecar for every completed (or partial) run
        _save_session_history()
        try:
            sidecar_path = os.path.join(state["save_path"], "MOTION.json")
            _write_motion_sidecar(sidecar_path)
        except Exception as _se:
            logger.warning(f"Motion sidecar write failed: {_se}")


async def _timelapse_worker_inner(base_interval: float):
    global _latest_shot, _motion_triggered, _timelapse_paused_liveview, _picam_bg_capture_task

    # Clear cached lens data so a lens swap between sequences gets re-detected
    state.pop("_sony_focal_mm",   None)
    state.pop("_sony_lens_model", None)
    # Clear card filename counter so it re-bootstraps from the new sequence's
    # first frame (card counter may have rolled or changed between sessions)
    state.pop("_sony_card_seq",        None)
    state.pop("_last_sony_basename",   None)

    # Ensure engine.keyframes is in sync with _prog_move keyframes before the
    # sequence starts.  The motion gate at the per-frame step checks
    # `if len(engine.keyframes) >= 2` — if sync was missed it silently skips
    # every motor move.  Calling sync here guarantees the gate passes whenever
    # the user has programmed a move, regardless of what happened before.
    sync_all_keyframes()
    logger.info(
        f"Timelapse start: engine.keyframes={len(engine.keyframes)}, "
        f"prog_move keyframes={len(_prog_move.keyframes) if _prog_move else 0}"
    )

    # Pause InertiaEngine before Bresenham stepping starts — both use the same
    # STEP pins and would conflict if running concurrently.
    if _inertia and _inertia._running:
        _inertia.stop()

    # Stop Sony liveview during the timelapse.  The A7III only allows one HTTP
    # consumer at a time; the exposure-set + capture API calls already occupy
    # that slot.  _timelapse_paused_liveview lets the outer timelapse_worker
    # finally block restart it even if this inner function raises an exception.
    _timelapse_paused_liveview = False
    if state.get("active_camera") == "sony" and _sony_liveview_running:
        _timelapse_paused_liveview = True
        _stop_sony_liveview()
        logger.info("Sony WiFi liveview paused for timelapse sequence")
    elif state.get("active_camera") == "sony_usb" and _sony_usb_liveview_running:
        _timelapse_paused_liveview = True
        _stop_sony_usb_liveview()
        logger.info("Sony USB liveview paused for timelapse sequence (gphoto2 needs exclusive USB)")

    # When a picam_motion_* trigger is active, ensure _last_frame stays fresh
    # for motion_detection_loop() throughout the sequence.
    #
    # Why this is needed:
    #   • Sony active: /video_feed serves Sony liveview (or nothing), so the
    #     PiCam MJPEG generator never runs → _last_frame goes stale immediately.
    #   • PiCam active: /video_feed does feed _last_frame, but only while a
    #     browser tab is streaming it — a disconnected browser leaves the buffer
    #     stale and motion never fires.
    # The background task runs independently of the web feed in both cases.
    _picam_bg_capture_task = None
    if state["trigger_mode"].startswith("picam_motion") and _HAS_PICAM and picam:
        _picam_bg_capture_task = asyncio.create_task(_picam_bg_capture_loop())
        cam_label = state.get("active_camera", "?")
        logger.info(f"PiCam background capture started for motion-detect timelapse "
                    f"(capture camera: {cam_label}).")

    state["is_running"]    = True
    state["current_frame"] = 0
    state.pop("_capture_error", None)   # clear any stale error from a previous run
    state["_stop_reason"]  = ""         # clear previous stop reason for new run
    os.makedirs(state["save_path"], exist_ok=True)

    # ── Warn if reusing a folder that already has frames ──────────────────────
    _existing_frames = [
        f for f in os.listdir(state["save_path"])
        if f.startswith("FRAME_") and (f.endswith(".dng") or f.endswith(".ARW"))
    ]
    if _existing_frames:
        _warn_msg = (
            f"⚠ Save folder already contains {len(_existing_frames)} existing frame(s). "
            "New frames will overwrite them. Use a new folder to keep the old sequence."
        )
        logger.warning(_warn_msg)
        await broadcast({"type": "log", "msg": _warn_msg})

    # ── Scheduled start: wait until start time if set ─────────────────────────
    schedule_start = state.get("schedule_start")
    if schedule_start:
        try:
            from zoneinfo import ZoneInfo as _ZoneInfo
            _sched_tz_name = state.get("schedule_tz",
                                       state.get("timezone", "America/Chicago"))
            _sched_tz = _ZoneInfo(_sched_tz_name)

            # JS sends the raw datetime-local value ("2026-04-24T04:30") plus
            # schedule_tz so we can localise it correctly.  Old clients may
            # still send a UTC ISO string ending in "Z" or "+00:00" — handle
            # both so a server restart doesn't break in-flight sessions.
            if schedule_start.endswith("Z") or "+" in schedule_start[10:]:
                # Legacy UTC ISO path
                start_dt = datetime.datetime.fromisoformat(
                    schedule_start.replace("Z", "+00:00"))
            else:
                # New path: treat as local time in the configured app timezone
                start_dt = datetime.datetime.fromisoformat(
                    schedule_start).replace(tzinfo=_sched_tz)

            now_dt  = datetime.datetime.now(datetime.timezone.utc)
            wait_s  = (start_dt - now_dt).total_seconds()
            _target_local = start_dt.astimezone(_sched_tz).strftime('%I:%M %p %Z')
            logger.info(
                f"Scheduled start: target={_target_local} wait={wait_s:.0f}s "
                f"tz={_sched_tz_name}")

            if wait_s > 0:
                await broadcast({"type": "log",
                    "msg": f"⏳ Scheduled start: sequence will begin at "
                           f"{_target_local} (in {wait_s/60:.1f} min). "
                           f"Click E-Stop to cancel."})
                # Mark scheduled-wait so reconnecting browsers show countdown,
                # not "SEQUENCE RUNNING".
                state["_scheduled_waiting"] = True
                state["_scheduled_target"]  = _target_local
                # Broadcast scheduled_wait so the UI shows a countdown, NOT
                # run_state:True (which would show "SEQUENCE RUNNING").
                await broadcast({"type": "scheduled_wait",
                    "seconds_remaining": int(wait_s),
                    "target_time": _target_local})
                while wait_s > 0 and not state["stop_event"].is_set():
                    await asyncio.sleep(min(5.0, wait_s))
                    # Recompute from wall clock — avoids drift in long waits
                    wait_s = (start_dt - datetime.datetime.now(datetime.timezone.utc)).total_seconds()
                    if wait_s > 0:
                        await broadcast({"type": "scheduled_wait",
                            "seconds_remaining": int(wait_s),
                            "target_time": _target_local})
                state.pop("_scheduled_waiting", None)
                state.pop("_scheduled_target",  None)
                if state["stop_event"].is_set():
                    state["is_running"] = False
                    await broadcast({"type": "scheduled_wait_cancelled"})
                    await broadcast({"type": "run_state", "running": False})
                    await broadcast({"type": "log", "msg": "⛔ Scheduled start cancelled."})
                    return
                await broadcast({"type": "scheduled_wait_done"})
                await broadcast({"type": "log",
                    "msg": f"[✓] Scheduled start time reached — beginning sequence."})
            else:
                await broadcast({"type": "log",
                    "msg": f"⚠ Scheduled start time already passed by "
                           f"{-wait_s/60:.1f} min — starting now."})
        except Exception as e:
            logger.warning(f"schedule_start error ({e}) — starting immediately.")
            await broadcast({"type": "log",
                "msg": f"⚠ Schedule parse error: {e} — starting immediately."})

    await broadcast({"type": "run_state", "running": True})

    # Reset HG anchor so each sequence gets a fresh calibration
    hg.settings.anchor_shutter_s = None
    hg.settings.anchor_iso       = None
    hg.settings.anchor_ev        = None

    # Clear graph history — new sequence = new graph
    global _timelapse_run_id, _seq_wall_start
    _timelapse_run_id  = str(int(time.time()))
    _seq_wall_start    = time.time()
    state["_motion_start_wall"] = _seq_wall_start
    _session_history.clear()
    try:
        _SESSION_HISTORY_FILE.write_text('{"frames":[]}')
    except Exception:
        pass
    await broadcast({"type": "graph_reset", "run_id": _timelapse_run_id})

    # ── Disk space pre-flight ──────────────────────────────────────────────────
    try:
        usage = shutil.disk_usage(state["save_path"])
        cam  = state.get("active_camera", "picam")
        if cam == "sony":
            # Sony WiFi mode: RAW/JPEG stay on the camera card — the Pi only
            # stores a thumbnail (~300 KB) and a tiny XMP sidecar per frame.
            # Don't try to estimate full-size RAW space on the Pi.
            mb_per_frame = 0.5   # thumbnail + XMP only
            fmt_label    = "thumb+XMP"
        elif cam == "sony_usb":
            # Sony USB/tethered: full ARW downloaded to Pi (~30 MB each)
            mb_per_frame = 30.0
            fmt_label    = "ARW"
        else:
            # PiCamera: DNG ≈ 25 MB (IMX477 sensor raw)
            mb_per_frame = 25.0
            fmt_label    = "DNG"
        needed_mb  = state["total_frames"] * mb_per_frame
        free_mb    = usage.free / (1024 * 1024)
        if free_mb < needed_mb:
            msg = (f"⚠ DISK WARNING: ~{needed_mb:.0f} MB needed "
                   f"({state['total_frames']} × ~{mb_per_frame:.1f} MB {fmt_label}), "
                   f"only {free_mb:.0f} MB free. Sequence may fail before completion.")
            await broadcast({"type": "log", "msg": msg})
            logger.warning(msg)
        if free_mb < mb_per_frame * 2:
            # Less than 2 frames of space — abort
            err = f"⛔ DISK FULL: only {free_mb:.0f} MB free on Pi. Sequence aborted."
            await broadcast({"type": "disk_full", "msg": err})
            logger.error(err)
            state["is_running"] = False
            await broadcast({"type": "run_state", "running": False})
            return
    except Exception as e:
        logger.warning(f"Disk pre-flight check failed: {e}")

    # ── HG Calibration Phase ─────────────────────────────────────────────────
    # The calibration phase runs BEFORE the sequence loop and frame counter.
    # Motion does NOT start until calibration is complete.
    # Files are saved as CAL_0000.dng / CAL_0001.dng — separate from the
    # sequence FRAME_xxxx files — so they never appear in the edit timeline.
    #
    # Step 1: AE-settle shot (preview only, no file saved)
    #         → seeds EV tracker from camera metadata
    # Step 2: CAL_0000 — first real captured frame
    #         → reads EXIF aperture (if available) and refines anchor_ev
    #         → pushes full-res frame into HG sky analyser
    # Step 3: CAL_0001 — second captured frame at the now-correct exposure
    #         → further warms the tracker so frame 1 of the sequence is stable
    #
    _night_cold_start = False   # set True below if HG calibration detects night
    _dawn_cal_failures = 0      # consecutive dawn AE calibration failures

    # Holy Grail requires the Sony WiFi HTTP API to set exposure each frame.
    # S2 cable mode has no API link — warn user and skip HG for this run.
    if hg.settings.enabled and state.get("active_camera") in ("s2", "sony_s2"):
        await broadcast({"type": "log",
            "msg": ("⚠ Holy Grail skipped — S2 cable mode cannot control camera "
                    "exposure settings.  Manage exposure manually on the camera body.")})
        logger.warning("HG disabled for this run: active_camera=%s has no API link.",
                       state.get("active_camera"))
        hg.settings.enabled = False

    if hg.settings.enabled:
        await broadcast({"type": "log", "msg": "HG: starting calibration…"})
        logger.info("HG: starting calibration phase (2 cal frames before sequence).")

        # Step 1 — AE settle + anchor (returns None for night cold-start)
        if state.get("active_camera") == "sony_usb":
            ev = await asyncio.to_thread(hg_calibration_shot_usb)
            # Flush any lens_info detected by the sync calibration function
            lens_info = state.pop("_pending_lens_info", None)
            if lens_info:
                await broadcast(lens_info)
        else:
            ev = await asyncio.to_thread(hg_calibration_shot)
        try:
            from astral.sun import elevation as _sun_el_cal0
            _sun_alt_cal0 = _sun_el_cal0(hg._location.observer,
                                         datetime.datetime.now(hg._tzinfo))
        except Exception:
            _sun_alt_cal0 = 0.0
        _night_cold_start = (ev is None and hg.settings.anchor_ev is None
                             and _sun_alt_cal0 < -6.0)
        await broadcast({"type": "log",
            "msg": (f"HG cal: night cold-start — using max night settings "
                    f"({hg.settings.shutter_max_night:.0f}s / "
                    f"ISO{hg.settings.iso_max_night}). No AE anchor.")
                   if _night_cold_start
                   else (f"HG cal: AE settled EV≈{ev:.2f}" if ev
                         else "HG cal: camera unavailable, starting cold.")})

        # Steps 2 & 3 — two real captured frames, named CAL_xxxx
        for cal_idx in range(2):
            if state["stop_event"].is_set():
                break

            cal_params  = hg.get_next_shot_parameters()
            cal_phase   = cal_params.get("phase", "day")
            cal_shutter = cal_params.get("shutter_s", 1/125)

            # Apply HG exposure to camera
            if state.get("active_camera") == "sony_usb":
                await asyncio.to_thread(apply_sony_usb_from_hg, cal_params)
            elif state.get("active_camera") == "sony":
                await asyncio.to_thread(apply_sony_from_hg, cal_params)
            elif _HAS_PICAM and picam:
                await asyncio.to_thread(apply_picam_from_hg, cal_params)

            # Capture to CAL file — not counted, not moved by rig
            _cal_ok = False   # True if shutter fired successfully
            if state.get("active_camera") == "sony_usb":
                cal_dest = os.path.join(state["save_path"], f"CAL_{cal_idx:04d}.ARW")
                status_leds.set_sequence_phase("shutter", cal_shutter)
                cal_path = await asyncio.to_thread(
                    capture_sony_usb, f"CAL_{cal_idx:04d}", cal_shutter)
                _cal_ok = bool(cal_path)
            elif state.get("active_camera") == "sony":
                # Sony WiFi: shutter fires, JPEG thumbnail set in _latest_shot.
                # Anchor EV will be computed from the JPEG (not the ARW —
                # cv2/PIL read raw sensor data as nearly-black from ARW files).
                status_leds.set_sequence_phase("shutter", cal_shutter)
                cal_result = await asyncio.to_thread(
                    capture_sony, f"CAL_{cal_idx:04d}", cal_shutter)
                # "" = success (file on camera card); None = failed
                cal_path = cal_result   # keep for EXIF aperture read if ARW downloaded
                _cal_ok  = (cal_result is not None)
            else:
                cal_dest = os.path.join(state["save_path"], f"CAL_{cal_idx:04d}.dng")
                status_leds.set_sequence_phase("shutter", cal_shutter)
                cal_path = await asyncio.to_thread(_capture_cal_frame, cal_dest)
                _cal_ok = bool(cal_path)
            status_leds.set_sequence_phase("waiting")

            if _cal_ok:
                if state.get("active_camera") == "sony":
                    # ── Sony WiFi cal: anchor + seed tracker from JPEG thumbnail ──
                    # ARW pixel data is raw/linear and undecodable by PIL/cv2 into
                    # a valid RGB histogram — produces nearly-black and a wrong EV.
                    # _latest_shot is the JPEG postview set by _generate_sony_thumb
                    # during capture_sony; same image data the tracker meters at
                    # runtime, so the EV scale is consistent.
                    if cal_idx == 0 and not _night_cold_start:
                        _jpeg = _latest_shot
                        if _jpeg:
                            try:
                                import io as _io
                                from PIL import Image as _PILImg
                                import numpy as _np
                                _img = _PILImg.open(_io.BytesIO(_jpeg)).convert("RGB")
                                _img = _img.resize((640, 480), _PILImg.LANCZOS)
                                _arr = _np.array(_img)
                                _lum = (0.2126 * _arr[:,:,0].astype(float)
                                      + 0.7152 * _arr[:,:,1].astype(float)
                                      + 0.0722 * _arr[:,:,2].astype(float))
                                _p50 = float(_np.percentile(_lum, 50))
                                _lin = (max(_p50, 1.0) / 255.0) ** 2.2
                                cal_ev = math.log2(max(_lin, 1e-9) / 0.18) + 12.0
                                old_anchor = hg.settings.anchor_ev
                                hg.settings.anchor_ev        = cal_ev
                                hg.settings.anchor_shutter_s = cal_params.get("shutter_s", hg.settings.anchor_shutter_s)
                                hg.settings.anchor_iso       = cal_params.get("iso", hg.settings.anchor_iso)
                                _fmt = lambda v, spec: format(v, spec) if v is not None else "—"
                                msg = (f"HG Sony WiFi anchor from JPEG thumbnail: "
                                       f"EV {_fmt(old_anchor, '.2f')}→{cal_ev:.2f} "
                                       f"SS={_fmt(hg.settings.anchor_shutter_s, '.3f')}s "
                                       f"ISO={hg.settings.anchor_iso}")
                                logger.info(msg)
                                await broadcast({"type": "log", "msg": f"📷 {msg}"})
                            except Exception as _ae:
                                logger.warning(f"HG Sony WiFi JPEG anchor failed: {_ae}")
                        else:
                            await broadcast({"type": "log",
                                "msg": "⚠ HG Sony WiFi: no JPEG thumbnail for anchor — using astral model."})

                        # EXIF aperture from ARW if it was downloaded locally
                        if cal_path:
                            aperture_from_exif = await asyncio.to_thread(
                                _read_aperture_from_exif, cal_path
                            )
                            if aperture_from_exif is not None:
                                hg.settings.aperture_day   = aperture_from_exif
                                hg.settings.aperture_night = aperture_from_exif
                                await broadcast({"type": "log",
                                    "msg": f"HG EXIF aperture: f/{aperture_from_exif}"})

                    # Seed the tracker from the same JPEG the anchor was computed from
                    await asyncio.to_thread(meter_sony_from_shot)

                else:
                    # ── PiCam / Sony USB: pixel EV from decoded file ──────────
                    cal_frame = await asyncio.to_thread(_load_cal_frame_rgb, cal_path)

                    # Cal frame 0: re-anchor from actual capture pixels.
                    # SKIP for night cold-start: the no-anchor path already
                    # targets the correct max-night settings from hardware limits.
                    if cal_idx == 0 and cal_frame is not None and not _night_cold_start:
                        import numpy as _np
                        lum_arr = (0.2126 * cal_frame[:,:,0].astype(float)
                                 + 0.7152 * cal_frame[:,:,1].astype(float)
                                 + 0.0722 * cal_frame[:,:,2].astype(float))
                        hi = float(_np.percentile(lum_arr, 90))
                        lo = float(_np.percentile(lum_arr,  5))
                        valid = (lum_arr >= lo) & (lum_arr <= hi) & (lum_arr > 0)
                        lum_cal = float(_np.mean(lum_arr[valid])) if _np.any(valid) else 128.0
                        lum_lin = (max(lum_cal, 1.0) / 255.0) ** 2.2
                        cal_ev  = math.log2(max(lum_lin, 1e-6) / 0.18) + 12.0
                        old_anchor = hg.settings.anchor_ev
                        hg.settings.anchor_ev        = cal_ev
                        hg.settings.anchor_shutter_s = cal_params.get("shutter_s", hg.settings.anchor_shutter_s)
                        hg.settings.anchor_iso       = cal_params.get("iso", hg.settings.anchor_iso)
                        _fmt = lambda v, spec: format(v, spec) if v is not None else "—"
                        msg = (f"HG anchor re-calibrated from CAL_0000 pixels: "
                               f"EV {_fmt(old_anchor, '.2f')}→{cal_ev:.2f} "
                               f"SS={_fmt(hg.settings.anchor_shutter_s, '.3f')}s "
                               f"ISO={hg.settings.anchor_iso}")
                        logger.info(msg)
                        await broadcast({"type": "log", "msg": f"📷 {msg}"})

                        # Also check EXIF for aperture refinement (Sony USB only)
                        aperture_from_exif = await asyncio.to_thread(
                            _read_aperture_from_exif, cal_path
                        )
                        if aperture_from_exif is not None:
                            hg.settings.aperture_day   = aperture_from_exif
                            hg.settings.aperture_night = aperture_from_exif
                            await broadcast({"type": "log",
                                "msg": f"HG EXIF aperture: f/{aperture_from_exif}"})

                    # Push cal frame into HG tracker
                    if cal_frame is not None:
                        await asyncio.to_thread(hg.push_capture_frame, cal_frame)

                await broadcast({"type": "log",
                    "msg": f"HG cal frame {cal_idx + 1}/2 captured."})
            else:
                await broadcast({"type": "log",
                    "msg": f"HG cal frame {cal_idx + 1}/2 failed — continuing."})

            # Brief settle between cal frames
            await asyncio.sleep(0.5)

        await broadcast({"type": "log", "msg": "[✓] HG calibration complete — starting sequence."})
        logger.info("HG calibration phase complete.")

    # ── Start motion detection task — AFTER calibration ──────────────────────
    motion_task = None
    if state["trigger_mode"].startswith("picam_motion"):
        _motion_triggered = False
        motion_task = asyncio.create_task(motion_detection_loop())
        # Tell the user what's happening so the setup is clear
        cap_cam   = state.get("active_camera", "?")
        tmode_lbl = state["trigger_mode"]
        if cap_cam == "sony":
            await broadcast({"type": "log",
                "msg": (f"📷 Sony WiFi = capture camera  |  🎞 Pi Camera = motion detector  "
                        f"({tmode_lbl}).  Sequence fires on motion.")})
        elif cap_cam == "sony_usb":
            await broadcast({"type": "log",
                "msg": (f"📷 Sony USB = capture camera (files → Pi SSD)  |  "
                        f"🎞 Pi Camera = motion detector ({tmode_lbl}).  "
                        f"Sequence fires on motion.")})
        elif cap_cam == "sony_s2":
            await broadcast({"type": "log",
                "msg": (f"📷 Sony S2 cable = capture camera (~5ms trigger)  |  "
                        f"🎞 Pi Camera = motion detector ({tmode_lbl}).  "
                        f"Sequence fires on motion.")})
        else:
            await broadcast({"type": "log",
                "msg": f"📷 Pi Camera = capture + motion detector ({tmode_lbl})."})

    save_session()
    logger.info(f"Timelapse: {state['total_frames']} frames, mode={state['trigger_mode']}")

    # ── LED: sequence started ──────────────────────────────────────────────────
    active_mode = state.get("active_mode", "timelapse")
    status_leds.set_mode("timelapse_run" if active_mode == "timelapse" else "macro_run")
    status_leds.set_sequence_phase("waiting")

    _consecutive_failures = 0          # consecutive frame-save failures
    _MAX_CONSECUTIVE_FAILURES = 5      # halt sequence after this many in a row

    # ── Pre-position: move to first keyframe before the sequence starts ───────
    # Without this the first shot is taken wherever the rig happens to be,
    # causing the start position to not match keyframe[0].
    if _prog_move and len(_prog_move.keyframes) >= 2:
        try:
            orig_s = _prog_move.origin_slider
            orig_p = _prog_move.origin_pan
            orig_t = _prog_move.origin_tilt
            kf0 = _prog_move.keyframes[0]
            pre_s = kf0.slider_mm + orig_s
            pre_p = clamp_pan(kf0.pan_deg + orig_p)
            pre_t = clamp_tilt(kf0.tilt_deg + orig_t)
            ds = int((pre_s - slider_axis.current_mm) * slider_axis.steps_per_mm)
            dp = int((pre_p - pan_axis.current_deg)   * pan_axis.steps_per_deg)
            dt = int((pre_t - tilt_axis.current_deg)  * tilt_axis.steps_per_deg)
            if any([ds, dp, dt]):
                await broadcast({"type": "log",
                    "msg": f"Moving to first keyframe before sequence start…"})
                status_leds.set_sequence_phase("motors")
                await asyncio.to_thread(hw.move_axes_simultaneous, ds, dp, dt, 3.0)
                # Update position tracking — CRITICAL for focus rail macro mode
                slider_axis.current_steps += ds  # Track steps moved
                slider_axis.current_mm = pre_s
                pan_axis.current_deg   = pre_p
                tilt_axis.current_deg  = pre_t
                status_leds.set_sequence_phase("waiting")
                await asyncio.sleep(0.5)   # brief vibration settle
        except Exception as _pre_err:
            logger.warning(f"Pre-position to keyframe 0 failed: {_pre_err}")

    _interval_extended = False  # tracks whether interval is currently being auto-extended

    for i in range(state["total_frames"]):
        if state["stop_event"].is_set():
            break

        loop_start = asyncio.get_event_loop().time()

        # 1. HG parameters
        params   = hg.get_next_shot_parameters()
        # Only use HG's per-phase interval when HG is actually enabled.
        # When HG is disabled, get_next_shot_parameters() still returns
        # interval_sec (defaults 5.0) which would override the user's manual interval.
        interval = params.get("interval", base_interval) if hg.settings.enabled else base_interval

        # ── Night cold-start → sunrise anchor ────────────────────────────────
        # When the sequence started at night (anchor_ev=None), ev_smooth is on
        # the camera-EV scale from the night floor computation.  As the tracker
        # warms up it slowly drifts toward pixel-EV, but the scale mismatch
        # means the no-anchor day path runs 5-7 stops overexposed for up to an
        # hour after sunrise.
        #
        # Fix: the first time sun_alt crosses -6° during a night cold-start,
        # immediately run a fresh AE calibration to set anchor_ev.
        # hg_calibration_shot() handles AE settle, pixel-EV measurement, sets
        # anchor_shutter_s / anchor_iso / anchor_ev, locks camera to manual,
        # and seeds the tracker — exactly the same as the pre-sequence cal.
        # Trigger matches hg_calibration_shot()'s own night threshold (-6°).
        # At civil-twilight onset the sky is bright enough for AE to settle
        # reliably; we anchor here and every subsequent frame uses delta mode.
        if (hg.settings.enabled
                and _night_cold_start
                and hg.settings.anchor_ev is None
                and params.get("sun_alt", -90.0) > -6.0):
            _sun_alt_str = f"{params.get('sun_alt', 0.0):.1f}"
            await broadcast({"type": "log",
                "msg": f"☀ Civil-twilight threshold reached (sun alt={_sun_alt_str}°) — "
                       f"running dawn AE calibration to anchor daytime exposure…"})
            ev_dawn = await asyncio.to_thread(hg_calibration_shot)
            if ev_dawn is not None:
                # hg_calibration_shot() has already set anchor_ev/shutter/iso
                # and seeded the tracker with the pixel-EV from the AE frame.
                _night_cold_start = False   # don't re-trigger
                _dawn_cal_failures = 0
                await broadcast({"type": "log",
                    "msg": f"☀ Sunrise anchor set: EV={ev_dawn:.2f} "
                           f"SS={hg.settings.anchor_shutter_s:.4f}s "
                           f"ISO{hg.settings.anchor_iso} — "
                           f"switching to anchor-delta exposure mode."})
                # Re-fetch params with anchor now active so this frame
                # uses the correct delta-mode exposure, not the stale cold-start value.
                params   = hg.get_next_shot_parameters()
                interval = params.get("interval", base_interval) if hg.settings.enabled else base_interval
            else:
                _dawn_cal_failures += 1
                # Give up after 3 failures — hg_calibration_shot() returns None
                # for any non-PiCam setup (Sony WiFi/USB) and would otherwise
                # spam a calibration attempt every single frame for the rest of
                # the day.  After giving up, HG continues with anchor-free
                # exposure computation (EV targets from sun altitude only).
                if _dawn_cal_failures >= 3:
                    _night_cold_start = False
                    await broadcast({"type": "log",
                        "msg": (f"☀ Dawn AE calibration unavailable (no PiCam metering) — "
                                f"continuing with sun-altitude EV model. "
                                f"Sun alt={params.get('sun_alt', 0.0):.1f}°")})
                else:
                    # Clear anchor so HG recomputes from scratch next frame
                    hg.settings.anchor_shutter_s = None
                    hg.settings.anchor_iso       = None
                    hg.settings.anchor_ev        = None
                    await broadcast({"type": "log",
                        "msg": f"⚠ Sunrise AE calibration failed ({_dawn_cal_failures}/3) — will retry on next frame."})

        # Update HG phase LED
        status_leds.set_hg_phase(params.get("phase", "unknown"))

        # Update progress LED
        pct = (i / max(state["total_frames"] - 1, 1))
        status_leds.set_progress(pct)

        # 2. Apply exposure
        if hg.settings.enabled:
            if state.get("active_camera") == "sony_usb":
                await asyncio.to_thread(apply_sony_usb_from_hg, params)
            elif state.get("active_camera") == "sony":
                await asyncio.to_thread(apply_sony_from_hg, params)
            else:
                await asyncio.to_thread(apply_picam_from_hg, params)

        # 3. Trigger gating — LED: waiting
        status_leds.set_sequence_phase("waiting")
        state["aux_triggered"] = False
        _motion_triggered      = False
        tmode = state["trigger_mode"]

        def _triggered():
            """True if either aux GPIO or motion detection fired."""
            return state["aux_triggered"] or _motion_triggered

        if tmode in ("aux_only", "picam_motion_only"):
            while not _triggered() and not state["stop_event"].is_set():
                await asyncio.sleep(0.05)
            if state["stop_event"].is_set():
                break

        elif tmode in ("aux_hybrid", "picam_motion_hybrid"):
            deadline = loop_start + interval
            while (not _triggered()
                   and asyncio.get_event_loop().time() < deadline
                   and not state["stop_event"].is_set()):
                await asyncio.sleep(0.05)
            if state["stop_event"].is_set():
                break

        # 4. Shoot — LED: shutter
        # For motion timelapse the motors haven't moved yet (step 8 is AFTER
        # capture), so retrying here naturally holds the rig at the correct
        # position for this frame.  Each retry eats into the interval budget
        # rather than extending the total sequence, keeping motion smooth.
        shutter_s = params.get("shutter_s", 1/125)
        status_leds.set_sequence_phase("shutter", shutter_s)
        state["current_frame"] = i + 1
        file_path  = None
        capture_blocked = False
        _has_motion = len(engine.keyframes) >= 2
        _MAX_SHOT_RETRIES   = 3     # attempts after the first before giving up
        _SHOT_RETRY_DELAY_S = 2.0   # seconds to wait between retries

        _disk_full = False
        for _shot_attempt in range(_MAX_SHOT_RETRIES + 1):
            file_path       = None
            capture_blocked = False
            try:
                cam = state["active_camera"]
                if cam == "picam":
                    file_path = await asyncio.to_thread(capture_picam, f"{i:04d}")
                    capture_blocked = True
                    _last_capture_time = time.time()   # ISP settle guard
                elif cam == "sony":
                    file_path = await asyncio.to_thread(capture_sony, f"{i:04d}", shutter_s)
                elif cam == "sony_usb":
                    file_path = await asyncio.to_thread(capture_sony_usb, f"{i:04d}", shutter_s)
                elif cam == "sony_s2":
                    # Low-latency S2 hardware trigger — exposure managed on camera body.
                    # ~5ms GPIO pulse via optocoupler; files saved to camera SD card.
                    await asyncio.to_thread(hw.trigger_camera, 0.2)
                    file_path = ""   # S2 trigger always "succeeds" (no local download)
                elif cam == "s2":
                    await asyncio.to_thread(hw.trigger_camera, 0.2)
                    file_path = ""   # s2 trigger always "succeeds" (no download)
            except OSError as e:
                if e.errno == 28:  # ENOSPC — disk full, no point retrying
                    err = f"⛔ DISK FULL at frame {i+1}: sequence halted. Free space and restart."
                    logger.error(err)
                    await broadcast({"type": "disk_full", "msg": err})
                    _disk_full = True
                    break
                logger.error(f"Shot {i} attempt {_shot_attempt + 1}: {e}")
            except Exception as e:
                logger.error(f"Shot {i} attempt {_shot_attempt + 1}: {e}")

            # 5. Wait for exposure + margin (skip if PiCam blocked for us)
            if not capture_blocked:
                await asyncio.sleep(max(0.05, shutter_s + state["exp_margin"]))

            if file_path is not None:
                break   # success (file_path == "" is still a shutter success)

            # Capture failed — retry if attempts remain
            if _shot_attempt < _MAX_SHOT_RETRIES and not state["stop_event"].is_set():
                hold_msg = (
                    f"⚠ Frame {i + 1} capture failed "
                    f"(attempt {_shot_attempt + 1}/{_MAX_SHOT_RETRIES + 1})"
                    + (", holding position — retrying…" if _has_motion else " — retrying…")
                )
                logger.warning(hold_msg)
                await broadcast({"type": "log", "msg": hold_msg})
                status_leds.set_save_error()
                await asyncio.sleep(_SHOT_RETRY_DELAY_S)
                status_leds.set_sequence_phase("shutter", shutter_s)

        if _disk_full:
            break

        # 6. Sidecar metadata + post-capture HG tracker push + LED save status
        # file_path semantics:
        #   None  = capture failed (error)
        #   ""    = shutter triggered OK, file saved to camera card (Sony HTTP mode)
        #   "..." = captured and downloaded to local path
        shutter_fired = file_path is not None   # None = failure; "" or path = success
        if shutter_fired:
            _consecutive_failures = 0   # reset on any success
            if file_path:    # non-empty → local file available
                # Fold bulb timing error into sidecar params so write_sidecar
                # can emit an accurate crs:Exposure2012 correction.
                bulb_ev_error = state.get("_bulb_ev_error", 0.0)
                sidecar_params = dict(params)
                if bulb_ev_error:
                    sidecar_params["_bulb_ev_error"] = bulb_ev_error

                # ── Sidecar naming ────────────────────────────────────────────
                # Sony WiFi: the file is saved locally as FRAME_####.ARW but
                # the original on the camera card is DSC####.ARW.  Write the
                # sidecar using the camera's filename so Lightroom auto-applies
                # when importing from the card.  We don't need a FRAME_####.xmp
                # because editing is done from the camera card files.
                # PiCam / Sony USB: file IS the canonical copy, use file_path.
                sony_bn = state.pop("_last_sony_basename", None)
                if sony_bn and state.get("active_camera") in ("sony", "sony_s2"):
                    sidecar_target = os.path.join(state["save_path"], f"{sony_bn}.ARW")
                else:
                    sidecar_target = file_path
                await asyncio.to_thread(write_sidecar, sidecar_target, sidecar_params)

                # ── Focal length auto-detect (first frame only) ────────────────
            elif state.get("active_camera") in ("sony", "sony_s2"):
                # Sony HTTP / S2 mode: file lives on the camera card, not the Pi.
                # Use the camera-assigned filename (e.g. "DSC01234") if we captured
                # it from the actTakePicture URL — that way the XMP basename matches
                # the ARW on the card and Lightroom will auto-apply corrections.
                # Fall back to FRAME_{i:04d} for bulb/S2 shots where the URL may
                # not be available.
                bulb_ev_error = state.get("_bulb_ev_error", 0.0)
                sidecar_params = dict(params)
                if bulb_ev_error:
                    sidecar_params["_bulb_ev_error"] = bulb_ev_error
                sony_bn = state.pop("_last_sony_basename", None)
                phantom_name = f"{sony_bn}.ARW" if sony_bn else f"FRAME_{i:04d}.ARW"
                phantom = os.path.join(state["save_path"], phantom_name)
                await asyncio.to_thread(write_sidecar, phantom, sidecar_params)

                # ── WiFi focal length auto-detect (first frame only) ──────────
                # _generate_sony_thumb() already read the focal length from the
                # JPEG postview EXIF into state["_sony_focal_mm"]. Broadcast it
                # now so the UI can offer to update HFOV/VFOV and perspective.
                if i == 0:
                    fl = state.get("_sony_focal_mm")
                    lm = state.get("_sony_lens_model", "")
                    if fl and fl > 0:
                        cam  = state.get("active_camera", "sony")
                        ori  = state.get("camera_orientation", "landscape")
                        hfov, vfov = _compute_fov(fl, cam, ori)
                        await broadcast({
                            "type":       "lens_info",
                            "focal_mm":   fl,
                            "lens_model": lm,
                            "hfov":       hfov,
                            "vfov":       vfov,
                            "source":     "wifi_postview",
                        })
                        logger.info(
                            f"WiFi focal from JPEG EXIF: {fl:.0f}mm {lm} "
                            f"→ HFOV={hfov}° VFOV={vfov}°"
                        )
            await broadcast({"type": "shutter_event"})
            status_leds.set_save_ok()

            # NOTE: We do NOT push _last_frame here. The first preview frame
            # after a still capture is unreliable — the ISP hasn't settled back
            # to the manual exposure level yet. Pushing it causes a sawtooth
            # pattern (bad reading → overcorrect → correct back → repeat).
            # The preview loop pushes a correct metering frame every 2s instead.
        else:
            # ── Capture failed — tell the user immediately ─────────────────────
            _consecutive_failures += 1
            capture_err = state.pop("_capture_error",
                                    "Unknown error — check server logs.")
            err_msg = (
                f"⛔ Frame {i + 1} FAILED to save: {capture_err} "
                f"({_consecutive_failures} consecutive failure(s))"
            )
            logger.error(err_msg)
            await broadcast({"type": "log", "msg": err_msg})
            status_leds.set_save_error()

            # Halt if too many consecutive frames fail — something is systemically
            # wrong (disk full, camera error, wrong path, etc.) and continuing
            # silently would just waste the session.
            if _consecutive_failures >= _MAX_CONSECUTIVE_FAILURES:
                halt_msg = (
                    f"⛔ SEQUENCE HALTED — {_consecutive_failures} consecutive frames "
                    f"failed to save. Check: camera connection, save path "
                    f"({state['save_path']}), disk space, and server logs."
                )
                logger.error(halt_msg)
                state["_stop_reason"] = halt_msg   # persisted for reconnecting browser
                await broadcast({"type": "log",        "msg": halt_msg})
                await broadcast({"type": "seq_halted", "msg": halt_msg})
                break

        # Broadcast disc-entry warnings if sun/moon about to enter frame
        if hg.settings.enabled:
            disc = params.get("disc_entry", {})
            for body, info in disc.items():
                mins = info.get("minutes", 99)
                if mins <= 5:
                    await broadcast({"type": "log",
                        "msg": f"⚠ {body.upper()} enters frame in ~{mins:.1f} min "
                               f"(az={info['az']:.0f}° alt={info['alt']:.1f}°)"})

        # 7. Progress + telemetry broadcast
        tracker_status = hg.get_tracker_status() if hg.settings.enabled else {}
        progress = _estimate_progress(state["current_frame"], interval)
        frame_data = {
            "type":       "status",
            "frame":      state["current_frame"],
            "frame_id":   f"{i:04d}",   # actual filename ID used for THUMB_{frame_id}.jpg
            "has_thumb":  True,
            "total":      state["total_frames"],
            "hg_phase":   params.get("phase", "—"),
            "hg_sun_alt": float(params.get("sun_alt", 0.0)),
            "hg_ev":      float(params.get("ev_final",   params.get("ev_target", 0.0))),
            "hg_ev_scene": float(params.get("ev_blended", params.get("ev_target", 0.0))),
            "hg_iso":     params.get("iso", 0),
            "hg_shutter": params.get("shutter", "—"),
            "hg_shutter_s": float(params.get("shutter_s", 1/125)),
            "hg_kelvin":  params.get("kelvin", 0),
            "hg_condition":    tracker_status.get("condition", ""),
            "hg_ev_slope":     tracker_status.get("ev_slope", 0.0),
            "hg_confidence":   tracker_status.get("confidence", 0.0),
            "hg_tracker_warm": tracker_status.get("warm", False),
            # Motor positions at shot time (before the inter-frame move)
            "pos_s": round(slider_axis.current_mm, 2),
            "pos_p": round(pan_axis.current_deg,   2),
            "pos_t": round(tilt_axis.current_deg,  2),
            # Sidecar fields — phase (0→1) and wall-clock offset from sequence start
            "motion_phase": round(i / max(state["total_frames"] - 1, 1), 6),
            "real_time_s":  round(time.time() - _seq_wall_start, 3),
            **progress,
        }
        _session_history.append(frame_data)
        await broadcast(frame_data)

        # Persist frame count so reconnect shows correct progress
        state["current_frame"] = i + 1
        if (i + 1) % 10 == 0:
            save_session()
            _save_session_history()

        # 8. Move rig + meter shot (concurrent) — LED: motors
        #
        # The motor movement window is the ONLY time the camera is
        # guaranteed free regardless of trigger mode:
        #   - normal: camera is between shots
        #   - aux_only / picam_motion_only: trigger hasn't fired yet for
        #     the *next* frame, but current shot is done — motors are moving
        #   - aux_hybrid / picam_motion_hybrid: same as above
        #   - no motion: falls through to step 9 vibration settle window
        #
        # The meter shot runs concurrently with the motors on a thread-pool
        # thread. It takes ~300ms (ISP settle + frame grab) and the motor
        # move takes 1.5s, so it always completes before motors stop.
        _meter_future = None
        _has_motor_move = False

        if hg.settings.enabled and state.get("active_camera") == "picam":
            frame_idx_for_meter = i
            _meter_future = asyncio.get_event_loop().run_in_executor(
                None, lambda fi=frame_idx_for_meter: _take_meter_shot(fi)
            )

        if len(engine.keyframes) >= 2:
            try:
                # Read the current origin offset (delta from design-space to physical space).
                # Set via "Set Reference" / cinematic_set_origin.  Zero if never set.
                # NOTE: variables are named orig_s/p/t (not os/op/ot) because
                # 'os' as a local name would shadow the top-level `import os`
                # across the ENTIRE function, causing UnboundLocalError at
                # os.makedirs() near the top of _timelapse_worker_inner.
                orig_s = _prog_move.origin_slider if _prog_move else 0.0
                orig_p = _prog_move.origin_pan    if _prog_move else 0.0
                orig_t = _prog_move.origin_tilt   if _prog_move else 0.0

                pm_mode    = _prog_move.path_mode       if _prog_move else "linear"
                pm_easing  = _prog_move.global_easing   if _prog_move else "cycloid"
                pm_tension = _prog_move.catmull_tension if _prog_move else 0.5
                n_frames   = state["total_frames"]

                cache_key = (len(engine.keyframes), n_frames,
                             orig_s, orig_p, orig_t, pm_mode, pm_easing, pm_tension)
                if not hasattr(timelapse_worker, '_traj_cache') or \
                   timelapse_worker._traj_cache.get('key') != cache_key:
                    # Use the unified trajectory generator so timelapse frames share
                    # the same spatial path, easing, and Catmull-Rom spline as cinema.
                    if _prog_move and len(_prog_move.keyframes) >= 2:
                        traj_s, traj_p, traj_t = _prog_move.generate_unified_trajectory(
                            n_frames,
                            orig_s, orig_p, orig_t,
                            for_timelapse=True,
                        )
                    else:
                        # Fallback: build a plain shifted MotionEngine (no easing applied).
                        from slider import LinearAxis as _LA, RotationAxis as _RA
                        shifted_engine = MotionEngine()
                        for kf in engine.keyframes:
                            shifted_engine.add_keyframe(
                                kf["slider_mm"] + orig_s,
                                kf["pan_deg"]   + orig_p,
                                kf["tilt_deg"]  + orig_t,
                            )
                        traj_s, traj_p, traj_t = shifted_engine.generate_trajectory(
                            duration_s=float(n_frames),
                            fps=1,
                            easing_curve="linear",
                        )
                    timelapse_worker._traj_cache = {
                        'key':    cache_key,
                        'n_kf':   len(engine.keyframes),
                        'total':  n_frames,
                        'traj_s': traj_s, 'traj_p': traj_p, 'traj_t': traj_t,
                    }

                idx = min(i + 1, state["total_frames"] - 1)
                cache = timelapse_worker._traj_cache
                target_s = float(cache['traj_s'][idx])
                target_p = clamp_pan(float(cache['traj_p'][idx]))
                target_t = clamp_tilt(float(cache['traj_t'][idx]))

                ds = int((target_s - slider_axis.current_mm) * slider_axis.steps_per_mm)
                dp = int((target_p - pan_axis.current_deg)   * pan_axis.steps_per_deg)
                dt = int((target_t - tilt_axis.current_deg)  * tilt_axis.steps_per_deg)
                if any([ds, dp, dt]):
                    _has_motor_move = True
                    status_leds.set_sequence_phase("motors")
                    # Ensure motors are enabled — _move_to (Return to Start) used to
                    # disable them; this is now a safety net for any future path that
                    # might leave the EN pin high.
                    hw.enable_motors(True)
                    # Adaptive motor duration: scale with interval but floor at the
                    # minimum time physically required at max bit-bang step rate (~800 steps/sec).
                    _MAX_STEP_RATE = 800  # steps/sec — practical bit-bang GPIO limit
                    _max_steps = max(abs(ds), abs(dp), abs(dt))
                    _min_motor_dur = _max_steps / _MAX_STEP_RATE if _max_steps else 0.0
                    _target_dur = max(0.4, min(1.5, interval * 0.22))
                    _motor_dur = max(_min_motor_dur, _target_dur)
                    # Auto-extend interval for this frame if motor + camera won't fit.
                    # Estimate camera overhead: Sony WiFi is slower than PiCam.
                    _cam_overhead = 2.5 if state.get("active_camera") in ("sony", "sony_usb") else 1.5
                    _needed_interval = _min_motor_dur + _cam_overhead
                    if _needed_interval > interval:
                        interval = _needed_interval
                        if not _interval_extended:
                            _interval_extended = True
                            await broadcast({"type": "log",
                                "msg": f"⏱ Interval extended to {interval:.1f}s at frame {i+1} "
                                       f"— movement requires {_min_motor_dur:.1f}s."})
                    else:
                        if _interval_extended:
                            _interval_extended = False
                            await broadcast({"type": "log",
                                "msg": f"⏱ Interval returned to {interval:.1f}s at frame {i+1}."})
                    await asyncio.to_thread(hw.move_axes_simultaneous, ds, dp, dt, _motor_dur)
                    # Update position tracking — CRITICAL for focus rail macro mode
                    slider_axis.current_steps += ds  # Track steps moved
                    slider_axis.current_mm = target_s
                    pan_axis.current_deg   = target_p
                    tilt_axis.current_deg  = target_t
                    status_leds.set_sequence_phase("waiting")
            except Exception as e:
                logger.error(f"Motion step {i}: {e}", exc_info=True)
                await broadcast({"type": "log",
                    "msg": f"⚠ Motion error at frame {i+1}: {e}"})

        # Await the meter shot future if it was launched but motors didn't move
        # (or just let it finish in the background if motors ran concurrently).
        # Either way we collect the result here so any exception is surfaced.
        if _meter_future is not None:
            try:
                await _meter_future
            except Exception as e:
                logger.warning(f"Meter shot future frame {i}: {e}")

        # 9. Anti-vibration settle — LED: waiting
        # If there was no motor move and no meter shot yet (aux_only /
        # motion_only with no keyframes), use the settle window for the
        # meter shot now. It's still guaranteed dead time.
        status_leds.set_sequence_phase("waiting")
        vibe_delay = state["vibe_delay"]

        if (not _has_motor_move
                and _meter_future is None
                and hg.settings.enabled
                and state.get("active_camera") == "picam"):
            # Run meter shot during settle — allow at least 0.5s settle after
            meter_start = asyncio.get_event_loop().time()
            frame_idx_for_meter = i
            await asyncio.get_event_loop().run_in_executor(
                None, lambda fi=frame_idx_for_meter: _take_meter_shot(fi)
            )
            meter_elapsed = asyncio.get_event_loop().time() - meter_start
            # Sleep any remaining settle time after the meter shot finishes
            remaining_settle = vibe_delay - meter_elapsed
            if remaining_settle > 0:
                await asyncio.sleep(remaining_settle)
        else:
            await asyncio.sleep(vibe_delay)

        # 10. Wait interval remainder (normal / hybrid modes)
        # Sony: grab a liveview frame during the wait for HG sky metering
        if tmode in ("normal", "aux_hybrid", "picam_motion_hybrid"):
            elapsed   = asyncio.get_event_loop().time() - loop_start
            remainder = interval - elapsed

            # Sony inter-shot HG metering — use the captured JPEG, not liveview.
            # For sony_wifi: _latest_shot is set by _generate_sony_thumb.
            # For sony_usb:  _latest_shot is set by _extract_arw_thumb inside capture_sony_usb.
            # Both paths feed the same meter_sony_from_shot() function.
            if (hg.settings.enabled
                    and state.get("active_camera") in ("sony", "sony_usb")):
                meter_sony_from_shot()
                elapsed   = asyncio.get_event_loop().time() - loop_start
                remainder = interval - elapsed

            if remainder > 0:
                await asyncio.sleep(remainder)
            elif remainder < -0.5:
                await broadcast({"type": "log",
                    "msg": f"⚠ Interval overrun at frame {i+1}: "
                           f"loop took {elapsed:.1f}s vs {interval:.1f}s target "
                           f"({abs(remainder):.1f}s over). Motor/shutter time exceeded interval."})

        # 11. Thumb recovery — use leftover idle time to fill in missing thumbnails.
        # Sony HTTP bulb captures and slow-card writes can leave frame_ids in
        # _thumb_retry_queue.  We try to recover from local files (ARW/JPG on
        # disk) and broadcast thumb_ready so the graph updates immediately.
        # Frames whose files are still on the camera card return False and stay
        # in the queue for a subsequent interval.
        if _thumb_retry_queue and state.get("active_camera") in ("sony", "sony_usb"):
            _still_missing = []
            for _fid in list(_thumb_retry_queue):
                try:
                    _ok = await asyncio.to_thread(_attempt_thumb_recovery, _fid)
                except Exception:
                    _ok = False
                if _ok:
                    await broadcast({"type": "thumb_ready", "frame_id": _fid})
                    logger.debug(f"Thumb recovery succeeded for frame {_fid}")
                else:
                    _still_missing.append(_fid)
            _thumb_retry_queue[:] = _still_missing

    # Cleanup
    if motion_task:
        motion_task.cancel()
    state["is_running"] = False
    state["stop_event"].clear()

    # Record why/how the sequence ended so a reconnecting browser can show it
    if not state.get("_stop_reason"):   # don't overwrite a halt/error reason set earlier
        state["_stop_reason"] = (
            f"Completed — {state['current_frame']} of {state['total_frames']} frames saved."
        )
    _save_session_history()   # final flush so reconnecting browser sees full graph
    save_session()
    stop_msg = state["_stop_reason"]
    await broadcast({"type": "run_state",  "running": False})
    await broadcast({"type": "log",        "msg": f"[✓] Sequence ended: {stop_msg}"})
    await broadcast({"type": "stop_reason","msg": stop_msg})
    logger.info(f"Timelapse ended: {stop_msg}")
    # Restart InertiaEngine so joystick/gamepad are immediately usable again
    if _inertia:
        _inertia.set_target(0, 0, 0)   # zero stale targets before restart
        _inertia.set_preset("responsive")
        if not _inertia._running:
            _inertia.start()
    # Return LEDs to idle breathing
    active_mode = state.get("active_mode", "timelapse")
    status_leds.set_mode("timelapse_idle" if active_mode != "macro" else "macro_idle")


# ─── BROADCAST ────────────────────────────────────────────────────────────────
# Only one active WebSocket client is allowed at a time.
# The active client is tracked here; broadcast always targets just this one.
connected_clients: set = set()   # kept for legacy broadcast() signature
_active_ws: Optional[WebSocket] = None
_graph_clients: set = set()       # read-only graph tabs — never kicked, never control

async def broadcast(payload: dict):
    """Send to the active control client and any read-only graph clients."""
    global _active_ws
    if _active_ws is not None:
        try:
            await _active_ws.send_json(payload)
        except Exception:
            _active_ws = None
    # Also forward to all graph tabs (read-only, never kicked).
    # Snapshot the set before iterating — a graph-tab disconnect (discard) can
    # happen at any await point, causing "Set changed size during iteration".
    dead = set()
    for gc in set(_graph_clients):
        try:
            await gc.send_json(payload)
        except Exception:
            dead.add(gc)
    _graph_clients.difference_update(dead)


# ─── INIT PACKET ──────────────────────────────────────────────────────────────
def _build_init_packet() -> dict:
    """Full state packet sent to every new WS client on connect."""
    import dataclasses
    hg_d = {k: v for k, v in dataclasses.asdict(hg.settings).items()
            if not isinstance(v, datetime.datetime)}
    return {
        "type":              "init",
        "running":           state["is_running"],
        "scheduled_waiting": state.get("_scheduled_waiting", False),
        "scheduled_target":  state.get("_scheduled_target", ""),
        "interrupted":     state.get("_was_interrupted", False),
        "current_frame":   state["current_frame"],
        "total_frames":    state["total_frames"],
        "trigger_mode":    state["trigger_mode"],
        "pan_min":         state["pan_min"],
        "pan_max":         state["pan_max"],
        "tilt_min":        state["tilt_min"],
        "tilt_max":        state["tilt_max"],
        "save_path":       state["save_path"],
        "pan_deg":         pan_axis.current_deg,
        "tilt_deg":        tilt_axis.current_deg,
        "slider_mm":       slider_axis.current_mm,
        "hg_settings":     hg_d,
        "motion_roi":      state["motion_roi"],
        "motion_threshold": state["motion_threshold"],
        "motion_warmup":   state.get("motion_warmup_frames", 10),
        "motion_cooldown": state.get("motion_cooldown", 2.0),
        "active_camera":   state["active_camera"],
        "preview_camera":  state.get("preview_camera", state["active_camera"]),
        "vibe_delay":      state["vibe_delay"],
        "exp_margin":      state["exp_margin"],
        "manual_interval": state.get("manual_interval", 5.0),
        "tl_preroll_s":    state.get("tl_preroll_s", 0.0),
        "active_mode":     state.get("active_mode", "timelapse"),
        "camera_orientation": state.get("camera_orientation", "landscape"),
        "cine_fps":           state.get("cine_fps", 24),
        "slider_inverted":    state.get("slider_inverted", False),
        "pan_inverted":       state.get("pan_inverted", False),
        "tilt_inverted":      state.get("tilt_inverted", False),
        # Gamepad connection state — sent so a browser that connects after the
        # controller was already paired still shows the correct indicator.
        "gamepad_connected":  bool(_gamepad_reader and _gamepad_reader.connected),
        # Last stop reason — shown to reconnecting browser so they know why the
        # sequence ended (user stop, error halt, completed, crash, etc.)
        "stop_reason":        state.get("_stop_reason", ""),
        # Crash report file — shown to user if present so they know where to look
        "crash_report_path":  (os.path.join(state["save_path"], "CRASH_REPORT.txt")
                               if os.path.exists(os.path.join(state["save_path"],
                                                              "CRASH_REPORT.txt"))
                               else ""),
        # Per-run ID so the graph page can bust its thumbnail cache when a new
        # sequence starts (even if the save_path is the same as a previous run).
        "run_id":             _timelapse_run_id,
    }


# ─── WEBSOCKET ────────────────────────────────────────────────────────────────
@app.get("/api/gps")
async def api_gps():
    """
    Server-side location lookup — runs on the Pi, not the browser.
    Avoids Chrome's HTTPS-only restriction on navigator.geolocation.
    Uses ip-api.com for approximate location based on the Pi's public IP.
    Falls back to the currently configured HG lat/lon if unavailable.
    """
    try:
        import urllib.request as _ur
        import json as _json
        with _ur.urlopen("http://ip-api.com/json/?fields=lat,lon,city,timezone", timeout=4) as r:
            data = _json.loads(r.read())
        lat = round(float(data["lat"]), 5)
        lon = round(float(data["lon"]), 5)
        tz  = data.get("timezone", "America/Winnipeg")
        city = data.get("city", "")
        return JSONResponse({"lat": lat, "lon": lon, "timezone": tz, "city": city})
    except Exception as e:
        # Return currently configured values so UI doesn't error
        return JSONResponse({
            "lat": hg.settings.lat,
            "lon": hg.settings.lon,
            "timezone": state.get("timezone", "America/Winnipeg"),
            "city": "",
            "error": str(e),
        })


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    global _active_ws, _macro_task, _cinematic_mode, _cinematic_live_active, _prog_task, _pending_play, _recording, _record_start_time
    global _last_input_time

    await websocket.accept()

    # ── TCP_NODELAY — disable Nagle's algorithm on this connection ────────────
    # Nagle buffers small outgoing packets (joystick ACKs, status msgs, ~50 B)
    # for up to 200 ms waiting to fill an MTU.  On a local WiFi network this
    # is the dominant source of perceived control latency.  Setting NODELAY
    # sends each packet the moment it's ready.
    # Path: websocket._send is uvicorn's bound WebSocketProtocol.asgi_send;
    # .__self__ is the protocol instance which holds the asyncio transport.
    try:
        _proto = websocket._send.__self__
        _raw_sock = _proto.transport.get_extra_info("socket")
        if _raw_sock is not None:
            _raw_sock.setsockopt(_socket.IPPROTO_TCP, _socket.TCP_NODELAY, 1)
            logger.debug("TCP_NODELAY enabled on WS connection")
    except Exception:
        pass   # non-fatal — best-effort; no-ops on non-uvicorn ASGI servers

    # ── Single-instance enforcement ───────────────────────────────────────────
    # If another tab/window is already connected, kick it with a clear message
    # then take over as the new active client.
    if _active_ws is not None:
        try:
            await _active_ws.send_json({
                "type": "kicked",
                "msg":  "⚠ A new browser window connected — this tab has been replaced. "
                        "Close this tab; the new one is now in control."
            })
            await _active_ws.close(code=4001, reason="replaced_by_new_client")
        except Exception:
            pass   # old socket may already be dead
        logger.info("WS: previous client kicked — new client taking over.")

    _active_ws = websocket
    connected_clients.discard(websocket)   # not used for routing, kept for compatibility
    connected_clients.add(websocket)

    await websocket.send_json(_build_init_packet())

    # ── Replay session graph history to the reconnecting browser ─────────────
    # The graph is built from "status" frames broadcast during the sequence.
    # A browser that connects mid-sequence (or after it ends) would otherwise
    # see a blank graph. Send the full in-memory history as a single batch so
    # the graph populates immediately without waiting for the next frame.
    if _session_history:
        try:
            await websocket.send_json({
                "type":   "history_replay",
                "frames": list(_session_history),
            })
        except Exception:
            pass   # client may disconnect during send — non-fatal

    try:
        while True:
            data = await websocket.receive_text()
            msg  = json.loads(data)
            cmd  = msg.get("command")

            # ── PING keepalive (browser → server every ~25 s) ─────────────────
            # Prevents home-router NAT timeouts from silently dropping the WS.
            if cmd == "ping":
                await websocket.send_json({"type": "pong"})
                continue

            # ── JOYSTICK ──────────────────────────────────────────────────────
            if cmd == "joystick":
                _last_input_time = time.time()
                # Block joystick during automated sequences (timelapse, programmed
                # move) but ALLOW it during cinematic_live_start — that mode is
                # specifically joystick-driven.
                if state["is_running"] and not _cinematic_live_active: continue
                try:
                    vx = float(msg.get("vx", 0))   # pan   [-1..1]
                    vy = float(msg.get("vy", 0))   # tilt  [-1..1]
                    vz = float(msg.get("vz", 0))   # slider[-1..1]
                    # Update UI axis cache then compute merged target that includes
                    # any live gamepad contribution.  This prevents the UI from
                    # zeroing out a gamepad axis that is actively being held.
                    _ui_axes["slider"] = vz
                    _ui_axes["pan"]    = vx
                    _ui_axes["tilt"]   = vy
                    # NOTE: set_target() is NOT called here.
                    # _joystick_target_sync() reads _ui_axes once per
                    # InertiaEngine tick (50 Hz) and calls set_target() with
                    # the CURRENT position.  Calling it here would replay every
                    # queued WS message in sequence, causing the motor to chase
                    # the full history of stick positions rather than jumping
                    # straight to where the stick is now.
                    if _inertia and not _inertia._running:
                        _inertia.start()
                except Exception as _je:
                    logger.error(f"WS joystick error: {_je}", exc_info=True)
                    await websocket.send_json({"type": "log", "msg": f"⚠ Joystick error: {_je}"})

            # ── NUDGE (precision step-and-stop) ───────────────────────────────
            # Nudge respects soft limits when they are set.
            # - If soft limits are NOT enforced (360° rotation or unset), nudge can move anywhere
            # - If soft limits ARE enforced, nudge is clamped to stay within [soft_min, soft_max]
            # This allows users to fine-tune positions within calibrated bounds.
            #
            # IMPORTANT: this handler must NEVER use await asyncio.sleep() directly.
            # Doing so blocks this WS handler coroutine from reading subsequent
            # messages (stop commands, etc.) until the sleep ends, causing the
            # appearance of a "stuck" motor and making stop buttons unresponsive.
            # We use asyncio.create_task() instead so the WS loop is always free.
            elif cmd == "nudge_axis":
                if state["is_running"]: continue
                axis = msg.get("axis", "pan")
                deg  = float(msg.get("deg", 0))
                if abs(deg) < 0.001 or not _inertia: continue

                if not _inertia._running:
                    _inertia.start()

                # Cancel any pending step for this axis so rapid clicks
                # restart the pulse rather than stacking up sleeps.
                prior = _nudge_axis_tasks.get(axis)
                if prior and not prior.done():
                    prior.cancel()

                sign = 1.0 if deg > 0 else -1.0

                # Step speeds and durations.  Faster than d-pad crawl so 10° steps
                # don't take 3 seconds, but slow enough for repeatable accuracy.
                STEP_SPEEDS = {
                    "pan":    8.0,   # deg/s
                    "tilt":   6.0,   # deg/s
                    "slider": 20.0,  # mm/s
                }
                spd = STEP_SPEEDS.get(axis, 8.0)
                dur = max(0.03, abs(deg) / spd)   # minimum 30 ms pulse

                # Bump generation counter so the previous task's finally block
                # will not zero the hardware after this new task has started.
                _nudge_axis_gen[axis] += 1
                gen = _nudge_axis_gen[axis]

                async def _step_nudge(axis=axis, sign=sign, spd=spd, dur=dur, gen=gen):
                    """Run motor via InertiaEngine nudge for dur seconds, then stop.

                    Uses set_nudge_pt/set_nudge_slider so the InertiaEngine physics
                    tick outputs exactly the nudge speed instead of its own computed
                    velocity — prevents the 50 Hz physics loop from fighting our
                    command.  Motor stops in the same tick the nudge is cleared.

                    The generation counter (gen) ensures only the LATEST click's
                    finally block clears the nudge.  When a new click arrives and
                    cancels this task, the new task has already incremented
                    _nudge_axis_gen[axis], so our captured gen no longer matches
                    and we skip the cleanup entirely — leaving the new task's nudge
                    state undisturbed.

                    Soft limit enforcement: Only bypass guard if soft limits are NOT set.
                    - Rotation axes: bypass only if soft limits span 360° (±180°)
                    - Slider (focus): bypass only if soft_min_steps == soft_max_steps (unset)
                    """
                    # Determine if soft limits are actually enforced for this axis
                    enforce_limits = True
                    if axis == "pan":
                        enforce_limits = not (pan_axis.soft_min == -180.0 and pan_axis.soft_max == 180.0)
                    elif axis == "tilt":
                        enforce_limits = not (tilt_axis.soft_min == -180.0 and tilt_axis.soft_max == 180.0)
                    elif axis == "slider":
                        enforce_limits = not (slider_axis.soft_min_steps == slider_axis.soft_max_steps)

                    try:
                        # Only bypass guard if limits are NOT enforced (recalibration mode)
                        _inertia._guard_bypass = not enforce_limits
                        if axis == "pan":
                            _inertia.set_nudge_pt(sign * spd, _inertia._nudge_tilt)
                        elif axis == "tilt":
                            _inertia.set_nudge_pt(_inertia._nudge_pan, sign * spd)
                        elif axis == "slider":
                            _inertia.set_nudge_slider(sign * spd)
                        # Send periodic position updates during nudge (~10 Hz)
                        t_end = time.time() + dur
                        while time.time() < t_end:
                            await asyncio.sleep(0.1)
                            try:
                                await websocket.send_json({"type": "status",
                                    "pos_s": round(slider_axis.current_mm, 2),
                                    "pos_p": round(pan_axis.current_deg, 2),
                                    "pos_t": round(tilt_axis.current_deg, 2)})
                            except Exception:
                                pass
                    except asyncio.CancelledError:
                        pass   # superseded by a newer click — new task takes over
                    finally:
                        # Only clean up if no newer click has taken over.
                        # A newer click increments _nudge_axis_gen[axis] BEFORE
                        # creating its own task, so a stale gen means we must
                        # not touch the motor — the new task owns it.
                        if _nudge_axis_gen.get(axis) == gen:
                            if axis in ("pan", "tilt"):
                                _inertia.set_nudge_pt(
                                    0.0 if axis == "pan"  else _inertia._nudge_pan,
                                    0.0 if axis == "tilt" else _inertia._nudge_tilt,
                                )
                                if _inertia._nudge_pan == 0.0 and _inertia._nudge_tilt == 0.0:
                                    _inertia.clear_nudge_pt()
                            else:
                                _inertia.clear_nudge_slider()
                            _inertia._guard_bypass = False

                _nudge_axis_tasks[axis] = asyncio.create_task(_step_nudge())

            # ── UI JOG NUDGE (hold-to-move buttons) ───────────────────────────
            # ui_nudge_start: motor moves at nudge speed while button held.
            # ui_nudge_stop:  motor stops instantly (same 20ms tick).
            # Routes through InertiaEngine.set_nudge_*() which bypasses physics
            # so there is absolutely no coast on release.
            #
            # Soft limit enforcement: Only bypass guard if soft limits are NOT set.
            elif cmd == "ui_nudge_start":
                if state["is_running"]: continue
                if _inertia:
                    if not _inertia._running:
                        _inertia.start()
                    axis = msg.get("axis", "pan")
                    dir  = float(msg.get("dir", 1))
                    now  = time.time()
                    # Optional speed override — GUI jog buttons send a higher speed
                    # than the d-pad default so they feel responsive for general
                    # navigation, while the d-pad retains crawl precision.
                    spd_override = msg.get("speed")

                    # Determine if soft limits are actually enforced for this axis
                    enforce_limits = True
                    if axis == "pan":
                        spd = float(spd_override) if spd_override is not None else NUDGE_SPEED_PAN
                        logger.info(f"ui_nudge_start: pan dir={dir:+.0f} spd={spd:.1f}")
                        enforce_limits = not (pan_axis.soft_min == -180.0 and pan_axis.soft_max == 180.0)
                        _inertia._guard_bypass = not enforce_limits
                        _inertia.set_nudge_pt(dir * spd, _inertia._nudge_tilt)
                        _nudge_heartbeat["pan"] = now
                        _nudge_source["pan"]    = "ui"
                    elif axis == "tilt":
                        spd = float(spd_override) if spd_override is not None else NUDGE_SPEED_TILT
                        logger.info(f"ui_nudge_start: tilt dir={dir:+.0f} spd={spd:.1f}")
                        enforce_limits = not (tilt_axis.soft_min == -180.0 and tilt_axis.soft_max == 180.0)
                        _inertia._guard_bypass = not enforce_limits
                        _inertia.set_nudge_pt(_inertia._nudge_pan, dir * spd)
                        _nudge_heartbeat["tilt"] = now
                        _nudge_source["tilt"]    = "ui"
                    elif axis == "slider":
                        spd = float(spd_override) if spd_override is not None else NUDGE_SPEED_SLIDER
                        logger.info(f"ui_nudge_start: slider dir={dir:+.0f} spd={spd:.1f}")
                        enforce_limits = not (slider_axis.soft_min_steps == slider_axis.soft_max_steps)
                        _inertia._guard_bypass = not enforce_limits
                        _inertia.set_nudge_slider(dir * spd)
                        _nudge_heartbeat["slider"] = now
                        _nudge_source["slider"]    = "ui"
                    _last_input_time = now

            elif cmd == "ui_nudge_stop":
                if _inertia:
                    axis = msg.get("axis", "pan")
                    logger.info(f"ui_nudge_stop: {axis}")
                    # Clear source and heartbeat for this axis
                    if axis in _nudge_source:    _nudge_source[axis]    = None
                    if axis in _nudge_heartbeat: _nudge_heartbeat[axis] = 0.0
                    if axis in ("pan", "tilt"):
                        # Zero only the requested axis; keep the other one if still held
                        if axis == "pan":
                            _inertia.set_nudge_pt(0.0, _inertia._nudge_tilt)
                        else:
                            _inertia.set_nudge_pt(_inertia._nudge_pan, 0.0)
                        # If both are zero after this, clear and hard-stop immediately
                        if _inertia._nudge_pan == 0.0 and _inertia._nudge_tilt == 0.0:
                            _inertia.clear_nudge_pt()
                            hw.stop_all_axes()   # belt-and-suspenders hard stop
                    elif axis == "slider":
                        _inertia.clear_nudge_slider()
                        hw.stop_all_axes()   # belt-and-suspenders hard stop

            # ── SOFT LIMITS ───────────────────────────────────────────────────
            elif cmd == "set_limits":
                axis  = msg.get("axis", "pan")
                which = msg.get("which", "max")
                val   = msg.get("value", None)
                if val is None:
                    val = pan_axis.current_deg if axis == "pan" else tilt_axis.current_deg
                state[f"{axis}_{which}"] = float(val)
                # Sync with cinematic soft guard
                guard_ax = getattr(_soft_guard, axis, None)
                if guard_ax:
                    if which == "min": guard_ax.min_unit = float(val)
                    else: guard_ax.max_unit = float(val)
                    guard_ax._update_cal()
                save_session()
                await websocket.send_json({"type": "limits_updated",
                    "pan_min": state["pan_min"], "pan_max": state["pan_max"],
                    "tilt_min": state["tilt_min"], "tilt_max": state["tilt_max"]})

            # ── CALIBRATION ───────────────────────────────────────────────────
            elif cmd == "hardware_zero":
                # Park the rig at the hardware reference position (rail centre,
                # camera level + perpendicular to rail) and press this button to
                # zero all axis position counters.  This creates an absolute
                # physical coordinate frame that all keyframes and soft limits
                # are measured relative to.  Soft limits and cinematic keyframes
                # set before zeroing are invalidated — a warning message is sent.
                had_limits = (
                    _soft_guard.slider.min_unit is not None or
                    _soft_guard.pan.min_unit    is not None or
                    _soft_guard.tilt.min_unit   is not None
                )
                had_keyframes = bool(_prog_move and _prog_move.keyframes)

                # Ensure any motion is halted before resetting counters
                hw.stop_all_axes()
                hw.enable_motors(False)
                # Zero axis counters — CRITICAL for focus rail macro mode
                # Both mm AND steps must be zeroed to track absolute position from home
                slider_axis.current_mm  = 0.0
                slider_axis.current_steps = 0  # ← CRITICAL: Reset step counter to 0 at home
                pan_axis.current_deg    = 0.0
                tilt_axis.current_deg   = 0.0
                # Re‑enable motors for subsequent operations (will be re‑enabled on move commands)
                hw.enable_motors(True)

                # Clear soft limits — they were relative to old zero
                for ax in (_soft_guard.slider, _soft_guard.pan, _soft_guard.tilt):
                    ax.min_unit  = None
                    ax.max_unit  = None
                    ax._update_cal()
                state["slider_min"] = None
                state["slider_max"] = None
                state["pan_min"]    = -90.0    # restore clamp defaults
                state["pan_max"]    =  90.0
                state["tilt_min"]   = -30.0
                state["tilt_max"]   =  30.0

                # Also reset slider soft limit steps (will be recalculated when limits are set)
                # Use STEPS_PER_MM = 800.0 (macro mode) not slider_axis.steps_per_mm (timelapse mode)
                slider_axis.soft_min_steps = 0
                slider_axis.soft_max_steps = int(slider_axis.max_mm * STEPS_PER_MM)

                # Clear keyframes — they referenced old coordinate frame
                if _prog_move:
                    _prog_move.clear_keyframes()
                    _prog_move.set_origin(0.0, 0.0, 0.0)
                    _prog_move.reference_slider = None
                    _prog_move.reference_pan    = None
                    _prog_move.reference_tilt   = None
                state["cinematic_origin"]    = {}
                state["cinematic_reference"] = {}

                save_session()

                # Tell UI: new zero, limits cleared, keyframes cleared
                await broadcast({"type": "cinematic_limits",
                                 "limits": _soft_guard.status()})
                await broadcast({"type": "cinematic_keyframes", "keyframes": []})
                await broadcast({
                    "type": "status",
                    "pos_s": 0.0, "pos_p": 0.0, "pos_t": 0.0,
                    "slider_mm": 0.0, "pan_deg": 0.0, "tilt_deg": 0.0,
                })
                warn = ""
                if had_limits or had_keyframes:
                    warn = " ⚠ Previous soft limits and keyframes cleared — re-set them from new zero."
                await broadcast({"type": "log",
                    "msg": f"[✓] Hardware reference zeroed — all axes reset to 0.{warn}"})
                await broadcast({"type": "hardware_zeroed",
                                 "slider_mm": 0.0, "pan_deg": 0.0, "tilt_deg": 0.0})

            elif cmd == "calibrate_origin":
                bearing = float(msg.get("bearing_deg", 0))
                state["origin_az"]   = (bearing - pan_axis.current_deg) % 360
                hg.settings.cam_az   = bearing
                hg.settings.cam_alt  = -tilt_axis.current_deg
                save_session()
                await websocket.send_json({"type": "calibration_done",
                    "origin_az": state["origin_az"],
                    "cam_az": bearing, "cam_alt": -tilt_axis.current_deg})

            # ── TRIGGERS ──────────────────────────────────────────────────────
            elif cmd == "aux_trigger":
                state["aux_triggered"] = True

            elif cmd == "set_trigger_mode":
                mode = msg.get("mode", "normal")
                valid = ("normal","aux_only","aux_hybrid",
                         "picam_motion_only","picam_motion_hybrid")
                if mode in valid:
                    state["trigger_mode"] = mode
                    save_session()

            elif cmd == "set_motion_roi":
                state["motion_roi"]           = msg.get("roi",       state["motion_roi"])
                state["motion_threshold"]     = msg.get("threshold", state["motion_threshold"])
                state["motion_warmup_frames"] = msg.get("warmup",    state.get("motion_warmup_frames", 10))
                state["motion_cooldown"]      = msg.get("cooldown",  state.get("motion_cooldown", 2.0))
                save_session()

            # ── CAMERA ────────────────────────────────────────────────────────
            elif cmd == "detect_sony_usb":
                result = await asyncio.to_thread(detect_sony_usb)
                await broadcast({
                    "type":  "sony_usb_status",
                    "found": result["found"],
                    "model": result["model"],
                    "port":  result["port"],
                })

            elif cmd == "sony_usb_liveview_start":
                if state.get("active_camera") == "sony_usb":
                    _start_sony_usb_liveview()
                    await broadcast({"type": "log",
                        "msg": "📹 Sony USB liveview started (~2-3fps via gphoto2 preview)."})

            elif cmd == "sony_usb_liveview_stop":
                _stop_sony_usb_liveview()
                await broadcast({"type": "log", "msg": "📹 Sony USB liveview stopped."})

            elif cmd == "set_camera":
                state["active_camera"] = msg.get("value", "picam")
                # preview_camera always follows active_camera on a camera switch
                # (user can override independently via set_preview_camera afterwards)
                state["preview_camera"] = state["active_camera"]
                hg.settings.continuous_shutter = (state["active_camera"] == "picam")
                # Start/stop liveview workers to match camera selection
                if state["active_camera"] == "sony" and state.get("sony_ip"):
                    global _sony_last_ae
                    _sony_last_ae = None  # re-send exposure mode on first settings push
                    _stop_sony_usb_liveview()
                    _start_sony_liveview()
                elif state["active_camera"] == "sony_usb":
                    _stop_sony_liveview()
                    # Do NOT auto-start USB liveview here — user controls it via
                    # the Liveview button to avoid racing with detect_sony_usb
                else:
                    _stop_sony_liveview()
                    _stop_sony_usb_liveview()
                save_session()

            elif cmd == "set_mode":
                mode = msg.get("value", "timelapse")
                state["active_mode"] = mode
                # Switch picam preview aspect ratio
                if _HAS_PICAM and picam and not state["is_running"]:
                    try:
                        cfg = PREVIEW_CONFIG_169 if mode == "cinematic" else PREVIEW_CONFIG_43
                        picam.stop()
                        picam.configure(picam.create_video_configuration(main=cfg))
                        picam.start()
                        logger.info(f"Preview config switched for mode: {mode}")
                    except Exception as e:
                        logger.error(f"Preview config switch: {e}")
                # Apply per-mode microstepping
                try:
                    hw.set_mode_microstepping(mode)
                except Exception as e:
                    logger.warning(f"Microstepping set failed: {e}")
                # Auto-fan: cinematic = 60% (motors run continuously), others = 20%
                auto_fan = {"cinematic": 60, "timelapse": 20, "macro": 20}.get(mode, 20)
                mstep    = {"macro": 256, "timelapse": 16, "cinematic": 1}.get(mode, 16)
                # LED mode update
                led_mode = {"timelapse": "timelapse_idle",
                            "macro":     "macro_idle",
                            "cinematic": "cinematic"}.get(mode, "timelapse_idle")
                status_leds.set_mode(led_mode)
                try:
                    hw.set_fan(auto_fan)
                    await websocket.send_json({"type": "log",
                        "msg": f"Mode: {mode.upper()} — fan {auto_fan}%, microstepping 1/{mstep}"})
                except Exception:
                    pass
                # On any mode switch: coast to stop, restore responsive preset
                if _inertia:
                    _inertia.set_target(0, 0, 0)
                    _inertia.set_preset("responsive")
                    if not _inertia._running:
                        _inertia.start()
                _last_input_time = time.time()
                logger.info(f"Mode switch to {mode}: InertiaEngine responsive, joystick ready.")
                save_session()

            elif cmd == "set_camera_orientation":
                orient = msg.get("value", "landscape")
                if orient in ("landscape", "portrait_cw", "portrait_ccw", "inverted"):
                    state["camera_orientation"] = orient
                    save_session()
                    await broadcast({"type": "camera_orientation", "value": orient})

            elif cmd == "set_cine_fps":
                fps = int(msg.get("value", 24))
                if fps in (24, 25, 30, 60):
                    state["cine_fps"] = fps
                    save_session()
                    await websocket.send_json({"type": "log",
                        "msg": f"Recording frame rate set to {fps}fps"})

            elif cmd == "set_save_path":
                new_path = msg.get("value", state["save_path"])
                # Validate path accessibility before accepting
                try:
                    parent = os.path.dirname(new_path)
                    if not parent:
                        parent = new_path
                    if not os.path.exists(parent):
                        await websocket.send_json({"type":"log",
                            "msg": f"⚠ Path does not exist: {parent}"})
                        return
                    if not os.access(parent, os.W_OK):
                        await websocket.send_json({"type":"log",
                            "msg": f"⚠ No write permission: {parent}"})
                        return
                    # Try to create a test file to verify writability
                    test_file = os.path.join(parent, ".pislider_test")
                    try:
                        with open(test_file, 'w') as f:
                            f.write("test")
                        os.remove(test_file)
                    except Exception as e:
                        await websocket.send_json({"type":"log",
                            "msg": f"⚠ Cannot write to path: {e}"})
                        return

                    state["save_path"] = new_path
                    save_session()
                    await websocket.send_json({"type":"log",
                        "msg": f"✓ Save path set: {new_path}"})
                    logger.info(f"Save path changed to: {new_path}")
                except Exception as e:
                    await websocket.send_json({"type":"log",
                        "msg": f"⚠ Save path error: {e}"})
                    logger.error(f"set_save_path error: {e}")

            elif cmd == "take_preview":
                prev = os.path.join(state["save_path"], "preview.jpg")
                os.makedirs(state["save_path"], exist_ok=True)
                cam = state.get("active_camera", "picam")
                if cam == "sony":
                    # Fire the Sony shutter via actTakePicture — saves the full-quality
                    # still to the camera card and returns a thumbnail URL we can display.
                    # The liveview frame is NOT used here because it's a low-res JPEG
                    # that doesn't reflect actual exposure (shutter capped at ~0.25 s).
                    await websocket.send_json({"type": "log",
                                              "msg": "Sony: firing preview shutter…"})
                    try:
                        result = await asyncio.to_thread(capture_sony,
                                                         "preview", state.get("picam_shutter_s", 1/125))
                        if result is not None:   # "" = shutter fired, file on card; path = local file
                            name = os.path.basename(result) if result else "preview (on camera card)"
                            await websocket.send_json({"type": "log",
                                "msg": f"[✓] Sony preview captured: {name}"})
                            if result:
                                await broadcast({"type": "new_frame",
                                                 "path": f"/shots/{os.path.basename(result)}"})
                        else:
                            cap_err = state.pop("_capture_error", "check camera is in Still mode and shutter speed is ≤30s")
                            await websocket.send_json({"type": "log",
                                "msg": f"⚠ Sony preview failed: {cap_err}"})
                    except Exception as e:
                        await websocket.send_json({"type": "log",
                            "msg": f"⚠ Sony preview error: {e}"})
                elif _HAS_PICAM and picam:
                    try:
                        frame = await asyncio.to_thread(picam.capture_array)
                        _, buf = cv2.imencode('.jpg', frame)
                        with open(prev, 'wb') as f: f.write(buf.tobytes())
                        await websocket.send_json({"type":"log","msg":f"Preview: {prev}"})
                    except Exception as e:
                        await websocket.send_json({"type":"log","msg":f"Preview fail: {e}"})

            # ── PICAM MANUAL SETTINGS ─────────────────────────────────────────
            elif cmd == "set_picam_settings":
                ae        = msg.get("ae",        True)
                awb       = msg.get("awb",       True)
                shutter_s = float(msg.get("shutter_s", 1/125))
                iso       = int(  msg.get("iso",     400))
                kelvin    = int(  msg.get("kelvin",  5500))
                state["picam_ae"]        = ae
                state["picam_awb"]       = awb
                state["picam_shutter_s"] = shutter_s
                state["picam_iso"]       = iso
                state["picam_kelvin"]    = kelvin
                async def _apply_sony_settings():
                    errs = await asyncio.to_thread(
                        set_sony_settings, ae, awb, shutter_s, iso, kelvin)
                    if errs:
                        await websocket.send_json(
                            {"type": "log", "msg": "Sony settings error: " + "; ".join(errs)})
                if state.get("active_camera") == "sony":
                    asyncio.create_task(_apply_sony_settings())
                elif state.get("active_camera") == "sony_usb":
                    asyncio.get_event_loop().run_in_executor(
                        None, set_sony_settings_usb, ae, awb, shutter_s, iso, kelvin)
                else:
                    apply_picam_settings()
                save_session()

            # ── SONY MANUAL SETTINGS ──────────────────────────────────────────
            elif cmd == "set_sony_settings":
                ae        = msg.get("ae",        True)
                awb       = msg.get("awb",       True)
                shutter_s = float(msg.get("shutter_s", 1/125))
                iso       = int(  msg.get("iso",   400))
                kelvin    = int(  msg.get("kelvin", 5500))
                state["picam_ae"]        = ae
                state["picam_awb"]       = awb
                state["picam_shutter_s"] = shutter_s
                state["picam_iso"]       = iso
                state["picam_kelvin"]    = kelvin
                async def _apply_sony_settings2():
                    errs = await asyncio.to_thread(
                        set_sony_settings, ae, awb, shutter_s, iso, kelvin)
                    if errs:
                        await websocket.send_json(
                            {"type": "log", "msg": "Sony settings error: " + "; ".join(errs)})
                if state.get("active_camera") == "sony_usb":
                    asyncio.get_event_loop().run_in_executor(
                        None, set_sony_settings_usb, ae, awb, shutter_s, iso, kelvin)
                else:
                    asyncio.create_task(_apply_sony_settings2())
                save_session()

            # ── HOLY GRAIL SETTINGS ───────────────────────────────────────────
            elif cmd == "set_hg_settings":
                try:
                    s = hg.settings
                    s_interval_sec = getattr(s, 'interval_sec', getattr(s, 'interval_day', 5.0))
                    hg.set_settings(HGSettings(
                        enabled           = msg.get("enabled",            s.enabled),
                        continuous_shutter= (state.get("active_camera", "picam") == "picam"),
                        lat               = float(msg.get("lat",           s.lat)),
                        lon               = float(msg.get("lon",           s.lon)),
                        tz                = msg.get("tz",                  s.tz),
                        cam_az            = float(msg.get("cam_az",        s.cam_az)),
                        cam_alt           = float(msg.get("cam_alt",       s.cam_alt)),
                        hfov              = float(msg.get("hfov",          s.hfov)),
                        vfov              = float(msg.get("vfov",          s.vfov)),
                        interval_sec      = float(msg.get("interval_sec",  s_interval_sec)),
                        frames            = int(  msg.get("frames",        s.frames)),
                        vibration_delay   = float(msg.get("vibration_delay",s.vibration_delay)),
                        exposure_margin   = float(msg.get("exposure_margin",s.exposure_margin)),
                        ev_day            = float(msg.get("ev_day",        s.ev_day)),
                        ev_golden         = float(msg.get("ev_golden",     s.ev_golden)),
                        ev_twilight       = float(msg.get("ev_twilight",   s.ev_twilight)),
                        ev_night          = float(msg.get("ev_night",      s.ev_night)),
                        kelvin_day        = int(  msg.get("kelvin_day",    s.kelvin_day)),
                        kelvin_golden     = int(  msg.get("kelvin_golden", s.kelvin_golden)),
                        kelvin_twilight   = int(  msg.get("kelvin_twilight",s.kelvin_twilight)),
                        kelvin_night      = int(  msg.get("kelvin_night",  s.kelvin_night)),
                        interval_day      = float(msg.get("interval_day",  s.interval_day)),
                        interval_golden   = float(msg.get("interval_golden",s.interval_golden)),
                        interval_twilight = float(msg.get("interval_twilight",s.interval_twilight)),
                        interval_night    = float(msg.get("interval_night",s.interval_night)),
                        iso_min           = int(  msg.get("iso_min",       s.iso_min)),
                        iso_max           = int(  msg.get("iso_max",       s.iso_max)),
                        iso_max_night     = int(  msg.get("iso_max_night", s.iso_max_night or s.iso_max)),
                        shutter_max_night = float(msg.get("shutter_max_night", s.shutter_max_night)),
                        shutter_max_twilight = float(msg.get("shutter_max_twilight", s.shutter_max_twilight)),
                        night_prefer_low_iso = bool(msg.get("night_prefer_low_iso", s.night_prefer_low_iso)),
                        highlight_clip_limit = float(msg.get("highlight_clip_limit", s.highlight_clip_limit)),
                        shadow_floor_limit   = float(msg.get("shadow_floor_limit",   s.shadow_floor_limit)),
                        agility_day       = float(msg.get("agility_day",    s.agility_day)),
                        agility_golden    = float(msg.get("agility_golden", s.agility_golden)),
                        agility_twilight  = float(msg.get("agility_twilight", s.agility_twilight)),
                        agility_night     = float(msg.get("agility_night",    s.agility_night)),
                        aperture_day      = float(msg.get("aperture_day",  s.aperture_day)),
                        aperture_night    = float(msg.get("aperture_night",s.aperture_night)),
                    ))
                    save_session()
                    await websocket.send_json({"type":"log","msg":"HG settings applied."})
                except Exception as e:
                    await websocket.send_json({"type":"log","msg":f"HG error: {e}"})

            # ── SEQUENCE ──────────────────────────────────────────────────────
            elif cmd == "set_total_frames":
                state["total_frames"] = int(msg.get("value", 300))
                save_session()

            elif cmd == "set_manual_interval":
                val = float(msg.get("value", 5.0))
                state["manual_interval"] = val
                save_session()
                logger.info(f"manual_interval updated to {val:.1f}s")

            elif cmd == "set_tl_preroll":
                state["tl_preroll_s"] = float(msg.get("seconds", 0.0))
                save_session()

            elif cmd == "record_and_run":
                asyncio.create_task(_record_and_run(msg, websocket))

            elif cmd == "read_lens_data":
                # Read focal length + compute FOV, then broadcast lens_info.
                # Priority order:
                #   1. State cache (_sony_focal_mm) — set by _generate_sony_thumb()
                #      from the WiFi JPEG postview EXIF. Works for Sony WiFi without
                #      any local RAW file.
                #   2. Most recent local RAW/DNG/JPG — for PiCam, Sony USB, or a
                #      Sony WiFi JPEG that was saved as the primary download.
                cam  = state.get("active_camera", "picam")
                ori  = state.get("camera_orientation", "landscape")
                save = state.get("save_path", "/home/tim/Pictures/PiSlider")

                fl = state.get("_sony_focal_mm")
                lm = state.get("_sony_lens_model", "")

                if not fl:
                    # Scan for the most recently modified RAW or JPEG on disk
                    import glob as _gl
                    recent = None
                    for pattern in ("*.ARW", "*.arw", "*.DNG", "*.dng",
                                    "*.JPG", "*.jpg", "*.JPEG", "*.jpeg"):
                        matches = sorted(_gl.glob(os.path.join(save, pattern)),
                                         key=os.path.getmtime, reverse=True)
                        if matches:
                            recent = matches[0]
                            break
                    if recent:
                        fl, lm = await asyncio.to_thread(_read_focal_from_exif, recent)

                if fl and fl > 0:
                    hfov, vfov = _compute_fov(fl, cam, ori)
                    label = f"{lm} — " if lm else ""
                    logger.info(f"read_lens_data: {label}{fl:.0f}mm → HFOV={hfov}° VFOV={vfov}°")
                    await websocket.send_json({
                        "type":       "lens_info",
                        "focal_mm":   fl,
                        "lens_model": lm or "",
                        "hfov":       hfov,
                        "vfov":       vfov,
                        "source":     "manual_read",
                    })
                else:
                    await websocket.send_json({"type": "log",
                        "msg": "⚠ No focal length found. Take a preview shot first. "
                               "For manual lenses, set focal length in Sony IBIS menu before shooting."})

            elif cmd == "start_run":
                # Wait briefly for any in-flight worker to finish cleanup
                # (the worker clears stop_event as its very last act)
                deadline = asyncio.get_event_loop().time() + 3.0
                while state["stop_event"].is_set() and asyncio.get_event_loop().time() < deadline:
                    await asyncio.sleep(0.05)
                state["stop_event"].clear()   # safety clear after wait

                if not state["is_running"]:
                    state["total_frames"]    = int(  msg.get("total_frames",  state["total_frames"]))
                    state["vibe_delay"]      = float(msg.get("vibe_delay",    state["vibe_delay"]))
                    state["exp_margin"]      = float(msg.get("exp_margin",    state["exp_margin"]))
                    state["save_path"]       = msg.get("save_path",           state["save_path"])
                    state["trigger_mode"]    = msg.get("trigger_mode",        state["trigger_mode"])
                    state["manual_interval"] = float(msg.get("interval",      state.get("manual_interval", 5.0)))
                    state["schedule_start"]  = msg.get("schedule_start", None)
                    state["schedule_tz"]     = msg.get("schedule_tz",
                                                       state.get("timezone", "America/Chicago"))
                    _iv_log = (
                        f"HG per-phase (day={hg.settings.interval_day:.0f}s "
                        f"golden={hg.settings.interval_golden:.0f}s "
                        f"twilight={hg.settings.interval_twilight:.0f}s "
                        f"night={hg.settings.interval_night:.0f}s)"
                        if hg.settings.enabled
                        else f"{state['manual_interval']:.1f}s"
                    )
                    logger.info(
                        f"start_run: interval={_iv_log}  "
                        f"HG={'ON' if hg.settings.enabled else 'OFF'}  "
                        f"frames={state['total_frames']}  trigger={state['trigger_mode']}"
                    )
                    await broadcast({"type": "log",
                        "msg": f"▶ Server: interval={_iv_log}  "
                               f"HG={'ON' if hg.settings.enabled else 'OFF'}  "
                               f"frames={state['total_frames']}"})

                    # ── Disk space preflight ───────────────────────────────────
                    # Bytes saved to Pi per frame varies by camera mode:
                    #   picam      — full DNG ~25 MB
                    #   sony_usb   — full ARW downloaded ~30 MB
                    #   sony/s2    — thumbnail JPEG ~100 KB + XMP ~5 KB only
                    #                (RAW stays on camera SD card)
                    _cam = state["active_camera"]
                    # Sony WiFi / S2: only thumbnails + XMP saved to Pi — negligible space.
                    # Skip the Pi disk warning for those modes (camera card is checked below).
                    if _cam not in ("sony", "sony_s2"):
                        BYTES_PER_FRAME = 25_000_000 if _cam == "picam" else 30_000_000  # sony_usb
                        needed = state["total_frames"] * BYTES_PER_FRAME
                        try:
                            save_p = state["save_path"] if os.path.exists(state["save_path"]) else "/"
                            free   = shutil.disk_usage(save_p).free
                            if free < needed:
                                free_gb   = free   / 1e9
                                needed_gb = needed / 1e9
                                warn = (f"⚠ LOW DISK SPACE: {free_gb:.1f} GB free but ~{needed_gb:.1f} GB needed "
                                        f"for {state['total_frames']} frames. Sequence may fail mid-run.")
                                logger.warning(warn)
                                # broadcast instead of send_json — avoids duplicate warnings
                                # when multiple browser tabs are open.
                                await broadcast({"type": "disk_warn", "msg": warn,
                                                 "free_gb": round(free_gb, 2),
                                                 "needed_gb": round(needed_gb, 2)})
                        except Exception as de:
                            logger.warning(f"Disk preflight check failed: {de}")

                    # ── Sony card space preflight ──────────────────────────────
                    # Query the Sony camera for remaining shots and warn if the
                    # card will run out before the sequence completes.
                    if state.get("active_camera") == "sony":
                        try:
                            def _get_sony_remaining():
                                res = _sony_api("getStorageInformation", [])
                                entries = res.get("result", [[]])[0]
                                for entry in entries:
                                    if entry.get("recordTarget", "").lower() in ("yes", "true", "1"):
                                        return int(entry.get("numberOfRecordableImages",
                                                             entry.get("remainingCount", -1)))
                                # fallback: first entry
                                if entries:
                                    return int(entries[0].get("numberOfRecordableImages",
                                                              entries[0].get("remainingCount", -1)))
                                return -1

                            remaining_shots = await asyncio.to_thread(_get_sony_remaining)
                            if remaining_shots >= 0:
                                planned = state["total_frames"]
                                if remaining_shots < planned:
                                    short_by = planned - remaining_shots
                                    warn = (
                                        f"⚠ SONY CARD SPACE: camera reports {remaining_shots} shots remaining "
                                        f"but sequence needs {planned} frames — will run short by {short_by}. "
                                        f"Swap card or reduce frame count before starting."
                                    )
                                    logger.warning(warn)
                                    await websocket.send_json({
                                        "type": "sony_storage_warn",
                                        "msg": warn,
                                        "remaining": remaining_shots,
                                        "needed": planned,
                                        "short_by": short_by,
                                    })
                                    # Warn only — don't block start; let user decide
                                else:
                                    logger.info(
                                        f"Sony storage OK: {remaining_shots} shots remaining "
                                        f"(need {planned})"
                                    )
                        except Exception as se:
                            logger.warning(f"Sony storage preflight skipped: {se}")

                    if hg.settings.enabled:
                        base_iv = getattr(hg.settings, 'interval_sec',
                                          getattr(hg.settings, 'interval_day', 5.0))
                    else:
                        base_iv = state["manual_interval"]

                    # ── Path preflight: check motion path vs soft limits ───────
                    if _prog_move and len(_prog_move.keyframes) >= 2:
                        _ts, _tp, _tt = _prog_move.generate_unified_trajectory(
                            state["total_frames"],
                            _prog_move.origin_slider, _prog_move.origin_pan, _prog_move.origin_tilt,
                            for_timelapse=True)
                        violations = _check_path_vs_limits(_ts, _tp, _tt)
                        if violations:
                            _pending_play = {"context": "timelapse", "base_iv": base_iv}
                            await websocket.send_json({
                                "type": "path_limit_warning",
                                "context": "timelapse",
                                "violations": violations,
                            })
                            return   # wait for user confirmation

                    save_session()
                    asyncio.create_task(timelapse_worker(base_iv))

            elif cmd == "stop":
                _was_timelapse_running = state["is_running"]
                state["stop_event"].set()
                hw.stop_all_axes()
                state["is_running"] = False
                _cinematic_live_active = False   # clear live gate so L1/R1 works after stop
                state["_stop_reason"] = (
                    f"Stopped by user at frame {state['current_frame']} "
                    f"of {state['total_frames']}."
                )
                _save_session_history()
                save_session()
                await broadcast({"type": "run_state",  "running": False})
                await broadcast({"type": "stop_reason","msg": state["_stop_reason"]})
                await broadcast({"type": "log", "msg": f"[✓] {state['_stop_reason']}"})
                # InertiaEngine restart strategy:
                #   • If a timelapse WAS running, InertiaEngine was already stopped
                #     for Bresenham stepping.  move_axes_simultaneous may still be
                #     executing in a thread — starting InertiaEngine now would cause
                #     tx_pwm (InertiaEngine) vs gpio_write (Bresenham) conflicts on
                #     the same STEP pins, crashing the engine.  Let the timelapse
                #     finally-block restart it after the thread finishes.
                #     Signal UI to show "✅ Safe to Move" button.
                #   • If NO timelapse was running (e.g. cinematic play, manual jog),
                #     restart InertiaEngine immediately for instant movement control.
                if not _was_timelapse_running:
                    if _inertia:
                        _inertia.set_target(0, 0, 0)
                        _inertia.set_preset("responsive")
                        if not _inertia._running:
                            _inertia.start()
                    # Safety net: if no timelapse worker is running to clear stop_event,
                    # clear it ourselves after a short delay so nudge/joystick still works.
                    async def _deferred_clear_non_tl():
                        await asyncio.sleep(3.0)
                        state["stop_event"].clear()
                        logger.info("stop_event auto-cleared (non-timelapse stop).")
                    asyncio.create_task(_deferred_clear_non_tl())
                else:
                    # Timelapse WAS running: DO NOT auto-clear stop_event here.
                    # The timelapse worker's finally-block clears it after it exits.
                    # Auto-clearing here was the bug — it wiped the signal before the
                    # worker (mid-frame, 9-10s loop) had a chance to see it.
                    await broadcast({"type": "estop_fired"})

            elif cmd == "resume_control":
                # User confirms it is safe to move after an E-stop.
                # Cleanly restarts InertiaEngine and clears any residual stop state.
                state["stop_event"].clear()
                state["is_running"] = False
                if _inertia:
                    # Full stop → start cycle to reset internal physics state
                    if _inertia._running:
                        _inertia.stop()
                        await asyncio.sleep(0.1)   # let task cancel propagate
                    _inertia.set_target(0, 0, 0)
                    _inertia.set_preset("responsive")
                    _inertia.start()
                hw.stop_all_axes()   # ensure clean step-pin state before InertiaEngine takes over
                await broadcast({"type": "control_resumed"})
                await broadcast({"type": "log", "msg": "✅ Movement control restored."})
                logger.info("resume_control: InertiaEngine restarted, stop_event cleared.")

            elif cmd == "reset_session":
                    # ── Full recovery reset — works even during a running sequence ──
                    # Force-stop any running sequence first
                    if state["is_running"]:
                        state["stop_event"].set()
                        state["is_running"] = False
                        await asyncio.sleep(0.3)   # let sequence loop wake and exit

                    state["stop_event"].clear()
                    if _macro_task and not _macro_task.done():
                        _macro_task.cancel()
                    if _inertia:
                        _inertia.stop()
                    if _prog_move:
                        _prog_move.stop()
                    hw.stop_all_axes()
                    try: hw.set_relay1(False); hw.set_relay2(False)
                    except Exception: pass

                    # Recover camera if it's in a bad state
                    if _HAS_PICAM and picam:
                        try:
                            _restart_picam()
                        except Exception:
                            pass

                    # Wipe session so restarted process starts at frame 0
                    reset_session()
                    await websocket.send_json({"type": "log",
                        "msg": "♻ Restarting server process…"})
                    await asyncio.sleep(0.3)
                    hw.cleanup()
                    import os as _os
                    _os.execv(_os.sys.executable,
                              [_os.sys.executable] + _os.sys.argv)

            # ── MOTION TEST ───────────────────────────────────────────────────
            elif cmd == "run_motion_test":
                if state["is_running"]: continue
                axis = msg.get("axis","slider"); curve = msg.get("curve","linear")
                total = float(msg.get("total",100)); intervals = int(msg.get("intervals",50))
                # Pause InertiaEngine — Bresenham stepping and set_axis_speed both use STEP pins
                if _inertia and _inertia._running:
                    _inertia.stop()
                def _mt():
                    weights = normalize(CURVE_FUNCTIONS.get(curve, CURVE_FUNCTIONS["linear"])(intervals))
                    for w in weights:
                        if state["stop_event"].is_set(): break
                        if axis == "slider":
                            hw.move_axes_simultaneous(int(w*total*slider_axis.steps_per_mm),0,0,0.5)
                        elif axis == "pan":
                            hw.move_axes_simultaneous(0,int(w*total*pan_axis.steps_per_deg),0,0.5)
                        elif axis == "tilt":
                            hw.move_axes_simultaneous(0,0,int(w*total*tilt_axis.steps_per_deg),0.5)
                async def _motion_test_runner():
                    await asyncio.to_thread(_mt)
                    if _inertia:
                        _inertia.set_target(0, 0, 0)
                        _inertia.set_preset("responsive")
                        if not _inertia._running:
                            _inertia.start()
                asyncio.create_task(_motion_test_runner())

            elif cmd == "home_axis":
                await websocket.send_json({"type": "log", "msg": "🏠 Starting soft‑home sequence..."})
                # Ensure motors are enabled.
                hw.enable_motors(True)

                # Helper to move axes to target deltas.
                async def move_axes(delta_slider: int, delta_pan: int, delta_tilt: int, description: str):
                    if delta_slider == delta_pan == delta_tilt == 0:
                        logger.info(f"Soft‑home: {description} – already at target")
                        return
                    max_delta = max(abs(delta_slider), abs(delta_pan), abs(delta_tilt))
                    duration = max(0.5, max_delta / 500.0)  # modest speed
                    logger.info(f"Soft‑home: moving {description} – s:{delta_slider} p:{delta_pan} t:{delta_tilt}, duration {duration:.2f}s")
                    await asyncio.to_thread(hw.move_axes_simultaneous, delta_slider, delta_pan, delta_tilt, duration)
                    await asyncio.sleep(duration + 0.2)

                # Determine target positions from saved state (fallback to zero).
                target_slider_mm = state.get("macro_rail_start_mm", 0.0)
                target_slider_steps = int(target_slider_mm * STEPS_PER_MM)
                delta_slider = target_slider_steps - slider_axis.current_steps

                target_pan_deg = state.get("macro_rotation_start_deg", 0.0)
                target_pan_steps = int(target_pan_deg * pan_axis.steps_per_deg)
                delta_pan = target_pan_steps - getattr(pan_axis, "current_steps", 0)

                target_tilt_deg = state.get("macro_aux_start_deg", 0.0)
                target_tilt_steps = int(target_tilt_deg * tilt_axis.steps_per_deg)
                delta_tilt = target_tilt_steps - getattr(tilt_axis, "current_steps", 0)

                await move_axes(delta_slider, delta_pan, delta_tilt, "to configured start positions")

                # Update internal counters.
                slider_axis.current_mm = target_slider_mm
                slider_axis.current_steps = target_slider_steps
                pan_axis.current_deg = target_pan_deg
                if hasattr(pan_axis, "current_steps"):
                    pan_axis.current_steps = target_pan_steps
                tilt_axis.current_deg = target_tilt_deg
                if hasattr(tilt_axis, "current_steps"):
                    tilt_axis.current_steps = target_tilt_steps

                # Broadcast new zeroed position.
                await broadcast({"type": "hardware_zeroed", "slider_mm": slider_axis.current_mm,
                                 "pan_deg": pan_axis.current_deg, "tilt_deg": tilt_axis.current_deg})
                await websocket.send_json({"type": "log", "msg": "✓ Soft‑home complete. Axes positioned at start values."})

            elif cmd == "add_node":
                engine.add_keyframe(slider_axis.current_mm, pan_axis.current_deg, tilt_axis.current_deg)
                # Sync into Cinematic engine too.
                # Timelapse keyframes are ABSOLUTE world positions, so origin must
                # be 0,0,0.  Reset it here so a stale cinematic deployment origin
                # doesn't corrupt return_to_start or the sequence pre-position.
                if _prog_move:
                    _prog_move.add_keyframe(slider_axis.current_mm, pan_axis.current_deg, tilt_axis.current_deg)
                    _prog_move.set_origin(0.0, 0.0, 0.0)
                state["cinematic_origin"] = {}
                sync_all_keyframes()
                await broadcast_points()

            elif cmd == "clear_nodes":
                engine.clear_keyframes()
                if _prog_move:
                    _prog_move.clear_keyframes()
                    _prog_move.set_origin(0.0, 0.0, 0.0)   # reset stale deployment offset
                state["cinematic_origin"] = {}
                sync_all_keyframes()
                await broadcast_points()

            elif cmd == "set_relay":
                relay = int(msg.get("relay",1)); on = bool(msg.get("on",False))
                if relay == 1:
                    hw.set_relay1(on)
                    status_leds.set_relay(1, on)
                elif relay == 2:
                    hw.set_relay2(on)
                    status_leds.set_relay(2, on)

            elif cmd == "set_fan":
                hw.set_fan(int(msg.get("value",0)))

            elif cmd == "build_sky_map":
                await websocket.send_json({"type":"log",
                    "msg":"Sky Map: wiring requires motor + camera connection — test pending."})

            elif cmd == "set_preview_camera":
                # Toggle preview source independently from capture camera
                cam = msg.get("camera", state["active_camera"])
                state["preview_camera"] = cam
                # Cache-bust stream — client must refresh video_feed after this
                await websocket.send_json({"type": "preview_camera_changed",
                                           "camera": cam,
                                           "label": "Sony (framing)" if cam == "sony" else "PiCam (motion zone)"})

            elif cmd == "create_folder":
                parent = msg.get("path", str(Path.home() / "Pictures"))
                name   = msg.get("name", "").strip().replace("/", "_").replace("..", "")
                if not name:
                    await websocket.send_json({"type":"log","msg":"Create folder: name required."})
                else:
                    new_path = os.path.join(parent, name)
                    try:
                        os.makedirs(new_path, exist_ok=True)
                        await websocket.send_json({"type":"folder_created","path":new_path})
                    except Exception as e:
                        await websocket.send_json({"type":"log","msg":f"Create folder error: {e}"})

            elif cmd == "connect_sony_wifi":
                ssid     = msg.get("ssid",""); password = msg.get("password","")
                state["sony_ssid"] = ssid
                async def _sc():
                    await websocket.send_json({"type":"sony_status","connected":False,
                                               "msg":"Dropping current wlan1 connection…"})
                    try:
                        # Step 1: explicitly disconnect wlan1 before reconnecting.
                        # NetworkManager's auto-connect can race against us if we just
                        # call "connect" while wlan1 is already managed by another profile.
                        await asyncio.to_thread(lambda: subprocess.run(
                            ["nmcli", "dev", "disconnect", "wlan1"],
                            capture_output=True, text=True, timeout=10))
                        await asyncio.sleep(0.5)   # let NM settle

                        await websocket.send_json({"type":"sony_status","connected":False,
                                                   "msg":f"Joining {ssid}…"})

                        # Step 2: connect wlan1 to the Sony camera AP
                        res = await asyncio.to_thread(lambda: subprocess.run(
                            ["nmcli","dev","wifi","connect",ssid,"password",password,"ifname","wlan1"],
                            capture_output=True, text=True, timeout=30))

                        if res.returncode != 0:
                            err = res.stderr.strip() or res.stdout.strip()
                            await websocket.send_json({"type":"sony_status","connected":False,"error":err})
                            return

                        # Step 3: lock wlan1 — prevent NM from roaming to a stronger
                        # known network (home WiFi, Starlink) mid-shoot.
                        await asyncio.to_thread(_wlan1_lock)

                        # Step 4: discover the real camera IP from the wlan1 gateway.
                        # We NEVER use a hardcoded IP — Sony WiFi Direct assigns addresses
                        # dynamically and the gateway IS the camera.
                        await websocket.send_json({"type":"sony_status","connected":False,
                                                   "msg":"WiFi joined — waiting for DHCP…"})
                        await asyncio.sleep(3.0)   # give DHCP time to assign an address

                        status = await asyncio.to_thread(_check_sony_wlan1)
                        if status["connected"]:
                            state["sony_http_port"] = status.get("http_port", 8080)
                            iface    = status.get("iface", "wlan1")
                            ptp_mode = status.get("ptp_mode", False)
                            http_mode= status.get("http_mode", False)
                            mode_str = ("PTP/IP+HTTP" if ptp_mode and http_mode
                                        else "HTTP only" if http_mode
                                        else "PTP/IP")
                            await websocket.send_json({
                                "type":      "sony_status",
                                "connected": True,
                                "ip":        status["ip"],
                                "ssid":      status["ssid"],
                                "iface":     iface,
                                "ptp_mode":  ptp_mode,
                                "http_mode": http_mode,
                                "model":     f"Sony A7III ({mode_str})",
                            })
                            _start_sony_liveview()
                        else:
                            # WiFi joined but camera not responding on either port.
                            # Report the IP we found (or attempted) to help diagnose.
                            cam_ip = status.get("ip") or state.get("sony_ip","?")
                            await websocket.send_json({
                                "type":  "sony_status",
                                "connected": False,
                                "error": (status.get("error") or
                                          f"WiFi joined but camera unreachable at {cam_ip}. "
                                          "Enable 'PC Remote' on the camera (Network → "
                                          "PC Remote Settings → Still Img. Save Dest → "
                                          "Computer only)."),
                            })
                    except Exception as e:
                        await websocket.send_json({"type":"sony_status","connected":False,"error":str(e)})
                asyncio.create_task(_sc())

            elif cmd == "check_sony_connection":
                # Check if wlan1 is already connected to a Sony camera (pre-connected workflow).
                # Discovers the camera IP from the wlan1 gateway — no password needed.
                async def _check_sc():
                    status = await asyncio.to_thread(_check_sony_wlan1)
                    if status["connected"]:
                        state["sony_http_port"] = status.get("http_port", 8080)
                        iface    = status.get("iface", "wlan")
                        ptp_mode = status.get("ptp_mode", False)
                        http_mode= status.get("http_mode", False)
                        mode_str = ("PTP/IP+HTTP" if ptp_mode and http_mode
                                    else "HTTP only" if http_mode
                                    else "PTP/IP")
                        await websocket.send_json({
                            "type":      "sony_status",
                            "connected": True,
                            "ip":        status["ip"],
                            "ssid":      status["ssid"],
                            "iface":     iface,
                            "ptp_mode":  ptp_mode,
                            "http_mode": http_mode,
                            "model":     f"Sony A7III ({mode_str})",
                        })
                        asyncio.create_task(asyncio.to_thread(_sony_api, "startRecMode"))
                        asyncio.create_task(asyncio.to_thread(_sony_set_current_time))
                        _start_sony_liveview()   # start background stream reader
                    else:
                        await websocket.send_json({
                            "type":      "sony_status",
                            "connected": False,
                            "error":     status["error"],
                        })
                asyncio.create_task(_check_sc())

            elif cmd == "sony_wifi_scan":
                # Scan wlan1 for visible WiFi networks (runs nmcli --rescan yes).
                async def _scan_sony():
                    await websocket.send_json({
                        "type": "sony_scan_status",
                        "msg":  f"Scanning {SONY_IFACE} for networks…"
                    })
                    try:
                        res = await asyncio.to_thread(lambda: subprocess.run(
                            ["nmcli", "-t", "-f", "SSID,SIGNAL,SECURITY",
                             "device", "wifi", "list",
                             "ifname", SONY_IFACE, "--rescan", "yes"],
                            capture_output=True, text=True, timeout=25
                        ))
                        networks = []
                        seen: set = set()
                        for line in res.stdout.splitlines():
                            parts = line.split(":")
                            ssid = parts[0].strip() if parts else ""
                            # nmcli escapes colons as \: — unescape
                            ssid = ssid.replace("\\:", ":")
                            if not ssid or ssid in seen:
                                continue
                            seen.add(ssid)
                            signal   = parts[1] if len(parts) > 1 else "?"
                            security = parts[2] if len(parts) > 2 else ""
                            networks.append({
                                "ssid":     ssid,
                                "signal":   signal,
                                "security": security,
                            })
                        # Sort: Sony/DIRECT- first, then by signal desc
                        def _rank(n):
                            s = n["ssid"]
                            is_sony = s.startswith("DIRECT-") or "SONY" in s.upper() or "ILCE" in s.upper()
                            sig = int(n["signal"]) if n["signal"].lstrip("-").isdigit() else 0
                            return (0 if is_sony else 1, -sig)
                        networks.sort(key=_rank)
                        await websocket.send_json({
                            "type":     "sony_scan_result",
                            "networks": networks,
                        })
                    except Exception as exc:
                        await websocket.send_json({
                            "type":     "sony_scan_result",
                            "networks": [],
                            "error":    str(exc),
                        })
                asyncio.create_task(_scan_sony())

            elif cmd == "disconnect_sony_wifi":
                # Drop the Sony camera connection and unlock wlan1 so NM can
                # resume normal management (e.g. reconnect to home WiFi/Starlink).
                async def _drop_sony():
                    await websocket.send_json({"type":"sony_status","connected":False,
                                               "msg":"Dropping Sony connection…"})
                    try:
                        # Unlock first — must happen before disconnect so NM is
                        # free to manage the device again immediately after.
                        await asyncio.to_thread(_wlan1_unlock)
                        await asyncio.to_thread(lambda: subprocess.run(
                            ["nmcli", "dev", "disconnect", SONY_IFACE],
                            capture_output=True, text=True, timeout=10))
                        state["sony_ip"] = ""
                        _stop_sony_liveview()    # stop background stream reader
                        await websocket.send_json({"type":"sony_status","connected":False,
                                                   "msg":"Disconnected — wlan1 released"})
                        logger.info("Sony WiFi disconnected; wlan1 autoconnect restored")
                    except Exception as exc:
                        await websocket.send_json({"type":"sony_status","connected":False,
                                                   "error":f"Disconnect error: {exc}"})
                asyncio.create_task(_drop_sony())

            # ── MACRO MODE ────────────────────────────────────────────────────
            elif cmd == "macro_set_soft_limits":
                # Update software travel limits for all three axes
                # For slider (focus), also calculate and store the step-based limits
                # NOTE: Use STEPS_PER_MM = 800.0 (macro mode) not slider_axis.steps_per_mm (timelapse mode)
                if "rail_min" in msg:
                    slider_axis.soft_min = float(msg["rail_min"])
                    slider_axis.soft_min_steps = int(slider_axis.soft_min * STEPS_PER_MM)
                if "rail_max" in msg:
                    slider_axis.soft_max = float(msg["rail_max"])
                    slider_axis.soft_max_steps = int(slider_axis.soft_max * STEPS_PER_MM)
                if "pan_min"  in msg: pan_axis.soft_min    = float(msg["pan_min"])
                if "pan_max"  in msg: pan_axis.soft_max    = float(msg["pan_max"])
                if "tilt_min" in msg: tilt_axis.soft_min   = float(msg["tilt_min"])
                if "tilt_max" in msg: tilt_axis.soft_max   = float(msg["tilt_max"])
                save_session()
                # Log in both mm and steps for clarity
                logger.info(f"Soft limits updated: "
                           f"rail [{slider_axis.soft_min:.1f}…{slider_axis.soft_max:.1f}]mm "
                           f"([{slider_axis.soft_min_steps}…{slider_axis.soft_max_steps}] steps)  "
                           f"pan [{pan_axis.soft_min:.1f}…{pan_axis.soft_max:.1f}]°  "
                           f"tilt [{tilt_axis.soft_min:.1f}…{tilt_axis.soft_max:.1f}]°")
                await websocket.send_json({"type": "log",
                    "msg": f"Soft limits updated: rail [{slider_axis.soft_min:.1f}…{slider_axis.soft_max:.1f}] mm  "
                           f"pan [{pan_axis.soft_min:.1f}…{pan_axis.soft_max:.1f}]°  "
                           f"tilt [{tilt_axis.soft_min:.1f}…{tilt_axis.soft_max:.1f}]°  "
                           f"(nudge will be restricted to these limits)"})

            elif cmd == "macro_allow_full_rotation":
                # Allow pan and/or tilt to rotate full 360° (bypass soft limits)
                axis = msg.get("axis", "both")  # "pan" | "tilt" | "both"
                enable = msg.get("enable", True)
                if axis in ("pan", "both"):
                    if enable:
                        pan_axis.soft_min = -180.0
                        pan_axis.soft_max = 180.0
                    else:
                        pan_axis.soft_min = -90.0
                        pan_axis.soft_max = 90.0
                if axis in ("tilt", "both"):
                    if enable:
                        tilt_axis.soft_min = -180.0
                        tilt_axis.soft_max = 180.0
                    else:
                        tilt_axis.soft_min = -30.0
                        tilt_axis.soft_max = 30.0
                save_session()
                status = "enabled" if enable else "disabled"
                await websocket.send_json({"type": "log",
                    "msg": f"Full rotation {status} for {axis}: "
                           f"pan [{pan_axis.soft_min:.0f}…{pan_axis.soft_max:.0f}]° "
                           f"tilt [{tilt_axis.soft_min:.0f}…{tilt_axis.soft_max:.0f}]°"})

            elif cmd == "diagnostic_inertia_status":
                # Check if InertiaEngine is initialized and running
                if not _inertia:
                    await websocket.send_json({"type": "log",
                        "msg": "[X] InertiaEngine: NOT INITIALIZED (critical error)"})
                else:
                    status = "[●] RUNNING" if _inertia._running else "[●] STOPPED"
                    await websocket.send_json({"type": "log",
                        "msg": f"InertiaEngine: {status}\n"
                               f"  Positions: slider={_inertia._slider.current_mm:.2f}mm, "
                               f"pan={_inertia._pan.current_deg:.2f}°, "
                               f"tilt={_inertia._tilt.current_deg:.2f}°\n"
                               f"  Nudge: pan={_inertia._nudge_pan:.1f}°/s, "
                               f"tilt={_inertia._nudge_tilt:.1f}°/s, "
                               f"slider={_inertia._nudge_slider:.1f}mm/s"
                               })

            elif cmd == "diagnostic_motor_test":
                # Brief motor nudge to test responsiveness
                if not _inertia:
                    await websocket.send_json({"type": "log",
                        "msg": "[X] InertiaEngine not available"})
                else:
                    axis = msg.get("axis", "slider")  # slider | pan | tilt
                    if not _inertia._running:
                        _inertia.start()
                    _inertia._guard_bypass = True
                    if axis == "slider":
                        _inertia.set_nudge_slider(20.0)
                        await asyncio.sleep(0.2)
                        _inertia.set_nudge_slider(0.0)
                        await asyncio.sleep(0.05)
                    elif axis == "pan":
                        _inertia.set_nudge_pt(10.0, _inertia._nudge_tilt)
                        await asyncio.sleep(0.2)
                        _inertia.set_nudge_pt(0.0, _inertia._nudge_tilt)
                        await asyncio.sleep(0.05)
                    elif axis == "tilt":
                        _inertia.set_nudge_pt(_inertia._nudge_pan, 10.0)
                        await asyncio.sleep(0.2)
                        _inertia.set_nudge_pt(_inertia._nudge_pan, 0.0)
                        await asyncio.sleep(0.05)
                    _inertia._guard_bypass = False
                    await websocket.send_json({"type": "log",
                        "msg": f"[✓] Motor test pulse sent to {axis}. Did it move?"})

            elif cmd == "macro_set_rail_start":
                # Mark current rail position as the focus stack start
                # Record BOTH mm (for UI) and steps (for accurate macro movement)
                # NOTE: Steps are offset from last calibrated home position
                state["macro_rail_start_mm"] = slider_axis.current_mm
                state["macro_rail_start_steps"] = slider_axis.current_steps
                logger.info(f"Rail start marked: {slider_axis.current_mm:.2f}mm ({slider_axis.current_steps} steps from home)")
                # IMPORTANT: User expects this to carry over.
                # If no keyframes exist, this is effectively our Point A.
                if _prog_move:
                    if len(_prog_move.keyframes) == 0:
                        _prog_move.add_keyframe(slider_axis.current_mm, pan_axis.current_deg, tilt_axis.current_deg)
                    else:
                        _prog_move.update_keyframe(0, slider_mm=slider_axis.current_mm)
                    sync_all_keyframes()
                    await broadcast_points()

                save_session()
                await websocket.send_json({"type": "macro_rail_mark",
                    "which": "start", "mm": slider_axis.current_mm, "steps": slider_axis.current_steps})

            elif cmd == "macro_set_rail_end":
                # Record BOTH mm (for UI) and steps (for accurate macro movement)
                # NOTE: Steps are offset from last calibrated home position
                state["macro_rail_end_mm"] = slider_axis.current_mm
                state["macro_rail_end_steps"] = slider_axis.current_steps
                # Calculate actual step movement between start and end
                start_steps = state.get("macro_rail_start_steps", 0)
                total_steps = slider_axis.current_steps - start_steps
                logger.info(f"Rail end marked: {slider_axis.current_mm:.2f}mm ({slider_axis.current_steps} steps from home) "
                           f"→ Total movement: {total_steps} steps")
                if _prog_move:
                    if len(_prog_move.keyframes) < 2:
                        _prog_move.add_keyframe(slider_axis.current_mm, pan_axis.current_deg, tilt_axis.current_deg)
                    else:
                        _prog_move.update_keyframe(len(_prog_move.keyframes)-1, slider_mm=slider_axis.current_mm)
                    sync_all_keyframes()
                    await broadcast_points()
                save_session()
                await websocket.send_json({"type": "macro_rail_mark",
                    "which": "end", "mm": slider_axis.current_mm, "steps": slider_axis.current_steps})

            elif cmd == "macro_set_rotation_start":
                state["macro_rotation_start_deg"] = pan_axis.current_deg
                if _prog_move:
                    if len(_prog_move.keyframes) == 0:
                        _prog_move.add_keyframe(slider_axis.current_mm, pan_axis.current_deg, tilt_axis.current_deg)
                    else:
                        _prog_move.update_keyframe(0, pan_deg=pan_axis.current_deg)
                    sync_all_keyframes()
                    await broadcast_points()
                save_session()
                await websocket.send_json({"type": "macro_rotation_mark",
                    "which": "start", "deg": pan_axis.current_deg})

            elif cmd == "macro_set_rotation_end":
                state["macro_rotation_end_deg"] = pan_axis.current_deg
                if _prog_move:
                    if len(_prog_move.keyframes) < 2:
                        _prog_move.add_keyframe(slider_axis.current_mm, pan_axis.current_deg, tilt_axis.current_deg)
                    else:
                        _prog_move.update_keyframe(len(_prog_move.keyframes)-1, pan_deg=pan_axis.current_deg)
                    sync_all_keyframes()
                    await broadcast_points()
                save_session()
                await websocket.send_json({"type": "macro_rotation_mark",
                    "which": "end", "deg": pan_axis.current_deg})

            elif cmd == "macro_set_aux_start":
                state["macro_aux_start_deg"] = tilt_axis.current_deg
                if _prog_move:
                    if len(_prog_move.keyframes) == 0:
                        _prog_move.add_keyframe(slider_axis.current_mm, pan_axis.current_deg, tilt_axis.current_deg)
                    else:
                        _prog_move.update_keyframe(0, tilt_deg=tilt_axis.current_deg)
                    sync_all_keyframes()
                    await broadcast_points()
                save_session()
                await websocket.send_json({"type": "macro_aux_mark",
                    "which": "start", "deg": tilt_axis.current_deg})

            elif cmd == "macro_set_aux_end":
                state["macro_aux_end_deg"] = tilt_axis.current_deg
                if _prog_move:
                    if len(_prog_move.keyframes) < 2:
                        _prog_move.add_keyframe(slider_axis.current_mm, pan_axis.current_deg, tilt_axis.current_deg)
                    else:
                        _prog_move.update_keyframe(len(_prog_move.keyframes)-1, tilt_deg=tilt_axis.current_deg)
                    sync_all_keyframes()
                    await broadcast_points()
                save_session()
                await websocket.send_json({"type": "macro_aux_mark",
                    "which": "end", "deg": tilt_axis.current_deg})

            elif cmd == "macro_set_tilt_start":
                # Geodesic 2D grid: mark tilt start for limited range mode
                state["macro_tilt_start_deg"] = tilt_axis.current_deg
                if _prog_move:
                    if len(_prog_move.keyframes) == 0:
                        _prog_move.add_keyframe(slider_axis.current_mm, pan_axis.current_deg, tilt_axis.current_deg)
                    else:
                        _prog_move.update_keyframe(0, tilt_deg=tilt_axis.current_deg)
                    sync_all_keyframes()
                    await broadcast_points()
                save_session()
                await websocket.send_json({"type": "macro_tilt_mark",
                    "which": "start", "deg": tilt_axis.current_deg})

            elif cmd == "macro_set_tilt_end":
                # Geodesic 2D grid: mark tilt end for limited range mode
                state["macro_tilt_end_deg"] = tilt_axis.current_deg
                if _prog_move:
                    if len(_prog_move.keyframes) < 2:
                        _prog_move.add_keyframe(slider_axis.current_mm, pan_axis.current_deg, tilt_axis.current_deg)
                    else:
                        _prog_move.update_keyframe(len(_prog_move.keyframes)-1, tilt_deg=tilt_axis.current_deg)
                    sync_all_keyframes()
                    await broadcast_points()
                save_session()
                await websocket.send_json({"type": "macro_tilt_mark",
                    "which": "end", "deg": tilt_axis.current_deg})

            elif cmd == "macro_go_home":
                # Move all three axes CONCURRENTLY to home: pan=0°, tilt=0°, slider/focus=0mm
                # Works in all modes (timelapse, cinematic, macro).
                #
                # WHY current_steps (not current_mm) for the slider:
                #   InertiaEngine's VACTUAL conversion is calibrated for the belt drive
                #   (SLIDER_STEPS_PER_MM=50).  In macro mode the focus rail lead screw is
                #   800 steps/mm, so the motor physically moves at 1/16 the speed that
                #   InertiaEngine's mm/s internal units imply — current_mm is 16× too large.
                #   current_steps however accumulates int(delta_mm * 50) = exact motor pulses
                #   emitted by the VACTUAL→PWM bridge for BOTH axes.  So -current_steps is
                #   always the correct return distance in raw motor steps regardless of mode.
                await websocket.send_json({"type": "log",
                    "msg": "📍 Moving to home position (pan=0°, tilt=0°, slider=0)…"})

                try:
                    def shortest_angle(current, target):
                        delta = target - current
                        while delta > 180:  delta -= 360
                        while delta < -180: delta += 360
                        return current + delta

                    # Slider: use raw step count — valid for belt drive AND focus rail
                    delta_slider = -slider_axis.current_steps if slider_axis else 0

                    # Pan / Tilt: degree tracking IS correct for both modes
                    delta_pan  = 0
                    delta_tilt = 0
                    if pan_axis:
                        pan_target = shortest_angle(pan_axis.current_deg, 0.0)
                        delta_pan  = int((pan_target - pan_axis.current_deg) * pan_axis.steps_per_deg)
                    if tilt_axis:
                        tilt_target = shortest_angle(tilt_axis.current_deg, 0.0)
                        delta_tilt  = int((tilt_target - tilt_axis.current_deg) * tilt_axis.steps_per_deg)

                    active_mode = state.get("active_mode", "timelapse")
                    logger.info(f"🏠 Go to Home ({active_mode} mode):")
                    if slider_axis:
                        logger.info(f"   Slider: {slider_axis.current_steps:+7d} steps → 0  ({delta_slider:+7d} steps)")
                    if pan_axis:
                        logger.info(f"   Pan:    {pan_axis.current_deg:+7.1f}° → 0.0°  ({delta_pan:+7d} steps)")
                    if tilt_axis:
                        logger.info(f"   Tilt:   {tilt_axis.current_deg:+7.1f}° → 0.0°  ({delta_tilt:+7d} steps)")

                    if any([delta_slider, delta_pan, delta_tilt]):
                        if _inertia and getattr(_inertia, "_running", False):
                            _inertia.stop()
                            _inertia.set_target(0, 0, 0)

                        hw.enable_motors(True)

                        # GPIO bit-bang tops out at ~3,000 steps/sec
                        max_delta = max(abs(delta_slider), abs(delta_pan), abs(delta_tilt))
                        duration  = max(max_delta / 3000.0, 0.5)
                        logger.info(f"   Duration: {duration:.2f}s")

                        await asyncio.to_thread(hw.move_axes_simultaneous,
                                                delta_slider, delta_pan, delta_tilt, duration)

                        if slider_axis:
                            slider_axis.current_mm    = 0.0
                            slider_axis.current_steps = 0
                        if pan_axis:
                            pan_axis.current_deg  = 0.0
                        if tilt_axis:
                            tilt_axis.current_deg = 0.0

                        hw.enable_motors(True)

                    await websocket.send_json({"type": "log",
                        "msg": "✓ Home position reached."})
                except Exception as e:
                    logger.error(f"macro_go_home error: {e}", exc_info=True)
                    await websocket.send_json({"type": "log",
                        "msg": f"⚠ Error moving to home: {e}"})

            elif cmd == "macro_get_easing_curves":
                # Return list of available easing functions for UI dropdown
                easing_names = sorted(CURVE_FUNCTIONS.keys())
                await websocket.send_json({
                    "type": "macro_easing_curves",
                    "curves": easing_names
                })

            elif cmd == "macro_compute_grid":
                # Compute optimal pan_cols/tilt_rows for even surface area coverage
                # Geodesic 2D grid distribution respecting pan_mode and tilt_mode
                try:
                    total = int(msg.get("total_stacks", 36))
                    pan_min = float(msg.get("pan_min", -90.0))
                    pan_max = float(msg.get("pan_max", 90.0))
                    tilt_min = float(msg.get("tilt_min", -30.0))
                    tilt_max = float(msg.get("tilt_max", 30.0))
                    pan_mode = msg.get("pan_mode", "full")  # 'full' | 'range'
                    tilt_mode = msg.get("tilt_mode", "full")  # 'full' | 'limited'
                    pan_axis_tilt_deg = float(msg.get("pan_axis_tilt_deg",
                                              state.get("macro_pan_axis_tilt_deg", 90.0)))

                    pan_cols, tilt_rows = compute_geodesic_grid(
                        total, pan_min, pan_max, tilt_min, tilt_max,
                        pan_axis_tilt_deg=pan_axis_tilt_deg
                    )
                    actual_stacks = pan_cols * tilt_rows

                    mode_str = f"Pan:{pan_mode} Tilt:{tilt_mode}"

                    await websocket.send_json({
                        "type": "macro_grid_computed",
                        "total_requested": total,
                        "pan_cols": pan_cols,
                        "tilt_rows": tilt_rows,
                        "total_actual": actual_stacks,
                        "pan_mode": pan_mode,
                        "tilt_mode": tilt_mode,
                        "msg": f"Grid: {pan_cols}×{tilt_rows} = {actual_stacks} stacks ({mode_str})"
                    })
                except Exception as e:
                    await websocket.send_json({"type": "log",
                        "msg": f"Grid compute error: {e}"})

            elif cmd == "macro_calc":
                # Return live image count / storage estimate without starting
                try:
                    sess = _build_macro_session(msg)
                    is_grid = sess.scan_type == "grid_2d"
                    n_stacks = num_stacks_grid(sess) if is_grid else sess.num_stacks
                    travel_steps = abs(sess.rail_end_steps - sess.rail_start_steps)
                    travel_mm = travel_steps / STEPS_PER_MM
                    await websocket.send_json({
                        "type":               "macro_calc",
                        "scan_type":          sess.scan_type,
                        "frames_per_stack":   rail_frame_count(sess),
                        "total_stacks":       n_stacks,
                        "total_images":       total_image_count(sess),
                        "storage_gb":         round(estimated_storage_gb(sess), 2),
                        "travel_steps":       travel_steps,
                        "travel_mm":          round(travel_mm, 3),
                        "depth_per_image_um": round(depth_per_image_um(sess), 2),
                        "effective_pixel_um": round(effective_pixel_um(sess), 3),
                        # legacy key kept for old UI
                        "frames":             rail_frame_count(sess),
                    })
                except Exception as e:
                    await websocket.send_json({"type":"log","msg":f"Macro calc error: {e}"})

            elif cmd == "macro_start":
                logger.info(f"macro_start received: is_running={state['is_running']}")
                if state["is_running"]:
                    await websocket.send_json({"type":"log",
                        "msg":"⚠ Cannot start macro — another sequence is running."})
                else:
                    try:
                        logger.info("Building macro session...")
                        sess = _build_macro_session(msg)
                        logger.info(f"Session built: scan_type={sess.scan_type}, num_stacks={sess.num_stacks}")

                        # Calculate and set total frames for progress tracking
                        total_frames = total_image_count(sess)
                        state["total_frames"] = total_frames
                        state["current_frame"] = 0
                        logger.info(f"Macro: {sess.num_stacks} stacks × {sess.images_per_stack} images = {total_frames} total frames")

                        # Pause InertiaEngine before macro Bresenham stepping
                        if _inertia and _inertia._running:
                            logger.info("Stopping InertiaEngine for macro sequence...")
                            _inertia.stop()
                            # Wait for engine to fully stop (don't race with GPIO claims)
                            await asyncio.sleep(0.5)
                            logger.info("InertiaEngine stopped, GPIO pins ready for macro.")
                        state["is_running"] = True
                        save_session()
                        await broadcast({"type": "run_state", "running": True})

                        # Calculate and broadcast all planned stack positions for 3D visualization
                        try:
                            if sess.session_mode == "art":
                                # Art mode: use cinematic keyframes for motion path visualization
                                keyframe_data = []
                                for idx, kf in enumerate(_keyframes):
                                    keyframe_data.append({
                                        "index": idx,
                                        "pan_deg": float(getattr(kf, 'pan_deg', 0)),
                                        "tilt_deg": float(getattr(kf, 'tilt_deg', 0)),
                                    })
                                await broadcast({"type": "macro_art_keyframes", "keyframes": keyframe_data})
                            else:
                                # Scan mode: generate geodesic stack positions using the
                                # same function the engine executes — guaranteed to match.
                                # generate_scan_positions() calls orbit_positions() or
                                # grid_positions() internally, respecting rotation_mode,
                                # easing curves, snake order, and pan_axis_tilt_deg.
                                all_positions = generate_scan_positions(sess)
                                stack_positions = [
                                    {
                                        "stack":   i + 1,
                                        "pan_deg": p["pan"],
                                        "tilt_deg": p["tilt"],
                                        "eye":     p.get("eye", "mono"),
                                    }
                                    for i, p in enumerate(all_positions)
                                ]
                                await broadcast({"type": "macro_scan_start", "stack_positions": stack_positions})
                        except Exception as e:
                            logger.warning(f"Could not compute scan positions: {e}")

                        macro_eng = MacroEngine(
                            hardware        = hw,
                            capture_fn      = macro_capture,
                            apply_camera_fn = macro_apply_camera,
                            broadcast_fn    = broadcast,
                        )
                        # Sync engine position from live axis trackers.
                        # Use slider_axis.current_steps directly — it accumulates actual GPIO
                        # PWM pulses sent to the STEP pin (InertiaEngine: vs×50 steps/sec →
                        # int(delta_mm×50) per tick).  rail_start/end_steps are also stored from
                        # current_steps, so both are in the same unit and the delta is correct.
                        # DO NOT use current_mm × STEPS_PER_MM(800): current_mm is in belt-drive
                        # mm (50 steps/mm scale) so ×800 gives a value 16× too large.
                        macro_eng.rail_pos_steps = slider_axis.current_steps
                        macro_eng.pan_pos_deg    = pan_axis.current_deg
                        macro_eng.tilt_pos_deg   = tilt_axis.current_deg

                        start_mm   = sess.rail_start_steps / STEPS_PER_MM
                        end_mm     = sess.rail_end_steps   / STEPS_PER_MM
                        travel_steps = abs(sess.rail_end_steps - sess.rail_start_steps)
                        travel_mm    = travel_steps / STEPS_PER_MM

                        logger.info(f"Macro engine initial positions: rail={macro_eng.rail_pos_steps} steps "
                                   f"(≈{macro_eng.rail_pos_steps / STEPS_PER_MM:.2f}mm focus rail), "
                                   f"will move to {start_mm:.2f}mm to begin stack. "
                                   f"pan={macro_eng.pan_pos_deg:.1f}°, tilt={macro_eng.tilt_pos_deg:.1f}°")
                        logger.info(f"Rail scan range: {sess.rail_start_steps}→{sess.rail_end_steps} steps "
                                   f"(≈{start_mm:.1f}→{end_mm:.1f}mm, {travel_steps} steps / ≈{travel_mm:.1f}mm travel, "
                                   f"{sess.images_per_stack} images, {sess.step_increment_steps}-step increments)")
                        logger.info(f"Pan/Tilt initial positions: pan={pan_axis.current_deg:.1f}° tilt={tilt_axis.current_deg:.1f}°")

                        async def _macro_run():
                            try:
                                logger.info("Macro run started, calling macro_eng.run()...")
                                await macro_eng.run(sess)
                                logger.info("Macro run completed successfully")
                            except (FileNotFoundError, PermissionError, OSError) as e:
                                # Path/filesystem errors
                                logger.error(f"Macro file system error: {e}", exc_info=True)
                                error_msg = f"⚠ Save path error: {e}\nPlease check: USB SSD connected? Path valid? Write permission?"
                                await broadcast({"type": "log", "msg": error_msg})
                            except Exception as e:
                                logger.error(f"Macro run error: {e}", exc_info=True)
                                await broadcast({"type": "log", "msg": f"⚠ Macro error: {e}"})
                            finally:
                                state["is_running"] = False
                                # Sync axis positions back from engine after macro completes.
                                # Convert from macro's step tracking (800 steps/mm) back to mm.
                                slider_axis.current_mm    = macro_eng.rail_pos_steps / STEPS_PER_MM
                                slider_axis.current_steps = macro_eng.rail_pos_steps  # keep in sync
                                pan_axis.current_deg      = macro_eng.pan_pos_deg
                                tilt_axis.current_deg     = macro_eng.tilt_pos_deg
                                # Restore safe soft limits (was synced for macro)
                                slider_axis.soft_min = 0.0
                                slider_axis.soft_max = slider_axis.max_mm
                                save_session()
                                await broadcast({"type": "run_state", "running": False})
                                # Restart InertiaEngine for joystick/gamepad use
                                if _inertia:
                                    logger.info("Restarting InertiaEngine after macro sequence...")
                                    hw.stop_all_axes()  # Cleanly stop any lingering PWM
                                    _inertia.set_target(0, 0, 0)   # zero stale targets before restart
                                    _inertia.set_preset("responsive")
                                    if not _inertia._running:
                                        _inertia.start()

                        _macro_task = asyncio.create_task(_macro_run())
                        logger.info("Macro task created and scheduled")
                    except Exception as e:
                        state["is_running"] = False
                        await websocket.send_json({"type":"log","msg":f"Macro start error: {e}"})

            elif cmd == "macro_stop":
                if _macro_task and not _macro_task.done():
                    # Signal the engine — it will clean up and broadcast macro_done
                    state["stop_event"].set()
                    # Also flag via state for the worker loop
                    state["is_running"] = False
                    hw.stop_all_axes()
                    await broadcast({"type": "run_state", "running": False})
                    await broadcast({"type": "log", "msg": "Macro sequence stopped."})

            elif cmd == "macro_save_lens_profile":
                state["macro_lens_profile"] = msg.get("profile", {})
                save_session()
                await websocket.send_json({"type":"log","msg":"Lens profile saved."})

            elif cmd == "macro_load_lens_profiles":
                profiles = state.get("macro_saved_lens_profiles", {})
                await websocket.send_json({"type":"macro_lens_profiles","profiles":profiles})

            elif cmd == "macro_store_lens_profile":
                name    = msg.get("name","default").strip()
                profile = msg.get("profile", {})
                if name:
                    profs = state.get("macro_saved_lens_profiles", {})
                    profs[name] = profile
                    state["macro_saved_lens_profiles"] = profs
                    save_session()
                    await websocket.send_json({"type":"log","msg":f"Lens profile '{name}' stored."})
                    await websocket.send_json({"type":"macro_lens_profiles","profiles":profs})

            # ── CINEMATIC MODE ────────────────────────────────────────────────

            elif cmd == "cinematic_set_mode":
                _cinematic_mode = msg.get("value", "live")
                # When switching away from live, coast to stop and restore responsive preset
                if _cinematic_mode != "live" and _inertia:
                    _inertia.set_target(0, 0, 0)
                    _inertia.set_preset("responsive")

            elif cmd == "cinematic_set_rail_tilt":
                deg = float(msg.get("degrees", 0.0))
                state["rail_tilt_deg"] = deg
                _arctan.set_rail_tilt(deg)
                save_session()
                await websocket.send_json({"type": "log",
                    "msg": f"Rail tilt set to {deg:.1f}°"})

            elif cmd == "cinematic_set_high_power":
                enabled = bool(msg.get("enabled", False))
                state["high_power_mode"] = enabled
                # 16 = standard (~800mA), 24 = high power (~1.2A)
                current = 24 if enabled else 16
                for addr in (0, 1, 2):
                    hw.set_tmc_current(addr, run_current=current, hold_current=current//2)
                save_session()
                await websocket.send_json({"type": "log",
                    "msg": f"High power mode {'ON' if enabled else 'OFF'} "
                           f"({'1.2A' if enabled else '800mA'} run current)."})

            # ── SOFT LIMITS ───────────────────────────────────────────────────
            elif cmd == "cinematic_calibrate_limit":
                axis = msg.get("axis", "slider")   # slider | pan | tilt
                end  = msg.get("end",  "min")       # min | max
                ax_obj = {"slider": slider_axis, "pan": pan_axis, "tilt": tilt_axis}.get(axis)
                guard_ax = getattr(_soft_guard, axis, None)
                if ax_obj and guard_ax:
                    pos = ax_obj.current_mm if axis == "slider" else ax_obj.current_deg
                    # Record position to whichever end was pressed, then auto-sort:
                    # if both limits are set and min > max, swap them so the
                    # system always uses the smaller as min regardless of button order.
                    if end == "min":
                        guard_ax.set_min(pos)
                    else:
                        guard_ax.set_max(pos)

                    # Auto-sort: swap if both set and in wrong order
                    if guard_ax.min_unit is not None and guard_ax.max_unit is not None:
                        if guard_ax.min_unit > guard_ax.max_unit:
                            guard_ax.min_unit, guard_ax.max_unit = (
                                guard_ax.max_unit, guard_ax.min_unit)
                            logger.info(f"Soft limit auto-sort: {axis} min/max swapped")

                    state[f"{axis}_min"] = guard_ax.min_unit
                    state[f"{axis}_max"] = guard_ax.max_unit
                    save_session()
                    await broadcast({"type": "cinematic_limits",
                                     "limits": _soft_guard.status()})
                    lim_str = (f"min={guard_ax.min_unit:.1f}  max={guard_ax.max_unit:.1f}"
                               if guard_ax.cal_state == guard_ax.CAL_BOTH
                               else f"{end}={pos:.1f}")
                    await websocket.send_json({"type": "log",
                        "msg": f"Soft limit captured: {axis} {lim_str}"})

            elif cmd == "cinematic_clear_limit":
                axis = msg.get("axis", "slider")
                guard_ax = getattr(_soft_guard, axis, None)
                if guard_ax:
                    guard_ax.min_unit = None
                    guard_ax.max_unit = None
                    guard_ax._update_cal()
                    await broadcast({"type": "cinematic_limits",
                                     "limits": _soft_guard.status()})

            elif cmd == "cinematic_get_limits":
                await websocket.send_json({"type": "cinematic_limits",
                                           "limits": _soft_guard.status()})

            # ── LIVE / INERTIA ────────────────────────────────────────────────
            elif cmd == "cinematic_live_start":
                if not state["is_running"] and _inertia:
                    state["is_running"] = True
                    _cinematic_live_active = True
                    _cinematic_mode = "live"
                    # Apply user's chosen preset (or explicit mass/drag from UI)
                    preset = msg.get("preset", "light")
                    if preset in ("responsive", "light", "standard", "heavy", "tracking"):
                        _inertia.set_preset(preset)
                    _inertia.mass = float(msg.get("mass", _inertia.mass))
                    _inertia.drag = float(msg.get("drag", _inertia.drag))
                    if not _inertia._running:
                        _inertia.start()
                    # Zero any stale target so the motor doesn't lurch from a
                    # previous joystick position.  The user must push the stick
                    # intentionally to start motion.
                    _inertia.set_target(0, 0, 0)
                    _inertia.instant_stop_pt()
                    _inertia.instant_stop_slider()
                    hw.set_fan(80)   # boost fan during continuous motor movement
                    await broadcast({"type": "run_state", "running": True})
                    await broadcast({"type": "log",
                        "msg": f"Live cinematic mode — preset: {preset} (mass={_inertia.mass:.2f}, drag={_inertia.drag:.2f})"})

            elif cmd == "cinematic_live_stop":
                _cinematic_live_active = False
                if _inertia:
                    _inertia.instant_stop_pt()
                    _inertia.instant_stop_slider()
                    _inertia.set_target(0, 0, 0)          # clear stale target
                    _inertia.set_preset("responsive")      # back to responsive for positioning
                state["is_running"] = False
                hw.set_fan(60)   # back to cinematic idle fan
                save_session()
                await broadcast({"type": "run_state", "running": False})

            elif cmd == "cinematic_set_inertia":
                if _inertia:
                    preset = msg.get("preset")
                    if preset and preset in RIG_PRESETS:
                        # apply_pt_scale=True so preset defaults are restored,
                        # UNLESS the user explicitly passed a pt_sensitivity override.
                        _inertia.set_preset(preset)
                    else:
                        _inertia.set_params(
                            float(msg.get("mass", _inertia.mass)),
                            float(msg.get("drag", _inertia.drag))
                        )
                    await websocket.send_json({"type": "cinematic_inertia",
                        "mass": _inertia.mass, "drag": _inertia.drag,
                        "pan_tilt_scale": _inertia.pan_tilt_scale})

            elif cmd == "cinematic_set_pt_sensitivity":
                # Live pan/tilt speed scale — works in all modes, no preset change.
                # Useful for telephoto: 0.05 = ultra-fine (0.25 deg/s), 1.0 = full.
                if _inertia:
                    scale = float(msg.get("scale", 1.0))
                    _inertia.set_pt_sensitivity(scale)
                    await websocket.send_json({"type": "cinematic_inertia",
                        "mass": _inertia.mass, "drag": _inertia.drag,
                        "pan_tilt_scale": _inertia.pan_tilt_scale})

            elif cmd == "set_inversion":
                axis = msg.get("axis")   # slider | pan | tilt
                inv  = bool(msg.get("inverted", False))
                if axis in ("slider", "pan", "tilt"):
                    state[f"{axis}_inverted"] = inv
                    hw.set_inversions(
                        state["slider_inverted"],
                        state["pan_inverted"],
                        state["tilt_inverted"]
                    )
                    save_session()
                    await broadcast({"type": "inversions_updated", 
                                     "slider": state["slider_inverted"],
                                     "pan":    state["pan_inverted"],
                                     "tilt":   state["tilt_inverted"]})

            # ── ARCTAN TRACKER ────────────────────────────────────────────────
            elif cmd == "arctan_add_point":
                _arctan.add_point(
                    slider_mm = slider_axis.current_mm,
                    pan_deg   = pan_axis.current_deg,
                    tilt_deg  = tilt_axis.current_deg,
                )
                result = {
                    "type":     "arctan_status",
                    "points":   len(_arctan.points),
                    "solved":   _arctan.is_solved,
                    "residual": round(_arctan.residual_deg, 3),
                    "warning":  _arctan.warning,
                    "subject":  _arctan.subject.tolist() if _arctan.subject is not None else None,
                }
                await broadcast(result)
                await websocket.send_json({"type": "log",
                    "msg": f"Arctan: point {len(_arctan.points)} marked. "
                           + (f"Solved — RMS {_arctan.residual_deg:.2f}°" if _arctan.is_solved else "Need more points.")
                           + (f" ⚠ {_arctan.warning}" if _arctan.warning else "")})

            elif cmd == "arctan_clear":
                _arctan.clear_points()
                if _inertia:
                    _inertia.arctan_active = False
                await broadcast({"type": "arctan_status", "points": 0,
                                 "solved": False, "residual": 0, "warning": ""})

            elif cmd == "arctan_enable":
                enabled = bool(msg.get("enabled", False))
                if enabled and not _arctan.is_solved:
                    await websocket.send_json({"type": "log",
                        "msg": "⚠ Arctan lock needs calibration points first."})
                else:
                    if _inertia:
                        _inertia.arctan_active = enabled
                    if _prog_move:
                        _prog_move.tracker = _arctan if enabled else None
                    await broadcast({"type": "arctan_enabled", "enabled": enabled})
                    await websocket.send_json({"type": "log",
                        "msg": f"Arctan lock {'enabled' if enabled else 'disabled'}."})

            # ── PROGRAMMED MOVES ──────────────────────────────────────────────
            elif cmd == "cinematic_set_origin":
                if _prog_move:
                    # Compute delta: map keyframe[0] to the current physical position.
                    # This anchors the entire move to where the user is parked now,
                    # regardless of where the keyframes were originally designed.
                    # Works for both absolute keyframes (recorded in-place) and
                    # relative keyframes (designed from 0,0,0).
                    if _prog_move.keyframes:
                        kf0 = _prog_move.keyframes[0]
                        origin_s = slider_axis.current_mm - kf0.slider_mm
                        origin_p = pan_axis.current_deg   - kf0.pan_deg
                        origin_t = tilt_axis.current_deg  - kf0.tilt_deg
                    else:
                        origin_s = slider_axis.current_mm
                        origin_p = pan_axis.current_deg
                        origin_t = tilt_axis.current_deg
                    _prog_move.set_origin(origin_s, origin_p, origin_t)
                    state["cinematic_origin"] = {
                        "slider_mm": origin_s,
                        "pan_deg":   origin_p,
                        "tilt_deg":  origin_t,
                        # Also store the physical anchor point for display in UI
                        "anchor_slider_mm": slider_axis.current_mm,
                        "anchor_pan_deg":   pan_axis.current_deg,
                        "anchor_tilt_deg":  tilt_axis.current_deg,
                    }
                    # Invalidate timelapse trajectory cache — origin shift changed
                    if hasattr(timelapse_worker, '_traj_cache'):
                        del timelapse_worker._traj_cache
                    save_session()
                    await broadcast({"type": "cinematic_origin_set",
                                     "slider_mm": origin_s,
                                     "pan_deg":   origin_p,
                                     "tilt_deg":  origin_t,
                                     "anchor_slider_mm": slider_axis.current_mm,
                                     "anchor_pan_deg":   pan_axis.current_deg,
                                     "anchor_tilt_deg":  tilt_axis.current_deg})
                    await websocket.send_json({"type": "log",
                        "msg": f"Origin set — move anchored to current position "
                               f"(s={slider_axis.current_mm:.1f}mm "
                               f"pan={pan_axis.current_deg:.1f}° "
                               f"tilt={tilt_axis.current_deg:.1f}°)"})

            elif cmd == "clear_origin":
                if _prog_move:
                    _prog_move.set_origin(0.0, 0.0, 0.0)
                    if hasattr(timelapse_worker, '_traj_cache'):
                        del timelapse_worker._traj_cache
                state["cinematic_origin"] = {}
                save_session()
                await broadcast({"type": "cinematic_origin_set",
                                 "slider_mm": 0.0, "pan_deg": 0.0, "tilt_deg": 0.0})
                await broadcast({"type": "log",
                    "msg": "[✓] Origin cleared — motion path back to recorded positions."})

            elif cmd == "cinematic_save_reference_point":
                # Save the current physical position as the reproducible reference point.
                # Must be called AFTER cinematic_set_origin so the origin is known.
                if _prog_move:
                    if _prog_move.origin_slider == 0.0 and _prog_move.origin_pan == 0.0 \
                            and _prog_move.origin_tilt == 0.0 \
                            and not state.get("cinematic_origin"):
                        await websocket.send_json({"type": "log",
                            "msg": "⚠ Set origin at start first, then save a reference point."})
                    else:
                        _prog_move.save_reference_point(
                            slider_axis.current_mm,
                            pan_axis.current_deg,
                            tilt_axis.current_deg,
                        )
                        ref = {
                            "slider_mm": round(_prog_move.reference_slider, 2),
                            "pan_deg":   round(_prog_move.reference_pan, 2),
                            "tilt_deg":  round(_prog_move.reference_tilt, 2),
                            # Physical position for display
                            "phys_slider_mm": round(slider_axis.current_mm, 1),
                            "phys_pan_deg":   round(pan_axis.current_deg, 1),
                            "phys_tilt_deg":  round(tilt_axis.current_deg, 1),
                        }
                        state["cinematic_reference"] = ref
                        save_session()
                        await broadcast({"type": "cinematic_reference_saved", **ref})
                        await websocket.send_json({"type": "log",
                            "msg": f"Reference point saved "
                                   f"(s={slider_axis.current_mm:.1f}mm "
                                   f"pan={pan_axis.current_deg:.1f}° "
                                   f"tilt={tilt_axis.current_deg:.1f}°). "
                                   f"Load this move later, jog to this spot, then tap "
                                   f"'I'm at Reference'."})

            elif cmd == "cinematic_at_reference":
                # User is physically at the saved reference point. Compute origin from it
                # so keyframe[0] can be found precisely without manual positioning at start.
                if _prog_move:
                    ok = _prog_move.apply_reference_point(
                        slider_axis.current_mm,
                        pan_axis.current_deg,
                        tilt_axis.current_deg,
                    )
                    if ok:
                        origin_s = _prog_move.origin_slider
                        origin_p = _prog_move.origin_pan
                        origin_t = _prog_move.origin_tilt
                        state["cinematic_origin"] = {
                            "slider_mm": origin_s, "pan_deg": origin_p, "tilt_deg": origin_t,
                            "anchor_slider_mm": slider_axis.current_mm,
                            "anchor_pan_deg":   pan_axis.current_deg,
                            "anchor_tilt_deg":  tilt_axis.current_deg,
                        }
                        if hasattr(timelapse_worker, '_traj_cache'):
                            del timelapse_worker._traj_cache
                        save_session()
                        await broadcast({"type": "cinematic_origin_set",
                                         "slider_mm": origin_s,
                                         "pan_deg":   origin_p,
                                         "tilt_deg":  origin_t,
                                         "anchor_slider_mm": slider_axis.current_mm,
                                         "anchor_pan_deg":   pan_axis.current_deg,
                                         "anchor_tilt_deg":  tilt_axis.current_deg})
                        await websocket.send_json({"type": "log",
                            "msg": f"[✓] Aligned from reference — "
                                   f"keyframe[0] is now at "
                                   f"s={origin_s + (_prog_move.keyframes[0].slider_mm if _prog_move.keyframes else 0):.1f}mm "
                                   f"pan={origin_p + (_prog_move.keyframes[0].pan_deg if _prog_move.keyframes else 0):.1f}°. "
                                   f"Use ↩ Return to Start to move there."})
                    else:
                        await websocket.send_json({"type": "log",
                            "msg": "⚠ No reference point saved for this move. "
                                   "Save one first while at your reference spot."})

            elif cmd == "cinematic_add_keyframe":
                if _prog_move:
                    idx = _prog_move.add_keyframe(
                        slider_mm  = float(msg.get("slider_mm", slider_axis.current_mm)),
                        pan_deg    = float(msg.get("pan_deg",   pan_axis.current_deg)),
                        tilt_deg   = float(msg.get("tilt_deg",  tilt_axis.current_deg)),
                        duration_s = float(msg.get("duration_s", 3.0)),
                        easing     = msg.get("easing", "gaussian"),
                    )
                    sync_all_keyframes()
                    await broadcast_points()
                    await websocket.send_json({"type": "log",
                        "msg": f"Keyframe {idx+1} added (synced to all modes)."})

            elif cmd == "cinematic_update_keyframe":
                if _prog_move:
                    idx = int(msg.get("index", 0))
                    kwargs = {k: msg[k] for k in
                              ("slider_mm","pan_deg","tilt_deg","duration_s","easing")
                              if k in msg}
                    _prog_move.update_keyframe(idx, **kwargs)
                    sync_all_keyframes()
                    await broadcast_points()

            elif cmd == "cinematic_remove_keyframe":
                if _prog_move:
                    _prog_move.remove_keyframe(int(msg.get("index", 0)))
                    sync_all_keyframes()
                    await broadcast_points()

            elif cmd == "cinematic_reverse_move":
                if _prog_move and len(_prog_move.keyframes) >= 2:
                    _prog_move.reverse_keyframes()
                    state["_move_reversed"] = not state.get("_move_reversed", False)
                    sync_all_keyframes()
                    await broadcast_points()
                    await broadcast({"type": "cinematic_status",
                                     "msg": f"Move {'reversed' if state['_move_reversed'] else 'restored to original direction'}."})

            elif cmd == "cinematic_clear_keyframes":
                if _prog_move:
                    _prog_move.clear_keyframes()
                    _prog_move.set_origin(0.0, 0.0, 0.0)   # reset stale deployment offset
                state["cinematic_origin"] = {}
                state["_move_reversed"] = False
                sync_all_keyframes()
                await broadcast_points()

            elif cmd == "cinematic_set_preroll":
                if _prog_move:
                    _prog_move.preroll_s = float(msg.get("seconds", 3.0))

            elif cmd == "cinematic_set_loop":
                if _prog_move:
                    _prog_move.loop = bool(msg.get("enabled", False))

            elif cmd == "cinematic_play":
                global _prog_task
                if not state["is_running"] and _prog_move and len(_prog_move.keyframes) >= 2:
                    # ── Pre-flight: check trajectory against soft limits ───────
                    n_check = max(2, int(
                        sum(max(0.1, kf.duration_s) for kf in _prog_move.keyframes[:-1]) * 60))
                    _ts, _tp, _tt = _prog_move.generate_unified_trajectory(
                        n_check,
                        _prog_move.origin_slider, _prog_move.origin_pan, _prog_move.origin_tilt,
                        for_timelapse=False)
                    violations = _check_path_vs_limits(_ts, _tp, _tt)
                    if violations:
                        _pending_play = {"context": "cinematic"}
                        await websocket.send_json({
                            "type": "path_limit_warning",
                            "context": "cinematic",
                            "violations": violations,
                        })
                        return   # wait for user to confirm or cancel

                    # ── All clear — launch the move ───────────────────────────
                    # Stop InertiaEngine — TrajectoryPlayer and InertiaEngine
                    # both call set_axis_speed and would fight over the same pins.
                    if _inertia and _inertia._running:
                        _inertia.stop()
                    state["is_running"] = True
                    _cinematic_mode = "programmed"
                    await broadcast({"type": "run_state", "running": True})

                    # Stop liveview — not needed during a programmed move and
                    # gphoto2/WiFi stream threads compete with the motor loop.
                    global _cinematic_paused_liveview, _sony_liveview_running
                    _cinematic_paused_liveview = False
                    if _sony_usb_liveview_running:
                        _stop_sony_usb_liveview()
                        _cinematic_paused_liveview = True
                    if _sony_liveview_running:
                        _sony_liveview_running = False
                        _cinematic_paused_liveview = True

                    async def _play_wrapper():
                        try:
                            await _prog_move.play()
                        finally:
                            state["is_running"] = False
                            save_session()
                            await broadcast({"type": "run_state", "running": False})
                            # Restart liveview now the move is done
                            _restart_cinematic_liveview()
                            # Restart InertiaEngine for immediate joystick use after move
                            if _inertia:
                                _inertia.set_target(0, 0, 0)
                                _inertia.set_preset("responsive")
                                if not _inertia._running:
                                    _inertia.start()

                    _prog_task = asyncio.create_task(_play_wrapper())
                else:
                    await websocket.send_json({"type": "log",
                        "msg": "⚠ Need at least 2 keyframes and system must be idle."})

            elif cmd == "cinematic_stop":
                if _prog_move:
                    _prog_move.stop()
                state["is_running"] = False
                hw.stop_all_axes()
                hw.enable_motors(True)   # ensure motors re-enabled after any _move_to
                save_session()
                await broadcast({"type": "run_state", "running": False})
                # Restart InertiaEngine so joystick/Live mode works immediately
                if _inertia:
                    _inertia.set_target(0, 0, 0)
                    _inertia.set_preset("responsive")
                    if not _inertia._running:
                        _inertia.start()

            # ── PATH LIMIT WARNING RESPONSES ──────────────────────────────────
            elif cmd == "path_expand_and_play":
                # User confirmed: widen soft limits to fit the path, then launch.
                if not _pending_play or not _prog_move:
                    return
                context = _pending_play.get("context", "cinematic")

                # Recompute trajectory to get bounds
                if context == "cinematic":
                    n_check = max(2, int(
                        sum(max(0.1, kf.duration_s) for kf in _prog_move.keyframes[:-1]) * 60))
                    _ts, _tp, _tt = _prog_move.generate_unified_trajectory(
                        n_check,
                        _prog_move.origin_slider, _prog_move.origin_pan, _prog_move.origin_tilt,
                        for_timelapse=False)
                else:
                    _ts, _tp, _tt = _prog_move.generate_unified_trajectory(
                        state["total_frames"],
                        _prog_move.origin_slider, _prog_move.origin_pan, _prog_move.origin_tilt,
                        for_timelapse=True)

                _expand_limits_to_fit(_ts, _tp, _tt)
                save_session()
                await broadcast({"type": "cinematic_limits", "limits": _soft_guard.status()})
                await websocket.send_json({"type": "log",
                    "msg": "[✓] Soft limits expanded to fit motion path."})

                pending_ctx = _pending_play.copy()
                _pending_play.clear()

                if context == "cinematic" and not state["is_running"] and len(_prog_move.keyframes) >= 2:
                    if _inertia and _inertia._running:
                        _inertia.stop()
                    state["is_running"] = True
                    _cinematic_mode = "programmed"
                    await broadcast({"type": "run_state", "running": True})

                    async def _play_wrapper_expanded():
                        try:
                            await _prog_move.play()
                        finally:
                            state["is_running"] = False
                            save_session()
                            await broadcast({"type": "run_state", "running": False})
                            if _inertia:
                                _inertia.set_target(0, 0, 0)
                                _inertia.set_preset("responsive")
                                if not _inertia._running:
                                    _inertia.start()

                    asyncio.create_task(_play_wrapper_expanded())

                elif context == "timelapse":
                    save_session()
                    asyncio.create_task(timelapse_worker(pending_ctx.get("base_iv", 5.0)))

            elif cmd == "path_cancel_play":
                _pending_play.clear()
                await websocket.send_json({"type": "log",
                    "msg": "Motion path check cancelled — adjust keyframes or soft limits and try again."})

            elif cmd == "cinematic_return_to_start":
                if _prog_move and not state["is_running"]:
                    # Stop InertiaEngine — _move_to uses set_tmc_velocity / set_axis_speed
                    # on the same STEP pins; they must not run concurrently.
                    if _inertia and _inertia._running:
                        _inertia.stop()
                    state["is_running"] = True
                    await broadcast({"type": "run_state", "running": True})
                    async def _return():
                        try:
                            await _prog_move.return_to_start()
                        finally:
                            state["is_running"] = False
                            hw.enable_motors(True)   # _move_to leaves EN pin in unknown state
                            await broadcast({"type": "run_state", "running": False})
                            # Restart InertiaEngine for immediate joystick use
                            if _inertia:
                                _inertia.set_target(0, 0, 0)
                                _inertia.set_preset("responsive")
                                if not _inertia._running:
                                    _inertia.start()
                    asyncio.create_task(_return())

            # ── MOVE LIBRARY ──────────────────────────────────────────────────
            elif cmd == "cinematic_save_move":
                if _prog_move and _prog_move.keyframes:
                    name  = msg.get("name", "").strip()
                    notes = msg.get("notes", "").strip()
                    if not name:
                        await websocket.send_json({"type":"log","msg":"Enter a name for the move."})
                    else:
                        try:
                            # Include the reference point if one has been saved
                            ref = None
                            if _prog_move.reference_slider is not None:
                                ref = {
                                    "slider_mm": round(_prog_move.reference_slider, 3),
                                    "pan_deg":   round(_prog_move.reference_pan,    3),
                                    "tilt_deg":  round(_prog_move.reference_tilt,   3),
                                }
                            _move_library.save_move(name, _prog_move.keyframes, notes,
                                                    reference=ref)
                            ref_note = " + reference point" if ref else ""
                            # Broadcast to all clients so the Move Library updates everywhere
                            await broadcast({"type": "cinematic_moves",
                                "moves": _move_library.list_moves()})
                            await broadcast({"type": "log",
                                "msg": f"[✓] Move '{name}' saved "
                                       f"({len(_prog_move.keyframes)} keyframes{ref_note})."})
                        except Exception as e:
                            await websocket.send_json({"type":"log","msg":f"Save failed: {e}"})
                else:
                    await websocket.send_json({"type":"log","msg":"No keyframes to save."})

            elif cmd == "cinematic_load_move":
                name = msg.get("name","")
                try:
                    kfs, ref = _move_library.load_move(name)
                    if _prog_move:
                        _prog_move.clear_keyframes()
                        _prog_move.set_origin(0.0, 0.0, 0.0)
                        # Restore reference point if the move has one
                        if ref:
                            _prog_move.reference_slider = ref["slider_mm"]
                            _prog_move.reference_pan    = ref["pan_deg"]
                            _prog_move.reference_tilt   = ref["tilt_deg"]
                        else:
                            _prog_move.reference_slider = None
                            _prog_move.reference_pan    = None
                            _prog_move.reference_tilt   = None
                        for kf in kfs:
                            _prog_move.keyframes.append(kf)
                    state["cinematic_origin"]    = {}
                    state["cinematic_reference"] = ref or {}
                    sync_all_keyframes()
                    if hasattr(timelapse_worker, '_traj_cache'):
                        del timelapse_worker._traj_cache
                    await broadcast({"type": "cinematic_keyframes",
                                     "keyframes": _keyframes_to_list()})
                    await broadcast({"type": "cinematic_origin_set",
                                     "slider_mm": 0.0, "pan_deg": 0.0, "tilt_deg": 0.0})
                    # Tell UI whether this move has a reference point
                    if ref:
                        await broadcast({"type": "cinematic_reference_saved",
                                         "slider_mm":      ref["slider_mm"],
                                         "pan_deg":        ref["pan_deg"],
                                         "tilt_deg":       ref["tilt_deg"],
                                         "phys_slider_mm": None,
                                         "phys_pan_deg":   None,
                                         "phys_tilt_deg":  None})
                    else:
                        await broadcast({"type": "cinematic_reference_cleared"})
                    await broadcast({"type": "cinematic_moves",
                                     "moves": _move_library.list_moves()})
                    ref_hint = (
                        " Jog to your reference spot and tap 'I'm at Reference → Auto-Align'."
                        if ref else
                        " Zero hardware at home position, then use ↩ Return to Start."
                    )
                    await broadcast({"type": "log",
                        "msg": f"[✓] Move '{name}' loaded ({len(kfs)} keyframes)."
                               + ref_hint})
                except KeyError as e:
                    await websocket.send_json({"type":"log","msg":str(e)})

            elif cmd == "cinematic_delete_move":
                name = msg.get("name","")
                _move_library.delete_move(name)
                await websocket.send_json({"type": "cinematic_moves",
                    "moves": _move_library.list_moves()})
                await websocket.send_json({"type":"log","msg":f"Move '{name}' deleted."})

            elif cmd == "cinematic_rename_move":
                old = msg.get("old_name",""); new = msg.get("new_name","").strip()
                if old and new:
                    try:
                        _move_library.rename_move(old, new)
                        await websocket.send_json({"type": "cinematic_moves",
                            "moves": _move_library.list_moves()})
                    except KeyError as e:
                        await websocket.send_json({"type":"log","msg":str(e)})

            elif cmd == "cinematic_list_moves":
                await websocket.send_json({"type": "cinematic_moves",
                    "moves": _move_library.list_moves()})

            # ── VIDEO RECORDING ───────────────────────────────────────────────
            elif cmd == "record_start":
                if _recording:
                    await websocket.send_json({"type":"log","msg":"Already recording."})
                else:
                    cam = state.get("active_camera","picam")
                    ts   = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                    dest = os.path.join(state.get("save_path", str(Path.home() / "Pictures/PiSlider")),
                                        f"CINE_{ts}.mp4")
                    os.makedirs(os.path.dirname(dest), exist_ok=True)

                    if cam == "picam":
                        fps = int(state.get("cine_fps", 24))
                        ok = await asyncio.to_thread(_start_picam_video, dest, fps)
                        if ok:
                            status_leds.set_recording(True)
                            await broadcast({"type": "record_state", "recording": True,
                                             "path": dest})
                            await websocket.send_json({"type":"log",
                                "msg":f"● Recording: {os.path.basename(dest)}"})
                        else:
                            await websocket.send_json({"type":"log",
                                "msg":"⚠ Camera recording failed — check logs."})
                    elif cam == "sony":
                        ok, msg_txt = await _toggle_sony_record(start=True)
                        if ok:
                            _recording = True
                            _record_start_time = time.time()
                            status_leds.set_recording(True)
                            await broadcast({"type": "record_state", "recording": True,
                                             "path": "Sony camera"})
                        else:
                            if msg_txt == "MOVIE_MODE_REQUIRED":
                                user_msg = (
                                    "⚠ Sony camera is not in Movie mode. "
                                    "Turn the mode dial to 🎬 Movie on the camera, then try again."
                                )
                            else:
                                user_msg = f"⚠ Sony record failed: {msg_txt}"
                            await websocket.send_json({"type": "log", "msg": user_msg})
                            await broadcast({"type": "sony_record_error", "msg": user_msg})

            elif cmd == "record_stop":
                if not _recording:
                    await websocket.send_json({"type":"log","msg":"Not recording."})
                else:
                    cam = state.get("active_camera","picam")
                    saved_path = _video_output_path  # capture before _stop clears it
                    elapsed = time.time() - (_record_start_time or time.time())
                    if cam == "picam":
                        await asyncio.to_thread(_stop_picam_video)
                    elif cam == "sony":
                        ok, msg_txt = await _toggle_sony_record(start=False)
                        if not ok:
                            await websocket.send_json({"type":"log",
                                "msg":f"⚠ Sony record stop failed: {msg_txt}"})
                        _recording = False
                    status_leds.set_recording(False)
                    await broadcast({"type": "record_state", "recording": False})
                    # Report file size so user knows it was actually saved
                    size_msg = ""
                    if saved_path and os.path.exists(saved_path):
                        size_mb = os.path.getsize(saved_path) / 1_048_576
                        size_msg = f" ({size_mb:.1f}MB)"
                    elif saved_path:
                        size_msg = " ⚠ file not found — is ffmpeg installed?"
                    await websocket.send_json({"type":"log",
                        "msg":f"■ Recording stopped. {elapsed:.1f}s{size_msg}"})

            elif cmd == "cinematic_get_state":
                # Full cinematic state restore on reconnect
                min_dur, min_axis = (0.0, "") if not _prog_move else _prog_move.compute_min_duration()
                await websocket.send_json({
                    "type":      "cinematic_state",
                    "limits":    _soft_guard.status(),
                    "keyframes": _keyframes_to_list(),
                    "moves":     _move_library.list_moves(),
                    "arctan":    {
                        "points":   len(_arctan.points),
                        "solved":   _arctan.is_solved,
                        "residual": round(_arctan.residual_deg, 3),
                        "warning":  _arctan.warning,
                    },
                    "inertia": {
                        "mass":          _inertia.mass           if _inertia else 0.4,
                        "drag":          _inertia.drag           if _inertia else 0.55,
                        "pan_tilt_scale": _inertia.pan_tilt_scale if _inertia else 1.0,
                    },
                    "recording":     _recording,
                    "rail_tilt":     state.get("rail_tilt_deg", 0.0),
                    "high_power":    state.get("high_power_mode", False),
                    "origin":        state.get("cinematic_origin", {}),
                    "reference":     state.get("cinematic_reference", {}),
                    # Path planning
                    "path_mode":       _prog_move.path_mode        if _prog_move else "linear",
                    "global_easing":   _prog_move.global_easing    if _prog_move else "cycloid",
                    "catmull_tension": _prog_move.catmull_tension  if _prog_move else 0.5,
                    "total_duration_s": _prog_move.total_duration_s() if _prog_move else 0.0,
                    "min_duration":    min_dur,
                    "min_duration_axis": min_axis,
                })

            elif cmd == "cinematic_set_path_mode":
                # "linear" | "catmull_rom"
                mode = str(msg.get("mode", "linear"))
                if _prog_move:
                    _prog_move.set_path_mode(mode)
                    if hasattr(timelapse_worker, '_traj_cache'):
                        del timelapse_worker._traj_cache
                await broadcast({"type": "log",
                                  "msg": f"Path mode → {mode}"})

            elif cmd == "cinematic_set_tension":
                tension = float(msg.get("tension", 0.5))
                if _prog_move:
                    _prog_move.set_catmull_tension(tension)
                    if hasattr(timelapse_worker, '_traj_cache'):
                        del timelapse_worker._traj_cache
                await broadcast({"type": "log",
                                  "msg": f"Catmull-Rom tension → {tension:.2f}"})

            elif cmd == "cinematic_set_global_easing":
                curve = str(msg.get("curve", "cycloid"))
                if _prog_move:
                    _prog_move.set_global_easing(curve)
                    if hasattr(timelapse_worker, '_traj_cache'):
                        del timelapse_worker._traj_cache
                await broadcast({"type": "cinematic_global_easing",
                                  "curve": curve})
                await broadcast({"type": "log",
                                  "msg": f"Global easing → {curve}"})

            elif cmd == "cinematic_apply_global_easing":
                # Reset every segment's easing to the current global curve
                if _prog_move:
                    _prog_move.apply_global_easing_to_all()
                    if hasattr(timelapse_worker, '_traj_cache'):
                        del timelapse_worker._traj_cache
                await broadcast({"type": "cinematic_keyframes",
                                  "keyframes": _keyframes_to_list()})
                await broadcast({"type": "log",
                                  "msg": "Applied global easing to all segments."})

            elif cmd == "cinematic_set_segment_pct":
                # Set one segment's % of total; the next segment absorbs the difference.
                # Last segment is read-only (auto-fills as the remainder).
                seg_idx = int(msg.get("index", 0))
                pct     = float(msg.get("pct", 50.0))
                if _prog_move:
                    _prog_move.set_segment_pct(seg_idx, pct)
                    if hasattr(timelapse_worker, '_traj_cache'):
                        del timelapse_worker._traj_cache
                    await broadcast({"type": "cinematic_keyframes",
                                      "keyframes": _keyframes_to_list()})

            elif cmd == "cinematic_scale_duration":
                # Rescale all segment durations proportionally to reach new_total_s.
                # Per-segment proportions are preserved — only the total changes.
                new_total = float(msg.get("total_s", 10.0))
                if _prog_move and new_total > 0.0:
                    actual = _prog_move.scale_total_duration(new_total)
                    if hasattr(timelapse_worker, '_traj_cache'):
                        del timelapse_worker._traj_cache
                    await broadcast({"type": "cinematic_keyframes",
                                      "keyframes": _keyframes_to_list()})
                    await broadcast({"type": "log",
                                      "msg": f"Scaled total duration to {actual:.1f}s "
                                             f"(proportions preserved)."})

            # ── TELEMETRY TICK ────────────────────────────────────────────────
            try:
                await websocket.send_json({
                    "type":  "status",
                    "frame": state["current_frame"],
                    "total": state["total_frames"],
                    "pos_s": slider_axis.current_mm,
                    "pos_p": pan_axis.current_deg,
                    "pos_t": tilt_axis.current_deg,
                })
            except Exception as _tel_err:
                logger.warning(f"WS telemetry send failed [{cmd}]: {_tel_err}")
                break
            await asyncio.sleep(0.05)

    except WebSocketDisconnect:
        logger.info("WS client disconnected (browser closed).")
    except Exception as _ws_err:
        logger.error(f"WS handler crashed: {_ws_err}", exc_info=True)
    finally:
        connected_clients.discard(websocket)
        # Only clear _active_ws if it's still pointing at THIS socket.
        # If a new client already took over (kicked us), don't clobber their reference.
        if _active_ws is websocket:
            _active_ws = None
            # Do NOT stop any running sequence on disconnect.
            # Timelapse, macro, programmed cinematic moves, and live InertiaEngine
            # all continue in the background — the gamepad can still drive live
            # cinematic control even without a browser open.
            # The joystick watchdog handles the "no input at all" safety case:
            # it zeros VACTUAL after 1s of silence from both browser and gamepad.
            if not state.get("is_running", False):
                # Zero the UI axis cache so stale joystick/slider targets can't
                # restart motors when the browser reconnects (e.g. after a page
                # reload). Without this, _merged_targets() returns the last
                # held-stick value, and InertiaEngine immediately drives toward it.
                for k in _ui_axes:
                    _ui_axes[k] = 0.0
                if _inertia:
                    _inertia.set_target(0, 0, 0)
                    _inertia.clear_nudge_pt()
                    _inertia.clear_nudge_slider()
                for k in _nudge_source:    _nudge_source[k]    = None
                for k in _nudge_heartbeat: _nudge_heartbeat[k] = 0.0
                hw.stop_all_axes()  # VACTUAL=0, motors hold position
                logger.info("WS client disconnected — no active sequence, motors holding.")
            else:
                logger.info("WS client disconnected — sequence/live control running in background.")




# ─── GRAPH WEBSOCKET (read-only, never kicked) ────────────────────────────────
@app.websocket("/ws-graph")
async def websocket_graph(websocket: WebSocket):
    """
    Read-only WebSocket for the session graph tab.
    Receives all broadcasts (status, log, run_state, etc.) but:
    - never triggers the single-instance kick logic
    - never accepts control commands
    Multiple graph tabs can be open simultaneously alongside the main UI.
    """
    await websocket.accept()
    _graph_clients.add(websocket)
    # Send current run state so the graph tab can bust its thumb cache and
    # show the correct progress immediately on (re)connect.
    await websocket.send_json({
        "type":          "init",
        "running":       state["is_running"],
        "current_frame": state["current_frame"],
        "total_frames":  state["total_frames"],
        "save_path":     state["save_path"],
        "run_id":        _timelapse_run_id,
    })
    # Send full history so the graph populates immediately without waiting
    # for the 400ms HTTP fallback — critical when opening during a live sequence.
    if _session_history:
        try:
            await websocket.send_json({
                "type":   "history_replay",
                "frames": list(_session_history),
            })
        except Exception:
            pass
    try:
        while True:
            # Drain incoming messages (graph tab doesn't send commands, but
            # we need to receive to detect disconnects)
            await websocket.receive_text()
    except Exception:
        pass
    finally:
        _graph_clients.discard(websocket)

# ─── STATIC & ROOT ────────────────────────────────────────────────────────────
app.mount("/static", StaticFiles(directory="web"), name="static")
app.mount("/night_thumbs", StaticFiles(directory="night thumbs"), name="night_thumbs")



@app.get("/thumbs/{frame_id}")
async def get_thumb(frame_id: str):
    """Serve a sequence thumbnail for the graph timelapse player."""
    from fastapi.responses import FileResponse, Response
    thumb_path = os.path.join(state["save_path"], "thumbs", f"THUMB_{frame_id}.jpg")
    if os.path.exists(thumb_path):
        # Cache real thumbs for 1 year — they never change once written
        return FileResponse(thumb_path, media_type="image/jpeg",
                            headers={"Cache-Control": "public, max-age=31536000"})
    # Thumb not found — return 404 so browser doesn't cache the miss
    from fastapi import HTTPException
    raise HTTPException(status_code=404, detail="Thumb not ready yet")



@app.get("/api/thumb-list")
async def thumb_list():
    """Return list of frame IDs that have saved thumbnails, for the graph strip.

    Only returns IDs that belong to the *current* session (i.e. appear in
    _session_history) and exist on disk.  This prevents thumbnails from a
    previous run in the same save folder from leaking into a new session.
    """
    from fastapi.responses import JSONResponse
    thumb_dir = os.path.join(state["save_path"], "thumbs")
    # Collect frame_ids that are part of the current run
    session_ids = {
        f.get("frame_id", "")
        for f in _session_history
        if f.get("frame_id")
    }
    if not os.path.isdir(thumb_dir) or not session_ids:
        return JSONResponse({"run_id": _timelapse_run_id, "frame_ids": []})
    # Only return IDs that are both in this session AND written to disk
    ids = sorted([
        fid for fid in session_ids
        if os.path.exists(os.path.join(thumb_dir, f"THUMB_{fid}.jpg"))
    ])
    return JSONResponse({"run_id": _timelapse_run_id, "frame_ids": ids})

@app.get("/graph")
async def graph_page():
    with open("web/graph.html") as f:
        return HTMLResponse(content=f.read())


@app.get("/macro_img")
async def serve_macro_img(p: str):
    """Serve a macro JPEG preview image by absolute path (must be within user home dir)."""
    import pathlib
    home = str(pathlib.Path.home())
    full = os.path.normpath(p)
    if not full.startswith(home):
        raise HTTPException(status_code=403, detail="Path outside home")
    if not os.path.isfile(full):
        raise HTTPException(status_code=404, detail="File not found")
    return FileResponse(full, media_type="image/jpeg")

@app.get("/macro_graph")
async def macro_graph_page():
    with open("web/macro-graph.html") as f:
        return HTMLResponse(content=f.read())


@app.get("/api/session-history")
async def session_history():
    """Return buffered frame history for graph tab initial load / reconnect."""
    from fastapi.responses import JSONResponse
    return JSONResponse({"frames": list(_session_history)})


@app.get("/")
async def index():
    with open("web/index.html") as f:
        return HTMLResponse(content=f.read())


# ─── CINEMATIC / VIDEO RECORDING ─────────────────────────────────────────────

def _start_picam_video(output_path: str, fps: int = 24) -> bool:
    """
    Record to a raw .h264 file using FileOutput (no ffmpeg quoting issues),
    then remux to .mp4 on stop using subprocess with proper arg list.
    This avoids FfmpegOutput's internal shell command breaking on spaces in paths.
    """
    global _recording, _record_start_time, _video_output_path
    if not _HAS_PICAM or not picam:
        return False
    try:
        from picamera2.encoders import H264Encoder
        from picamera2.outputs import FileOutput

        bitrate = {24: 20_000_000, 25: 20_000_000,
                   30: 18_000_000, 60: 12_000_000}.get(fps, 18_000_000)

        encoder = H264Encoder(bitrate=bitrate)

        # Write raw H264 — FileOutput takes a plain file path, no shell involved
        h264_path = output_path.replace('.mp4', '.h264')
        output = FileOutput(h264_path)
        picam.start_encoder(encoder, output)

        _recording         = True
        _record_start_time = time.time()
        _video_output_path = output_path   # final .mp4 destination
        logger.info(f"Recording started (raw H264): {h264_path} @ {bitrate//1_000_000}Mbps")
        return True
    except Exception as e:
        logger.error(f"PiCam video start: {e}")
        _recording         = False
        _record_start_time = None
        return False


def _stop_picam_video():
    """Stop encoder, remux raw H264 → MP4, verify output file."""
    global _recording, _record_start_time, _video_output_path
    if not _HAS_PICAM or not picam:
        return
    mp4_path  = _video_output_path
    h264_path = mp4_path.replace('.mp4', '.h264') if mp4_path else None

    try:
        picam.stop_encoder()
        time.sleep(0.3)   # let encoder flush final NAL units
    except Exception as e:
        logger.warning(f"PiCam stop encoder: {e}")

    _recording         = False
    _record_start_time = None
    _video_output_path = None

    if not h264_path or not os.path.exists(h264_path):
        logger.error(f"H264 source not found: {h264_path}")
        return

    h264_size = os.path.getsize(h264_path)
    logger.info(f"H264 raw file: {h264_path} ({h264_size//1024}KB) — remuxing to MP4…")

    if h264_size < 1000:
        logger.warning("H264 file suspiciously small — encoder may have failed.")
        return

    # Remux using subprocess arg list — no shell, no quoting issues with spaces
    try:
        result = subprocess.run(
            ["ffmpeg", "-y",
             "-framerate", "25",        # container framerate hint
             "-i", h264_path,
             "-c:v", "copy",            # stream copy — instant, lossless
             mp4_path],
            capture_output=True, text=True, timeout=60
        )
        if result.returncode == 0 and os.path.exists(mp4_path):
            mp4_size = os.path.getsize(mp4_path)
            logger.info(f"MP4 saved: {mp4_path} ({mp4_size//1024}KB)")
            os.remove(h264_path)        # clean up raw file
        else:
            logger.error(f"ffmpeg remux failed (rc={result.returncode}): {result.stderr[-300:]}")
            logger.info(f"Raw H264 kept at: {h264_path}")
    except FileNotFoundError:
        logger.error("ffmpeg not found — install with: sudo apt install ffmpeg")
        logger.info(f"Raw H264 available at: {h264_path}")
    except subprocess.TimeoutExpired:
        logger.error("ffmpeg remux timed out")


async def _record_and_run(msg: dict, websocket):
    """Start camera recording, hold pre-roll, then begin motion (timelapse or cinematic).

    Flow:
      1. Start recording (Sony WiFi: startMovieRec; others: log only — user arms manually)
      2. Broadcast countdown during pre-roll
      3. Kick off the programmed motion (start_run or cinematic play)
      4. On completion stop recording and write motion sidecar named after the clip
    """
    mode        = msg.get("mode", state.get("active_mode", "timelapse"))
    preroll_s   = float(msg.get("preroll_s", 0.0))
    flash_sync  = bool(msg.get("flash_sync", False))
    camera      = state.get("active_camera", "picam")

    # ── Apply run parameters from message (mirrors start_run handler) ────────
    if mode != "cinematic":
        state["total_frames"]    = int(  msg.get("total_frames",  state["total_frames"]))
        state["vibe_delay"]      = float(msg.get("vibe_delay",    state["vibe_delay"]))
        state["exp_margin"]      = float(msg.get("exp_margin",    state["exp_margin"]))
        state["save_path"]       = msg.get("save_path",           state["save_path"])
        state["trigger_mode"]    = msg.get("trigger_mode",        state["trigger_mode"])
        state["manual_interval"] = float(msg.get("interval",      state.get("manual_interval", 5.0)))
        state["tl_preroll_s"]    = preroll_s
        save_session()
    else:
        # Cinematic: guard — need at least 2 keyframes
        if _prog_move is None or len(_prog_move.keyframes) < 2:
            await broadcast({"type": "log",
                             "msg": "⚠ Record + Run: no cinematic keyframes set"})
            return

        # Move to start position before starting the camera recording
        await broadcast({"type": "log", "msg": "Moving to start position…"})
        await _prog_move.return_to_start()

    # ── Start camera recording ───────────────────────────────────────────────
    recording_started = False
    if camera == "sony":
        ok, err = await _toggle_sony_record(start=True)
        if ok:
            recording_started = True
            await broadcast({"type": "log", "msg": "⏺ Sony recording started"})
        else:
            await broadcast({"type": "log",
                             "msg": f"⚠ Sony record failed: {err} — continuing without record"})
    else:
        await broadcast({"type": "log",
                         "msg": f"⏺ Record + Run: arm camera manually ({camera})"})

    # ── Pre-roll — with optional flash sync marker ───────────────────────────
    # For cinematic mode we handle pre-roll here (not inside play()) so we can
    # fire the flash at a precise moment and set motion_start_wall accurately.
    state["_flash_sync_wall"] = None

    async def _do_preroll(duration_s: float, use_flash: bool):
        """Wait duration_s with countdown ticks.  If use_flash, fire the flash
        at the halfway point and record flash_sync_wall in state."""
        if duration_s <= 0:
            return
        await broadcast({"type": "log",
                         "msg": f"⏳ Pre-roll: {duration_s:.0f}s before motion starts…"})
        flash_fired   = False
        flash_at      = duration_s / 2.0   # fire halfway through pre-roll
        elapsed       = 0.0
        step          = 0.25
        while elapsed < duration_s:
            await asyncio.sleep(step)
            elapsed += step
            # Fire flash once, at the halfway point
            if use_flash and not flash_fired and elapsed >= flash_at:
                flash_sync_wall = time.time()
                await asyncio.to_thread(hw.trigger_flash, 0.010)
                state["_flash_sync_wall"] = flash_sync_wall
                flash_fired = True
                await broadcast({"type": "log", "msg": "⚡ Flash sync fired"})
            remaining = max(0.0, duration_s - elapsed)
            if remaining <= 10.0 or int(remaining) != int(remaining + step):
                await broadcast({"type": "log", "msg": f"⏳ Motion in {remaining:.0f}s…"})

    if mode != "cinematic" and preroll_s > 0:
        await _do_preroll(preroll_s, flash_sync)
    elif mode == "cinematic" and preroll_s > 0:
        await _do_preroll(preroll_s, flash_sync)

    # Mark wall-clock time when phase=0 fires — set here (after explicit pre-roll)
    # so it accurately reflects when the motors actually start moving.
    state["_motion_start_wall"] = time.time()

    # ── Start motion ─────────────────────────────────────────────────────────
    if mode == "cinematic":
        # Stop all liveview during programmed moves — gphoto2 subprocess loops
        # and WiFi stream reconnects compete with the 60 Hz motor thread.
        global _cinematic_paused_liveview, _sony_liveview_running
        _cinematic_paused_liveview = False
        if _sony_usb_liveview_running:
            _stop_sony_usb_liveview()
            _cinematic_paused_liveview = True
        if _sony_liveview_running:
            _sony_liveview_running = False   # signal WiFi worker to exit
            _cinematic_paused_liveview = True

        # Stop InertiaEngine — both it and the motor thread call set_tmc_velocity /
        # lgpio.tx_pwm() on the same STEP pins.  InertiaEngine fires at 50 Hz with
        # velocity=0 (no stick input) and overwrites the 60 Hz motor commands,
        # causing the jitter and position errors seen during Record + Run.
        if _inertia and _inertia._running:
            _inertia.stop()

        # Pre-roll was handled above — start motion immediately.
        _prog_move.preroll_s = 0
        asyncio.create_task(_prog_move.play(skip_first_return=True))
        await broadcast({"type": "log", "msg": "▶ Cinematic move started"})
    else:
        state["stop_event"].clear()
        if hg.settings.enabled:
            base_iv = getattr(hg.settings, "interval_sec",
                              getattr(hg.settings, "interval_day", 5.0))
        else:
            base_iv = state["manual_interval"]
        asyncio.create_task(timelapse_worker(base_iv))

    # ── After completion: stop recording + write named sidecar ───────────────
    if recording_started:
        asyncio.create_task(_stop_record_after_run(mode, preroll_s))


async def _get_last_sony_clip_name() -> Optional[str]:
    """Return the stem of the most recently recorded Sony clip (e.g. 'C0090').

    Tries getContentList first (most reliable — returns clips sorted by date).
    Falls back to scanning the getEvent result array for any MP4 URL.
    Returns None if both approaches fail.
    """
    sony_ip = state.get("sony_ip", "")
    if not sony_ip:
        return None

    # ── Primary: getContentList sorted descending, take first item ────────────
    for storage_uri in ("storage:memoryCard1", "storage:memoryCard2"):
        try:
            resp = await asyncio.to_thread(
                lambda uri=storage_uri: _req.post(
                    f"http://{sony_ip}/sony/avContent",
                    json={"method": "getContentList",
                          "params": [{"uri": uri, "type": "movie",
                                      "sort": "descending", "count": 1}],
                          "id": 1, "version": "1.3"},
                    timeout=8,
                ).json()
            )
            items = (resp.get("result") or [[]])[0] or []
            if items:
                # title field is the clip stem; fall back to parsing the URL
                title = items[0].get("title", "")
                if title:
                    logger.info(f"Got clip name from getContentList: {title}")
                    return title
                url = ((items[0].get("content", {})
                                .get("original", [{}]) or [{}])[0]
                               .get("url", ""))
                if url:
                    stem = Path(url).stem
                    logger.info(f"Got clip name from getContentList URL: {stem}")
                    return stem
        except Exception as e:
            logger.debug(f"getContentList({storage_uri}) failed: {e}")

    # ── Fallback: scan every item in getEvent result for an MP4 URL ───────────
    try:
        ev = await asyncio.to_thread(
            lambda: _req.post(
                f"http://{sony_ip}/sony/camera",
                json={"method": "getEvent", "params": [False], "id": 1, "version": "1.0"},
                timeout=8,
            ).json()
        )
        for slot in (ev.get("result") or []):
            if not isinstance(slot, list):
                continue
            for item in slot:
                if not isinstance(item, dict):
                    continue
                for orig in (item.get("content", {}).get("original", []) or []):
                    url = orig.get("url", "")
                    if url and Path(url).suffix.upper() in (".MP4", ".MOV", ".MTS"):
                        stem = Path(url).stem
                        logger.info(f"Got clip name from getEvent: {stem}")
                        return stem
    except Exception as e:
        logger.warning(f"getEvent fallback failed: {e}")

    return None


async def _stop_record_after_run(mode: str, preroll_s: float = 0.0):
    """Wait for motion to complete, stop Sony recording, fetch clip name, write sidecar."""
    # Small yield so the motion task has time to set its running flag
    await asyncio.sleep(0.5)

    # Wait for motion to finish
    if mode == "cinematic":
        # Wait for play() to set _running, then wait for it to clear
        for _ in range(40):           # up to 2s for _running to go True
            if _prog_move and _prog_move._running:
                break
            await asyncio.sleep(0.05)
        while _prog_move and _prog_move._running:
            await asyncio.sleep(1.0)
    else:
        while state.get("is_running", False):
            await asyncio.sleep(1.0)

    ok, err = await _toggle_sony_record(start=False)
    if not ok:
        logger.warning(f"_stop_record_after_run: stopMovieRec failed: {err}")
        await broadcast({"type": "log", "msg": f"⚠ Stop record failed: {err}"})
        return

    await broadcast({"type": "log", "msg": "⏹ Sony recording stopped"})

    # Give camera a moment to finalise the file before querying
    await asyncio.sleep(3.0)

    clip_name = await _get_last_sony_clip_name()
    if not clip_name:
        clip_name = f"clip_{int(time.time())}"
        logger.warning(f"Could not get clip name from camera — using fallback: {clip_name}")

    # Write mode-appropriate sidecar named after the clip
    save_path = state.get("save_path", "")
    if save_path:
        os.makedirs(save_path, exist_ok=True)
        sidecar_path = os.path.join(save_path, f"{clip_name}.json")
        if mode == "cinematic":
            _write_cinema_sidecar(sidecar_path, preroll_s=preroll_s)
        else:
            _write_motion_sidecar(sidecar_path)
        await broadcast({"type": "log",
                         "msg": f"💾 Motion sidecar saved: {clip_name}.json"})

    # Restart liveview now that recording and sidecar writing are both done
    _restart_cinematic_liveview()

    # Restart InertiaEngine for immediate joystick use after move
    if _inertia:
        _inertia.set_target(0, 0, 0)
        _inertia.set_preset("responsive")
        if not _inertia._running:
            _inertia.start()


def _write_motion_sidecar(path: str):
    """Write per-frame motion log with phase, position, and timing to a JSON sidecar file."""
    frames = list(_session_history)
    if not frames:
        logger.warning("_write_motion_sidecar: no session history — skipping")
        return

    out_frames = []
    for idx, f in enumerate(frames):
        out_frames.append({
            "frame_idx":   f.get("frame", idx),
            "phase":       f.get("motion_phase", round(idx / max(len(frames) - 1, 1), 6)),
            "real_time_s": f.get("real_time_s", 0.0),
            "pos_s":       f.get("pos_s", 0.0),
            "pos_p":       f.get("pos_p", 0.0),
            "pos_t":       f.get("pos_t", 0.0),
        })

    camera = state.get("active_camera", "picam")
    sidecar = {
        "run_id":               _timelapse_run_id,
        "mode":                 state.get("active_mode", "timelapse"),
        "camera":               camera,
        "camera_fps":           state.get("sony_video_fps")         if camera == "sony" else None,
        "camera_quality":       state.get("sony_video_quality_str") if camera == "sony" else None,
        "camera_hfr":           state.get("sony_video_hfr", False)  if camera == "sony" else False,
        "focal_mm":             state.get("_sony_focal_mm"),
        "motion_start_wall":    state.get("_motion_start_wall", 0.0),
        "capture_interval_s":   state.get("manual_interval", 0.0),
        "frames":               out_frames,
    }
    try:
        with open(path, "w") as fh:
            json.dump(sidecar, fh, separators=(",", ":"))
        logger.info(f"Motion sidecar written: {path} ({len(out_frames)} frames)")
    except OSError as e:
        logger.error(f"_write_motion_sidecar: {e}")


def _write_cinema_sidecar(path: str, preroll_s: float = 0.0):
    """Write per-frame motion sidecar for a cinematic video recording.

    Generates the trajectory at camera_fps resolution using the same
    unified trajectory generator that drove the motors, so phase values
    are accurate to the actual easing curve used.
    """
    if _prog_move is None or len(_prog_move.keyframes) < 2:
        logger.warning("_write_cinema_sidecar: no programmed move — skipping")
        return

    camera     = state.get("active_camera", "picam")
    camera_fps = state.get("sony_video_fps") or 60

    total_dur = sum(max(0.1, kf.duration_s) for kf in _prog_move.keyframes[:-1])
    n_cam     = max(2, int(total_dur * camera_fps))

    # Generate trajectory at motor resolution (60 fps) then resample
    motor_fps     = 60
    n_motor       = max(2, int(total_dur * motor_fps))
    try:
        traj_s, traj_p, traj_t = _prog_move.generate_unified_trajectory(
            n_motor,
            _prog_move.origin_slider, _prog_move.origin_pan, _prog_move.origin_tilt,
            for_timelapse=False,
        )
    except Exception as e:
        logger.error(f"_write_cinema_sidecar: trajectory generation failed: {e}")
        return

    from motion_engine import generate_time_array
    import numpy as np
    t_norm       = generate_time_array(n_motor, _prog_move.global_easing)
    motor_times  = np.linspace(0.0, total_dur, n_motor)
    cam_times    = np.linspace(0.0, total_dur, n_cam)

    phase_cam = np.interp(cam_times, motor_times, t_norm)
    pos_s_cam = np.interp(cam_times, motor_times, traj_s)
    pos_p_cam = np.interp(cam_times, motor_times, traj_p)
    pos_t_cam = np.interp(cam_times, motor_times, traj_t)

    out_frames = [
        {
            "frame_idx":   i,
            "phase":       round(float(phase_cam[i]), 6),
            "real_time_s": round(float(cam_times[i]),  4),
            "pos_s":       round(float(pos_s_cam[i]),  2),
            "pos_p":       round(float(pos_p_cam[i]),  2),
            "pos_t":       round(float(pos_t_cam[i]),  2),
        }
        for i in range(n_cam)
    ]

    sidecar = {
        "run_id":            _timelapse_run_id,
        "mode":              "cinematic",
        "camera":            camera,
        "camera_fps":        camera_fps,
        "camera_quality":    state.get("sony_video_quality_str") if camera == "sony" else None,
        "camera_hfr":        state.get("sony_video_hfr", False)  if camera == "sony" else False,
        "camera_sq":         state.get("sony_video_sq",  False)  if camera == "sony" else False,
        "motion_start_wall": state.get("_motion_start_wall", 0.0),
        "flash_sync_wall":   state.get("_flash_sync_wall"),
        "pre_roll_s":        preroll_s,
        "duration_s":        total_dur,
        "easing":            _prog_move.global_easing,
        "focal_mm":          state.get("_sony_focal_mm"),
        "reversed":          bool(state.get("_move_reversed", False)),
        "frames":            out_frames,
    }
    try:
        with open(path, "w") as fh:
            json.dump(sidecar, fh, separators=(",", ":"))
        logger.info(f"Cinema sidecar written: {path} "
                    f"({n_cam} frames @ {camera_fps}fps, dur={total_dur:.1f}s)")
    except OSError as e:
        logger.error(f"_write_cinema_sidecar: {e}")


async def _toggle_sony_record(start: bool = True):
    """Start or stop Sony video recording.

    Flow for start:
      1. Query current shoot mode. If already in "movie" or "s&q", leave it alone.
         Only switch to "movie" if the camera is in a non-recording mode (e.g. "still").
         Preserves S&Q mode so 120fps slow-motion recording works correctly.
      2. Query video quality (getMovieQuality + S&Q-specific API for capture fps).
      3. Call startMovieRec via HTTP API.
      4. Fall back to gphoto2 PTP if HTTP fails and PTP mode is active.

    Flow for stop:
      1. Call stopMovieRec via HTTP API.
      2. Restore the shoot mode that was active before recording started
         (not always "still" — could be "s&q").
    """
    method = "startMovieRec" if start else "stopMovieRec"

    # ── 0. Query shoot mode and video quality at the moment recording starts ──
    if start:
        # Read current shoot mode so we can preserve it (especially "s&q")
        try:
            sm_res = await asyncio.to_thread(_sony_api, "getShootMode", [])
            current_shoot_mode = (sm_res.get("result") or ["still"])[0]
            state["_pre_record_shoot_mode"] = current_shoot_mode
            logger.info(f"Sony shoot mode at record start: {current_shoot_mode}")
        except Exception as _sme:
            state["_pre_record_shoot_mode"] = "still"
            logger.warning(f"Could not query getShootMode: {_sme}")

        # Detect S&Q mode and get capture fps
        is_sq = state["_pre_record_shoot_mode"] in ("s&q", "sanq", "slowAndQuick")
        state["sony_video_sq"] = is_sq

        # For S&Q: try the dedicated S&Q quality API first, fall back to getMovieQuality
        sq_capture_fps = None
        if is_sq:
            for sq_method in ("getSlowAndQuickMotionQuality", "getSloMotionQuality",
                              "getSlowAndQuickSetting"):
                try:
                    sqr = await asyncio.to_thread(_sony_api, sq_method, [])
                    sq_val = (sqr.get("result") or [None])[0]
                    if sq_val:
                        import re as _re2
                        m2 = _re2.search(r'(\d+)[pi]', str(sq_val))
                        if m2:
                            sq_capture_fps = int(m2.group(1))
                            logger.info(f"S&Q capture fps from {sq_method}: {sq_capture_fps}")
                            break
                except Exception:
                    pass

        try:
            qres = await asyncio.to_thread(_sony_api, "getMovieQuality", [])
            q = (qres.get("result") or [None])[0]   # e.g. "60p", "120p", "4K 24p"
            if q:
                import re as _re
                m = _re.search(r'(\d+)[pi]', q)
                fps = int(m.group(1)) if m else None
                # In S&Q mode prefer the dedicated API's capture fps if we got one
                if is_sq and sq_capture_fps:
                    fps = sq_capture_fps
                state["sony_video_fps"]         = fps
                state["sony_video_quality_str"] = q
                state["sony_video_hfr"] = bool(fps and fps >= 100)
                logger.info("Sony video quality: %s → %s fps%s%s",
                            q, fps,
                            " [HFR]" if state["sony_video_hfr"] else "",
                            " [S&Q]" if is_sq else "")
        except Exception as _qe:
            logger.warning("Could not query getMovieQuality: %s", _qe)
            if is_sq and sq_capture_fps:
                state["sony_video_fps"] = sq_capture_fps
                state["sony_video_hfr"] = True

    # ── 1. Sony HTTP Camera Remote API ───────────────────────────────────────
    try:
        # Switch shoot mode only if needed. "movie" and "s&q" can both accept
        # startMovieRec. Switching away from "s&q" would exit slow-motion mode.
        recording_modes = {"movie", "s&q", "sanq", "slowAndQuick"}
        if start:
            if state.get("_pre_record_shoot_mode") not in recording_modes:
                target_mode = "movie"
            else:
                target_mode = None   # already in a recording-capable mode — leave it
        else:
            # Restore the mode that was active before recording
            target_mode = state.get("_pre_record_shoot_mode", "still")
            if target_mode in recording_modes:
                target_mode = "still"   # always return to still after a stop

        if target_mode:
            try:
                mode_res = await asyncio.to_thread(_sony_api, "setShootMode", [target_mode])
                if "error" in mode_res:
                    err = mode_res["error"]
                    err_msg = err[1] if isinstance(err, list) and len(err) > 1 else str(err)
                    logger.warning(f"Sony setShootMode({target_mode}): {err_msg} (continuing anyway)")
                else:
                    logger.info(f"Sony setShootMode({target_mode}) OK")
            except Exception as mode_err:
                logger.warning(f"Sony setShootMode({target_mode}) failed: {mode_err} (continuing anyway)")

        res = await asyncio.to_thread(_sony_api, method, [])
        # Success: {"id":1,"result":[0]}   Error: {"id":1,"error":[code,"message"]}
        if "error" not in res:
            logger.info(f"Sony record: {method} OK via HTTP API")
            return True, f"Sony {method} OK"
        err_info = res["error"]
        err_code = err_info[0] if isinstance(err_info, list) else -1
        err_msg  = err_info[1] if isinstance(err_info, list) and len(err_info) > 1 else str(err_info)
        logger.warning(f"Sony record: {method} HTTP error {err_code}: {err_msg}")
        # Error code 3 = "Illegal Order" — camera is in the wrong shooting mode.
        # The setShootMode("movie") call above was either rejected (dial in M/A/S/P)
        # or not supported on this camera model.
        if err_code == 3 and start:
            # startMovieRec failed but setShootMode("movie") may have succeeded —
            # restore still mode so actTakePicture continues to work.
            try:
                await asyncio.to_thread(_sony_api, "setShootMode", ["still"])
                logger.info("Sony record: restored setShootMode(still) after startMovieRec failure")
            except Exception:
                pass
            return False, "MOVIE_MODE_REQUIRED"
        # Fall through to PTP fallback below
    except Exception as http_err:
        logger.warning(f"Sony record: HTTP API unavailable ({http_err}), trying PTP")

    # ── 2. gphoto2 PTP fallback ───────────────────────────────────────────────
    # Only attempted when PC Remote is enabled (port 15740 open).
    if state.get("ptp_mode"):
        try:
            import subprocess
            result = await asyncio.to_thread(lambda: subprocess.run(
                ["gphoto2", "--port", f"ptpip:{state['sony_ip']}",
                 "--set-config", "movie=1"],
                capture_output=True, text=True, timeout=10
            ))
            if result.returncode == 0:
                logger.info("Sony record: toggled via PTP/gphoto2")
                return True, "Sony record toggled via PTP."
            return False, result.stderr.strip() or "gphoto2 failed"
        except Exception as ptp_err:
            return False, f"PTP error: {ptp_err}"

    return False, (
        f"Sony {method} failed. Camera must be in 'Control with Smartphone' Wi-Fi mode. "
        f"If already connected, switch the camera to Movie mode manually then try again."
    )


async def _gamepad_event_processor():
    """
    Consume gamepad events and route to inertia engine / cinematic commands.
    Runs as a background task for the lifetime of the app.
    """
    global _inertia, _recording, _cinematic_mode, _cinematic_live_active, _last_input_time, _gp_broadcast_t
    global _gp_btn_l1, _gp_btn_r1

    # Timestamp after which axis events are accepted (skip initial burst on connect)
    gamepad_ignore_until = 0.0

    # Axis state is kept in the module-level _gp_axes dict so the UI joystick
    # handler can read it when computing merged targets (and vice-versa).

    # Axis event names that are stateless position reports.
    # Only the LATEST value for each axis matters — intermediate values are stale
    # by the time they're processed and should be discarded rather than replayed.
    # Button and connection events are NOT in this set — every edge matters.
    _AXIS_EVENTS = frozenset({
        "axis_pan", "axis_tilt", "axis_slider",
        "axis_l2",  "axis_r2",
        "dpad_x",   "dpad_y",
    })

    logger.info("Gamepad event processor started.")
    while True:
        # ── Wait for at least one event ───────────────────────────────────────
        try:
            first: GamepadEvent = await asyncio.wait_for(
                _gamepad_queue.get(), timeout=1.0
            )
        except asyncio.TimeoutError:
            continue
        except Exception:
            break

        # ── Drain any additional events that arrived while we were busy ───────
        # The gamepad fires at 125 Hz; the asyncio event loop can fall briefly
        # behind during camera I/O or broadcasts.  Without draining, the
        # processor replays stale stick positions in FIFO order — felt as
        # input lag followed by a catch-up lurch.
        #
        # Strategy: collect everything currently in the queue, then for axis
        # events keep ONLY the latest value per axis (current position is all
        # that matters).  Button / connection events are preserved in order
        # so every press and release is honoured.
        pending = [first]
        while True:
            try:
                pending.append(_gamepad_queue.get_nowait())
            except asyncio.QueueEmpty:
                break

        # Build latest-axis map and find the first occurrence of each axis
        # so we can replace it in-place and skip subsequent duplicates.
        latest_axis: dict = {}
        for ev in pending:
            if ev.name in _AXIS_EVENTS:
                latest_axis[ev.name] = ev   # last write wins

        # Reconstruct event list: non-axis events in original order, each axis
        # appearing exactly once (at its first queued position) with its latest value.
        collapsed: list = []
        seen_axes: set  = set()
        for ev in pending:
            if ev.name in _AXIS_EVENTS:
                if ev.name not in seen_axes:
                    collapsed.append(latest_axis[ev.name])   # use latest, not queued
                    seen_axes.add(ev.name)
                # else: stale intermediate — discard
            else:
                collapsed.append(ev)   # button / connection event — preserve all

        # ── Process the collapsed batch ───────────────────────────────────────
        for event in collapsed:
            name  = event.name
            value = event.value

            # Drop all axis/dpad events during the initial ignore window.
            # This prevents spurious values sent by the controller at connect time
            # (both analog drift and d-pad ghost events) from driving the motors.
            is_axis = name in ("axis_slider", "axis_pan", "axis_tilt",
                               "axis_l2", "axis_r2", "dpad_x", "dpad_y")
            if is_axis and time.time() < gamepad_ignore_until:
                continue

            # DEBUG: log all gamepad events so we can verify routing
            logger.info(f"Gamepad event: {name}={value:.3f}" if isinstance(value, float) else f"Gamepad event: {name}={value}")

            # Keep watchdog alive for gamepad input (not just WebSocket joystick)
            if is_axis:
                _last_input_time = time.time()

            # ── Axis events → InertiaEngine ──────────────────────────────────────
            # Control routing:
            #
            #   LEFT STICK  (axis_pan, axis_tilt)  → InertiaEngine physics (smooth, inertia)
            #   RIGHT STICK (axis_slider)           → InertiaEngine physics (smooth, inertia)
            #   D-PAD       (dpad_x, dpad_y)        → set_nudge_pt()  (direct, instant stop)
            #   L1/R1       (btn_l1, btn_r1)        → set_nudge_slider() (direct, instant stop)
            #
            # Nudge bypasses physics entirely so releasing the button stops the motor
            # in the same 20 ms tick — no coast, no inertia ramp-down.
            # This is critical for soft-limit and keyframe positioning.
            #
            # L1/R1 are digital buttons (not analog axes) chosen over L2/R2 because
            # they produce reliable press+release events on the 8BitDo in 2.4G mode.

            _stick_axes  = ("axis_slider", "axis_pan", "axis_tilt")
            _nudge_axes  = ("dpad_x", "dpad_y")          # L1/R1 handled in button section
            _all_axes    = _stick_axes + _nudge_axes

            if name in _all_axes:
                _gp_axes[name] = float(value)

            # ── Button display state — update BEFORE broadcast ────────────────────
            # _gp_btn_l1/r1 must be current when the broadcast fires so the GUI
            # shows the correct pressed/released state.  Motor-control side-effects
            # (slider nudge) are still gated in the elif block below.
            if name == "btn_l1":
                _gp_btn_l1 = bool(value)
            elif name == "btn_r1":
                _gp_btn_r1 = bool(value)

            # ── Mirror gamepad state to browser (rate-limited to 20 Hz) ──────────
            # Lets the GUI show joystick knob position, lit-up d-pad/L1/R1 buttons.
            if name in _all_axes or name in ("btn_l1", "btn_r1"):
                now_t = time.time()
                # Force-bypass the 20 Hz rate-limit for two critical cases:
                #
                # 1. Stick axis returning to zero: rate-limiter would suppress this
                #    update, leaving the GUI knob visually "stuck" off-center.
                #    No further events fire once the stick is at rest, so the next
                #    rate-limited slot never arrives.
                #
                # 2. Button press/release (L1, R1): press and release can arrive
                #    < 50 ms apart.  If the release is rate-limited, the GUI button
                #    highlight stays lit permanently.
                force_broadcast = (
                    name in ("axis_pan", "axis_tilt", "axis_slider")
                    and float(value) == 0.0
                ) or name in ("btn_l1", "btn_r1")

                if now_t - _gp_broadcast_t > 0.05 or force_broadcast:
                    _gp_broadcast_t = now_t
                    await broadcast({
                        "type":    "gamepad_input",
                        "pan":     _gp_axes["axis_pan"],
                        "tilt":    _gp_axes["axis_tilt"],
                        "slider":  _gp_axes["axis_slider"],
                        "l1":      _gp_btn_l1,
                        "r1":      _gp_btn_r1,
                        "dpad_x":  _gp_axes["dpad_x"],
                        "dpad_y":  _gp_axes["dpad_y"],
                    })

            if name in _all_axes and (not state.get("is_running") or _cinematic_live_active) and _inertia:
                _last_input_time = time.time()

                if not _inertia._running:
                    _inertia.start()

                # ── D-pad → pan/tilt nudge ────────────────────────────────────
                # D-pad sends ONE event on press (dpad=±1) and ONE on release (dpad=0).
                # We track a heartbeat on press so the watchdog can safety-stop the motor
                # if the wireless release event is lost (5-second timeout).
                #
                # Conflict guard: if L-stick is actively driving pan/tilt, ignore the
                # d-pad press entirely.  We always honour release events (value=0).
                if name in ("dpad_x", "dpad_y"):
                    pan_n  = _gp_axes["dpad_x"] * NUDGE_SPEED_PAN
                    # ABS_HAT0Y convention: +1 = DOWN, -1 = UP (same as screen Y).
                    # Negate so D-pad UP → positive tilt, matching the left-stick Y
                    # inversion (stick up = positive tilt).
                    tilt_n = -_gp_axes["dpad_y"] * NUDGE_SPEED_TILT
                    if pan_n != 0.0 or tilt_n != 0.0:
                        # Press — update heartbeat so watchdog knows button is held
                        _nudge_heartbeat["pan"]  = time.time()
                        _nudge_heartbeat["tilt"] = time.time()
                        stick_pt_active = (abs(_gp_axes["axis_pan"]) > 0.05
                                           or abs(_gp_axes["axis_tilt"]) > 0.05)
                        if not stick_pt_active:
                            if not _inertia._running:
                                _inertia.start()
                            _inertia.set_nudge_pt(pan_n, tilt_n)
                            _nudge_source["pan"]  = "gamepad"
                            _nudge_source["tilt"] = "gamepad"
                        # else: sticks active — silently ignore d-pad press
                    else:
                        # Release — always clear, regardless of stick state
                        _inertia.clear_nudge_pt()
                        _nudge_source["pan"]  = None
                        _nudge_source["tilt"] = None
                        _nudge_heartbeat["pan"]  = 0.0
                        _nudge_heartbeat["tilt"] = 0.0

                # ── Sticks → _gp_axes (target applied by sync task) ──────────
                # _gp_axes[name] was already updated above; _joystick_target_sync
                # reads the current _gp_axes + _ui_axes once per tick and calls
                # set_target().  No set_target() call here — doing so would chase
                # the drained event list rather than the live stick position.

            # ── L1/R1 → slider nudge ─────────────────────────────────────────────
            # Digital shoulder buttons: R1 = forward, L1 = backward.
            # Chosen over L2/R2 analog triggers because they give reliable
            # press-AND-release events on the 8BitDo in 2.4G wireless mode.
            # Gate: allowed whenever a timelapse is NOT running (is_running=False)
            # OR when cinematic live mode is active.  Never blocked during normal
            # jogging / positioning (is_running stays False for those).
            elif name in ("btn_l1", "btn_r1") and (not state.get("is_running") or _cinematic_live_active) and _inertia:
                # _gp_btn_l1/r1 already updated above for display; here we act on them.
                if not _inertia._running:
                    _inertia.start()
                stick_slider_active = abs(_gp_axes.get("axis_slider", 0)) > 0.05
                if _gp_btn_r1 and not _gp_btn_l1:
                    slider_n = NUDGE_SPEED_SLIDER
                elif _gp_btn_l1 and not _gp_btn_r1:
                    slider_n = -NUDGE_SPEED_SLIDER
                else:
                    slider_n = 0.0

                if slider_n != 0.0 and not stick_slider_active:
                    logger.info(f"L1/R1 slider nudge: {slider_n:+.1f} mm/s")
                    _inertia.set_nudge_slider(slider_n)
                    _nudge_source["slider"]    = "gamepad"
                    _nudge_heartbeat["slider"] = time.time()
                    _last_input_time = time.time()
                else:
                    _inertia.clear_nudge_slider()
                    _nudge_source["slider"]    = None
                    _nudge_heartbeat["slider"] = 0.0

            # ── Buttons ───────────────────────────────────────────────────────────

            elif name == "btn_shutter" and value:
                # ── A button — context-sensitive ───────────────────────────────
                # Cinema mode  → record toggle (same as GUI REC button).
                #                Star button physically maps to btn_record but the
                #                js0 index can be unreliable; A is always reachable.
                # AUX timelapse → fire the AUX trigger for the waiting frame loop.
                # All other     → single still capture (actTakePicture / PiCam).
                if _cinematic_mode in ("live", "programmed"):
                    # Delegate to the same path the GUI REC button uses
                    await broadcast({"type": "gamepad_btn", "btn": "record"})
                    logger.info("Gamepad A: cinema mode → record toggle")
                else:
                    tmode = state.get("trigger_mode", "normal")
                    if state.get("is_running") and tmode in ("aux_only", "aux_hybrid",
                                                              "picam_motion_only", "picam_motion_hybrid"):
                        state["aux_triggered"] = True
                        logger.info("Gamepad A: AUX trigger fired")
                        await broadcast({"type": "log", "msg": "📸 Gamepad trigger fired"})
                    else:
                        # Single capture — don't block the event loop
                        cam = state.get("active_camera", "picam")
                        logger.info(f"Gamepad A: manual shutter ({cam})")
                        if cam == "sony" and state.get("sony_ip"):
                            async def _gp_sony_capture():
                                shutter_s = state.get("picam_shutter_s", 1/125)
                                result = await asyncio.to_thread(capture_sony, "gp_shot", shutter_s)
                                if result:
                                    await broadcast({"type": "log",
                                        "msg": f"📸 Sony: {os.path.basename(result)}"})
                                    await broadcast({"type": "new_frame",
                                        "path": f"/shots/{os.path.basename(result)}"})
                                else:
                                    await broadcast({"type": "log",
                                        "msg": "⚠ Gamepad shutter: Sony capture failed"})
                            asyncio.create_task(_gp_sony_capture())
                        elif _HAS_PICAM and picam and not state.get("is_running"):
                            async def _gp_picam_capture():
                                shutter_s = state.get("picam_shutter_s", 1/125)
                                iso       = state.get("picam_iso", 400)
                                ts        = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                                dest      = os.path.join(state.get("save_path",
                                                str(Path.home() / "Pictures/PiSlider")),
                                                f"GP_{ts}.jpg")
                                os.makedirs(os.path.dirname(dest), exist_ok=True)
                                try:
                                    frame = await asyncio.to_thread(picam.capture_array)
                                    _, buf = cv2.imencode('.jpg', frame)
                                    with open(dest, 'wb') as f:
                                        f.write(buf.tobytes())
                                    await broadcast({"type": "log",
                                        "msg": f"📸 PiCam: {os.path.basename(dest)}"})
                                except Exception as e:
                                    await broadcast({"type": "log",
                                        "msg": f"⚠ Gamepad shutter: {e}"})
                            asyncio.create_task(_gp_picam_capture())

            elif name == "btn_record" and value:
                # ★ Star button → toggle video recording
                await broadcast({"type": "gamepad_btn", "btn": "record"})

            elif name == "btn_return" and value and _prog_move:
                if not state.get("is_running"):
                    async def _gp_return_to_start():
                        # Stop InertiaEngine so _move_to doesn't fight over motors
                        if _inertia and _inertia._running:
                            _inertia.stop()
                        state["is_running"] = True
                        await broadcast({"type": "run_state", "running": True})
                        try:
                            await _prog_move.return_to_start()
                        finally:
                            state["is_running"] = False
                            hw.enable_motors(True)   # _move_to leaves EN pin in unknown state
                            await broadcast({"type": "run_state", "running": False})
                            if _inertia:
                                _inertia.set_target(0, 0, 0)
                                _inertia.set_preset("responsive")
                                if not _inertia._running:
                                    _inertia.start()
                    asyncio.create_task(_gp_return_to_start())

            elif name == "btn_keyframe" and value and _prog_move:
                idx = _prog_move.add_keyframe(
                    slider_axis.current_mm, pan_axis.current_deg, tilt_axis.current_deg
                )
                await broadcast({"type": "cinematic_keyframe_added", "index": idx,
                                 "slider_mm": slider_axis.current_mm,
                                 "pan_deg":   pan_axis.current_deg,
                                 "tilt_deg":  tilt_axis.current_deg})

            elif name == "btn_play" and value:
                await broadcast({"type": "gamepad_btn", "btn": "play"})

            elif name == "btn_stop" and value:
                await broadcast({"type": "gamepad_btn", "btn": "stop"})

            elif name == "btn_arctan" and value:
                await broadcast({"type": "gamepad_btn", "btn": "arctan_toggle"})

            elif name == "btn_origin" and value:
                if _prog_move:
                    if _prog_move.keyframes:
                        kf0 = _prog_move.keyframes[0]
                        origin_s = slider_axis.current_mm - kf0.slider_mm
                        origin_p = pan_axis.current_deg   - kf0.pan_deg
                        origin_t = tilt_axis.current_deg  - kf0.tilt_deg
                    else:
                        origin_s = slider_axis.current_mm
                        origin_p = pan_axis.current_deg
                        origin_t = tilt_axis.current_deg
                    _prog_move.set_origin(origin_s, origin_p, origin_t)
                    if hasattr(timelapse_worker, '_traj_cache'):
                        del timelapse_worker._traj_cache
                    await broadcast({"type": "log",
                        "msg": f"📍 Gamepad: origin set — path anchored at current position "
                               f"(s={slider_axis.current_mm:.1f}mm "
                               f"pan={pan_axis.current_deg:.1f}° "
                               f"tilt={tilt_axis.current_deg:.1f}°). "
                               f"Tap '✕ Clear Origin' in UI to undo."})
                await broadcast({"type": "cinematic_origin_set",
                                 "slider_mm": origin_s if _prog_move else slider_axis.current_mm,
                                 "pan_deg":   origin_p if _prog_move else pan_axis.current_deg,
                                 "tilt_deg":  origin_t if _prog_move else tilt_axis.current_deg,
                                 "anchor_slider_mm": slider_axis.current_mm,
                                 "anchor_pan_deg":   pan_axis.current_deg,
                                 "anchor_tilt_deg":  tilt_axis.current_deg})

            elif name == "gamepad_connected":
                # Set 2-second ignore window to discard initial axis burst from controller
                gamepad_ignore_until = time.time() + 2.0
                # Reset all targets and nudge states to zero
                if _inertia:
                    _inertia.set_target(0, 0, 0)
                    _inertia.clear_nudge_pt()
                    _inertia.clear_nudge_slider()
                logger.info("Gamepad connected: 2s axis ignore window started.")
                await broadcast({"type": "gamepad_status", "connected": True})

            elif name == "gamepad_disconnected":
                if _inertia:
                    # Zero stick physics targets immediately so the motor doesn't
                    # keep running at whatever the last set_target() value was.
                    # (The watchdog is a delayed safety net; this is the primary stop
                    # for controller disconnect.)
                    _inertia.set_target(0, 0, 0)
                    _inertia.instant_stop_pt()
                    _inertia.instant_stop_slider()
                    _inertia.clear_nudge_pt()
                    _inertia.clear_nudge_slider()
                # Reset axis cache so _merged_targets() returns (0,0,0) immediately
                for k in _gp_axes: _gp_axes[k] = 0.0
                for k in _nudge_heartbeat: _nudge_heartbeat[k] = 0.0
                await broadcast({"type": "gamepad_status", "connected": False})


_last_input_time = 0.0

# ── UI nudge heartbeat timestamps ─────────────────────────────────────────────
# The nudge safety watchdog only applies to UI (browser button) nudges.
# Gamepad d-pad/trigger nudges are self-managing: dpad=1 starts, dpad=0 stops.
# No heartbeat needed for gamepad — and applying one would kill it immediately
# because the d-pad sends only one event on press, not a stream.
#
# _nudge_source: which source owns the current nudge for each axis.
#   "ui"      → started by UI button; expires if no keepalive within NUDGE_TIMEOUT_S
#   "gamepad" → started by d-pad/trigger; never expires via heartbeat
#   None      → nudge inactive
#
# _nudge_heartbeat: last time ui_nudge_start was received for that axis.
#   Only meaningful when _nudge_source[axis] == "ui".
NUDGE_TIMEOUT_S  = 0.5   # 500 ms; client keepalive fires every 150 ms
_nudge_source    = {"pan": None,  "tilt": None,  "slider": None}
_nudge_heartbeat = {"pan": 0.0,   "tilt": 0.0,   "slider": 0.0}

# Background tasks for precision step nudges (nudge_axis command).
# Stored so a new click can cancel any still-running step on the same axis.
_nudge_axis_tasks: dict = {"pan": None, "tilt": None, "slider": None}

# Generation counter — incremented on every new nudge_axis click.
# The task captures its generation at creation time; the finally block only
# zeroes hardware if its generation is still current (i.e. no newer click arrived).
_nudge_axis_gen: dict = {"pan": 0, "tilt": 0, "slider": 0}

# Rate-limiter for gamepad→browser mirror broadcasts (20 Hz cap).
_gp_broadcast_t: float = 0.0

# ── Shared input-axis state ───────────────────────────────────────────────────
# Both the gamepad event processor and the UI WebSocket joystick handler write
# into these dicts. _merged_targets() combines them so neither source can
# silently zero out what the other is doing.
#
# Without this, the gamepad and UI constantly overwrite each other's complete
# set_target() call — e.g. UI sends set_target(0,0,0) while gamepad holds 0.8
# on slider, motor loops between accelerating and braking at the soft limit.
# L1/R1 digital button state (True = pressed)
_gp_btn_l1: bool = False
_gp_btn_r1: bool = False

_gp_axes = {
    "axis_slider": 0.0, "axis_pan": 0.0, "axis_tilt": 0.0,
    "dpad_x": 0.0, "dpad_y": 0.0,
}
_ui_axes = {"slider": 0.0, "pan": 0.0, "tilt": 0.0}


def _merged_targets() -> tuple:
    """
    Combine gamepad STICK + UI joystick contributions into a single
    (slider, pan, tilt) target tuple clamped to [-1, 1].

    D-pad (pan/tilt nudge) and L2/R2 (slider nudge) are intentionally
    excluded here — they route directly to InertiaEngine.set_nudge_*()
    which bypasses physics so the motor stops the instant the button is
    released.  Including them here caused motors to coast after release
    because stick drift above the deadzone could re-set the target.
    """
    slider = max(-1.0, min(1.0,
        _gp_axes["axis_slider"]
        + _ui_axes["slider"]
    ))
    pan    = max(-1.0, min(1.0,
        _gp_axes["axis_pan"]
        + _ui_axes["pan"]
    ))
    tilt   = max(-1.0, min(1.0,
        _gp_axes["axis_tilt"]
        + _ui_axes["tilt"]
    ))
    return slider, pan, tilt


async def _joystick_target_sync():
    """
    Feed the current merged joystick/gamepad position to the InertiaEngine
    at 50 Hz — exactly once per physics tick.

    WHY THIS EXISTS
    ───────────────
    Both the WebSocket handler and the gamepad event processor update
    _ui_axes / _gp_axes (shared dicts) whenever input arrives.  Previously,
    each message also called set_target() immediately.  Because the WS handler
    can receive many buffered messages in rapid succession (the TCP receive
    buffer never drains between ticks), this caused set_target() to be called
    once for every queued message — playing back the full stick-movement
    history rather than jumping to the current position.  The motor appeared
    to chase its own tail: move left → right → release → motor still goes
    left, then right, then finally stops.

    By removing set_target() from both handlers and centralising it here,
    the InertiaEngine receives the stick position exactly ONCE per tick,
    always the CURRENT value in _ui_axes / _gp_axes, regardless of how many
    WS or gamepad messages arrived since the previous tick.
    """
    _pos_tick = 0
    while True:
        await asyncio.sleep(0.02)   # 50 Hz — matches InertiaEngine tick rate

        if not _inertia or not _inertia._running:
            continue

        # Respect automated-sequence lockout (timelapse, programmed move).
        # Cinematic live mode is exempt — it is joystick-driven by design.
        if state.get("is_running") and not _cinematic_live_active:
            continue

        s, p, t = _merged_targets()
        _inertia.set_target(slider=s, pan=p, tilt=t)

        # Broadcast live motor positions to UI at ~5 Hz (every 10th tick).
        # Fire-and-forget — never awaited so InertiaEngine ticks are never delayed.
        _pos_tick += 1
        if _pos_tick >= 10:
            _pos_tick = 0
            asyncio.create_task(broadcast({
                "type":  "status",
                "pos_s": round(slider_axis.current_mm,  2),
                "pos_p": round(pan_axis.current_deg,    2),
                "pos_t": round(tilt_axis.current_deg,   2),
            }))


async def _joystick_watchdog():
    """
    Safety net — two tiers:

    1. Nudge heartbeat (every 100 ms): if ui_nudge_start was last received
       more than NUDGE_TIMEOUT_S ago and the nudge is still set, kill it.
       Protects against a stuck motor when the browser drops the WebSocket
       mid-button-hold or pointer-up is never delivered to the server.

    2. Global inactivity (every 500 ms): if no joystick/gamepad input has
       arrived for 1.5 s, zero all targets and clear nudge.  Handles the
       case where the browser closes entirely while a stick is held.
    """
    global _last_input_time, _cinematic_live_active
    _tier2_tick = 0   # counts 100ms ticks; tier-2 check runs every 5th
    while True:
        await asyncio.sleep(0.1)
        # Skip watchdog during automated sequences, but keep it running during
        # cinematic_live so the UI nudge heartbeat timeout still protects us.
        if state.get("is_running") and not _cinematic_live_active:
            continue
        if not _inertia:
            continue

        now = time.time()

        # ── Tier 1a: UI nudge heartbeat timeout (500 ms) ─────────────────────
        # Browser jog buttons send ui_nudge_start keepalives every 150 ms.
        # If keepalive stops (pointer-up lost, WS drop), clear after 500 ms.
        if (_inertia._nudge_pan != 0.0 or _inertia._nudge_tilt != 0.0):
            pan_is_ui  = _nudge_source.get("pan")  == "ui"
            tilt_is_ui = _nudge_source.get("tilt") == "ui"
            pan_stuck  = (pan_is_ui  and _inertia._nudge_pan  != 0.0
                          and now - _nudge_heartbeat["pan"]  > NUDGE_TIMEOUT_S)
            tilt_stuck = (tilt_is_ui and _inertia._nudge_tilt != 0.0
                          and now - _nudge_heartbeat["tilt"] > NUDGE_TIMEOUT_S)
            if pan_stuck or tilt_stuck:
                logger.warning(
                    f"UI nudge timeout — clearing pan/tilt (age: "
                    f"pan={now-_nudge_heartbeat['pan']:.2f}s "
                    f"tilt={now-_nudge_heartbeat['tilt']:.2f}s)"
                )
                _inertia.clear_nudge_pt()
                _nudge_source["pan"]     = None
                _nudge_source["tilt"]    = None
                _nudge_heartbeat["pan"]  = 0.0
                _nudge_heartbeat["tilt"] = 0.0

        if _inertia._nudge_slider != 0.0 and _nudge_source.get("slider") == "ui":
            slider_age = now - _nudge_heartbeat["slider"]
            if slider_age > NUDGE_TIMEOUT_S:
                logger.warning(
                    f"UI nudge timeout — clearing slider (age={slider_age:.2f}s)"
                )
                _inertia.clear_nudge_slider()
                _nudge_source["slider"]    = None
                _nudge_heartbeat["slider"] = 0.0

        # ── Tier 1b: Gamepad nudge safety timeout (5 seconds) ────────────────
        # D-pad and L1/R1 are edge-triggered: one event on press, one on release.
        # If the wireless release event is dropped (2.4G dropout), the motor runs
        # indefinitely since there is no keepalive stream.  We store a heartbeat
        # on every D-pad/L1/R1 press and time out after GP_NUDGE_TIMEOUT_S.
        # 5 seconds covers the longest realistic single positioning move.
        GP_NUDGE_TIMEOUT_S = 5.0
        if (_inertia._nudge_pan != 0.0 or _inertia._nudge_tilt != 0.0):
            if (_nudge_source.get("pan") == "gamepad"
                    or _nudge_source.get("tilt") == "gamepad"):
                gp_age = now - max(_nudge_heartbeat.get("pan", 0),
                                   _nudge_heartbeat.get("tilt", 0))
                if gp_age > GP_NUDGE_TIMEOUT_S:
                    logger.warning(
                        f"Gamepad D-pad nudge timeout ({gp_age:.1f}s) — "
                        f"release event likely dropped. Clearing pan/tilt."
                    )
                    _inertia.clear_nudge_pt()
                    _nudge_source["pan"]     = None
                    _nudge_source["tilt"]    = None
                    _nudge_heartbeat["pan"]  = 0.0
                    _nudge_heartbeat["tilt"] = 0.0

        if (_inertia._nudge_slider != 0.0
                and _nudge_source.get("slider") == "gamepad"):
            gp_slider_age = now - _nudge_heartbeat.get("slider", 0)
            if gp_slider_age > GP_NUDGE_TIMEOUT_S:
                logger.warning(
                    f"Gamepad L1/R1 nudge timeout ({gp_slider_age:.1f}s) — "
                    f"release event likely dropped. Clearing slider."
                )
                _inertia.clear_nudge_slider()
                _nudge_source["slider"]    = None
                _nudge_heartbeat["slider"] = 0.0

        # ── Tier 2: global inactivity (checked at the slower 500 ms cadence) ─
        # We only run this check every 5 iterations (0.1 × 5 = 0.5 s).
        _tier2_tick = (_tier2_tick + 1) % 5
        if _tier2_tick != 0:
            continue

        if now - _last_input_time > 1.5:
            if _inertia._running:
                # ── Evdev delta-filter exemption ─────────────────────────────
                # evdev only fires an axis event when the value CHANGES.  A held
                # gamepad stick that is perfectly steady produces ZERO events, so
                # the watchdog would falsely declare "inactivity" and kill the
                # physics target mid-pan.  Before zeroing anything, check whether
                # any stick axis is currently held above the movement threshold.
                # If so, just refresh the input timer — the stick IS active, it
                # just isn't changing.  The genuine-inactivity path (controller
                # disconnected, browser closed) is covered by the gamepad_disconnected
                # event and the UI WebSocket close handler respectively.
                gp_stick_held = any(
                    abs(_gp_axes.get(k, 0.0)) > 0.05
                    for k in ("axis_pan", "axis_tilt", "axis_slider")
                )
                ui_stick_held = any(
                    abs(_ui_axes.get(k, 0.0)) > 0.05
                    for k in ("pan", "tilt", "slider")
                )
                if gp_stick_held or ui_stick_held:
                    # Stick is actively held — not genuine inactivity.
                    # Refresh timer so we re-check in another 1.5 s rather than
                    # spamming the log every 500 ms.
                    _last_input_time = now
                else:
                    # Genuine inactivity: no stick held, no recent events.
                    # Zero shared caches and stop motors.
                    for k in _gp_axes: _gp_axes[k] = 0.0
                    for k in _ui_axes: _ui_axes[k] = 0.0
                    _inertia.set_target(0, 0, 0)  # InertiaEngine coasts to stop
                    # Also clear any active nudge so motors stop immediately
                    _inertia.clear_nudge_pt()
                    _inertia.clear_nudge_slider()
                    for k in _nudge_heartbeat: _nudge_heartbeat[k] = 0.0


@app.on_event("startup")
async def _startup():
    """Start background tasks: gamepad reader + event processor."""
    global _gamepad_reader, _gamepad_task, _inertia, _prog_move, _soft_guard

    # Initialise cinematic objects with real hardware
    _inertia = InertiaEngine(
        hardware    = hw,
        guard       = _soft_guard,
        broadcast_fn = broadcast,
        slider_axis = slider_axis,
        pan_axis    = pan_axis,
        tilt_axis   = tilt_axis,
    )
    _prog_move = ProgrammedMove(
        hardware    = hw,
        guard       = _soft_guard,
        slider_axis = slider_axis,
        pan_axis    = pan_axis,
        tilt_axis   = tilt_axis,
        broadcast_fn = broadcast,
        arctan_tracker = _arctan,
    )

    # Restore path planning settings from previous session
    if SESSION_FILE.exists():
        try:
            _saved = json.loads(SESSION_FILE.read_text())
            if "path_mode"       in _saved: _prog_move.set_path_mode(_saved["path_mode"])
            if "global_easing"   in _saved: _prog_move.set_global_easing(_saved["global_easing"])
            if "catmull_tension" in _saved: _prog_move.set_catmull_tension(_saved["catmull_tension"])
        except Exception as _e:
            logger.warning(f"Path planning session restore failed: {_e}")

    # Initialise hardware inversions from state
    hw.set_inversions(
        state.get("slider_inverted", False),
        state.get("pan_inverted",    False),
        state.get("tilt_inverted",   False)
    )

    # HALL SENSOR DISABLED: Pre-calibrate all axes to CAL_BOTH
    # so full speed is available immediately without Hall/endstop calibration.
    # Remove these lines once endstops are connected.
    from cinematic_engine import AxisLimit
    for ax in (_soft_guard.slider, _soft_guard.pan, _soft_guard.tilt):
        ax.cal_state = AxisLimit.CAL_BOTH
    logger.info("Hall sensor bypassed: all axes pre-calibrated to full speed.")

    # Start InertiaEngine immediately in responsive preset — always-on for joystick/gamepad.
    # Switching to cinematic preset happens on cinematic_live_start.
    _inertia.set_preset("responsive")
    _inertia.start()
    logger.info("InertiaEngine started (responsive preset) — ready for joystick/gamepad.")

    # Start gamepad reader
    _gamepad_reader = GamepadReader(_gamepad_queue)
    _gamepad_task   = asyncio.create_task(_gamepad_reader.run())

    # Start gamepad event processor
    asyncio.create_task(_gamepad_event_processor())

    # Start joystick→InertiaEngine sync (50 Hz, always current position)
    asyncio.create_task(_joystick_target_sync())

    # Start Joystick Watchdog (prevents runaway if browser/network drops)
    asyncio.create_task(_joystick_watchdog())

    # Auto-detect (and if needed auto-reconnect) Sony camera on wlan1 at startup.
    # wlan0 is reserved for the Pi hotspot — never touched here.
    #
    # Strategy:
    #   1. Check if wlan1 is already connected to a Sony network.
    #   2. If not, and we have a saved SSID + Sony is the active camera, attempt
    #      to reconnect using the NetworkManager profile cached from the last
    #      manual connect.  NM stores the password so no password is needed here.
    #   3. Report success or failure via the normal sony_status broadcast.
    async def _startup_sony_check():
        await asyncio.sleep(3)   # give nmcli/networking a moment to settle

        status = await asyncio.to_thread(_check_sony_wlan1)
        if status["connected"]:
            state["sony_http_port"] = status.get("http_port", 8080)
            iface = status.get("iface", "wlan1")
            logger.info(f"Sony auto-detected at {status['ip']} via {iface} ({status['ssid']})")
            await broadcast({"type": "sony_status", "connected": True,
                             "ip": status["ip"], "ssid": status["ssid"],
                             "iface": iface, "model": f"Sony ({iface})"})
            _start_sony_liveview()
            return

        # Not yet connected — attempt auto-reconnect if we have a saved SSID
        saved_ssid = state.get("sony_ssid", "")
        if not saved_ssid or state.get("active_camera") != "sony":
            logger.info(f"Sony not detected at startup: {status['error']}")
            return

        logger.info(f"Sony not connected; attempting auto-reconnect to '{saved_ssid}'…")
        await broadcast({"type": "sony_status", "connected": False,
                         "msg": f"Auto-reconnecting to {saved_ssid}…"})
        try:
            # Disconnect first — clears any stale NM state on the interface
            await asyncio.to_thread(lambda: subprocess.run(
                ["nmcli", "dev", "disconnect", "wlan1"],
                capture_output=True, timeout=10))
            await asyncio.sleep(0.5)

            # Connect using the NM-cached profile (no password arg needed;
            # NM supplies it from the saved connection profile).
            res = await asyncio.to_thread(lambda: subprocess.run(
                ["nmcli", "dev", "wifi", "connect", saved_ssid, "ifname", "wlan1"],
                capture_output=True, text=True, timeout=30))

            if res.returncode != 0:
                err = (res.stderr.strip() or res.stdout.strip() or
                       "nmcli connect failed — camera may be off or out of range")
                logger.info(f"Sony auto-reconnect failed: {err}")
                await broadcast({"type": "sony_status", "connected": False, "error": err})
                return

            # WiFi joined — wait for DHCP then re-check
            await asyncio.sleep(3.0)
            status2 = await asyncio.to_thread(_check_sony_wlan1)
            if status2["connected"]:
                state["sony_http_port"] = status2.get("http_port", 8080)
                iface2 = status2.get("iface", "wlan1")
                logger.info(f"Sony auto-reconnected: {status2['ip']} ({status2['ssid']})")
                await broadcast({"type": "sony_status", "connected": True,
                                 "ip": status2["ip"], "ssid": status2["ssid"],
                                 "iface": iface2, "model": f"Sony ({iface2})"})
                _start_sony_liveview()
            else:
                logger.info(f"Sony auto-reconnect: WiFi joined but camera not responding: "
                            f"{status2['error']}")
                await broadcast({"type": "sony_status", "connected": False,
                                 "error": status2["error"]})

        except Exception as exc:
            logger.warning(f"Sony auto-reconnect exception: {exc}")
            await broadcast({"type": "sony_status", "connected": False,
                             "error": f"Auto-reconnect error: {exc}"})

    asyncio.create_task(_startup_sony_check())

    # Safety: always boot into timelapse mode regardless of saved session.
    # User must explicitly click CINEMATIC to enable motors.
    state["active_mode"] = "timelapse"
    logger.info("Cinematic engine ready — waiting for mode selection or gamepad input.")

    # Set LEDs to correct idle mode based on restored session state
    restored_mode = state.get("active_mode", "timelapse")
    led_mode = {"timelapse": "timelapse_idle",
                "macro":     "macro_idle",
                "cinematic": "cinematic"}.get(restored_mode, "timelapse_idle")
    status_leds.set_mode(led_mode)

    logger.info("Cinematic engine and gamepad reader started.")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8000,
        timeout_keep_alive=65,   # outlast most NAT/router idle timeouts (default 5s is too short)
    )
