#!/usr/bin/env python3
"""
gamepad.py — 8BitDo Pro 2 async gamepad reader for PiSlider.

Reads the controller via Linux evdev and publishes named events
to an asyncio queue consumed by the cinematic engine and joystick handler.

8BitDo Pro 2 axis/button map (Linux HID mode, XInput-style):
  ABS_X        Left stick X    → Pan
  ABS_Y        Left stick Y    → Tilt  (inverted: up = positive tilt)
  ABS_RX       Right stick X   → Slider
  ABS_RY       Right stick Y   → (future: focus / zoom)
  ABS_Z        L2 analog       → slow modifier (analog)
  ABS_RZ       R2 analog       → fast modifier (analog)
  ABS_HAT0X    D-pad X         → nudge pan
  ABS_HAT0Y    D-pad Y         → nudge tilt
  BTN_SOUTH    B               → return to start   (8BitDo physical B = XInput A/South)
  BTN_EAST     A               → shutter / AUX trigger  (8BitDo physical A = XInput B/East)
  BTN_NORTH    X               → add keyframe
  BTN_WEST     Y               → arctan lock toggle
  BTN_TL       L1              → slider nudge backward (held)
  BTN_TR       R1              → slider nudge forward  (held)
  BTN_START    Start/Menu      → play programmed move
  BTN_SELECT   Select/View     → stop / e-stop
  BTN_MODE     ★ Star          → video record start/stop
  BTN_THUMBL   Left stick click  → set origin
  BTN_THUMBR   Right stick click → (reserved)
"""

from __future__ import annotations

import asyncio
import logging
import os
from typing import Optional, Dict, Any

logger = logging.getLogger("PiSlider.Gamepad")

# ─── AXIS / BUTTON CONSTANTS ─────────────────────────────────────────────────
# (evdev event codes — same regardless of evdev import availability)
EV_ABS    = 3
EV_KEY    = 1
EV_SYN    = 0

ABS_X     = 0
ABS_Y     = 1
ABS_RX    = 3    # right stick X (standard XInput mapping)
ABS_RY    = 4
ABS_Z     = 2    # L2
ABS_RZ    = 5    # R2
ABS_BRAKE = 10   # right stick X on 8BitDo Compx 2.4G dongle in 2.4G mode
ABS_HAT0X = 16
ABS_HAT0Y = 17

BTN_SOUTH  = 304   # XInput A / 8BitDo physical B
BTN_EAST   = 305   # XInput B / 8BitDo physical A
BTN_NORTH  = 308   # Y
BTN_WEST   = 307   # X
BTN_TL     = 310   # L1
BTN_TR     = 311   # R1
BTN_SELECT = 314
BTN_START  = 315
BTN_MODE   = 316   # ★ Star / Home button (8BitDo Pro 2)
BTN_THUMBL = 317
BTN_THUMBR = 318

# Axis deadzone (raw value out of 32767 for sticks; triggers use their own range)
DEADZONE  = 2000      # for analog sticks (range ±32767)
AXIS_MAX  = 32767.0
# Deadzone as a fraction of axis range — used when axis max != 32767
DEADZONE_FRAC = DEADZONE / AXIS_MAX   # ≈ 6 %

# Trigger normalization — 8BitDo Pro 2 reports ABS_Z/ABS_RZ in range 0–255 on Linux
# (not ±32767 like sticks).  IMPORTANT: some firmware versions rest triggers at the
# midpoint (~128) rather than 0.  We read the actual resting value from evdev absinfo
# at connect time and subtract it so a fully-released trigger always reads 0.0.
TRIGGER_DEADZONE = 8    # small deadzone above rest position to ignore noise
TRIGGER_MAX      = 255.0   # default; overridden at connect time via absinfo

# Normalize analog stick value to [-1.0, 1.0] with deadzone
def _norm(raw: int, maxval: float = AXIS_MAX, deadzone: int = DEADZONE) -> float:
    if abs(raw) < deadzone:
        return 0.0
    sign = 1 if raw > 0 else -1
    return sign * (abs(raw) - deadzone) / (maxval - deadzone)

def _norm_trigger(raw: int, restval: float = 0.0, maxval: float = TRIGGER_MAX) -> float:
    """
    Normalize trigger to [0.0, 1.0] accounting for a non-zero resting position.
    restval: the raw value reported when the trigger is fully released (from absinfo).
    """
    effective_range = maxval - restval
    if effective_range <= 0:
        return 0.0
    adjusted = raw - restval
    if adjusted < TRIGGER_DEADZONE:
        return 0.0
    return min(1.0, adjusted / effective_range)


class GamepadEvent:
    """A named event from the gamepad."""
    __slots__ = ("name", "value", "raw")

    def __init__(self, name: str, value: Any, raw: int = 0):
        self.name  = name    # e.g. "axis_pan", "btn_record"
        self.value = value   # float for axes, True/False for buttons
        self.raw   = raw


class GamepadReader:
    """
    Async evdev gamepad reader.

    Usage:
        reader = GamepadReader(event_queue)
        asyncio.create_task(reader.run())

    Events published to event_queue (asyncio.Queue):
        axis_slider  float [-1, 1]   right stick X
        axis_pan     float [-1, 1]   left stick X
        axis_tilt    float [-1, 1]   left stick Y (inverted)
        axis_l2      float [0, 1]    L2 analog
        axis_r2      float [0, 1]    R2 analog
        dpad_x       int  -1/0/1     D-pad horizontal
        dpad_y       int  -1/0/1     D-pad vertical
        btn_shutter  bool            A button (physical A on 8BitDo) — shutter / AUX trigger
        btn_record   bool            ★ Star button — video record toggle
        btn_return   bool            B button — return to start
        btn_arctan   bool            Y button (toggle)
        btn_keyframe bool            X button (add keyframe)
        btn_l1       bool            L1 (slow modifier, held)
        btn_r1       bool            R1 (fast modifier, held)
        btn_play     bool            Start
        btn_stop     bool            Select (e-stop)
        btn_origin   bool            Left stick click (set origin)
    """

    def __init__(self, queue: asyncio.Queue, device_path: Optional[str] = None):
        self._queue  = queue
        self._path   = device_path
        self._stop   = asyncio.Event()
        self.connected = False

        # Per-axis max values discovered via evdev absinfo at connect time.
        # Populated in _read_loop; defaults assume standard stick range.
        # ABS_BRAKE (10) = right stick X on 8BitDo Compx 2.4G dongle; default
        # max 127 based on observed values (range appears to be -127..+127).
        self._axis_max: Dict[int, float] = {
            ABS_X:     AXIS_MAX,    # left stick X — pan
            ABS_Y:     AXIS_MAX,    # left stick Y — tilt
            ABS_RX:    AXIS_MAX,    # right stick X (standard XInput) — slider
            ABS_RY:    AXIS_MAX,    # right stick Y (unused)
            ABS_Z:     TRIGGER_MAX, # L2 trigger
            ABS_RZ:    TRIGGER_MAX, # R2 trigger
            ABS_BRAKE: 127.0,       # right stick X on Compx 2.4G — slider
        }

        # Trigger resting (released) positions — some 8BitDo firmware versions
        # report ~128 instead of 0 when the trigger is fully released.
        # Discovered from absinfo.value at connect time.
        self._trigger_rest: Dict[int, float] = {
            ABS_Z:  0.0,   # L2
            ABS_RZ: 0.0,   # R2
        }

        # Per-axis minimum values from evdev absinfo.
        # If min >= 0 the axis is unipolar (e.g. 0-255 from some 8BitDo firmware
        # versions for the left stick). We then centre it at (min+max)/2 before
        # normalising so the resting position maps to 0.0 instead of ~0.5.
        self._axis_min: Dict[int, float] = {
            ABS_X:     -AXIS_MAX,  # left stick X — assume bipolar until proven otherwise
            ABS_Y:     -AXIS_MAX,
            ABS_RX:    -AXIS_MAX,  # right stick X (standard)
            ABS_RY:    -AXIS_MAX,
            ABS_Z:     0.0,
            ABS_RZ:    0.0,
            ABS_BRAKE: -127.0,     # right stick X on Compx 2.4G — bipolar centred at 0
        }

        # Resting (center) position of stick axes.  Populated from evdev absinfo.value
        # at connect time.  Non-zero means the controller reports the stick in a
        # shifted range (e.g. 0-255 with rest at 127) even though absinfo.min says -32767.
        # We use this to re-centre the axis so the resting position maps to 0.0.
        # ABS_Z is also included here because on the 8BitDo Compx 2.4G dongle the
        # right stick X is reported on ABS_Z (code 2) instead of ABS_RX (code 3).
        # When rest > 0, ABS_Z is reclassified as a slider axis rather than L2 trigger.
        self._stick_rest: Dict[int, float] = {
            ABS_X: 0.0, ABS_Y: 0.0, ABS_RX: 0.0, ABS_RY: 0.0, ABS_BRAKE: 0.0,
            ABS_Z: 0.0,  # may be reclassified as slider if rest position is centred
        }

        # Current axis state (for publishing deltas only when changed)
        self._axes: Dict[str, float] = {
            "axis_slider": 0.0,
            "axis_pan":    0.0,
            "axis_tilt":   0.0,
            "axis_l2":     0.0,
            "axis_r2":     0.0,
        }
        self._buttons: Dict[str, bool] = {}

    def stop(self):
        self._stop.set()

    async def run(self):
        """Main loop — auto-detects device, reads events, publishes to queue."""
        last_error = ""
        while not self._stop.is_set():
            path = self._path or await self._find_device()
            if not path:
                if last_error != "no_device":
                    logger.info("Gamepad: no device found, waiting for connection...")
                    last_error = "no_device"
                await asyncio.sleep(5)
                continue

            if last_error != "connected":
                logger.info(f"Gamepad: opening {path}")
                last_error = "connected"
                
            try:
                await self._read_loop(path)
            except Exception as e:
                err_msg = str(e)
                if last_error != err_msg:
                    logger.warning(f"Gamepad: disconnected ({err_msg}), retrying in background...")
                    last_error = err_msg
                self.connected = False
                await self._queue.put(GamepadEvent("gamepad_disconnected", False))
                await asyncio.sleep(3)

    async def _find_device(self) -> Optional[str]:
        """
        Scan /dev/input/event* for a gamepad.  Priority order:

        1. Named gamepad (keyword match) that ALSO has ABS_X + ABS_Y stick axes.
           When multiple matches exist (e.g. Compx dongle creates several event
           nodes), pick the one with the most analog axes — that node carries the
           full controller HID interface, not just the keyboard part.
        2. Any clean-name device with ABS_X + ABS_Y and no reject keywords.
        3. Soft-reject-name device (e.g. "keyboard") that has ABS_X + ABS_Y —
           the 8BitDo Pro 2 in 2.4G mode presents as
           'Compx 2.4G Wireless Receiver Keyboard'; the keyboard interface
           can carry full analog axes on some kernel versions.

        NOTE: We no longer return on the first keyword match.  The Compx dongle
        registers multiple /dev/input/event* nodes under the same name; the first
        one encountered is often the keyboard-only HID interface with no real
        analog axes, while the second or third is the actual joystick interface.
        We scan ALL nodes and prefer the one with the most ABS codes.
        """
        GAMEPAD_KEYWORDS = ("8bitdo", "gamepad", "joystick", "controller", "pro 2",
                            "xbox", "ps4", "ps5", "dualshock", "dualsense", "pro controller",
                            "compx", "wireless receiver")   # Compx 2.4G dongle
        # Hard rejects — these never carry real analog axes
        HARD_REJECT = ("power button", "hdmi", "touchpad")
        # Soft rejects — accept only if the device also has stick axes
        SOFT_REJECT = ("keyboard", "mouse")

        try:
            import evdev

            # Collect all candidates so we can rank them
            keyword_candidates: list = []   # (path, name, abs_count)
            found_fallback  = None          # clean name + sticks
            found_soft_maybe = None         # soft-reject name + sticks

            for path in evdev.list_devices():
                try:
                    dev  = evdev.InputDevice(path)
                    name = dev.name.lower()
                    caps = dev.capabilities()

                    # Must have both analog axes and buttons to be a gamepad
                    if EV_ABS not in caps or EV_KEY not in caps:
                        continue

                    # Hard-reject — never a gamepad regardless of axes
                    if any(kw in name for kw in HARD_REJECT):
                        continue

                    abs_codes  = [code for code, _ in caps.get(EV_ABS, [])]
                    has_sticks = ABS_X in abs_codes and ABS_Y in abs_codes

                    if any(kw in name for kw in GAMEPAD_KEYWORDS):
                        if has_sticks:
                            # Collect ALL keyword matches; we'll pick best below
                            keyword_candidates.append((path, dev.name, len(abs_codes)))
                            logger.info(f"Gamepad finder: keyword match '{dev.name}' "
                                        f"at {path} ({len(abs_codes)} ABS axes)")
                        else:
                            logger.info(f"Gamepad finder: '{dev.name}' matches keyword "
                                        f"but has NO stick axes (abs={abs_codes}) — skipped")
                        continue

                    # Soft-reject names: accept as last-resort if it has sticks
                    if any(kw in name for kw in SOFT_REJECT):
                        if has_sticks and found_soft_maybe is None:
                            found_soft_maybe = (path, dev.name)
                            logger.info(f"Gamepad finder: '{dev.name}' soft-reject name "
                                        f"but has stick axes — kept as last-resort")
                        continue

                    # Clean name + sticks → fallback
                    if has_sticks and found_fallback is None:
                        found_fallback = (path, dev.name)

                except Exception:
                    pass

            # Pick keyword match with the most ABS axes (fullest HID interface)
            if keyword_candidates:
                keyword_candidates.sort(key=lambda x: x[2], reverse=True)
                best = keyword_candidates[0]
                # Log full ABS code list so axis mapping issues are immediately visible
                try:
                    _dev_caps = evdev.InputDevice(best[0]).capabilities()
                    _abs_codes = sorted([c for c, _ in _dev_caps.get(EV_ABS, [])])
                    logger.info(f"Gamepad finder: selected '{best[1]}' at {best[0]} "
                                f"({best[2]} ABS axes) — codes: {_abs_codes}")
                except Exception:
                    logger.info(f"Gamepad finder: selected '{best[1]}' at {best[0]} "
                                f"({best[2]} ABS axes)")
                return best[0]

            if found_fallback:
                logger.info(f"Gamepad finder: using fallback '{found_fallback[1]}' "
                            f"at {found_fallback[0]}")
                return found_fallback[0]

            if found_soft_maybe:
                logger.info(f"Gamepad finder: using soft-reject fallback "
                            f"'{found_soft_maybe[1]}' at {found_soft_maybe[0]}")
                return found_soft_maybe[0]

        except ImportError:
            pass

        # Last resort: js0/js1 joystick nodes
        for candidate in ["/dev/input/js0", "/dev/input/js1"]:
            if os.path.exists(candidate):
                return candidate
        return None

    async def _read_loop(self, path: str):
        """
        Read events from device. Supports both:
        - evdev InputDevice (preferred, event interface)
        - /dev/input/js* via raw struct read (fallback)
        """
        try:
            import evdev
            dev = evdev.InputDevice(path)

            # Learn per-axis ranges from evdev absinfo so normalization is correct
            # regardless of whether the controller uses ±32767 or 0–255 ranges.
            try:
                caps = dev.capabilities()
                for code, info in caps.get(EV_ABS, []):
                    if code not in self._axis_max:
                        continue
                    if hasattr(info, 'max') and info.max > 0:
                        self._axis_max[code] = float(info.max)

                    # Store minimum so we can detect unipolar axes (min >= 0).
                    # Some 8BitDo firmware versions report the left stick in 0–255
                    # range rather than ±32767.  Without knowing min we would
                    # normalise the resting-centre position (~127) as +0.47 speed.
                    if code in self._axis_min and hasattr(info, 'min'):
                        self._axis_min[code] = float(info.min)

                    # Detect trigger rest position — some 8BitDo versions rest at
                    # ~128 (midpoint) rather than 0 when fully released.
                    if code in self._trigger_rest and hasattr(info, 'value'):
                        rest = float(info.value)
                        if rest > TRIGGER_DEADZONE * 2:   # non-trivial offset
                            self._trigger_rest[code] = rest

                    # Detect stick rest (center) position.  If absinfo says min=-32767
                    # but the current resting value is e.g. 127, the controller is
                    # reporting in 0-255 range while claiming ±32767 — all values
                    # fall inside the bipolar deadzone and the axis reads as 0.0.
                    # We store the resting position and re-centre in _handle_abs.
                    #
                    # IMPORTANT: use 10% of the axis max as threshold, NOT the
                    # raw DEADZONE constant (2000).  For a 0-255 range axis the
                    # rest position is ~127, which is greater than 2000 is False —
                    # causing the Compx 2.4G dongle's left/right sticks to never
                    # get their rest position stored and therefore read as ~0.467
                    # at centre instead of 0.0.
                    if code in self._stick_rest and hasattr(info, 'value'):
                        rest = float(info.value)
                        max_for_code = self._axis_max.get(code, AXIS_MAX)
                        if rest > max_for_code * 0.10:   # e.g. 127 > 25.5 ✓ for 0-255 axes
                            self._stick_rest[code] = rest
                            logger.info(f"Gamepad absinfo: axis {code} resting at {rest:.0f} "
                                        f"(non-zero centre detected — will re-centre)")

                # Log the discovered ranges so axis mapping issues are visible
                # ABS_Z label depends on whether it was reclassified as a slider.
                abs_z_rest = self._stick_rest.get(ABS_Z, 0.0)
                abs_z_label = ('R.stick X [RECLASSIFIED as slider]'
                               if abs_z_rest > 0 else 'L2 trigger')
                names = {ABS_X:'L.stick X',       ABS_Y:'L.stick Y',
                         ABS_RX:'R.stick X (std)', ABS_RY:'R.stick Y',
                         ABS_Z: abs_z_label,        ABS_RZ:'R2',
                         ABS_BRAKE:'R.stick X (Compx 2.4G)'}
                for code, label in names.items():
                    mn  = self._axis_min.get(code, '?')
                    mx  = self._axis_max.get(code, '?')
                    rst = self._trigger_rest.get(code, '-')
                    rest_str = f" rest={rst:.0f}" if isinstance(rst, float) and rst > 0 else ""
                    stick_rest = self._stick_rest.get(code, 0.0)
                    srest_str = f" stick_rest={stick_rest:.0f}" if stick_rest > 0 else ""
                    unipolar = (isinstance(mn, float) and mn >= 0
                                and code not in (ABS_Z, ABS_RZ))
                    uni_str = " [UNIPOLAR - will centre]" if unipolar else ""
                    logger.info(f"Gamepad absinfo: {label} (axis {code}) "
                                f"min={mn} max={mx}{rest_str}{srest_str}{uni_str}")

            except Exception as e:
                logger.warning(f"Gamepad: could not read absinfo: {e}")

            self.connected = True
            await self._queue.put(GamepadEvent("gamepad_connected", True))
            logger.info(f"Gamepad connected: {dev.name}")

            async for event in dev.async_read_loop():
                if self._stop.is_set():
                    break
                self._handle_evdev_event(event.type, event.code, event.value)

        except ImportError:
            # Fallback: raw js0 protocol (16-byte struct: time, value, type, number)
            import struct
            self.connected = True
            await self._queue.put(GamepadEvent("gamepad_connected", True))
            logger.info(f"Gamepad (js0 fallback): {path}")

            with open(path, "rb") as f:
                while not self._stop.is_set():
                    data = await asyncio.to_thread(f.read, 8)
                    if len(data) < 8:
                        break
                    _, value, typ, number = struct.unpack("IhBB", data)
                    self._handle_js0_event(typ, number, value)
                    await asyncio.sleep(0)

    def _handle_evdev_event(self, typ: int, code: int, value: int):
        """Map evdev events to named gamepad events."""
        if typ == EV_ABS:
            # Log unmapped ABS axis codes so layout surprises are immediately visible.
            # Known codes: 0=L.X, 1=L.Y, 2=L2, 3=R.X, 4=R.Y, 5=R2, 10=R.X(Compx), 16/17=D-pad
            if code not in (0, 1, 2, 3, 4, 5, 10, 16, 17):
                logger.info(f"Gamepad: unknown ABS axis code={code} value={value}")
            self._handle_abs(code, value)
        elif typ == EV_KEY:
            self._handle_key(code, bool(value))

    def _handle_js0_event(self, typ: int, number: int, value: int):
        """Map js0 raw events (type 1=button, 2=axis) to named events."""
        if typ == 2:   # axis
            # js0 axis numbers for 8BitDo Pro 2 in 2.4G mode (confirmed via SDL
            # gamecontrollerdb.txt — SDL and js0 both assign sequential indices
            # from the evdev ABS capability bitmap in code-number order):
            #   ABS_X(0)→0, ABS_Y(1)→1, ABS_Z(2)→2, ABS_RX(3)→3,
            #   ABS_RY(4)→4, ABS_RZ(5)→5, ABS_HAT0X(16)→6, ABS_HAT0Y(17)→7
            #
            # D-pad Y convention: Linux ABS_HAT0Y sends -1 for physical UP,
            # +1 for physical DOWN — same in js0.  Do NOT invert; app.py
            # handles the motor-direction flip with tilt_n = -dpad_y * speed.
            js0_axis_map = {
                0: ("axis_pan",    False),   # Left stick X  (ABS_X)
                1: ("axis_tilt",   True),    # Left stick Y  (ABS_Y, invert)
                2: ("axis_slider", False),   # Right stick X (ABS_Z on 8BitDo 2.4G)
                3: (None,          False),   # Right stick Y (ABS_RX, unused)
                4: ("axis_r2",     False),   # R2 trigger    (ABS_RY)
                5: ("axis_l2",     False),   # L2 trigger    (ABS_RZ)
                6: ("dpad_x",      False),   # D-pad X       (ABS_HAT0X)
                7: ("dpad_y",      False),   # D-pad Y       (ABS_HAT0Y, no invert)
            }
            if number in js0_axis_map:
                name, invert = js0_axis_map[number]
                if name is None:
                    return
                if name.startswith("dpad"):
                    v = -1 if value < -DEADZONE else (1 if value > DEADZONE else 0)
                    if invert:
                        v = -v
                    self._put(GamepadEvent(name, v, value))
                else:
                    norm = _norm(value)
                    if invert:
                        norm = -norm
                    if abs(norm - self._axes.get(name, 0)) > 0.01:
                        self._axes[name] = norm
                        self._put(GamepadEvent(name, norm, value))

        elif typ == 1:  # button
            # 8BitDo Pro 2 in 2.4G mode reports ALL 15 buttons (BTN codes 304–318).
            # js0 button index = sequential position of each BTN code in the sorted
            # EV_KEY capability bitmap — confirmed from SDL gamecontrollerdb.txt:
            #
            #  0 = BTN_SOUTH (304) → physical B on 8BitDo → return to start
            #  1 = BTN_EAST  (305) → physical A on 8BitDo → shutter / AUX trigger
            #  2 = BTN_C     (306) → unused extra button
            #  3 = BTN_WEST  (307) → physical Y on 8BitDo (arctan helper)
            #  4 = BTN_NORTH (308) → physical X on 8BitDo (keyframe)
            #  5 = BTN_Z     (309) → unused extra button
            #  6 = BTN_TL    (310) → L1 shoulder → slider nudge backward
            #  7 = BTN_TR    (311) → R1 shoulder → slider nudge forward
            #  8 = BTN_TL2   (312) → digital L2 (unused)
            #  9 = BTN_TR2   (313) → digital R2 (unused)
            # 10 = BTN_SELECT(314) → Back/Select
            # 11 = BTN_START (315) → Start/Play
            # 12 = BTN_MODE  (316) → ★ Guide/Star → record toggle
            # 13 = BTN_THUMBL(317) → L3 → return to origin
            # 14 = BTN_THUMBR(318) → R3 → stop
            js0_btn_map = {
                0:  "btn_return",    # physical B (BTN_SOUTH = js0 b0) — return to start
                1:  "btn_shutter",   # physical A (BTN_EAST  = js0 b1) — shutter / AUX trigger
                3:  "btn_arctan",    # physical Y (BTN_WEST  = js0 b3) — arctan toggle
                4:  "btn_keyframe",  # physical X (BTN_NORTH = js0 b4) — add keyframe
                6:  "btn_l1",        # L1 shoulder (BTN_TL   = js0 b6) — slider nudge backward
                7:  "btn_r1",        # R1 shoulder (BTN_TR   = js0 b7) — slider nudge forward
                10: "btn_stop",      # Back/Select (BTN_SELECT = js0 b10)
                11: "btn_play",      # Start       (BTN_START  = js0 b11)
                12: "btn_record",    # ★ Star/Guide (BTN_MODE  = js0 b12) — video record toggle
                13: "btn_origin",    # L3          (BTN_THUMBL = js0 b13) — set origin
                14: "btn_stop_r",    # R3          (BTN_THUMBR = js0 b14)
            }
            if number in js0_btn_map:
                name = js0_btn_map[number]
                pressed = bool(value)
                logger.info(f"Gamepad js0: BTN number={number} → {name} pressed={pressed}")
                if self._buttons.get(name) != pressed:
                    self._buttons[name] = pressed
                    self._put(GamepadEvent(name, pressed, value))
            else:
                logger.info(f"Gamepad js0: unknown BTN number={number} value={value}")

    def _handle_abs(self, code: int, value: int):
        # ── Trigger codes — unipolar, may have non-zero rest position ──────────
        # EXCEPTION: ABS_Z may be reclassified as the right-stick X (slider) on
        # the 8BitDo Compx 2.4G dongle.  This is detected at connect time by
        # checking absinfo.value: if the rest position is centred (> 10% of max),
        # ABS_Z is a stick, not a trigger — we fall through to stick_mapping below.
        TRIGGER_CODES = {ABS_Z: "axis_l2", ABS_RZ: "axis_r2"}
        _abs_z_is_slider = self._stick_rest.get(ABS_Z, 0.0) > 0.0
        if code in TRIGGER_CODES and not (code == ABS_Z and _abs_z_is_slider):
            name    = TRIGGER_CODES[code]
            maxval  = self._axis_max.get(code, TRIGGER_MAX)
            restval = self._trigger_rest.get(code, 0.0)
            norm    = _norm_trigger(value, restval, maxval)
            if abs(norm - self._axes.get(name, 0)) > 0.005:
                self._axes[name] = norm
                self._put(GamepadEvent(name, norm, value))
            return

        # ── Analog stick codes — bipolar or unipolar ────────────────────────────
        # ABS_RX  (3): standard XInput right-stick X → slider
        # ABS_Z   (2): right-stick X on 8BitDo Compx 2.4G dongle (reclassified from L2)
        # ABS_BRAKE (10): right-stick X on 8BitDo Compx 2.4G dongle → slider
        # All map to axis_slider; whichever is present on the device wins.
        stick_mapping = {
            ABS_X:     ("axis_pan",    False),
            ABS_Y:     ("axis_tilt",   True),    # invert Y: stick up = positive tilt
            ABS_RX:    ("axis_slider", False),
            ABS_RY:    (None,          False),
            ABS_Z:     ("axis_slider", False),   # Compx 2.4G right-stick X (reclassified from L2)
            ABS_BRAKE: ("axis_slider", False),   # Compx 2.4G dongle right-stick X (alt code)
        }
        if code in stick_mapping:
            name, invert = stick_mapping[code]
            if name is None:
                return
            min_val  = self._axis_min.get(code, -AXIS_MAX)
            max_val  = self._axis_max.get(code, AXIS_MAX)
            rest_pos = self._stick_rest.get(code, 0.0)

            # Detect "pseudo-unipolar": absinfo claims bipolar (min < 0) but the
            # resting position is significantly positive — the controller sends
            # values in e.g. 0-255 range centred at 127 while lying about min.
            # Fix: treat as unipolar centred at rest_pos.
            pseudo_unipolar = (min_val < 0 and rest_pos > DEADZONE)

            # Detect "fake unipolar": absinfo says min >= 0 (looks unipolar) but
            # the resting position is AT or very near the minimum edge (e.g. a
            # right-stick that rests at 0 with absinfo min=0).  A true unipolar
            # axis (like a trigger) also rests at 0, so we can't distinguish just
            # from absinfo — but for STICK axes the rest is always the physical
            # centre.  Require rest > 20% of max before treating as centred.
            # If rest is near zero and absinfo says min=0, treat as bipolar -max…+max.
            truly_unipolar = (min_val >= 0 and rest_pos > max_val * 0.20)

            if truly_unipolar or pseudo_unipolar:
                # ── Unipolar / pseudo-unipolar: centre at rest_pos or midpoint ─
                if pseudo_unipolar:
                    center     = rest_pos
                    half_range = max(1.0, max_val - rest_pos)
                else:
                    center     = (min_val + max_val) / 2.0
                    half_range = max(1.0, (max_val - min_val) / 2.0)
                dz_abs = max(2.0, half_range * DEADZONE_FRAC)
                offset = float(value) - center
                if abs(offset) < dz_abs:
                    norm = 0.0
                else:
                    norm = max(-1.0, min(1.0, offset / half_range))
            else:
                # ── Bipolar axis: -max … +max, centre at 0 ───────────────────
                # Covers both truly-bipolar axes (min < 0) AND "fake unipolar"
                # axes where absinfo lies (min=0) but the stick actually rests at 0.
                deadzone = max(2, int(max_val * DEADZONE_FRAC))
                norm = _norm(value, max_val, deadzone)

            if invert:
                norm = -norm

            # Log right-stick (slider axis) raw values for diagnostics.
            _is_slider_code = (code in (ABS_RX, ABS_BRAKE)
                               or (code == ABS_Z and _abs_z_is_slider))
            if _is_slider_code:
                logger.info(f"Gamepad slider-axis code={code} raw={value} "
                            f"norm={norm:.3f} min={min_val:.0f} max={max_val:.0f} "
                            f"rest={rest_pos:.0f} pseudo_uni={pseudo_unipolar}")

            prev = self._axes.get(name, 0.0)
            if abs(norm - prev) > 0.005 or (norm == 0.0 and prev != 0.0):
                self._axes[name] = norm
                self._put(GamepadEvent(name, norm, value))
            return

        elif code == ABS_HAT0X:
            v = -1 if value < 0 else (1 if value > 0 else 0)
            self._put(GamepadEvent("dpad_x", v, value))
        elif code == ABS_HAT0Y:
            # Standard Linux ABS_HAT0Y convention: -1 = physical UP, +1 = physical DOWN.
            # The Compx 2.4G dongle follows this convention.
            # Pass through directly so dpad_y=-1 means UP everywhere.
            # app.py negates when computing tilt_n so UP → positive tilt direction.
            v = -1 if value < 0 else (1 if value > 0 else 0)
            logger.info(f"Gamepad: ABS_HAT0Y raw={value} → dpad_y={v}")
            self._put(GamepadEvent("dpad_y", v, value))

    def _handle_key(self, code: int, pressed: bool):
        btn_map = {
            # 8BitDo Pro 2 physical layout uses Nintendo-style ABXY.
            # BTN_SOUTH (304) = physical B (XInput A) → return to start / cancel
            # BTN_EAST  (305) = physical A (XInput B) → shutter trigger / AUX fire
            BTN_SOUTH:  "btn_return",   # physical B — return to start
            BTN_EAST:   "btn_shutter",  # physical A — shutter / AUX trigger
            BTN_NORTH:  "btn_keyframe", # physical X — add keyframe  (BTN_NORTH=308 = XInput Y but 8BitDo X)
            BTN_WEST:   "btn_arctan",   # physical Y — arctan toggle  (BTN_WEST=307 = XInput X but 8BitDo Y)
            BTN_TL:     "btn_l1",       # L1 shoulder — slider nudge backward
            BTN_TR:     "btn_r1",       # R1 shoulder — slider nudge forward
            BTN_START:  "btn_play",
            BTN_SELECT: "btn_stop",
            BTN_MODE:   "btn_record",   # ★ Star → video record toggle
            BTN_THUMBL: "btn_origin",
            BTN_THUMBR: "btn_stop_r",
        }
        if code in btn_map:
            name = btn_map[code]
            logger.info(f"Gamepad: KEY code={code} → {name} pressed={pressed}")
            if self._buttons.get(name) != pressed:
                self._buttons[name] = pressed
                self._put(GamepadEvent(name, pressed, int(pressed)))
        else:
            # Log unmapped keycodes so we can identify unknown buttons (e.g. star button,
            # shoulder buttons) and add them to btn_map if needed.
            logger.info(f"Gamepad: unknown KEY code={code} pressed={pressed}")

    def _put(self, event: GamepadEvent):
        """Non-blocking put — drop if queue is full (stale input)."""
        try:
            self._queue.put_nowait(event)
        except asyncio.QueueFull:
            pass
