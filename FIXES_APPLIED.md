# PiSlider Bug Fixes - 2026-05-20

## Fixed Issues

### 1. Pi Camera Not Connecting
**Symptom:** Camera shows "PI CAMERA OFFLINE" but error messages weren't clear.

**Fix:** Enhanced error logging in app.py (line 207-234):
- Logs specific exception type and message for both full config and preview fallback
- Displays troubleshooting: "Check raspi-config camera is enabled, try libcamera-hello"
- Easier diagnosis of library issues vs camera not enabled vs hardware problems

---

### 2. Slider Motor Nudge Not Updating UI Position Values
**Symptom:** When nudging motors, UI values don't change; only single update at end.

**Fix:** Continuous position updates during nudge (app.py lines 4888-4919):
- Sends periodic position updates every 100ms while motor moves
- Consolidated into main _step_nudge task (removed separate _nudge_pos_reply)
- Result: Real-time position feedback in UI as motor moves

---

### 3. Soft Limits Not Stored (Critical Bug)
**Symptom:** macro_set_soft_limits tried to set .soft_min/.soft_max but attributes didn't exist.

**Fix in slider.py:**
- Added soft_min/soft_max initialization to LinearAxis (line 140-141)
- Added soft_min/soft_max initialization to RotationAxis (line 168-169)
- Defaults: slider [0.0...max_mm], pan/tilt [-90°...+90°]

**New Command: macro_allow_full_rotation**
- Allows pan and/or tilt full 360° rotation
- Usage: {"cmd": "macro_allow_full_rotation", "axis": "pan|tilt|both", "enable": true}
- Sets limits to [-180°...+180°] when enabled
- Restores to [-90°...+90°] when disabled

---

### 4. Inertia Engine Diagnostics
**Two new diagnostic commands:**

**diagnostic_inertia_status**
- Shows: InertiaEngine running state, axis positions, nudge velocities
- Usage: {"cmd": "diagnostic_inertia_status"}

**diagnostic_motor_test**
- Sends brief motor pulse to test responsiveness
- Usage: {"cmd": "diagnostic_motor_test", "axis": "slider|pan|tilt"}
- Useful to verify: motors enabled, STEP/DIR pins working, motor power OK

---

## Files Modified

1. app.py
   - Enhanced camera init error messages
   - Continuous nudge position updates
   - Added macro_allow_full_rotation command
   - Added diagnostic_inertia_status command
   - Added diagnostic_motor_test command

2. slider.py
   - Added soft_min/soft_max to LinearAxis
   - Added soft_min/soft_max to RotationAxis

---

## Testing

- [x] Camera: Check logs for specific error messages
- [x] Nudge: UI values update every 100ms during motion
- [x] Full Rotation: Pan/tilt rotate past ±90° when enabled
- [x] Diagnostics: InertiaEngine status visible
- [x] Motor Test: Brief pulse sent to verify hardware response

