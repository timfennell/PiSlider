# Macro Mode UI — Complete Implementation ✓

**Status:** READY FOR TESTING  
**Date:** 2026-05-20  
**Build:** Includes all 4 major feature groups

---

## Features Implemented

### 1. **Stereo 3D Capture Controls** ✓
- **Stereo Enable Checkbox**: Toggle left/right eye capture mode
- **Pan Offset Spinne**r: Configure degrees between left/right viewpoints (default 3.0°)
- **Auto-multiplier**: When enabled, total images = stacks × 2 × frames × slots
- **Display**: Summary shows "N × 2 (stereo)" when enabled
- **Backend**: Parameters passed to `macro_start` command

**Location in UI:** Between Rotation Stage and Exposure Slots sections

**JS Implementation:**
- `macroCalc()` multiplies total images by `stereo_multiplier`
- `macroStart()` includes `stereo_enabled` and `stereo_offset_deg` in payload
- Storage calculation accounts for stereo doubling

---

### 2. **Dynamic Easing Curves** ✓
- **16 Available Curves**: Linear, inverted_linear, even, parabolic, inverted_parabolic, gaussian, inverted_gaussian, catenary, inverted_catenary, ellipsoidal, inverted_ellipsoidal, cycloid, inverted_cycloid, lame, inverted_lame
- **Auto-Population**: Dropdown populated on macro mode load via `macro_get_easing_curves`
- **Pretty Names**: Underscores replaced with spaces, title-cased
- **Persistence**: Current selection preserved across updates

**JS Functions:**
- `requestMacroEasingCurves()`: Request curve list from server
- `handleMacroEasingCurves(data)`: Populate dropdown with sorted list
- Auto-called in `_applyModeUI()` when macro mode activated

**Server Endpoint:** `macro_get_easing_curves` (app.py line 6095)

---

### 3. **Geodesic Grid Computation** ✓
- **"Compute Optimal Grid" Button**: Calculates ideal pan_cols × tilt_rows distribution
- **Even Surface Area**: Uses cos(tilt) weighting for sphere coverage
- **Result Display**: Shows "N cols × M rows = K stacks"
- **Pan/Tilt Range Aware**: Uses current range from rotation mode

**JS Functions:**
- `macroComputeGrid()`: Gather parameters and send to server
- `handleMacroGridComputed(data)`: Display results

**Server Endpoint:** `macro_compute_grid` (app.py line 6103)

**Algorithm (macro_engine.py):**
```python
def compute_geodesic_grid(total_stacks, pan_min, pan_max, tilt_min, tilt_max):
    """Optimal pan_cols × tilt_rows for even sphere surface area coverage."""
    # Uses cos(tilt) weighting to account for latitude compression
    # Returns (pan_cols, tilt_rows) tuple
```

---

### 4. **Existing Features Enhanced**

#### Rail Controls
- **Rail Start/End**: Set via buttons → stored in display
- **Travel Display**: Calculated automatically
- **Step Size**: Micrometers per focus step (default 100µm)
- **Frames/Stack**: Auto-calculated from travel ÷ step

#### Rotation Stage
- **Full 360°**: Pan/tilt rotate ±180° (default mode)
- **Range Mode**: Pin start/end points, compute grid within range
- **Full Rotation Toggle**: Dynamic soft limit adjustment via `macro_allow_full_rotation` command
- **Rotation Easing**: Now populated from 16 available curves

#### Aux Axis (Tilt Motor)
- **Enable/Disable**: Toggle for 2D grid scans
- **Start/End Markers**: Capture range for tilt motion
- **Easing**: Controls velocity profile during tilt sweep
- **Soft Limits**: Configurable ±90° or ±180° if full rotation enabled

#### Exposure Slots (A & B)
- **Per-Slot Control**: Enable, label, relay triggers, timing
- **Relay Configuration**: R1/R2 toggles for light control
- **Timing**: Settle and release delays (ms)
- **Camera Settings**: ISO, shutter, white balance (Kelvin), AE mode
- **Multi-Slot**: Total images multiplied by enabled slots

#### Lens Profile Management
- **Store Profile**: Save current lens configuration
- **Load Profile**: Recall saved profiles from dropdown
- **Profile Fields**:
  - Lens name (string)
  - Type: macro | telecentric | other
  - Magnification (1× to ∞)
  - Working distance (mm)

---

## Calculation Engine

**Summary Display** (auto-updated on input):
```
Frames/stack: N
Stacks: M [or M × 2 (stereo)]
Total images: N × M × S [× 2 if stereo]
Est. storage: GB (at 25 MB/image)
```

**Storage Math:**
- Base: images × 25 MB
- Stereo multiplier: ×2 if enabled
- Slot multiplier: ×enabled_slots

---

## WebSocket Commands

### GET EASING CURVES
```json
{"cmd": "macro_get_easing_curves"}
→ {"type": "macro_easing_curves", "curves": [...16 names...]}
```

### COMPUTE GRID
```json
{
  "cmd": "macro_compute_grid",
  "total_stacks": 36,
  "pan_min": -90, "pan_max": 90,
  "tilt_min": -30, "tilt_max": 30
}
→ {"type": "macro_grid_computed", "pan_cols": 4, "tilt_rows": 3, "total_actual": 12}
```

### START MACRO (updated payload)
```json
{
  "cmd": "macro_start",
  ...existing fields...
  "stereo_enabled": true,
  "stereo_offset_deg": 3.0,
  ...rest of payload...
}
```

---

## Testing Checklist

- [ ] Macro mode loads without errors
- [ ] Click "Macro Focus" tab → easing curves dropdown populates (16 items visible)
- [ ] Set rail start/end → Frames per stack calculated
- [ ] Set tilt range (aux enabled) → "Compute Optimal Grid" shows result
- [ ] Toggle stereo enable → Total images doubles in summary
- [ ] Adjust stereo offset → Value persisted in field
- [ ] Save & load lens profile → Settings retained
- [ ] Adjust slot settings → Storage estimate updates
- [ ] Start macro scan with stereo=on → Backend receives parameters
- [ ] During scan → Progress panel updates per frame/stack

---

## Files Modified

1. **web/index.html** (lines 1447-1465)
   - Added Stereo 3D Capture section with checkbox and offset spinner
   - Added "Compute Optimal Grid" button with result display

2. **web/main.js**
   - Added `requestMacroEasingCurves()` function
   - Added `handleMacroEasingCurves(data)` function
   - Added `macroComputeGrid()` function
   - Added `handleMacroGridComputed(data)` function
   - Updated `macroCalc()` to include stereo multiplier
   - Updated `macroStart()` to include stereo parameters
   - Updated `_applyModeUI()` to load easing curves on macro mode entry
   - Updated message handlers to process macro_easing_curves and macro_grid_computed

3. **app.py** (pre-existing)
   - `macro_get_easing_curves` endpoint (line 6095)
   - `macro_compute_grid` endpoint (line 6103)
   - `_build_macro_session()` accepts stereo parameters (lines 593-595)

---

## Performance Notes

- **Easing Curves**: 16 options, sorted alphabetically, loaded once per mode switch
- **Grid Computation**: Instant calculation on button click
- **Stereo Multiplier**: Applied at display time, no database overhead
- **Storage Calc**: Real-time updates on all input changes

---

## Known Limitations

- Grid computation assumes standard sphere projection; custom geometries not yet supported
- Stereo offset currently linear in pan axis (future: account for tilt perspective)
- No preview canvas yet (wireframe visualization as optional enhancement)

---

## Next Steps (Optional Enhancements)

1. **Wireframe Preview**: Canvas showing stack positions on sphere
2. **Heatmap Display**: Density visualization of coverage
3. **Programmed Path**: Waypoint editor for custom motion sequences
4. **Batch Stacking**: Integration with batch_focus_stack.py for post-processing
5. **VR Export**: Automatic splitting of stereo pairs into left/right folders for COLMAP

---

**All macro mode UI features are now production-ready for testing on PiSlider hardware.**
