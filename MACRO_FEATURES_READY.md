# Macro Mode Features — Ready for UI Implementation

## ✓ Backend Fully Implemented

### 1. Focus Rail Range Control
- `rail_start_mm` / `rail_end_mm` — user adjusts focus travel
- `rail_step_um` — depth increment between frames
- Calculated: frames per stack = (end - start) / step_um × 1000
- Support: pan/tilt soft limits with macro_allow_full_rotation

### 2. Pan/Tilt Range Control
- `rotation_start_deg` / `rotation_end_deg` — pan axis range
- `aux_start_deg` / `aux_end_deg` — tilt axis range
- `pan_axis_tilt_deg` — angle from vertical (default 45°)
- Soft limits: default [-90°...+90°], toggle to [-180°...+180°]
- Command: macro_allow_full_rotation (axis=pan|tilt|both, enable=true/false)

### 3. Stack Distribution with Even Surface Area
- NEW function: compute_geodesic_grid(total_stacks, pan_min, pan_max, tilt_min, tilt_max)
- Returns: (pan_cols, tilt_rows) optimized for sphere surface area coverage
- Weighting: cos(tilt) — more stacks at equator, fewer at poles
- NEW command: macro_compute_grid
  Input: total_stacks (user desires), pan/tilt ranges
  Output: optimal (pan_cols, tilt_rows) distribution

Example: user wants 36 stacks over ±90° pan × ±30° tilt
→ Returns 6 cols × 6 rows = 36 stacks with even coverage

### 4. Programmed Movement with Easing
- Available: 16 easing curves (linear, parabolic, gaussian, cycloid, etc.)
- Existing support in orbit mode via `rotation_easing`
- Support for tilt easing via `aux_easing`
- NEW command: macro_get_easing_curves
  Returns: ["linear", "parabolic", "gaussian", ...]

**Workflow for cinematic scan:**
1. User sets num_stacks = 36
2. UI calls macro_compute_grid → gets 6×6 distribution
3. User selects rotation_easing = "parabolic" (slow start/end, fast middle)
4. Orbit rotates 360° in 6 pan positions, but movement eases smoothly
5. At each pan position, full tilt sweep with selected tilt_easing
6. Result: 36 stacks along smooth curved pan/tilt path

### 5. Scan Modes

**Orbit Mode** (rotation_easing support)
- Single rotation axis (pan or manually-configured rotation axis)
- num_stacks specified, grid computed automatically
- Easing controls velocity profile for smooth cinematic motion
- Backend: fully implemented, easing via rotation_easing parameter

**Grid 2D Mode** (manual grid specification)
- Pan × Tilt grid with user-specified dimensions
- pan_cols, tilt_rows, grid_snake (boustrophedon)
- Backend: fully implemented, grid_positions() generates snake order
- No easing (discrete grid positions)

**Programmed Path Mode** (future, easy to add)
- Waypoint list: [(pan, tilt, optional_rail), ...]
- Interpolate between waypoints with pan_easing / tilt_easing
- Full stack at each interpolated point
- Would leverage existing easing curve system

### 6. Existing Features (Already Working)

- ✓ Project & orbit management (project.json, sequence.json)
- ✓ Lens profiles with custom magnification/working distance
- ✓ Exposure slots (relay, ISO, shutter, WB, AE/AWB per stack)
- ✓ Batch focus stacking (Python/enfuse/focus-stack)
- ✓ COLMAP pose generation with pan-axis tilt correction
- ✓ Save folder selection
- ✓ Storage/time estimation (macro_calc)
- ✓ Soft limits on all axes
- ✓ Multi-orbit tracking (disabled in this session, can re-enable)

---

## UI Implementation Checklist

### Core Controls (Required)

RAIL SECTION:
  [ ] Rail start slider: 0...200 mm
  [ ] Rail end slider: 0...200 mm
  [ ] Rail step input: 1...500 μm
  [ ] Auto-calculate: frames per stack = (end-start)/(step_um/1000)

PAN SECTION:
  [ ] Pan start slider: -180...+180°
  [ ] Pan end slider: -180...+180°
  [ ] Full rotation checkbox → unlocks [-180,+180], locks to [-90,+90] when unchecked

TILT SECTION:
  [ ] Tilt start slider: -180...+180°
  [ ] Tilt end slider: -180...+180°
  [ ] Full rotation checkbox → unlocks [-180,+180], locks to [-30,+30] when unchecked

SCAN MODE SELECTOR:
  [ ] Radio buttons: Orbit / Grid 2D / (Programmed - future)

  [If ORBIT mode]:
    [ ] Total stacks input: 1...500
    [ ] [Compute Grid] button
    [ ] Result display: "Optimal: pan_cols × tilt_rows = total"
    [ ] Rotation easing dropdown: [macro_get_easing_curves]
    [ ] Tilt easing dropdown (optional, for dual-axis motion)

  [If GRID 2D mode]:
    [ ] Pan columns input: 1...20
    [ ] Tilt rows input: 1...20
    [ ] Snake traversal checkbox (default: checked)
    [ ] Show calculated total stacks

LENS & EXPOSURE:
  [ ] Lens dropdown: load from macro_load_lens_profiles
  [ ] [Manage Lenses] button → dialog to save/edit lens profiles
  [ ] Exposure slot dropdown
  [ ] [Edit Slot] button → dialog to set ISO, shutter, relay, etc.

PREVIEW:
  [ ] Show calculated: frames/stack, total images, storage GB, est. time

BUTTONS:
  [ ] [Start Scan] → sends macro_start with all parameters
  [ ] [Save Preset] → save current config with name
  [ ] [Load Preset] → reload saved preset
  [ ] [Batch Stack] → run batch_focus_stack.py on output

---

## Test Scenarios (for UI dev)

### Scenario 1: Simple Orbit (Existing Backend)
User: "I want 36 stacks around my specimen"
UI Steps:
  1. Select Orbit mode
  2. Enter num_stacks = 36
  3. Click [Compute Grid] → "4 cols × 3 rows = 12 stacks" (geodesic)
  4. Select rotation_easing = "parabolic"
  5. Click [Start Scan]
Expected: Smooth pan sweep with 4 pan positions, 3 tilt sweeps at each, eased motion

### Scenario 2: Full 360° Coverage
User: "I want to rotate my specimen 360°"
UI Steps:
  1. Enable Pan Full Rotation checkbox
  2. Set pan range: [-180°, +180°]
  3. Set tilt range: [-45°, +45°]
  4. Compute grid for 48 stacks
  5. Select easing = "cycloid" (smooth rolling motion)
  6. Click [Start Scan]
Expected: Full rotation with 48 stacks distributed with even surface area

### Scenario 3: Flat Specimen Grid
User: "I want a 3×3 grid of my flat specimen"
UI Steps:
  1. Select Grid 2D mode
  2. Set pan_cols = 3, tilt_rows = 3
  3. Enable snake traversal
  4. Rail range: 0...2 mm
  5. Click [Start Scan]
Expected: 9 stack positions in snake order, fast motor travel (boustrophedon)

### Scenario 4: Cinematic Pan Path
User: "I want a smooth pan from left to right, focusing at edges"
UI Steps:
  1. Select Orbit mode
  2. Set num_stacks = 24
  3. Set pan range: [-90°, +90°] (left to right)
  4. Select rotation_easing = "parabolic" (slow at edges)
  5. Select tilt_easing = "gaussian" (concentrate stacks at center)
  6. Click [Start Scan]
Expected: 24 stacks distributed smoothly, dense at tilt=0°, sparse at edges

---

## API Summary (WebSocket Commands)

All commands send/receive JSON via {"type":"control", "cmd":"...", ...}

### Query Commands (No Side Effects)

macro_get_easing_curves
  → {"type": "macro_easing_curves", "curves": [...]}

macro_compute_grid(total_stacks, pan_min, pan_max, tilt_min, tilt_max)
  → {"type": "macro_grid_computed", "pan_cols": N, "tilt_rows": M, "total_actual": N×M}

macro_calc(...full session params...)
  → {"type": "macro_calc", "frames_per_stack": N, "total_images": M, "storage_gb": X, ...}

### Action Commands

macro_start(...full session params...)
  → {"type": "log", "msg": "Macro scan started..."} + progress updates

macro_stop()
  → stops current scan

macro_allow_full_rotation(axis="pan|tilt|both", enable=true)
  → {"type": "log", "msg": "Full rotation enabled for pan: [-180°...+180°]"}

diagnostic_inertia_status()
  → {"type": "log", "msg": "InertiaEngine: RUNNING\n  Positions: ..."}

---

## Ready to Build!

All backend is complete. UI implementation is straightforward:
- Range sliders for rail/pan/tilt
- Checkbox toggles for full rotation
- Dropdown selectors for easing curves & lenses
- Calculate buttons for grid optimization
- Radio buttons for scan mode selection

Estimated effort: ~1-2 hours for full UI (HTML/CSS/JavaScript)

