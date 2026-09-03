# Macro Mode UI Specification — 2026-05-20

## Available Easing Curves (16 options)

linear, inverted_linear, even, parabolic, inverted_parabolic, 
gaussian, inverted_gaussian, catenary, inverted_catenary, 
ellipsoidal, inverted_ellipsoidal, cycloid, inverted_cycloid, 
lame, inverted_lame

**Use cases:**
- Even spacing (default): `even`
- Cinematic slow pan: `parabolic` (slow start/fast middle/slow end)
- Dense at equator: `gaussian` (bell curve)
- Smooth circular motion: `cycloid`

---

## Key Macro Mode Features

1. **Orbit Mode** (single rotation axis, full focus stacks)
   - User specifies: total_stacks, rotation_easing
   - App computes: optimal pan_cols × tilt_rows distribution
   - Result: Even surface area coverage on sphere
   - Easing: Controls velocity profile (slow/fast/slow for cinematic)

2. **Grid 2D Mode** (pan × tilt grid with snake traversal)
   - User specifies: pan_cols, tilt_rows (or total stacks)
   - Result: Flat grid of viewpoints
   - Snake mode: minimize motor travel between rows

3. **Programmed Path** (future feature)
   - User specifies: waypoint list (pan, tilt, optional rail)
   - Each waypoint triggers a full focus stack
   - Pan & tilt easing control interpolation between waypoints
   - Result: Cinematic scan along custom path

---

## UI Controls Required

RAIL SECTION:
  - Rail Start: [0.0 mm] slider → [5.0 mm]
  - Rail Step: [100.0 μm] spinner
  - Calculated: Frames per stack, Total images

PAN/TILT SECTION:
  - Pan Range: [-90°] slider [+90°]
  - Pan Full Rotation: checkbox → [-180°...+180°]
  - Tilt Range: [-30°] slider [+30°]
  - Tilt Full Rotation: checkbox → [-180°...+180°]

SCAN MODE:
  - Mode selector: Orbit / Grid 2D / Programmed

  [If Orbit]
    - Total Stacks: [36] spinner
    - [Compute Grid] button → shows "4 cols × 3 rows = 12 stacks"
    - Rotation Easing: dropdown [even ▼]

  [If Grid 2D]
    - Pan Columns: [4] spinner
    - Tilt Rows: [3] spinner
    - Snake Traversal: checkbox (checked)
    - Total Stacks: calculated

LENS & CAPTURE:
  - Lens Profile: dropdown [50mm Macro ▼] [Manage]
  - Exposure Slot: dropdown [Slot A ▼] [Edit]

PREVIEW:
  - Wireframe sphere showing stack positions
  - Density heatmap (darker = more stacks)

CALCULATION:
  - Frames/Stack, Total Images, Storage GB, Est. Time

BUTTONS:
  - [Start Scan] [Cancel] [Save Preset] [Load Preset] [Batch Stack]

---

## WebSocket Commands

### macro_get_easing_curves
Get list of available easing functions
Request: {"cmd": "macro_get_easing_curves"}
Response: {"type": "macro_easing_curves", "curves": [...]}

### macro_compute_grid
Compute optimal pan_cols × tilt_rows for total stacks count
Request: {
  "cmd": "macro_compute_grid",
  "total_stacks": 36,
  "pan_min": -90, "pan_max": 90,
  "tilt_min": -30, "tilt_max": 30
}
Response: {
  "type": "macro_grid_computed",
  "pan_cols": 4, "tilt_rows": 3,
  "total_actual": 12
}

### macro_calc
Calculate frames, storage, time estimates
Request: {"cmd": "macro_calc", ...all session params...}
Response: {
  "type": "macro_calc",
  "frames_per_stack": 50,
  "total_stacks": 36,
  "total_images": 1800,
  "storage_gb": 5.4
}

### macro_start
Start orbit/grid/programmed scan
Request: {
  "cmd": "macro_start",
  "scan_type": "orbit",
  "rotation_easing": "parabolic",
  ...all rail, pan, tilt, lens params...
}

### macro_allow_full_rotation
Enable ±180° rotation
Request: {
  "cmd": "macro_allow_full_rotation",
  "axis": "pan|tilt|both",
  "enable": true
}

