# Stereo 3D Macro Scanning — Complete Feature Guide

## Overview

Capture stereo pairs (left/right eye images) at each position in your macro scan sequence. Perfect for creating:
- **VR 360° videos** (stereoscopic 360)
- **Side-by-side 3D videos** (TV/cinema format)
- **Anaglyph 3D** (red/cyan glasses)
- **Cross-eye/parallel viewing** (no equipment needed)
- **LightField reconstruction** (computational 3D)

---

## How It Works

### Normal Scan
```
Stack 1 at pan=0°   → focus stack (50 images)
Stack 2 at pan=15°  → focus stack (50 images)
Stack 3 at pan=30°  → focus stack (50 images)
Total: 3 stacks × 50 images = 150 images
```

### Stereo Scan (offset=3°)
```
Stack 1a at pan=0°   → LEFT  focus stack (50 images)
Stack 1b at pan=3°   → RIGHT focus stack (50 images)
Stack 2a at pan=15°  → LEFT  focus stack (50 images)
Stack 2b at pan=18°  → RIGHT focus stack (50 images)
Stack 3a at pan=30°  → LEFT  focus stack (50 images)
Stack 3b at pan=33°  → RIGHT focus stack (50 images)
Total: 6 stacks × 50 images = 300 images (2× without stereo)
```

**Key:** Right eye captures from pan+offset position. Both eyes shoot full focus stacks.

---

## Parameters

### stereo_enabled: boolean
- Default: `false`
- Enables stereo pair capture mode
- When enabled, total images double

### stereo_offset_deg: float
- Default: `3.0` degrees
- Pan offset between left and right eye viewpoints
- Range: `0.5°` to `10°` (typical: 2–5°)
- **Lower offset** = easier to fuse, less 3D pop
- **Higher offset** = more pronounced 3D depth, but harder to fuse

### How to Calculate Offset

**Physical baseline method:**
```
Desired baseline (mm) = IPD × magnification
  (typical IPD = 65mm for humans)

stereo_offset_deg = atan2(baseline_mm, working_distance_mm) × 180/π

Example:
  - Working distance: 70mm (macro close-up)
  - IPD: 65mm
  - Magnification: 1.0× (life-size)
  - baseline = 65mm (like human eyes)
  - offset = atan2(65, 70) × 180/π ≈ 42.7°

  - Working distance: 300mm (longer distance)
  - baseline: 65mm
  - offset = atan2(65, 300) × 180/π ≈ 12.2°

  - Working distance: 50mm, magnification 5×
  - baseline: 65 × 5 = 325mm (exaggerated for small macro)
  - offset = atan2(325, 50) × 180/π ≈ 81.2° (!)
```

**Practical recommendations:**
- Small specimen (macro close-up): **3–5°** (easier to fuse, still 3D)
- Medium distance (50–100mm): **2–4°**
- Large object (>200mm): **1–2°**
- Exaggerated 3D (special effects): **6–10°** (requires viewer effort)

---

## Output Structure

With stereo enabled, images organized as:

```
project_folder/
├── orbit_001/
│   ├── stack_001/
│   │   ├── slot_A/
│   │   │   ├── left/
│   │   │   │   ├── frame_0000.jpg
│   │   │   │   ├── frame_0001.jpg
│   │   │   │   └── ...
│   │   │   └── right/
│   │   │       ├── frame_0000.jpg
│   │   │       ├── frame_0001.jpg
│   │   │       └── ...
│   │   └── (other slots...)
│   ├── stack_002/ (LEFT eye position)
│   │   └── ... (L/R subfolders)
│   ├── stack_003/ (RIGHT eye position, offset pan)
│   │   └── ... (L/R subfolders)
│   └── ...
├── sequence.json (records stereo metadata)
└── project.json (stereo_enabled: true, stereo_offset_deg: 3.0)
```

### Metadata in sequence.json
```json
{
  "stacks": [
    {
      "stack_id": "stack_001",
      "pan_deg": 0.0,
      "tilt_deg": 0.0,
      "eye": "left",
      "folder": "stack_001",
      "completed": true,
      "frames": [...]
    },
    {
      "stack_id": "stack_002",
      "pan_deg": 3.0,
      "tilt_deg": 0.0,
      "eye": "right",
      "folder": "stack_002",
      "completed": true,
      "frames": [...]
    },
    ...
  ]
}
```

---

## Post-Processing Workflow

### 1. Batch Focus Stack (Existing)
Run `batch_focus_stack.py` as usual — it will:
- Stack frames for LEFT eye
- Stack frames for RIGHT eye
- Output: `orbit_001/stack_001/slot_A/left/best_focus.jpg` and `.../right/best_focus.jpg`

### 2. Merge Stereo Pairs into Video

**Option A: Side-by-side (simplest)**
```bash
# Concatenate left and right side-by-side for each frame
ffmpeg -i left_%04d.jpg -i right_%04d.jpg -filter_complex \
  "hstack" -c:v libx264 output_sbs.mp4
```

**Option B: Anaglyph (red/cyan glasses)**
```bash
ffmpeg -i left_%04d.jpg -i right_%04d.jpg -filter_complex \
  "colorchannelmixer=rr=1:gb=0:bb=0.5" output_anaglyph.mp4
```

**Option C: Stereo 360 VR (for VR headsets)**
- Use ffmpeg with `stereo3d` filter or commercial VR authoring tools
- Export as equirectangular stereo (over/under or left/right)
- Load into YouTube/Vimeo with stereo 3D support

### 3. COLMAP Reconstruction (Advanced)

For dense 3D reconstruction using stereo pairs:
1. Run `macro_compute_merged_colmap()` on stereo data
2. COLMAP sees stereo pairs as additional viewpoints
3. Result: Denser point cloud from stereo baseline
4. Can use for Gaussian splatting or mesh generation

---

## UI Changes Required

### Stereo Controls
```
[Checkbox] Stereo 3D Mode ☑
  Stereo Offset: [3.0 °] ← slider 0.5...10.0
  [?] Help: Calculate offset...
  
  Preview: "36 stacks → 72 stacks (left/right pairs)"
```

### Impact on Calculations
- Total stacks: **×2** when stereo enabled
- Total images: **×2**
- Storage: **×2**
- Time: **~2×** (plus repositioning pan between pairs)

### Example UI Update
```
Without Stereo:
  Scan: Orbit, 36 stacks
  Total images: 1,800
  Storage: 5.4 GB
  Time: ~40 min

With Stereo (3°):
  Scan: Orbit, 36 pairs (72 stacks)
  Total images: 3,600
  Storage: 10.8 GB
  Time: ~85 min (motor must move offset twice per position)
```

---

## WebSocket Commands

### Start Stereo Scan
```json
{
  "cmd": "macro_start",
  "scan_type": "orbit",
  "num_stacks": 36,
  "stereo_enabled": true,
  "stereo_offset_deg": 3.0,
  "rotation_easing": "parabolic",
  "rail_start_mm": 0,
  "rail_end_mm": 5,
  ...rest of params...
}
```

### Start Stereo Grid 2D
```json
{
  "cmd": "macro_start",
  "scan_type": "grid_2d",
  "pan_cols": 3,
  "tilt_rows": 2,
  "stereo_enabled": true,
  "stereo_offset_deg": 4.0,
  ...
}
```

### Calculation with Stereo
```json
{
  "cmd": "macro_calc",
  "stereo_enabled": true,
  "stereo_offset_deg": 3.0,
  ...
}
```

Response includes stereo multiplier:
```json
{
  "type": "macro_calc",
  "total_stacks": 72,  // 36 pairs × 2
  "total_images": 3600,  // 1800 × 2
  "storage_gb": 10.8,  // 5.4 × 2
  "stereo_pairs": 36,
  "stereo_offset_deg": 3.0
}
```

---

## Implementation Notes

### Backend
✓ `stereo_enabled` and `stereo_offset_deg` added to MacroSession
✓ `stereo_multiplier()` function for calculation multipliers
✓ `generate_scan_positions()` function generates (pan, tilt, eye) tuples
✓ `total_image_count()` accounts for stereo doubling
✓ `num_stacks_grid()` accounts for stereo doubling
✓ Parameters passed through `_build_macro_session()`

### Frontend (TODO)
- [ ] Stereo checkbox in macro UI
- [ ] Offset slider (0.5°–10°)
- [ ] "Calculate offset" helper dialog
- [ ] Updated image/storage estimates showing ×2 multiplier
- [ ] L/R folder structure display

### MacroEngine Changes
- [ ] `_run_sequence_orbit()` iterates through `generate_scan_positions()` 
- [ ] `_run_sequence_grid()` iterates through `generate_scan_positions()`
- [ ] Update `sequence.json` to record "eye" field per stack
- [ ] Update `project.json` to store stereo_enabled and stereo_offset_deg

---

## Advantages

✓ **2× baseline**: Captures stereo from separated viewpoints (left eye & right eye separated by offset)
✓ **Full focus stacks**: Both eyes get complete depth stacking (unlike traditional stereo where each only gets one frame)
✓ **Cinematic quality**: Can generate smooth 3D videos with easing curves
✓ **Backward compatible**: Non-stereo scans work exactly as before
✓ **Flexible post-processing**: Output structure supports multiple stereo formats (SBS, anaglyph, VR, etc.)

---

## Limitations & Tradeoffs

⚠ **2× time and storage** — each scan doubles duration and disk space
⚠ **Pan motion overhead** — must move pan motor twice per position (once for L, once for R)
⚠ **Offset must be calibrated** — wrong offset makes stereoscopy uncomfortable or ineffective
⚠ **Tilt offset not supported** (yet) — only pan offset; adding tilt would require 4 positions per scan point
⚠ **COLMAP fusion needed** — for 3DGS reconstruction, both L/R point clouds must be aligned post-hoc

---

## Test Scenario

**Simple Stereo Orbit Test:**
1. Setup: coin on rotating base
2. Settings:
   - Orbit, 8 stacks (base)
   - Rail: 0...1 mm, 50 μm steps
   - Stereo enabled, 3° offset
   - Easing: parabolic
3. Expected result:
   - 16 stacks total (8 left + 8 right)
   - 800 images (40 frames/stack × 20 stacks)
   - ~10 minutes execution
4. Verify:
   - Left and right images exist in separate folders
   - sequence.json records "eye" field
   - Can be stacked and merged into 3D video

