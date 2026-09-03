# Stereo 3D Capture — Physical Setup Requirements

**Status:** Configuration Guide  
**Date:** 2026-05-20  
**Priority:** IMPORTANT (read before stereo scanning)

---

## Critical: Stereo Requires Specific Axis Orientation

### Optimal Stereo Configuration
For stereo 3D pairs to be useful for binocular viewing and reconstruction:

```
From Camera View:
        UP (Tilt Axis - horizontal)
        |
LEFT ---|--- RIGHT (Pan Axis - vertical)
        |
       DOWN

Pan Axis:    Vertical (rotates left-right from camera perspective)
Tilt Axis:   Horizontal (rotates up-down from camera perspective)
Camera:      Looking straight at specimen
Stereo:      Pan offset (3° each side) = binocular baseline
```

**Why This Matters:**
- Stereo baseline (inter-ocular distance) must be **perpendicular to camera view**
- Pan motion creates proper eye convergence
- Tilt motion would create keystone distortion (not useful for 3D)
- COLMAP and stereo reconstruction assume standard stereo geometry

---

## Your Coverage Preference vs. Stereo

### Current Setup (Pan tilted 45° toward camera)
**Advantage:** Better specimen coverage
- Avoids "poles" of sphere (top/bottom compression)
- More uniform lighting angles
- Better for macro surface detail

**Disadvantage:** Breaks stereo geometry
- Pan axis is no longer vertical
- Introduces twist into stereo pairs
- COLMAP reconstruction ambiguity
- Post-processing harder to align

---

## Solutions

### Option 1: Two Scanning Modes (Recommended)

**Standard Coverage Mode (Current Setup)**
```json
{
  "pan_axis_tilt_deg": 45,  // Tilted toward camera
  "stereo_enabled": false,   // Mono capture for coverage
  "num_stacks": 72,          // Higher density for surface detail
  "scan_type": "orbit"
}
```
- Use for detailed surface scanning
- Post-process with focus stacking only
- No 3D reconstruction (use NeRF or photogrammetry)

**Stereo 3D Mode (Axis Reconfigured)**
```json
{
  "pan_axis_tilt_deg": 90,   // Vertical (reset hardware)
  "stereo_enabled": true,
  "stereo_offset_deg": 3.0,
  "num_stacks": 36,          // Standard for stereo
  "scan_type": "orbit"
}
```
- Reconfigure mechanical setup (straighten pan axis)
- Use for 3D reconstruction (COLMAP, Brush, etc.)
- Output: stereo pairs for VR/3D video

**In UI:** Add mode selector or document in project notes

---

### Option 2: Hybrid Approach

Capture **both** simultaneously (if hardware allows):
```json
{
  "pan_axis_tilt_deg": 45,   // Current setup
  "scan_type": "grid_2d",
  "pan_cols": 8,
  "tilt_rows": 4,            // Wide coverage
  "stereo_enabled": true,    // Also capture stereo pairs
  "stereo_offset_deg": 3.0,
  "num_stacks": 32           // 8 cols × 4 rows
}
```

**Result:**
- Folder: `standard/` — all frames at fixed axis angle (45°)
- Folder: `stereo_left/` — left eye frames
- Folder: `stereo_right/` — right eye frames

**Trade-off:**
- Hardware must support dual-output (if mechanical constraint)
- Or: Run two separate scans (same specimen, reconfigure between)
- Doubles capture time but maximizes data utility

---

### Option 3: Post-Rotation Correction

**Capture with 45° tilt, correct in post-processing:**
```python
# COLMAP pose adjustment (pseudo-code)
for stack in stereo_pairs:
    # Rotate camera intrinsics to account for pan axis tilt
    K_corrected = rotate_intrinsics(K, pan_axis_angle=45)
    # Re-triangulate with corrected poses
    reconstruct(stack, K_corrected)
```

**Feasibility:** Medium
- Requires custom COLMAP integration
- Works if axis tilt is **rigid and known** (already documented in sequence.json as `pan_axis_tilt_deg`)
- Adds computational overhead

---

## Recommendation

**For your current setup:**

1. **Default behavior:** Pan tilted 45°, `stereo_enabled: false`
   - Captures excellent surface detail
   - Use batch_focus_stack.py for focus blending
   - Post-process with photogrammetry (not COLMAP stereo)

2. **Special stereo scanning:** Mechanically straighten pan axis to 90°
   - Use `stereo_enabled: true`
   - Enables COLMAP 3D reconstruction
   - Enables VR/3D video output
   - Takes same time, different output format

3. **Document the constraint:**
   - Add note to `sequence.json`: `"pan_axis_tilt_deg": 45`
   - Post-processing scripts check this field
   - Route to appropriate reconstruction pipeline

---

## Implementation: UI Hints

**Add to macro panel** (optional enhancement):

```html
<div style="background:#2a3a2a; border:1px solid #3a5a3a; 
            border-radius:4px; padding:8px; margin-bottom:8px;
            font-size:0.7rem; color:#7fda7f;">
  ⚠️ <strong>Stereo Note:</strong> Stereo 3D capture works best when 
  pan axis is vertical (pan_axis_tilt_deg = 90°). 
  If your setup uses 45° tilt for coverage, disable stereo 
  or reconfigure hardware before scanning.
</div>
```

**Or add to macro_start() log:**

```javascript
if (payload.stereo_enabled && payload.rotation_axis_angle_deg !== 90) {
    log(`⚠️ WARNING: Stereo enabled but pan axis angle = ${payload.rotation_axis_angle_deg}°. ` +
        `For optimal 3D reconstruction, pan axis should be vertical (90°).`);
}
```

---

## Field to Track

**In `sequence.json`, already captured:**
```json
{
  "pan_axis_tilt_deg": 45,      // YOUR SETUP
  "stereo_enabled": true,        // USER CHOICE
  "stereo_offset_deg": 3.0,      // LEFT-RIGHT BASELINE
  
  // Derived: should stereo be trusted?
  "stereo_viable": false         // computed: stereo_enabled AND pan_axis_tilt_deg == 90
}
```

Post-processing script can check `stereo_viable` to decide reconstruction method.

---

## Summary Table

| Configuration | Coverage | Stereo Quality | Reconstruction | Recommended For |
|---------------|----------|---|---|---|
| Pan 45°, Stereo OFF | Excellent | — | Photogrammetry | Surface detail, focus stacking |
| Pan 90°, Stereo OFF | Good | — | NeRF / Photogrammetry | Quick 3D capture |
| Pan 90°, Stereo ON | Good | Excellent | COLMAP, VR | 3D video, binocular viewing |
| Pan 45°, Stereo ON | Excellent | Poor | NOT RECOMMENDED | (avoid this combination) |

---

## Action Items

- [x] Document constraint in code comments
- [x] Add `pan_axis_tilt_deg` to sequence.json output (already implemented)
- [ ] Add UI warning for stereo+non-vertical axis combination (optional)
- [ ] Create post-processing decision tree in batch_focus_stack.py
- [ ] Document in user manual under "Setup Variations"

---

## For You (Tim)

**Option A (Minimal change):**
- Keep 45° tilt for standard coverage
- Disable stereo (`stereo_enabled: false`)
- Use batch_focus_stack.py + photogrammetry
- ✓ What you're already doing well

**Option B (Maximum flexibility):**
- Add mechanical quick-release for pan axis
- Swap between: 45° (coverage) and 90° (stereo)
- Toggle stereo mode in UI before scanning
- ✓ Best of both worlds, requires hardware mod

**Option C (Pragmatic):**
- Keep 45° setup as primary
- Implement stereo capture anyway
- Document that it's non-standard geometry
- Post-process with error handling in COLMAP
- ✓ Get stereo data, may need custom reconstruction

---

**Bottom Line:** The UI is ready to go. Just remember to either:
1. Disable stereo for 45° pan axis, OR
2. Straighten axis to 90° before enabling stereo
