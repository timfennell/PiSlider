# Macro Mode UI — Test Plan

**Status:** Ready for QA  
**Implementation Date:** 2026-05-20  
**Test Date:** TBD

---

## Pre-Test Checklist

- [x] HTML/JS syntax validated (no parsing errors)
- [x] Python backend compiles successfully  
- [x] All WebSocket handlers registered in message dispatcher
- [x] Stereo parameters passed through `macro_start` payload
- [x] Easing curve list populated dynamically from server
- [x] Grid computation endpoint available
- [x] Storage calculation includes stereo multiplier
- [x] All form controls properly bound to `macroCalc()`

---

## UI Test Scenarios

### Scenario 1: Mode Activation & Easing Curves Load
**Steps:**
1. Load web interface
2. Click "Macro Focus" tab in header
3. Observe: Macro panel appears in sidebar
4. **Expected:** "Spacing Curve" dropdown contains ~16 options (linear, parabolic, gaussian, cycloid, etc.)
5. **Expected:** Log shows "[Macro] Loaded 16 easing curves"

### Scenario 2: Rail Configuration
**Steps:**
1. Click "Set Start" button → move slider to position A
2. Click "Set End" button → move slider to position B
3. Observe: "Rail Start" and "Rail End" display values
4. Change "Step Size (µm)" to different values
5. **Expected:** "Frames/Stack" auto-updates based on (travel_mm × 1000) / step_um
6. **Expected:** "Total images" in summary updates
7. **Expected:** "Est. storage" (GB) recalculates

### Scenario 3: Rotation Range (Full Mode)
**Steps:**
1. Confirm "360° Full" button is active (default)
2. Observe: Rotation range controls are hidden
3. Click "Range" button
4. Click "📍 Rot Start" → rotate pan to position A
5. Click "📍 Rot End" → rotate pan to position B
6. Observe: Start/End degrees display
7. **Expected:** "Compute Optimal Grid" button becomes functional

### Scenario 4: Grid Computation
**Steps:**
1. Set rotation range (or keep at ±180° full)
2. Set number of stacks to 36
3. (Optional) Enable aux axis and set tilt range
4. Click "📊 Compute Optimal Grid"
5. **Expected:** Result displays: "4 cols × 3 rows = 12 stacks" (or similar)
6. **Expected:** Log shows "[Macro Grid] ..." message
7. Change num_stacks to 100 → click button again
8. **Expected:** Grid result updates (larger distribution)

### Scenario 5: Stereo 3D Capture
**Steps:**
1. Leave "Enable Stereo" unchecked
2. Observe: "Stacks:" shows plain number (e.g., "36")
3. Observe: "Total images:" = frames × stacks × slots
4. Check "Enable Stereo" checkbox
5. **Expected:** "Stacks:" now shows "36 × 2 (stereo)"
6. **Expected:** "Total images:" doubles
7. **Expected:** "Est. storage:" doubles
8. Adjust "Pan Offset (°)" to 5.0
9. **Expected:** Value updates in UI, log notes change

### Scenario 6: Lens Profile Management
**Steps:**
1. Change "Lens Name" to "Canon MP-E 65mm"
2. Select "Type" = "Macro"
3. Set "Magnification" to 5.0
4. Set "Working Dist." to 25
5. Click "💾 Save Profile"
6. **Expected:** Log shows "Lens profile 'Canon MP-E 65mm' saved."
7. Change lens name to something else
8. Click "💾 Save Profile" again
9. Click dropdown "— Load saved —"
10. Click "Canon MP-E 65mm"
11. **Expected:** All fields restore to previous values

### Scenario 7: Exposure Slots
**Steps:**
1. Enable Slot A (diffuse lighting)
2. Set Slot A: Relay1 ON, ISO 400, Shutter 1/125s, 5500K
3. Enable Slot B (laser)
4. Set Slot B: Relay2 ON, ISO 100, Shutter 1/30s, 5500K
5. **Expected:** "Total images" shows calculation × 2 (two slots)
6. Disable Slot B
7. **Expected:** "Total images" recalculates to single-slot count

### Scenario 8: Aux Axis (Tilt)
**Steps:**
1. Check "Enable" under Aux Axis section
2. Observe: Aux controls appear (Start/End buttons, easing, soft limits)
3. Click "📍 Aux Start" → tilt motor to position A
4. Click "📍 Aux End" → tilt motor to position B
5. Select easing curve for aux
6. **Expected:** "Compute Optimal Grid" now accounts for tilt range

### Scenario 9: Start Macro Scan (Dry Run)
**Steps:**
1. Configure complete scan:
   - Rail: Start=0.5mm, End=3.0mm, Step=50µm → ~51 frames
   - Rotation: Full 360°, 36 stacks
   - Stereo: ON, 3.0° offset
   - Slot A: ON (diffuse)
2. Review summary:
   - Frames/Stack: 51
   - Stacks: 36 × 2 (stereo)
   - Total images: 51 × 36 × 2 = 3,672
   - Est. storage: ~91.8 GB
3. Observe: Log shows final configuration
4. Click "▶ Start Sequence"
5. **Expected:** Progress panel appears
6. **Expected:** Hardware begins motion (or errors gracefully if not connected)
7. **Expected:** WebSocket log shows "Starting macro: 36 stacks × 50µm steps + STEREO (3.0° offset)"

---

## Backend Integration Tests

### Test B1: Easing Curves Endpoint
```bash
# Send via WebSocket:
{"cmd": "macro_get_easing_curves"}

# Expected response:
{"type": "macro_easing_curves", "curves": [
    "catenary", "cycloid", "ellipsoidal", "even", 
    "gaussian", "inverted_catenary", "inverted_cycloid",
    ... (16 total)
]}
```

### Test B2: Grid Computation Endpoint
```bash
# Send via WebSocket:
{
  "cmd": "macro_compute_grid",
  "total_stacks": 36,
  "pan_min": -90, "pan_max": 90,
  "tilt_min": -30, "tilt_max": 30
}

# Expected response:
{"type": "macro_grid_computed", "pan_cols": 4, "tilt_rows": 3, "total_actual": 12}
```

### Test B3: Macro Start with Stereo
```bash
# Send via WebSocket macro_start with:
"stereo_enabled": true,
"stereo_offset_deg": 3.0

# Backend should:
1. Pass parameters to MacroSession dataclass
2. Call stereo_multiplier() → returns 2
3. Generate scan positions with (pan, tilt, "left"|"right") tuples
4. Capture left stack, then right stack at offset pan
5. Record eye field in sequence.json
```

---

## Hardware Tests (on Raspberry Pi)

### H1: Motor Control
- [ ] Slider motor nudges smoothly without UI lag
- [ ] Pan motor responds to range selection
- [ ] Tilt motor works when aux enabled

### H2: Camera Capture
- [ ] Pi camera captures frames during focus stack
- [ ] Frame count matches calculated value
- [ ] Exposure slot A applies correctly
- [ ] Exposure slot B applies correctly

### H3: Stereo Sequence
- [ ] Left eye captures → right eye captures with pan offset
- [ ] Output folder structure has left/ and right/ subdirectories
- [ ] sequence.json lists eye field per stack

### H4: Post-Processing
- [ ] batch_focus_stack.py processes stacked images
- [ ] Merged images appear in images_merged/
- [ ] COLMAP can reconstruct both left and right models

---

## Error Handling Tests

### E1: Missing Rail Endpoints
- **Test:** Click "Start Sequence" without setting rail start/end
- **Expected:** Warning message, no scan starts

### E2: Invalid Range
- **Test:** Set stereo offset to 0° (below minimum 0.1°)
- **Expected:** Input validation prevents (or corrects) invalid value

### E3: Hardware Timeout
- **Test:** Start scan with USB cable unplugged
- **Expected:** Hardware error message, graceful stop

### E4: Easing Curves Not Loaded
- **Test:** Manually navigate to macro mode with network delay
- **Expected:** Dropdown shows placeholder; curves load when available

---

## Performance Benchmarks

| Operation | Expected Time | Status |
|-----------|----------------|--------|
| Load easing curves | <100ms | TBD |
| Compute grid (36 stacks) | <10ms | TBD |
| Render progress panel | <50ms | TBD |
| Full 36-stack macro (no focus) | ~5-10 min | TBD |
| Focus stack 51 frames | ~30-60s | TBD |

---

## Sign-Off

**Tester Name:** ___________________  
**Date Tested:** ___________________  
**Issues Found:** [ ] None [ ] Minor [ ] Critical  
**Comments:**  
```
[Space for notes]
```

**Approved for Production:** [ ] Yes [ ] No [ ] Conditional

---

## Known Limitations (Document for Users)

1. **Grid computation** assumes orthogonal pan/tilt axis; real hardware may have slight coupling
2. **Stereo offset** is currently linear in pan; tilt perspective correction is future work
3. **Preview canvas** (wireframe visualization) not yet implemented
4. **Programmed path** mode (custom waypoint sequences) not yet available

---

## Rollback Plan

If critical issues found:
1. `git revert` most recent macro UI commits
2. Keep backend stereo logic (non-breaking change)
3. Restore previous easing dropdown (5 hardcoded options)
4. Document findings in ISSUES.md for next iteration
