# Macro Scan Graph — User Guide

**Status:** Ready for testing  
**Date:** 2026-05-20

---

## Overview

The **Macro Graph** is a real-time visualization of your macro scanning session, similar to the timelapse graph but optimized for stack-by-stack progress tracking.

### Launch

Open in a new window during a macro scan:
```
http://raspberrypi.local:5000/macro-graph.html
```

Or click a "View Graph" button on the main control panel (if implemented).

---

## Display Elements

### Header Stats
- **Stack**: Current stack number and total
- **Frames**: Frames per stack (focus rail steps)
- **Rail (mm)**: Current slider position
- **Pan (°)**: Current pan angle
- **Tilt (°)**: Current tilt angle

### Charts (Scrollable)

#### 1. **Focus Rail Position** (Thin chart, magenta)
- Tracks slider motor position across all stacks
- Shows how the focus rail sweeps through the specimen
- Useful for verifying focus sweep range

#### 2. **Pan Position** (Thin chart, cyan)
- Pan angle per stack
- Visualizes rotational coverage pattern
- Shows scan progression around specimen

#### 3. **Tilt Position** (Thin chart, gold)
- Tilt (auxiliary axis) angle per stack
- Only shown if tilt axis is enabled in scan
- Visible in grid-2D mode

#### 4. **ISO per Stack** (Thin chart, green)
- Exposure sensitivity for each stack
- Normally flat (single value) unless adjusted mid-scan

#### 5. **Shutter Speed** (Thin chart, magenta)
- Exposure time per stack (log scale)
- Normally flat unless adjusted mid-scan

#### 6. **Progress** (Tall chart, cyan)
- Stack count vs. total stacks
- Filled area chart showing scan completion
- X-axis = stacks, Y-axis = completion count

### Preview Bar (Bottom)

#### Left: Stack Flipbook
- **Thumbnail preview** of stack data
- Shows current stack details:
  - Stack number / total
  - Frame count
  - Rail position
  - Pan/tilt angles
  - ISO & shutter settings
- **Controls**:
  - `◀` / `▶` : Step through stacks
  - `▶` / `⏸` : Play/pause auto-advance (120ms per frame)
  - Label shows current position

#### Right: Live Camera Feed
- Real-time view from camera
- Updates during capture
- Useful for monitoring lighting, alignment

---

## Data Tracked Per Stack

Each macro_progress update includes:
- Stack number
- Frame count
- Rail position (mm)
- Pan angle (degrees)
- Tilt angle (degrees)
- ISO
- Shutter speed (seconds)

---

## Workflow Tips

1. **During Scan**
   - Open graph in separate window
   - Monitor motor positions for expected coverage
   - Watch ISO/shutter settings
   - Follow progress bar to estimate remaining time

2. **Verify Coverage**
   - Pan chart should show smooth progression across scan range
   - Rail chart should match expected focus sweep
   - Tilt chart should show expected tilt pattern (if 2D grid)

3. **Troubleshoot**
   - If motor positions look wrong, stop scan (E-Stop button)
   - Check hardware configuration
   - Verify soft limit settings

4. **Post-Scan Review**
   - Graph remains live after scan completes
   - Flip through stacks to verify captures
   - Check final statistics

---

## Graph Features

- **Real-time WebSocket updates** — data arrives as stacks complete
- **Auto-scaling axes** — min/max adjust to actual data
- **Responsive design** — works on desktop and tablet
- **Dark theme** — matches main control interface
- **Stack flipbook** — quick preview of captured data
- **Live camera feed** — monitor specimen during scan

---

## Comparison with Timelapse Graph

| Feature | Timelapse | Macro |
|---------|-----------|-------|
| **X-axis unit** | Frames | Stacks |
| **Focus tracking** | Optional (pan/tilt/slider) | Always (rail position) |
| **Exposure curves** | EV + sun altitude | ISO + shutter only |
| **Time-based** | Yes (frames/sec) | No (discrete stacks) |
| **Coverage viz** | Phase backgrounds | Progress bar |
| **Flipbook** | Full JPEG preview | Stack info text |

---

## Keyboard Shortcuts

- `Esc` or `Close button` : Close graph window
- Hover charts for axis labels
- Mouse over flipbook controls for tooltips

---

## Troubleshooting

### Graph not updating
- Check browser console (DevTools) for WebSocket errors
- Verify `ws://raspberrypi.local:5000/ws-graph` is accessible
- Try refreshing page (F5)

### Charts show no data
- Graph must be open **during** a macro scan
- Open it after scan starts, not before
- Wait for first stack to complete

### Flipbook shows "NO FRAMES YET"
- Stacks are being captured but thumbnails may not be ready yet
- Thumbnails are generated after focus stacking completes
- Check main control panel for stack completion status

### Live camera feed dark/blank
- Check camera is enabled and connected
- Verify lighting is adequate
- Try `/video_feed` endpoint directly in browser

---

## Performance

- **Graph responsiveness**: < 100ms per update
- **Flipbook load time**: < 500ms per stack thumbnail
- **Memory usage**: ~50MB for 100+ stacks
- **CPU usage**: Minimal (only during updates)

---

## Future Enhancements

1. **Heatmap display** — coverage density visualization on sphere/grid
2. **Wireframe overlay** — 3D position plot
3. **Easing curve visualization** — show pan/tilt motion profile
4. **Stack metadata export** — download CSV of all metrics
5. **Comparison mode** — overlay multiple scans

---

## Integration with Main Control Panel

When macro graph feature is fully integrated:
- "View Graph" button on macro panel
- Opens new window automatically on scan start
- Auto-closes with scan completion (optional)

---

**Questions?** See MACRO_READY_TO_TEST.md for testing scenarios or SESSION_IMPLEMENTATION_SUMMARY.md for implementation details.
