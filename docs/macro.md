# Macro 3D Scanning Mode

PiSlider's macro mode automates the capture of large multi-image datasets for **focus stacking** and **3D photogrammetry** (COLMAP). The slider positions the camera at dozens of orbital positions around an object, and at each position captures a full focus bracket stack by moving the rail through the focal range.

---

## Overview

Macro 3D scanning is the combination of two techniques:

1. **Focus stacking** — captures multiple frames at different focus distances, then merges them into a single all-in-focus image
2. **Photogrammetry** — uses photos from many viewpoints to reconstruct a 3D model of the object

The PiSlider automates both. A single macro scan session can capture hundreds to thousands of images across an entire orbital sweep, producing a dataset ready for COLMAP, RealityCapture, or Metashape.

---

## Scan Geometry

### Orbits

A **macro scan project** is organized into one or more **orbits**. Each orbit corresponds to one physical rig setup at a specific height and distance from the subject.

Within an orbit, the pan motor rotates the camera around the subject at evenly-spaced angular positions (e.g., 36 positions × 10° apart = 360° coverage). These positions are calculated using a **geodesic sphere distribution** — they cover the hemisphere with even angular spacing rather than latitude/longitude grid lines.

### Focus Stacks

At each pan/tilt position, the rail moves through the focus range:
- **Rail start** — the nearest focus distance (object edge closest to camera)
- **Rail end** — the furthest focus distance (object edge furthest from camera)
- **Images per stack** — number of frames captured across this range (typically 10–30)

The rail steps evenly through the focus range, capturing one frame at each position. These N frames form a **focus stack**.

### Exposure Slots

Each position can have multiple **exposure slots** — different lighting setups captured at every focus position. Slots are named (e.g., `slot_A` = diffuse, `slot_B` = side light). This allows you to capture the same geometry under different lighting conditions in a single automated session.

---

## Scan Modes

### SCAN Mode (Science-First)

- Even angular spacing across the full sphere
- Maximum COLMAP coverage — every view angle gets equal weight
- Best for 3D reconstruction accuracy
- Pan positions calculated from geodesic distribution

### ART Mode (Cinematic)

- Easing curves on pan/tilt movement
- Partial arcs (not necessarily full 360°)
- Optimized for look rather than geometric completeness
- Better for hero shots or artistic presentation

---

## Setting Up a Scan

### 1. Position the Subject

Place the subject on a turntable or fixed surface. The subject should not move during the scan.

### 2. Set Focus Rail Limits

Manually position the camera at the nearest edge of the subject. Note the rail position (shown in the UI, in steps from home). Then move to the furthest edge. These become **rail start** and **rail end**.

- **Rail start steps** — absolute step count from home position at the near edge
- **Rail end steps** — absolute step count from home position at the far edge

Using absolute steps (rather than mm) ensures precision across sessions — step counts are exact integers with no floating point conversion error.

### 3. Configure Stack Parameters

- **Images per stack** — how many frames to capture across the focus range. More = smoother merge, larger dataset, longer session.
- **Stacks** — total number of orbital positions
- **Vibration delay** — wait time after each motor movement before capturing (0.5–2s typically)

### 4. Configure Exposure Slots

Add one or more exposure slots. Each slot defines:
- Relay states (for controlling lights)
- ISO, shutter speed
- Settle time (wait after relay fires before shutter)

### 5. Run the Scan

Press **Start Scan**. The system will:

1. Move to the first geodesic position (pan + tilt)
2. Move the rail to `rail_start`
3. For each focus position:
   - Wait for vibration to settle
   - Capture frame (all exposure slots)
4. Move rail back to start at fast return speed
5. Move to next geodesic position
6. Repeat

Progress is logged in real time. Each completed stack is marked in `sequence.json` so the scan can resume from interruptions.

---

## Output Structure

Scan output follows this directory structure:

```
macro_project_667_2026-05-22/
└── orbit_001/
    ├── project.json        ← Orbit metadata
    ├── sequence.json       ← Per-stack completion tracking
    ├── stack_001/
    │   └── rot+000.000_aux-180.00/
    │       └── slot_A/
    │           ├── img_001.dng
    │           ├── img_002.dng
    │           └── ...
    ├── stack_002/
    │   └── rot+010.000_aux-171.00/
    │       └── slot_A/
    │           └── ...
    └── colmap/
        └── images/         ← Stacked outputs for 3D reconstruction
            ├── stack_001.tiff
            ├── stack_002.tiff
            └── ...
```

The `colmap/images/` folder is populated by StackBatch after focus stacking. COLMAP reads images from this folder to generate the 3D model.

---

## Focus Stacking with StackBatch

After the scan is complete, focus stack each set of raw frames using the **StackBatch** macOS app.

StackBatch:
1. Scans the project folder for all `stack_*` directories
2. For each stack, creates a temp directory with symlinks to raw files (DNG/ARW/TIFF only — no preview JPEGs, no macOS `._` resource files)
3. Runs shinestacker.app in a subprocess: aligns frames → stacks with PyramidStack
4. Copies the stacked output to `colmap/images/` for COLMAP input
5. Deletes temporary aligned frames automatically (no disk waste between jobs)

**StackBatch requires:**
- [shinestacker.app](https://www.shinestacker.com) installed in `/Applications`
- Homebrew Python 3 (`/opt/homebrew/bin/python3`)

The app processes jobs sequentially with real-time progress display — stack counter, phase indicator (aligning / stacking), current stack name, and a progress bar.

---

## COLMAP 3D Reconstruction

With stacked images in `colmap/images/`, run COLMAP to reconstruct the 3D model:

```bash
# Feature extraction
colmap feature_extractor \
    --database_path colmap/database.db \
    --image_path colmap/images

# Feature matching
colmap exhaustive_matcher \
    --database_path colmap/database.db

# Sparse reconstruction
colmap mapper \
    --database_path colmap/database.db \
    --image_path colmap/images \
    --output_path colmap/sparse

# Dense reconstruction (optional, requires GPU)
colmap image_undistorter \
    --image_path colmap/images \
    --input_path colmap/sparse/0 \
    --output_path colmap/dense

colmap patch_match_stereo \
    --workspace_path colmap/dense

colmap stereo_fusion \
    --workspace_path colmap/dense \
    --output_path colmap/dense/fused.ply
```

The output `fused.ply` point cloud can be imported into Blender, MeshLab, or any 3D software.

---

## Tips

- **Manual lens** — autofocus changes between frames will corrupt the stack. Use a manual lens or tape the focus ring.
- **Object stability** — the subject must not move during the entire scan. A heavy, stable turntable base helps. Even slight vibration from a nearby road can affect alignment.
- **Depth of field** — a narrower aperture (f/8–f/16) gives more DOF per frame, requiring fewer stack images. But diffraction softens images at very small apertures. f/8 is often the sweet spot.
- **Lighting** — consistent, diffuse lighting produces the most reliable stacks. Specular highlights can confuse alignment algorithms.
- **Test stack first** — before running a full multi-hour scan, run a single-stack test and verify it stacks cleanly in StackBatch. Check the output TIFF for alignment issues.
- **Disk space** — a scan with 36 positions × 20 images × 25MB DNG = 18GB of raw files. Make sure the output drive has adequate space.
