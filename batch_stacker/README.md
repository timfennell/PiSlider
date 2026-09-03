# StackBatch — Batch Focus Stacker

A standalone macOS app that batch processes PiSlider macro scan output — automatically focus stacking every bracket set and saving the results ready for COLMAP 3D reconstruction.

---

## What It Does

After a PiSlider macro scan session you'll have dozens (or hundreds) of folders, each containing a bracket of raw images shot at different focus distances. StackBatch finds all of these automatically, aligns and stacks each bracket using shinestacker, and copies the finished TIFFs into the `colmap/images/` folder ready for photogrammetry.

Processing is sequential — each stack is fully completed and cleaned up before the next one starts, so intermediate aligned TIFFs never accumulate on disk.

---

## Requirements

- **macOS** only
- [**shinestacker.app**](https://www.shinestacker.com) installed in `/Applications`
- **Homebrew Python 3** — `/opt/homebrew/bin/python3` or `/usr/local/bin/python3`

StackBatch does not bundle OpenCV, numpy, or any image processing libraries. It borrows them from shinestacker.app at runtime via a subprocess, keeping the app itself small (~27MB).

---

## Usage

1. Launch `batchstacker.app`
2. Click **Choose Folder** and select your macro scan project folder (the one containing `orbit_001/`, `orbit_002/`, etc.)
3. Click **Start** — StackBatch will find all stacks and begin processing
4. Watch progress in real time — stack counter, current phase (aligning / stacking), and a log of completed stacks
5. Finished TIFFs appear in `colmap/images/` inside your project folder

---

## Output Location

Stacked images are saved to:
```
your_project/
└── orbit_001/
    └── colmap/
        └── images/
            ├── stack_001.tiff
            ├── stack_002.tiff
            └── ...
```

These are the input images for COLMAP or other photogrammetry software.

---

## Building from Source

```bash
pip install pyinstaller
pyinstaller StackBatch.spec
```

The built app appears in `dist/StackBatch.app`.

**Requirements for building:**
- macOS
- Python 3.8+
- PyInstaller (`pip install pyinstaller`)
- shinestacker.app installed (required at runtime, not build time)

---

## Files

| File | Description |
|------|-------------|
| `gui_stacker.py` | Main application — GUI and batch processing logic |
| `shine_batch.py` | Command-line batch runner (no GUI) |
| `StackBatch.spec` | PyInstaller build configuration |
| `rthook_shinestacker.py` | PyInstaller runtime hook |

---

## How It Works

StackBatch writes a temporary Python script and executes it using the system Python in a subprocess. The subprocess loads shinestacker from `/Applications/shinestacker.app/Contents/Resources` and processes all stacks sequentially in a single Python session.

For each stack:
1. Creates a temp directory with symlinks to the raw source files (DNG/ARW/CR3/NEF/TIFF only — no preview JPEGs, no macOS `._` resource files)
2. Runs shinestacker: `AlignFrames → BalanceFrames → PyramidStack`
3. Copies the stacked output TIFF to `colmap/images/`
4. Deletes the temp directory (aligned TIFFs are gone before the next stack starts)

This approach means the StackBatch app itself contains no image processing code — it's just a thin GUI shell around a subprocess.
