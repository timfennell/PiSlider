# TiltCorrector — Perspective Warp Corrector

A standalone macOS app that corrects perspective distortion in TIFF images from PiSlider macro scan sessions where the camera rig was tilted. It reads the tilt angle embedded in the XMP sidecar metadata written by the PiSlider and applies the inverse geometric warp to straighten each image.

---

## What It Does

When the PiSlider camera rig is physically tilted (e.g. shooting at an angle down onto a subject), the resulting images have perspective distortion — parallel lines appear to converge. TiltCorrector reads the `Rig_Tilt_Deg` value recorded in the PiSlider XMP metadata and uses the lens focal length and sensor dimensions to calculate and apply the exact inverse warp, producing geometrically corrected TIFFs.

---

## Requirements

- **macOS** only
- [**ExifTool**](https://exiftool.org) — for reading focal length from image metadata
  ```bash
  brew install exiftool
  ```
- **Homebrew Python 3** with:
  ```bash
  pip install opencv-python numpy tifffile rawpy imageio
  ```

---

## Usage

1. Launch `TiltCorrector.app`
2. Click **Choose Input Folder** — select the folder containing your lens-corrected TIFFs and their XMP sidecars
3. Click **Choose Output Folder** — where corrected TIFFs will be saved
4. Click **Process** — the app reads the tilt angle from each XMP file and applies the correction
5. Corrected TIFFs appear in the output folder

---

## XMP Metadata

The PiSlider writes tilt angle data into XMP sidecar files alongside each captured image using the custom namespace `http://ns.pislider.io/1.0/`:

```xml
<ps:Rig_Tilt_Deg>-20.5</ps:Rig_Tilt_Deg>
```

TiltCorrector reads this value to determine the exact warp to apply. If no XMP sidecar is found, the image is skipped.

---

## How the Correction Works

1. **Read focal length** from EXIF via ExifTool
2. **Read tilt angle** from XMP sidecar (`Rig_Tilt_Deg`)
3. **Calculate field of view** from focal length and sensor dimensions (Sony A7 III: 35.6 × 23.8mm by default)
4. **Build perspective warp matrix** — the inverse of the tilt transformation
5. **Apply warp** using OpenCV `warpPerspective`
6. **Save** as TIFF

The correction is mathematically precise — it accounts for the actual lens FOV rather than using a generic approximation.

---

## Building from Source

```bash
pip install pyinstaller opencv-python numpy tifffile rawpy imageio
pyinstaller TiltCorrector.spec
```

The built app appears in `dist/TiltCorrector.app`.

> **Note:** The built app is large (~500MB) because OpenCV is bundled inside it. This is a known issue — a future version will use a subprocess architecture (like StackBatch) to avoid bundling cv2.

---

## Files

| File | Description |
|------|-------------|
| `gui_app.py` | Main application — GUI and correction pipeline |
| `correct_perspective.py` | Core perspective warp logic (command-line) |
| `process_tiffs.py` | Batch TIFF processor |
| `TiltCorrector.spec` | PyInstaller build configuration |
