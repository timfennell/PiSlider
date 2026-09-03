#!/usr/bin/env python3
"""
batch_focus_stack.py — PiSlider Focus Stack Batch Processor
=============================================================
Reads a PiSlider macro project folder and focus-stacks each capture
position into a single sharp image ready for COLMAP / Gaussian Splat.

The output matches the filenames in colmap_merged/images.txt exactly:
    images_merged/orbit_001/stack_001/best_focus.jpg
    images_merged/orbit_001/stack_002/best_focus.jpg
    images_merged/orbit_002/stack_001/best_focus.jpg  ...

STACKING ENGINES (tried in order):
  1. enfuse + align_image_stack   (brew install hugin  — best, recommended)
  2. focus-stack                  (brew install focus-stack — fast)
  3. Pure Python                  (built-in fallback, rawpy + numpy + Pillow)
     pip install rawpy            (reads Sony ARW, Canon CR3, Nikon NEF, etc.)

RAW conversion: rawpy (pip) → any raw format
                sips (macOS built-in) → HEIC/HEIF and common raws

Affinity Photo note:
  Affinity Photo 2 does not expose Focus Merge to scripts or AppleScript.
  For a manual high-quality alternative on individual stacks:
    File > New > Focus Merge → browse to a stack_NNN slot folder
  After this script finishes, use --open-in-affinity to view results.

Usage:
  python3 batch_focus_stack.py <project_folder>
  python3 batch_focus_stack.py <project_folder> --slot slot_A
  python3 batch_focus_stack.py <project_folder> --orbit orbit_002
  python3 batch_focus_stack.py <project_folder> --format tiff
  python3 batch_focus_stack.py <project_folder> --engine python
  python3 batch_focus_stack.py <project_folder> --dry-run
  python3 batch_focus_stack.py <project_folder> --open-in-affinity
"""

from __future__ import annotations
import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import List, Optional, Tuple
from PIL import ImageFilter as _ImageFilter

# ── Optional imports (degrade gracefully) ────────────────────────────────────
try:
    import numpy as np
    HAVE_NUMPY = True
except ImportError:
    HAVE_NUMPY = False

try:
    from PIL import Image
    HAVE_PIL = True
except ImportError:
    HAVE_PIL = False

try:
    import rawpy
    HAVE_RAWPY = True
except ImportError:
    HAVE_RAWPY = False


# ── RAW / image file extensions ──────────────────────────────────────────────
RAW_EXTS  = {'.arw', '.nef', '.cr2', '.cr3', '.raf', '.dng', '.rw2', '.orf',
             '.pef', '.srw', '.nrw', '.raw'}
IMG_EXTS  = {'.jpg', '.jpeg', '.tif', '.tiff', '.png', '.bmp'}
ALL_EXTS  = RAW_EXTS | IMG_EXTS


# =============================================================================
# Tool detection
# =============================================================================

def detect_engine() -> str:
    """Return the best available stacking engine name."""
    if shutil.which("enfuse") and shutil.which("align_image_stack"):
        return "enfuse"
    if shutil.which("focus-stack"):
        return "focus-stack"
    if HAVE_NUMPY and HAVE_PIL:
        return "python"
    return "none"


def check_requirements(engine: str) -> None:
    if engine == "none":
        print("ERROR: No stacking engine available.")
        print("Install one of:")
        print("  brew install hugin        # enfuse + align_image_stack")
        print("  brew install focus-stack")
        print("  pip install numpy Pillow  # pure-Python fallback")
        sys.exit(1)
    if engine == "python" and not HAVE_RAWPY:
        print("WARNING: rawpy not installed — RAW files will use sips (lower quality).")
        print("         pip install rawpy   for full RAW support")


# =============================================================================
# Project structure discovery
# =============================================================================

def find_stacks(project_folder: Path,
                orbit_filter: Optional[str],
                slot_filter: Optional[str]) -> List[dict]:
    """
    Walk a PiSlider project folder and return a list of stack entries.
    Each entry: {orbit_label, stack_id, slot_id, slot_dir, output_jpg}
    """
    proj_json = project_folder / "project.json"
    stacks = []

    if proj_json.exists():
        # Structured project — read project.json + sequence.json per orbit
        with open(proj_json) as f:
            proj = json.load(f)

        for orb in proj.get("orbits", []):
            if not orb.get("completed"):
                continue
            orbit_label = orb.get("orbit_id", orb.get("label", "orbit_001"))
            if orbit_filter and orbit_label != orbit_filter:
                continue
            orb_folder = project_folder / orb.get("folder", orbit_label)
            seq_json   = orb_folder / "sequence.json"
            if not seq_json.exists():
                print(f"  WARNING: no sequence.json in {orb_folder}, skipping")
                continue

            with open(seq_json) as f:
                seq = json.load(f)

            for stack in seq.get("stacks", []):
                if not stack.get("completed"):
                    continue
                stack_id   = stack.get("stack_id", "stack")
                stack_base = orb_folder / stack.get("folder", stack_id)
                # Find slot subfolders
                _add_slot_entries(stacks, orbit_label, stack_id,
                                  stack_base, slot_filter)
    else:
        # Unstructured: walk looking for slot folders containing images
        for orbit_dir in sorted(project_folder.iterdir()):
            if not orbit_dir.is_dir():
                continue
            if orbit_filter and orbit_dir.name != orbit_filter:
                continue
            for stack_dir in sorted(orbit_dir.iterdir()):
                if not stack_dir.is_dir():
                    continue
                _add_slot_entries(stacks, orbit_dir.name, stack_dir.name,
                                  stack_dir, slot_filter)

    return stacks


def _add_slot_entries(stacks: list, orbit: str, stack_id: str,
                      stack_base: Path, slot_filter: Optional[str]) -> None:
    """Find slot subfolders under stack_base and append entries."""
    if not stack_base.exists():
        return
    slot_dirs = sorted(d for d in stack_base.iterdir() if d.is_dir())
    if not slot_dirs:
        # No slot subdirectories — treat stack_base itself as the slot
        imgs = _find_images(stack_base)
        if imgs:
            stacks.append({
                "orbit":     orbit,
                "stack_id":  stack_id,
                "slot_id":   "default",
                "slot_dir":  stack_base,
                "images":    imgs,
            })
        return
    for slot_dir in slot_dirs:
        if slot_filter and slot_dir.name != slot_filter:
            continue
        imgs = _find_images(slot_dir)
        if imgs:
            stacks.append({
                "orbit":     orbit,
                "stack_id":  stack_id,
                "slot_id":   slot_dir.name,
                "slot_dir":  slot_dir,
                "images":    imgs,
            })


def _find_images(folder: Path) -> List[Path]:
    """Return sorted list of image/RAW files in a folder."""
    imgs = sorted(p for p in folder.iterdir()
                  if p.suffix.lower() in ALL_EXTS and not p.name.startswith('.'))
    return imgs


# =============================================================================
# RAW conversion
# =============================================================================

def load_as_pil(path: Path) -> "Image.Image":
    """Load any image file (including RAW) as a PIL Image (RGB, 8-bit)."""
    ext = path.suffix.lower()

    if ext in RAW_EXTS:
        if HAVE_RAWPY:
            return _load_raw_rawpy(path)
        else:
            return _load_raw_sips(path)
    else:
        return Image.open(path).convert("RGB")


def _load_raw_rawpy(path: Path) -> "Image.Image":
    with rawpy.imread(str(path)) as raw:
        rgb = raw.postprocess(
            use_camera_wb=True,
            no_auto_bright=False,
            output_bps=8,
            demosaic_algorithm=rawpy.DemosaicAlgorithm.AHD,
        )
    return Image.fromarray(rgb)


def _load_raw_sips(path: Path) -> "Image.Image":
    """Use macOS sips to convert RAW to JPEG, then load."""
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "converted.jpg"
        r = subprocess.run(
            ["sips", "-s", "format", "jpeg", str(path), "--out", str(out)],
            capture_output=True
        )
        if r.returncode != 0 or not out.exists():
            raise RuntimeError(f"sips failed on {path}: {r.stderr.decode()}")
        return Image.open(out).convert("RGB")


def convert_stack_to_tiff(images: List[Path], work_dir: Path) -> List[Path]:
    """
    Convert all images (including RAW) to TIFF in work_dir.
    Returns list of TIFF paths in same order.
    """
    tiffs = []
    for i, p in enumerate(images):
        out = work_dir / f"frame_{i:04d}.tiff"
        if p.suffix.lower() in RAW_EXTS or p.suffix.lower() not in {'.tif', '.tiff'}:
            img = load_as_pil(p)
            img.save(str(out), format="TIFF")
        else:
            shutil.copy(p, out)
        tiffs.append(out)
    return tiffs


# =============================================================================
# Focus stacking engines
# =============================================================================

def stack_enfuse(images: List[Path], output: Path, align: bool = True) -> None:
    """Stack using Hugin enfuse (brew install hugin)."""
    with tempfile.TemporaryDirectory() as td:
        work = Path(td)
        tiffs = convert_stack_to_tiff(images, work)

        if align:
            aligned_prefix = str(work / "aligned_")
            r = subprocess.run(
                ["align_image_stack", "-m", "-a", aligned_prefix] +
                [str(t) for t in tiffs],
                capture_output=True
            )
            if r.returncode != 0:
                print(f"    align_image_stack warning: {r.stderr.decode()[:200]}")
            # aligned files: aligned_0000.tif, aligned_0001.tif ...
            aligned = sorted(work.glob("aligned_*.tif*"))
            if not aligned:
                aligned = tiffs  # fallback: use unaligned
        else:
            aligned = tiffs

        out_tiff = work / "stacked.tiff"
        r = subprocess.run(
            ["enfuse", "--exposure-weight=0", "--saturation-weight=0",
             "--contrast-weight=1", "--contrast-window-size=9",
             "--hard-mask", "-o", str(out_tiff)] + [str(a) for a in aligned],
            capture_output=True
        )
        if r.returncode != 0:
            raise RuntimeError(f"enfuse failed: {r.stderr.decode()[:300]}")

        _save_output(Image.open(str(out_tiff)).convert("RGB"), output)


def stack_focus_stack(images: List[Path], output: Path) -> None:
    """Stack using focus-stack CLI tool (brew install focus-stack)."""
    with tempfile.TemporaryDirectory() as td:
        work = Path(td)
        tiffs = convert_stack_to_tiff(images, work)
        out_path = work / "stacked.jpg"
        r = subprocess.run(
            ["focus-stack", "--output=" + str(out_path)] +
            [str(t) for t in tiffs],
            capture_output=True
        )
        if r.returncode != 0:
            raise RuntimeError(f"focus-stack failed: {r.stderr.decode()[:300]}")
        _save_output(Image.open(str(out_path)).convert("RGB"), output)


def stack_python(images: List[Path], output: Path) -> None:
    """
    Pure numpy/Pillow Laplacian focus stacking.

    Algorithm:
      1. Load each image, compute per-pixel Laplacian magnitude (sharpness).
      2. Gaussian-blur each sharpness map to get a smooth focus weight.
      3. Normalise weights across the stack so they sum to 1 per pixel.
      4. Weighted blend of all images → output.
    """
    if not HAVE_NUMPY or not HAVE_PIL:
        raise RuntimeError("numpy and Pillow are required for python engine")

    print(f"    loading {len(images)} images...", end=" ", flush=True)
    t0 = time.time()

    # Load at a working scale to keep memory reasonable (full res for output)
    WORK_SCALE = 0.25   # compute focus maps at 1/4 res, apply blend at full res

    imgs_full: List[np.ndarray] = []
    imgs_small: List[np.ndarray] = []

    for p in images:
        pil = load_as_pil(p)
        imgs_full.append(np.array(pil, dtype=np.float32))
        small = pil.resize(
            (max(1, pil.width // 4), max(1, pil.height // 4)),
            Image.LANCZOS
        )
        imgs_small.append(np.array(small, dtype=np.float32))

    H_full, W_full = imgs_full[0].shape[:2]
    print(f"{time.time()-t0:.1f}s", flush=True)

    print(f"    computing focus maps...", end=" ", flush=True)
    t0 = time.time()

    # Compute Laplacian sharpness at reduced resolution
    focus_maps_small: List[np.ndarray] = []
    for img in imgs_small:
        gray = img.mean(axis=2)
        sharp = _laplacian_magnitude(gray)
        # Blur to smooth selection boundaries
        sharp_pil = Image.fromarray(
            np.clip(sharp / (sharp.max() + 1e-6) * 255, 0, 255).astype(np.uint8)
        ).filter(_ImageFilter.GaussianBlur(radius=8))
        focus_maps_small.append(np.array(sharp_pil, dtype=np.float32))

    print(f"{time.time()-t0:.1f}s", flush=True)
    print(f"    blending...", end=" ", flush=True)
    t0 = time.time()

    # Stack focus maps: [N, H_small, W_small]
    fm_stack = np.stack(focus_maps_small, axis=0)

    # Normalise so weights sum to 1 per pixel
    fm_sum = fm_stack.sum(axis=0, keepdims=True) + 1e-8
    weights_small = fm_stack / fm_sum                          # [N, H_s, W_s]

    # Upsample weight maps to full resolution
    weights_full = np.zeros((len(images), H_full, W_full), dtype=np.float32)
    for i, w in enumerate(weights_small):
        wpil = Image.fromarray(
            np.clip(w * 255, 0, 255).astype(np.uint8)
        ).resize((W_full, H_full), Image.LANCZOS)
        weights_full[i] = np.array(wpil, dtype=np.float32) / 255.0

    # Re-normalise after upscaling (bilinear can shift sums slightly)
    wsum = weights_full.sum(axis=0, keepdims=True) + 1e-8
    weights_full /= wsum

    # Weighted blend across all images
    imgs_arr = np.stack(imgs_full, axis=0)                     # [N, H, W, C]
    w4d      = weights_full[..., np.newaxis]                   # [N, H, W, 1]
    result   = (imgs_arr * w4d).sum(axis=0)                    # [H, W, C]
    result   = np.clip(result, 0, 255).astype(np.uint8)

    print(f"{time.time()-t0:.1f}s", flush=True)

    _save_output(Image.fromarray(result), output)


def _laplacian_magnitude(gray: np.ndarray) -> np.ndarray:
    """Fast absolute Laplacian using finite differences (numpy only)."""
    lap = (
        np.roll(gray, -1, axis=0) + np.roll(gray, 1, axis=0) +
        np.roll(gray, -1, axis=1) + np.roll(gray, 1, axis=1) -
        4.0 * gray
    )
    return np.abs(lap)


def _save_output(img: "Image.Image", output: Path) -> None:
    """Save result image in the format implied by the output path suffix."""
    output.parent.mkdir(parents=True, exist_ok=True)
    ext = output.suffix.lower()
    if ext in ('.jpg', '.jpeg'):
        img.save(str(output), format="JPEG", quality=95, subsampling=0)
    elif ext in ('.tif', '.tiff'):
        img.save(str(output), format="TIFF", compression="lzw")
    else:
        img.save(str(output))


# =============================================================================
# Output structure builder
# =============================================================================

def build_images_merged(project_folder: Path, results: List[dict],
                         fmt: str) -> Path:
    """
    Create project_folder/images_merged/<orbit>/<stack_id>/best_focus.<fmt>
    by symlinking (or copying) the stacked outputs.
    Returns the images_merged path.
    """
    merged = project_folder / "images_merged"
    merged.mkdir(exist_ok=True)

    for r in results:
        if not r.get("output") or not r["output"].exists():
            continue
        dest_dir = merged / r["orbit"] / r["stack_id"]
        dest_dir.mkdir(parents=True, exist_ok=True)
        dest = dest_dir / f"best_focus.{fmt}"
        if dest.exists():
            dest.unlink()
        # Symlink if on same filesystem, else copy
        try:
            dest.symlink_to(r["output"].resolve())
        except (OSError, NotImplementedError):
            shutil.copy2(r["output"], dest)

    return merged


# =============================================================================
# Main processing loop
# =============================================================================

def process_stack(entry: dict, engine: str, output_dir: Path,
                  fmt: str, align: bool, dry_run: bool) -> dict:
    """
    Process one slot within one stack.
    Returns updated entry dict with 'output', 'status', 'elapsed'.
    """
    orbit    = entry["orbit"]
    stack_id = entry["stack_id"]
    slot_id  = entry["slot_id"]
    images   = entry["images"]

    out_path = output_dir / orbit / stack_id / slot_id / f"best_focus.{fmt}"

    prefix = f"  [{orbit}/{stack_id}/{slot_id}]"
    print(f"{prefix}  {len(images)} frames  →  {out_path.name}", flush=True)

    if dry_run:
        return {**entry, "output": out_path, "status": "dry-run", "elapsed": 0}

    if len(images) < 2:
        print(f"{prefix}  SKIP (only {len(images)} image)", flush=True)
        # Copy single image as-is
        if images:
            out_path.parent.mkdir(parents=True, exist_ok=True)
            img = load_as_pil(images[0])
            _save_output(img, out_path)
            return {**entry, "output": out_path, "status": "copied", "elapsed": 0}
        return {**entry, "output": None, "status": "empty", "elapsed": 0}

    t0 = time.time()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        if engine == "enfuse":
            stack_enfuse(images, out_path, align=align)
        elif engine == "focus-stack":
            stack_focus_stack(images, out_path)
        else:
            stack_python(images, out_path)
        elapsed = time.time() - t0
        print(f"{prefix}  ✓  {elapsed:.1f}s", flush=True)
        return {**entry, "output": out_path, "status": "ok", "elapsed": elapsed}
    except Exception as e:
        print(f"{prefix}  ✗  {e}", flush=True)
        return {**entry, "output": None, "status": f"error: {e}", "elapsed": 0}


def open_in_affinity(paths: List[Path]) -> None:
    """Open result images in Affinity Photo 2 for review."""
    affinity = Path("/Applications/Affinity Photo 2.app")
    if not affinity.exists():
        print("Affinity Photo 2 not found at /Applications/Affinity Photo 2.app")
        return
    jpg_paths = [p for p in paths if p and p.exists()]
    if not jpg_paths:
        print("No output images to open.")
        return
    print(f"Opening {len(jpg_paths)} images in Affinity Photo 2...")
    subprocess.run(["open", "-a", str(affinity)] + [str(p) for p in jpg_paths])


# =============================================================================
# Entry point
# =============================================================================

def main():
    ap = argparse.ArgumentParser(
        description="PiSlider batch focus stacker — produces COLMAP-ready images"
    )
    ap.add_argument("project",    help="Project folder (contains project.json)")
    ap.add_argument("--orbit",    help="Process only this orbit label (e.g. orbit_001)")
    ap.add_argument("--slot",     help="Process only this slot id (e.g. slot_A)")
    ap.add_argument("--engine",   choices=["auto","enfuse","focus-stack","python"],
                    default="auto", help="Stacking engine (default: auto-detect)")
    ap.add_argument("--format",   choices=["jpg","jpeg","tiff","tif"],
                    default="jpg",  help="Output format (default: jpg)")
    ap.add_argument("--no-align", action="store_true",
                    help="Skip alignment step (enfuse engine only)")
    ap.add_argument("--dry-run",  action="store_true",
                    help="Show what would be done without processing")
    ap.add_argument("--open-in-affinity", action="store_true",
                    help="Open results in Affinity Photo 2 when done")
    ap.add_argument("--output-dir",
                    help="Custom output directory (default: project/stacked_output)")
    args = ap.parse_args()

    project = Path(args.project).expanduser().resolve()
    if not project.is_dir():
        print(f"ERROR: {project} is not a directory"); sys.exit(1)

    # Engine selection
    engine = args.engine
    if engine == "auto":
        engine = detect_engine()
        print(f"Engine: {engine}  (auto-detected)")
    else:
        print(f"Engine: {engine}")
    check_requirements(engine)

    fmt = args.format.lstrip(".")
    if fmt == "jpeg": fmt = "jpg"

    # Output directory
    out_dir = Path(args.output_dir) if args.output_dir \
              else project / "stacked_output"
    out_dir.mkdir(parents=True, exist_ok=True)

    # Discover stacks
    print(f"\nScanning {project} ...")
    stacks = find_stacks(project, args.orbit, args.slot)

    if not stacks:
        print("No stacks found. Check folder structure or --orbit/--slot filters.")
        sys.exit(1)

    # Summary
    total_images = sum(len(s["images"]) for s in stacks)
    print(f"\n{'DRY RUN — ' if args.dry_run else ''}Found {len(stacks)} stacks,"
          f" {total_images} source images\n")

    # Process
    results = []
    t_total = time.time()
    for i, entry in enumerate(stacks):
        print(f"[{i+1}/{len(stacks)}]", end=" ")
        result = process_stack(
            entry, engine, out_dir, fmt,
            align=not args.no_align,
            dry_run=args.dry_run
        )
        results.append(result)

    elapsed_total = time.time() - t_total

    # Build images_merged/ folder (symlinks to outputs for COLMAP)
    if not args.dry_run:
        merged = build_images_merged(project, results, fmt)
        print(f"\nCOLMAP images_merged → {merged}")

    # Summary report
    ok    = sum(1 for r in results if r["status"] == "ok")
    skip  = sum(1 for r in results if r["status"] in ("copied", "dry-run"))
    error = sum(1 for r in results if r["status"].startswith("error"))
    print(f"\n{'='*50}")
    print(f"Done in {elapsed_total:.0f}s")
    print(f"  Stacked : {ok}")
    print(f"  Skipped : {skip}  (single-image or dry-run)")
    print(f"  Errors  : {error}")

    if error:
        print("\nFailed stacks:")
        for r in results:
            if r["status"].startswith("error"):
                print(f"  {r['orbit']}/{r['stack_id']}/{r['slot_id']}: {r['status']}")

    # Affinity Photo handoff
    if args.open_in_affinity:
        outputs = [r.get("output") for r in results if r.get("output")]
        open_in_affinity(outputs)
    elif ok > 0 and not args.dry_run:
        print("\nTo review in Affinity Photo:")
        print(f"  python3 {Path(__file__).name} {project} --open-in-affinity")
        print("\nFor higher-quality stacking of individual positions in Affinity Photo:")
        print("  File > New > Focus Merge → select a stack_NNN/slot_A/ folder")

    print()


if __name__ == "__main__":
    main()
