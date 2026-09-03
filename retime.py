#!/usr/bin/env python3
"""
retime.py — PiSlider Motion Sidecar Time Remapper

Reads MOTION.json sidecars from multiple passes of the same programmed move
and produces output clips that are phase-aligned to a designated plate clip.
All outputs have identical frame counts — stack them in DaVinci at T=0 and
they are in sync with no manual trimming or speed adjustment needed.

─── Two entry points ───────────────────────────────────────────────────────

  CLI (timelapse image sequences):
    python retime.py --plate /Footage/plate --clips /Footage/clipB /Footage/clipC

  CLI (Sony video clips):
    python retime.py --plate /Footage/plate --clips /Footage/C0001.MP4 \
                     --sidecars /Footage/C0001.json

  DaVinci Resolve console (paste script first, then call):
    exec(open("/path/to/retime.py").read())
    retime_from_timeline(plate_track=1)

─── How the sync works ─────────────────────────────────────────────────────

  Every run writes a MOTION.json sidecar with:
    frames[i].phase      : 0.0 → 1.0 position in the programmed move
    frames[i].real_time_s: seconds since motion start for that frame
    motion_start_wall    : Unix timestamp when phase=0 fired

  For each plate output frame N:
    1. Look up phase_plate[N]
    2. Find frame M in clip B where phase_clip[M] == phase_plate[N]
       (solved via CubicSpline inverse on the clip's phase array)
    3. Output frame N of clip B = source frame M

  Result: all output sequences have the same frame count, every frame i
  has matched motion phase across all clips. Drop into DaVinci at T=0.

─── Output ─────────────────────────────────────────────────────────────────

  Timelapse sequences → retimed FRAME_XXXX.* files in a new folder
  Sony video clips    → ffmpeg command (run it; re-import the output)
  DaVinci mode        → retimed sequences written, script reports import paths

"""

import json
import math
import os
import re
import shutil
import sys
import urllib.parse
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline


# ── Sidecar loading ──────────────────────────────────────────────────────────

def find_sidecar(folder) -> Path:
    """Return the sidecar JSON path inside a folder.

    Checks MOTION.json first, then <clip_name>.json, then any .json
    that contains a 'frames' key.  Raises FileNotFoundError if none found.
    """
    folder = Path(folder)
    # 1. Canonical name
    if (folder / "MOTION.json").exists():
        return folder / "MOTION.json"
    # 2. Any .json with a 'frames' key (covers clip_XXXXXXXX.json, C0090.json, etc.)
    for f in sorted(folder.glob("*.json")):
        try:
            data = json.loads(f.read_text())
            if "frames" in data:
                return f
        except Exception:
            continue
    raise FileNotFoundError(f"No sidecar found in {folder}")


def load_sidecar(path):
    """Load a motion sidecar JSON.

    path can be:
      - a .json file directly
      - a folder (searches for any sidecar via find_sidecar)
      - a video/image file (looks for <stem>.json or any sidecar next to it)
    """
    p = Path(path)
    if p.is_dir():
        p = find_sidecar(p)
    elif p.suffix.lower() not in (".json",):
        # Treat as video/image — look for sidecar next to it
        candidates = [p.parent / f"{p.stem}.json", p.parent / "MOTION.json"]
        for c in candidates:
            if c.exists():
                p = c
                break
        else:
            p = find_sidecar(p.parent)
    with open(p) as f:
        data = json.load(f)
    _validate_sidecar(data, p)
    return data


def _validate_sidecar(data, path):
    if "frames" not in data or len(data["frames"]) < 2:
        raise ValueError(f"Sidecar has fewer than 2 frames: {path}")
    required = {"frame_idx", "phase", "real_time_s"}
    missing  = required - set(data["frames"][0].keys())
    if missing:
        raise ValueError(f"Sidecar missing fields {missing}: {path}")


# ── Core retime math ─────────────────────────────────────────────────────────

def compute_retime_map(plate_sidecar, clip_sidecar):
    """
    For each plate frame index N, return the source frame index in clip
    that has the same motion phase.

    Uses CubicSpline interpolation on the clip's (phase → frame_idx) curve
    so the result is smooth even when clip frame density differs from plate.

    Returns: float array, length = number of plate frames.
    """
    plate_frames = plate_sidecar["frames"]
    clip_frames  = clip_sidecar["frames"]

    plate_phases = np.array([f["phase"] for f in plate_frames])
    clip_phases  = np.array([f["phase"] for f in clip_frames])
    clip_indices = np.array([f["frame_idx"] for f in clip_frames], dtype=float)

    # Reversed clip: flip frame indices so phase=0 maps to the last clip frame.
    # This produces decreasing source_frames → negative speed keyframes in the XML.
    # Resolve plays the clip backward, matching a forward-moving plate.
    if clip_sidecar.get("reversed", False):
        clip_indices = clip_indices[::-1].copy()

    # Enforce monotonicity — floating point drift can cause tiny regressions
    for arr in (plate_phases, clip_phases):
        for i in range(1, len(arr)):
            if arr[i] <= arr[i - 1]:
                arr[i] = arr[i - 1] + 1e-9

    # Build clip inverse spline: phase → frame_idx
    clip_inv = CubicSpline(clip_phases, clip_indices)

    # Clamp plate phases to the clip's covered range before inverting
    lo, hi = clip_phases[0], clip_phases[-1]
    clamped = np.clip(plate_phases, lo, hi)
    source_frames = clip_inv(clamped)

    # Clamp result to valid frame range (min/max regardless of direction)
    idx_min = float(np.min(clip_indices))
    idx_max = float(np.max(clip_indices))
    source_frames = np.clip(source_frames, idx_min, idx_max)

    return source_frames


def retime_summary(plate_sidecar, clip_sidecar, source_frames):
    """Print a human-readable summary of the retime operation."""
    n_out = len(source_frames)
    n_src = len(clip_sidecar["frames"])

    plate_dur = plate_sidecar["frames"][-1]["real_time_s"]
    clip_dur  = clip_sidecar["frames"][-1]["real_time_s"]

    speeds = np.diff(source_frames)           # source frames consumed per output frame
    avg_speed  = float(np.mean(speeds))
    min_speed  = float(np.min(speeds))
    max_speed  = float(np.max(speeds))

    print(f"  Plate:  {n_out} frames  ({plate_dur:.1f}s real time)")
    print(f"  Source: {n_src} frames  ({clip_dur:.1f}s real time)")
    print(f"  Speed ratio  avg={avg_speed:.2f}×  min={min_speed:.2f}×  max={max_speed:.2f}×")
    if min_speed < 0.05:
        print(f"  ⚠ Very slow region detected — consider optical flow interpolation")


# ── Tilt keystoning correction ───────────────────────────────────────────────

def _resolve_tilt_factor(sidecar, tilt_factor):
    """Return tilt_factor to use, auto-computing from focal_mm when possible.

    Reference: tilt_factor = 35.0 / focal_mm  (full-frame 35mm equivalent).
    For APS-C cameras compensate for crop factor before calling this,
    or pass tilt_factor explicitly after calibrating on a known scene.

    Calibration: find the Pitch value that eliminates keystoning at a known
    tilt angle θ, then: tilt_factor = pitch_needed / sin(radians(θ)).
    """
    if tilt_factor is not None:
        return tilt_factor
    focal_mm = sidecar.get("focal_mm")
    if focal_mm and focal_mm > 0:
        return 35.0 / focal_mm
    return None


def _compute_tilt_correction(sidecar, tilt_factor=None):
    """Compute per-frame DaVinci Pitch correction values for tilt keystoning.

    Each frame stores pos_t (tilt axis degrees).  The perspective distortion
    introduced by tilting the camera is approximated as:

        pitch_correction = sin(radians(pos_t)) * tilt_factor

    where tilt_factor encodes lens FOV sensitivity (smaller focal length →
    more keystoning per degree of tilt → larger tilt_factor).

    Returns:
        list of (frame_idx, pitch_value) tuples, or None if pos_t is missing.
    """
    tf = _resolve_tilt_factor(sidecar, tilt_factor)
    if tf is None:
        print("  ⚠ tilt_factor unknown — pass tilt_factor= or add focal_mm to sidecar")
        return None

    frames = sidecar.get("frames", [])
    if not frames or "pos_t" not in frames[0]:
        print("  ⚠ Sidecar has no pos_t data — tilt correction skipped")
        return None

    corrections = []
    for f in frames:
        pos_t = f.get("pos_t", 0.0)
        pitch = math.sin(math.radians(pos_t)) * tf
        corrections.append((f["frame_idx"], round(pitch, 4)))

    print(f"  Tilt correction: focal_mm={sidecar.get('focal_mm','?')}  "
          f"tilt_factor={tf:.3f}  "
          f"pitch range [{min(c[1] for c in corrections):.3f} "
          f"… {max(c[1] for c in corrections):.3f}]")
    return corrections


def apply_tilt_correction_davinci(timeline_item, corrections, fps):
    """Apply per-frame Pitch keyframes to a DaVinci timeline item.

    DaVinci's Python API does not expose a per-frame SetProperty keyframe
    interface for Pitch/Yaw/Roll in the timeline inspector.  The cleanest
    approach is to export a CSV that the user imports via a Fusion tool or
    DaVinci Color page node.

    If SetProperty("Pitch", ...) becomes available in a future API version
    this function will use it directly; for now it writes a CSV and prints
    import instructions.
    """
    # Try the direct API first (may work in some Resolve versions)
    try:
        item = timeline_item
        # SetClipProperty exists but Pitch is on the inspector level
        # Try anyway — fails silently on unsupported versions
        for frame_idx, pitch in corrections:
            tc = frame_idx  # frame number
            item.SetProperty("Pitch", pitch)   # last value wins if no keyframe API
        print("  Applied Pitch via SetProperty (static — no per-frame keyframes)")
    except Exception:
        pass

    # Reliable fallback: write a CSV for manual keyframe import
    try:
        src_path = _source_path(timeline_item)
        if src_path:
            csv_path = src_path.parent / f"{src_path.stem}_tilt_correction.csv"
            _write_tilt_csv(corrections, fps, csv_path)
    except Exception as e:
        print(f"  Warning: could not write tilt CSV: {e}")


def _write_tilt_csv(corrections, fps, csv_path):
    """Write frame_idx, timecode, pitch to a CSV for manual DaVinci import."""
    csv_path = Path(csv_path)
    with open(csv_path, "w") as fh:
        fh.write("frame_idx,timecode_s,pitch\n")
        for frame_idx, pitch in corrections:
            tc_s = frame_idx / fps if fps else frame_idx
            fh.write(f"{frame_idx},{tc_s:.6f},{pitch}\n")
    print(f"  Tilt correction CSV → {csv_path}")
    print(f"  In DaVinci: add an 'Edit > Inspector > Transform > Pitch' keyframe track,")
    print(f"  then paste these values or use a Fusion CSV input node.")


def write_tilt_sidecar(sidecar, output_folder, tilt_factor=None):
    """Write a _tilt_correction.csv next to output_folder's MOTION.json.

    Call this after retime_image_sequence() to save keystoning corrections
    alongside the retimed frames, so DaVinci colour operators can read them.
    """
    corrections = _compute_tilt_correction(sidecar, tilt_factor)
    if corrections is None:
        return None
    fps = sidecar.get("camera_fps") or 24
    csv_path = Path(output_folder) / "tilt_correction.csv"
    _write_tilt_csv(corrections, fps, csv_path)
    return csv_path


# ── Image sequence retiming ──────────────────────────────────────────────────

def retime_image_sequence(plate_sidecar, clip_sidecar, clip_folder, output_folder,
                          correct_tilt=False, tilt_factor=None):
    """
    Produce a retimed image sequence in output_folder.

    For each plate frame N, copies the source frame M (nearest integer)
    from clip_folder. Output has the same frame count as the plate.

    RAW files (ARW/DNG/RAF) are copied losslessly — no re-encoding.
    The output MOTION.json is the plate sidecar so this clip can itself
    be used as a plate for further passes.

    If correct_tilt=True, also writes a tilt_correction.csv alongside the
    retimed frames. Import the CSV into a DaVinci Fusion/Color pitch node
    to correct vertical keystoning caused by tilt axis movement.
    tilt_factor overrides the auto-computed value (35.0 / focal_mm).
    """
    clip_folder   = Path(clip_folder)
    output_folder = Path(output_folder)
    output_folder.mkdir(parents=True, exist_ok=True)

    source_frames = compute_retime_map(plate_sidecar, clip_sidecar)
    retime_summary(plate_sidecar, clip_sidecar, source_frames)

    # Find source image files
    source_files = _find_frame_files(clip_folder)
    if not source_files:
        raise FileNotFoundError(f"No FRAME_* image files found in {clip_folder}")

    ext     = source_files[0].suffix
    max_idx = len(source_files) - 1
    n_out   = len(source_frames)

    print(f"  Writing {n_out} frames to {output_folder} …")
    for out_idx, src_float in enumerate(source_frames):
        src_int  = int(round(float(src_float)))
        src_int  = max(0, min(max_idx, src_int))
        src_file = source_files[src_int]
        dst_file = output_folder / f"FRAME_{out_idx + 1:04d}{ext}"
        shutil.copy2(src_file, dst_file)

        if (out_idx + 1) % 50 == 0 or out_idx == n_out - 1:
            print(f"    [{out_idx + 1}/{n_out}] ← {src_file.name}")

    # Write plate sidecar into retimed folder so DaVinci sees it as phase-aligned
    out_sidecar = dict(plate_sidecar)
    out_sidecar["retimed_from"] = clip_sidecar.get("run_id", str(clip_folder))
    with open(output_folder / "MOTION.json", "w") as fh:
        json.dump(out_sidecar, fh, separators=(",", ":"))

    if correct_tilt:
        write_tilt_sidecar(clip_sidecar, output_folder, tilt_factor=tilt_factor)

    print(f"  ✓ {output_folder}")
    return output_folder


_VIDEO_EXTS = {".mp4", ".mov", ".mxf", ".avi", ".m4v"}

def _find_frame_files(folder):
    """Return sorted list of FRAME_XXXX.* image files in folder."""
    folder = Path(folder)
    for ext in (".ARW", ".DNG", ".RAF", ".JPG", ".JPEG", ".PNG", ".TIFF"):
        files = sorted(folder.glob(f"FRAME_*{ext}"))
        if files:
            return files
        files = sorted(folder.glob(f"FRAME_*{ext.lower()}"))
        if files:
            return files
    # Fallback: any numbered image sequence
    return sorted(folder.glob("FRAME_*.*"))


def _find_video_file(folder):
    """Return the first video file (MP4/MOV/etc.) in folder, or None."""
    folder = Path(folder)
    for f in sorted(folder.iterdir()):
        if f.suffix.lower() in _VIDEO_EXTS:
            return f
    return None


def detect_flash_frame(video_path, max_seconds: float = 60.0):
    """Find the video frame with peak average luminance (the flash frame).

    Uses ffprobe's signalstats lavfi filter to measure per-frame YAVG.
    Returns (frame_index, peak_luma) or (None, None) on failure.

    Accuracy: ±1 frame.  Requires ffprobe in PATH.
    """
    import subprocess, shutil
    video_path = Path(video_path)
    if not video_path.exists() or not shutil.which("ffprobe"):
        return None, None

    # Single-quote the path so spaces work in the lavfi filter string.
    # Escape any single quotes or colons already in the path itself.
    escaped = (str(video_path.resolve())
               .replace("\\", "\\\\")
               .replace("'",  "\\'")
               .replace(":",  "\\:"))
    lavfi_input = f"movie='{escaped}',signalstats"

    try:
        result = subprocess.run(
            [
                "ffprobe", "-v", "error",
                "-f", "lavfi", "-i", lavfi_input,
                "-show_entries", "frame_tags=lavfi.signalstats.YAVG",
                "-read_intervals", f"%+{int(max_seconds)}",
                "-of", "csv=nokey=1:p=0",
            ],
            capture_output=True, text=True,
            timeout=int(max_seconds) + 30,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None, None

    peak_frame = None
    peak_luma  = -1.0
    for i, line in enumerate(result.stdout.splitlines()):
        line = line.strip()
        if not line:
            continue
        try:
            y = float(line)
            if y > peak_luma:
                peak_luma  = y
                peak_frame = i
        except ValueError:
            continue

    return (peak_frame, peak_luma) if peak_frame is not None else (None, None)


# ── Sony video clip retiming via ffmpeg ──────────────────────────────────────

def retime_video_ffmpeg(plate_sidecar, clip_sidecar, clip_video_path,
                        output_path=None, video_fps=None):
    """Generate an ffmpeg command that produces a phase-aligned retimed video clip.

    The sidecar's frames[] describe the motion period only (phase=0 onward).
    If the video file was started before motion (Record + Run pre-roll), the
    sidecar stores pre_roll_s so we offset into the file correctly.

    Output matches the plate frame count; minterp provides smooth interpolation
    between pulled source frames for time-ramp regions.
    """
    clip_path = Path(clip_video_path)
    if output_path is None:
        output_path = clip_path.parent / f"{clip_path.stem}_retimed{clip_path.suffix}"

    # FPS: explicit arg > sidecar camera_fps > unknown
    if video_fps is None:
        video_fps = clip_sidecar.get("camera_fps")

    # HFR advisory
    if clip_sidecar.get("camera_hfr"):
        q    = clip_sidecar.get("camera_quality", "?")
        cfps = video_fps or "?"
        print(f"\n  HFR recording: {q} ({cfps}fps in sidecar)")
        print(f"  A7III XAVC S HD: container IS {cfps}fps — no adjustment needed.")
        print(f"  S&Q / other bodies: if DaVinci shows 24/30fps, pass --fps <container_fps>")
        print(f"  Verify: ffprobe -v 0 -select_streams v:0 -show_entries stream=r_frame_rate "
              f"-of csv=p=0 \"{clip_path}\"\n")

    source_frames = compute_retime_map(plate_sidecar, clip_sidecar)
    retime_summary(plate_sidecar, clip_sidecar, source_frames)

    # Pre-roll: frames in the video file before phase=0.
    # Stored directly in cinema sidecars; timelapse sidecars have none.
    pre_roll_s      = float(clip_sidecar.get("pre_roll_s", 0.0))
    pre_roll_frames = round(pre_roll_s * video_fps) if video_fps else 0

    # Absolute video frame indices (motion frames offset by pre-roll)
    abs_frames = [int(round(float(f))) + pre_roll_frames for f in source_frames]

    if video_fps:
        frame_dur   = 1.0 / video_fps
        concat_path = clip_path.parent / f"{clip_path.stem}_retime_list.txt"
        with open(concat_path, "w") as fh:
            for abs_f in abs_frames:
                src_pts = abs_f * frame_dur
                fh.write(f"file '{clip_path.resolve()}'\n")
                fh.write(f"inpoint  {src_pts:.6f}\n")
                fh.write(f"outpoint {src_pts + frame_dur:.6f}\n")

        cmd = (
            f"ffmpeg -f concat -safe 0 -i \"{concat_path}\" \\\n"
            f"  -vf minterp=fps={video_fps}:mi_mode=mci \\\n"
            f"  -c:v libx264 -crf 18 -preset slow \\\n"
            f"  \"{output_path}\""
        )
    else:
        # FPS unknown — emit a simpler select command the user can adapt
        select_expr = "+".join([f"eq(n\\,{f})" for f in abs_frames])
        cmd = (
            f"# Set FPS to match your clip's actual frame rate\n"
            f"ffmpeg -i \"{clip_path}\" \\\n"
            f"  -vf \"select='{select_expr}',setpts=N/FPS/TB\" \\\n"
            f"  -vsync vfr \"{output_path}\""
        )

    print(f"\n  ffmpeg command (run this, then import {output_path.name} into DaVinci):")
    print(f"  {cmd}\n")
    return cmd, output_path


# ── DaVinci Resolve console entry point ─────────────────────────────────────

def retime_from_timeline(plate_track=1, output_root=None,
                         correct_tilt=False, tilt_factor=None):
    """
    Run this from the DaVinci Resolve scripting console after exec()-ing this file.

    Finds clips on the current timeline, reads their MOTION.json sidecars,
    produces phase-aligned retimed sequences, and reports the import paths.
    All output sequences have identical frame counts — import them and place
    at T=0 on separate tracks; they are in sync.

    Args:
      plate_track  : 1-based video track number containing the plate clip.
      output_root  : root folder for retimed output (default: sibling of each
                     source folder, named <folder>_retimed).
      correct_tilt : Write tilt_correction.csv next to each retimed sequence.
                     Corrects vertical keystoning from tilt-axis movement.
                     Auto-uses focal_mm from sidecar (35.0/focal_mm factor).
      tilt_factor  : Override tilt sensitivity (pitch_deg per sin(tilt_rad)).
                     Calibrate once: tilt_factor = pitch_needed / sin(radians(θ)).
                     Omit to auto-compute from focal_mm stored in sidecar.

    Usage:
      exec(open("/home/tim/retime.py").read())
      retime_from_timeline(plate_track=1)
      retime_from_timeline(plate_track=1, correct_tilt=True)
      retime_from_timeline(plate_track=1, correct_tilt=True, tilt_factor=1.2)
    """
    resolve = _get_resolve()
    if resolve is None:
        return

    project  = resolve.GetProjectManager().GetCurrentProject()
    timeline = project.GetCurrentTimeline()
    if not timeline:
        print("No active timeline.")
        return

    n_tracks = timeline.GetTrackCount("video")
    print(f"Timeline: '{timeline.GetName()}'  ({n_tracks} video tracks)")

    # ── Load plate ────────────────────────────────────────────────────────────
    plate_items = timeline.GetItemListInTrack("video", plate_track) or []
    if not plate_items:
        print(f"No clips on plate track {plate_track}.")
        return

    plate_src = _source_path(plate_items[0])
    if plate_src is None:
        print("Cannot determine source path for plate clip.")
        return

    plate_sidecar = _sidecar_for_path(plate_src)
    if plate_sidecar is None:
        print(f"No MOTION.json found for plate clip at {plate_src}")
        return

    print(f"\nPlate  track {plate_track}: {plate_src.name}")
    print(f"         {len(plate_sidecar['frames'])} frames  "
          f"run_id={plate_sidecar.get('run_id','?')}")

    import_paths = []   # paths to tell the user to import

    # ── Process each non-plate track ─────────────────────────────────────────
    for track_idx in range(1, n_tracks + 1):
        if track_idx == plate_track:
            continue

        items = timeline.GetItemListInTrack("video", track_idx) or []
        for item in items:
            src = _source_path(item)
            if src is None:
                continue

            sidecar = _sidecar_for_path(src)
            if sidecar is None:
                print(f"\n  Track {track_idx}: no sidecar for {src.name} — skipping")
                continue

            print(f"\n  Track {track_idx}: {src.name}")
            print(f"    {len(sidecar['frames'])} frames  "
                  f"run_id={sidecar.get('run_id','?')}")

            mode = sidecar.get("mode", "timelapse")

            if mode == "timelapse":
                clip_folder   = src.parent
                out_dir       = (Path(output_root) / clip_folder.name
                                 if output_root
                                 else clip_folder.parent / f"{clip_folder.name}_retimed")
                retime_image_sequence(plate_sidecar, sidecar, clip_folder, out_dir,
                                      correct_tilt=correct_tilt, tilt_factor=tilt_factor)
                import_paths.append(out_dir)

            else:
                # Video clip — emit ffmpeg command; tilt CSV written separately
                _, out_path = retime_video_ffmpeg(plate_sidecar, sidecar, src)
                if correct_tilt:
                    write_tilt_sidecar(sidecar, out_path.parent, tilt_factor=tilt_factor)
                import_paths.append(out_path.parent)

    # ── Import summary ────────────────────────────────────────────────────────
    if import_paths:
        print("\n" + "─" * 60)
        print("Import these into DaVinci and place at T=0 on separate tracks:")
        for p in import_paths:
            print(f"  {p}")
        print("\nAll clips are phase-aligned to the plate — no speed adjustment needed.")
        print("Set each clip to Optical Flow for smooth time-ramped playback.")
    else:
        print("\nNo clips processed.")


# ── DaVinci API helpers ───────────────────────────────────────────────────────

def _get_resolve():
    """Connect to DaVinci Resolve — works from console (resolve already bound)
    or from an external script via DaVinciResolveScript."""
    # Console: 'resolve' is a built-in global
    g = globals()
    if "resolve" in g and g["resolve"] is not None:
        return g["resolve"]
    # Built-in in newer console versions
    try:
        import builtins
        if hasattr(builtins, "resolve"):
            return builtins.resolve
    except Exception:
        pass
    # External script path
    try:
        import DaVinciResolveScript as dvr
        return dvr.scriptapp("Resolve")
    except ImportError:
        pass
    print("Cannot connect to DaVinci Resolve.  Run this from the Resolve scripting console.")
    return None


def _source_path(timeline_item):
    """Return the source file Path for a TimelineItem, or None."""
    try:
        mpi = timeline_item.GetMediaPoolItem()
        if mpi is None:
            return None
        p = mpi.GetClipProperty("File Path")
        return Path(p) if p else None
    except Exception:
        return None


def _sidecar_for_path(src_path):
    """Find the MOTION.json sidecar closest to src_path."""
    candidates = [
        src_path.parent / f"{src_path.stem}.json",   # C0001.json
        src_path.parent / "MOTION.json",              # folder-level sidecar
    ]
    for c in candidates:
        if c.exists():
            try:
                return load_sidecar(c)
            except Exception as e:
                print(f"  Warning: could not parse {c}: {e}")
    return None


# ── FCP7 XML timeline generator with speed keyframes ─────────────────────────

def _graphdict_keyframes(source_frames, pre_roll=0, step=3):
    """Convert a source-frame map to (output_frame, absolute_source_frame) keyframes.

    Downsampled by `step` to keep keyframe count manageable.
    """
    N  = len(source_frames)
    kf = []
    indices = list(range(0, N - 1, step)) + [N - 1]
    seen = set()
    for i in sorted(set(indices)):
        if i in seen:
            continue
        seen.add(i)
        val = round(float(source_frames[i]) + pre_roll, 3)
        kf.append((i, val))
    return kf


def generate_fcp7_xml(plate_folder, clip_folders, output_xml,
                      fps=24, sequence_name="PiSlider Retime"):
    """Write a FCP7 XML timeline Resolve can import with retime curves intact.

    Each clip goes on its own video track. The plate (track 1) plays at
    normal speed. Every other clip has speed keyframes that encode the
    phase-matching retime — import the XML, set each retimed clip to
    Optical Flow in Resolve's Inspector, then render.

    References ORIGINAL clip folders — no frame copying required.

    Import: File → Import Timeline → select .xml
    """
    plate_folder  = Path(plate_folder)
    plate_sidecar = load_sidecar(plate_folder)
    N             = len(plate_sidecar["frames"])

    ntsc     = fps in (23.976, 29.97, 47.952, 59.94)
    timebase = {23.976: 24, 29.97: 30, 47.952: 48, 59.94: 60}.get(fps, int(fps))
    ns       = "TRUE" if ntsc else "FALSE"

    def rate(ind=""):
        return f"{ind}<rate><timebase>{timebase}</timebase><ntsc>{ns}</ntsc></rate>"

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<!DOCTYPE xmeml>',
        '<xmeml version="5">',
        f'<sequence id="seq1">',
        f'  <name>{sequence_name}</name>',
        f'  <duration>{N}</duration>',
        f'  {rate()}',
        '  <media><video>',
        '    <format><samplecharacteristics>',
        f'      {rate()}',
        '    </samplecharacteristics></format>',
    ]

    all_folders = [plate_folder] + [Path(c) for c in clip_folders]

    for idx, folder in enumerate(all_folders):
        is_plate = (idx == 0)
        label    = "plate" if is_plate else f"clip_{idx}"

        frame_files = _find_frame_files(folder)
        video_file  = None if frame_files else _find_video_file(folder)

        if not frame_files and not video_file:
            print(f"  Warning: no frames or video in {folder} — skipping")
            continue

        # Retime map for non-plate clips (load sidecar regardless of source type)
        source_frames = None
        folder_sidecar = None
        if not is_plate:
            try:
                folder_sidecar = load_sidecar(folder)
                source_frames  = compute_retime_map(plate_sidecar, folder_sidecar)
                retime_summary(plate_sidecar, folder_sidecar, source_frames)
            except Exception as e:
                print(f"  Warning: retime failed for {folder.name}: {e}")

        cid = f"clipitem{idx + 1}"
        fid = f"file{idx + 1}"

        if frame_files:
            # Image sequence — no pre-roll concept; frame 0 is always motion frame 0
            source_file     = frame_files[0]
            src_count       = len(frame_files)
            clip_name       = source_file.name
            pre_roll_frames = 0
        else:
            # Video file — derive frame count and pre-roll offset from sidecar
            source_file = video_file
            sc          = folder_sidecar or (plate_sidecar if is_plate else None)
            if sc:
                cam_fps         = float(sc.get("camera_fps") or fps)
                dur_s           = float(sc.get("duration_s") or sc["frames"][-1]["real_time_s"])
                pre_roll_s      = float(sc.get("pre_roll_s") or 0.0)
                src_count       = max(1, round(cam_fps * dur_s))
                pre_roll_frames = round(pre_roll_s * cam_fps)
            else:
                src_count       = N
                pre_roll_frames = 0
            clip_name = source_file.name

            # Flash-sync correction: if the sidecar recorded a flash_sync_wall
            # timestamp, find the flash frame in the video and compute the exact
            # pre-roll rather than relying on the nominal pre_roll_s value.
            # formula: pre_roll = (motion_start_wall - flash_sync_wall) * fps + flash_frame
            flash_sync_wall  = sc.get("flash_sync_wall") if sc else None
            motion_start_wall = sc.get("motion_start_wall") if sc else None
            if flash_sync_wall and motion_start_wall:
                flash_frame, flash_luma = detect_flash_frame(
                    source_file, max_seconds=max(30.0, pre_roll_s * 2 + 5))
                if flash_frame is not None and flash_luma is not None and flash_luma > 150:
                    corrected = int(round(
                        (motion_start_wall - flash_sync_wall) * cam_fps
                    )) + flash_frame
                    if corrected >= 0:
                        print(f"  ⚡ Flash sync: frame {flash_frame} "
                              f"(luma={flash_luma:.0f}), "
                              f"pre-roll {pre_roll_frames} → {corrected} frames "
                              f"({corrected/cam_fps:.3f}s)")
                        pre_roll_frames = corrected
                    else:
                        print(f"  ⚠ Flash sync: frame {flash_frame} gives negative "
                              f"pre-roll — using nominal {pre_roll_frames} frames")
                elif flash_frame is not None:
                    print(f"  ⚠ Flash sync: brightest frame {flash_frame} "
                          f"(luma={flash_luma:.0f}) below threshold — no flash detected, "
                          f"using nominal pre-roll")
                else:
                    print(f"  ⚠ Flash sync: could not analyse {source_file.name} "
                          f"(ffprobe unavailable?) — using nominal pre-roll")

            print(f"  Video source: {source_file.name}  ({src_count} frames @ {fps} fps, "
                  f"{pre_roll_frames} pre-roll frames)")

        # Compute source in/out so pre-roll is skipped and the full motion is covered.
        # Adding pre_roll_frames converts to absolute video frame indices.
        total_video_frames = pre_roll_frames + src_count
        if is_plate:
            clip_in  = pre_roll_frames
            clip_out = pre_roll_frames + N
        elif source_frames is not None:
            clip_in  = int(round(float(np.min(source_frames)))) + pre_roll_frames
            clip_out = int(round(float(np.max(source_frames)))) + pre_roll_frames + 1
        else:
            clip_in  = pre_roll_frames
            clip_out = pre_roll_frames + src_count

        # URL encode absolute path — Resolve's XML importer requires strictly compliant
        # URIs, escaping spaces and special characters.
        path_url = "file://localhost" + urllib.parse.quote(str(source_file.resolve()), safe='/:')

        lines += [
            f'    <track>',
            f'      <clipitem id="{cid}">',
            f'        <name>{clip_name}</name>',
            f'        <duration>{N}</duration>',
            f'        {rate("        ")}',
            f'        <start>0</start>',
            f'        <end>{N}</end>',
            f'        <in>{clip_in}</in>',
            f'        <out>{clip_out}</out>',
            f'        <file id="{fid}">',
            f'          <name>{clip_name}</name>',
            f'          <pathurl>{path_url}</pathurl>',
            f'          {rate("          ")}',
            f'          <duration>{total_video_frames}</duration>',
            f'          <timecode>',
            f'            {rate("            ")}',
            f'            <string>00:00:00:00</string>',
            f'            <frame>0</frame>',
            f'            <displayformat>NDF</displayformat>',
            f'          </timecode>',
            f'          <media><video>',
            f'            <duration>{total_video_frames}</duration>',
            f'            <samplecharacteristics>{rate()}</samplecharacteristics>',
            f'          </video></media>',
            f'        </file>',
        ]

        if source_frames is not None:
            kfs = _graphdict_keyframes(source_frames, pre_roll=pre_roll_frames)
            lines += [
                f'        <filter><effect>',
                f'          <name>Time Remap</name>',
                f'          <effectid>timeremap</effectid>',
                f'          <effecttype>motion</effecttype>',
                f'          <mediatype>video</mediatype>',
                f'          <parameter id="graphdict">',
                f'            <name>graphdict</name>',
                f'            <valuemin>0</valuemin>',
                f'            <valuemax>{total_video_frames}</valuemax>',
            ]
            for when, src_frame in kfs:
                lines.append(
                    f'            <keyframe><when>{when}</when>'
                    f'<value>{src_frame}</value>'
                    f'<interp>linear</interp></keyframe>'
                )
            lines += [
                f'          </parameter>',
                f'        </effect></filter>',
            ]

        lines += [
            f'      </clipitem>',
            f'    </track>',
        ]

    lines += ['  </video></media>', '</sequence>', '</xmeml>']

    out = Path(output_xml)
    out.write_text("\n".join(lines), encoding="utf-8")
    print(f"\n  Timeline XML → {out}")
    return out


def retime_and_generate_xml(plate_folder, clip_folders,
                             output_root=None, fps=24,
                             correct_tilt=False, tilt_factor=None,
                             sequence_name="PiSlider Retime"):
    """Compute phase-match retime curves and write an importable Resolve XML.

    No frames are copied. Original clip folders are referenced directly.
    Speed keyframes in the XML encode the retime so Resolve can apply
    Optical Flow and render the final output.

    After import:
      1. Select each retimed clip on tracks 2+
      2. Inspector → Retime and Scaling → Retime Process → Optical Flow
      3. Render

    Returns path to the generated .xml file.
    """
    plate_folder = Path(plate_folder)
    print(f"Plate : {plate_folder.name}")
    for i, c in enumerate(clip_folders):
        print(f"Clip {i+1}: {Path(c).name}")

    if sequence_name == "PiSlider Retime":
        video_file = _find_video_file(plate_folder)
        if video_file:
            sequence_name = f"{video_file.stem}_Retime"
        else:
            frames = _find_frame_files(plate_folder)
            if frames:
                sequence_name = f"{plate_folder.name}_Retime"

    xml_path = (
        Path(output_root) / f"{sequence_name}.xml"
        if output_root
        else plate_folder.parent / f"{sequence_name}.xml"
    )

    generate_fcp7_xml(
        plate_folder=plate_folder,
        clip_folders=clip_folders,
        output_xml=xml_path,
        fps=fps,
        sequence_name=sequence_name,
    )

    if correct_tilt:
        plate_sidecar = load_sidecar(plate_folder)
        for clip_path in clip_folders:
            clip_path = Path(clip_path)
            try:
                sc = load_sidecar(clip_path)
                write_tilt_sidecar(sc, clip_path, tilt_factor=tilt_factor)
            except Exception as e:
                print(f"  Tilt CSV skipped for {clip_path.name}: {e}")

    print(f"\n{'─'*60}")
    print(f"Import into DaVinci Resolve:")
    print(f"  File → Import Timeline → {xml_path.name}")
    print(f"")
    print(f"Then for each retimed clip (tracks 2+):")
    print(f"  Inspector → Retime and Scaling → Retime Process → Optical Flow")
    return xml_path


# ── CLI entry point ───────────────────────────────────────────────────────────

def _cli():
    import argparse

    parser = argparse.ArgumentParser(
        description="PiSlider motion sidecar time remapper",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--plate", required=True,
        help="Plate: folder containing MOTION.json, or path to MOTION.json itself",
    )
    parser.add_argument(
        "--clips", required=True, nargs="+",
        help="One or more clip folders (timelapse) or video files (Sony)",
    )
    parser.add_argument(
        "--sidecars", nargs="*", default=None,
        help="Sidecar JSON paths for each --clips entry (video only; "
             "defaults to <stem>.json next to the video file)",
    )
    parser.add_argument(
        "--output", default=None,
        help="Root output folder (default: sibling of each clip folder)",
    )
    parser.add_argument(
        "--fps", type=float, default=None,
        help="Source video frame rate override — read from sidecar automatically "
             "when captured via Sony WiFi; only needed if sidecar lacks camera_fps",
    )
    parser.add_argument(
        "--correct-tilt", action="store_true", default=False,
        help="Write tilt_correction.csv alongside each retimed output for "
             "per-frame DaVinci Pitch keystoning correction",
    )
    parser.add_argument(
        "--tilt-factor", type=float, default=None,
        help="Tilt sensitivity override (pitch_deg / sin(tilt_rad)). "
             "Auto-computed from focal_mm in sidecar when omitted.",
    )
    args = parser.parse_args()

    plate_sidecar = load_sidecar(args.plate)
    print(f"Plate: {args.plate}")
    print(f"  {len(plate_sidecar['frames'])} frames  "
          f"run_id={plate_sidecar.get('run_id','?')}")

    for i, clip_arg in enumerate(args.clips):
        clip_path = Path(clip_arg)
        print(f"\nClip {i + 1}: {clip_arg}")

        # Resolve sidecar
        if args.sidecars and i < len(args.sidecars):
            clip_sidecar = load_sidecar(args.sidecars[i])
        else:
            clip_sidecar = load_sidecar(clip_path)

        print(f"  {len(clip_sidecar['frames'])} frames  "
              f"run_id={clip_sidecar.get('run_id','?')}")

        mode = clip_sidecar.get("mode", "timelapse")

        if mode == "timelapse" or clip_path.is_dir():
            clip_folder = clip_path if clip_path.is_dir() else clip_path.parent
            out_dir     = (Path(args.output) / f"clip{i + 1:02d}_retimed"
                           if args.output
                           else clip_folder.parent / f"{clip_folder.name}_retimed")
            retime_image_sequence(plate_sidecar, clip_sidecar, clip_folder, out_dir,
                                  correct_tilt=args.correct_tilt,
                                  tilt_factor=args.tilt_factor)

        else:
            _, out_path = retime_video_ffmpeg(plate_sidecar, clip_sidecar, clip_path,
                                              video_fps=args.fps)
            if args.correct_tilt:
                write_tilt_sidecar(clip_sidecar, out_path.parent,
                                   tilt_factor=args.tilt_factor)

    print("\nDone.")


if __name__ == "__main__":
    _cli()
