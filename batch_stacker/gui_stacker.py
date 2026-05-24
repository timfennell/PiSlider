#!/usr/bin/env python3
"""
PiSlider Batch Focus Stacker — standalone GUI app.
Depends on shinestacker.app installed in /Applications/.

All stacking runs in a single subprocess that loads shinestacker once, then
processes each stack as a complete StackJob (align → merge) in sequence.
Each job's intermediate aligned TIFFs live in an isolated temp directory and
are deleted immediately after that job finishes — before the next one starts.
"""
import os
import sys
import glob
import json
import shutil
import subprocess
import tempfile
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext
from pathlib import Path

# ── Constants ─────────────────────────────────────────────────────────────────
_SS_APP_RESOURCES = "/Applications/shinestacker.app/Contents/Resources"

# ── Batch runner script ───────────────────────────────────────────────────────
# Written to a temp file and executed by the system Python once per batch run.
# Receives a JSON jobs file; processes every stack in sequence; prints markers.
_STACK_RUNNER_SCRIPT = '''\
#!/usr/bin/env python3
"""
Shinestacker batch runner — one subprocess for the entire job.
Reads a JSON list of {"src": "...", "dest": "..."} entries.
For each entry: align all raw frames, merge, save result, clean up temp files.
Aligned TIFFs exist only for the duration of their own stack job.
"""
import sys, os, json, traceback, tempfile, shutil
from pathlib import Path

ss_res = '/Applications/shinestacker.app/Contents/Resources'
if os.path.isdir(ss_res) and ss_res not in sys.path:
    sys.path.insert(0, ss_res)

# Raw/DNG/TIFF only — no preview JPEGs, no macOS ._ resource-fork files
RAW_EXTS = {'.dng', '.arw', '.cr3', '.nef', '.tif', '.tiff'}

try:
    from shinestacker import StackJob, CombinedActions, FocusStack
    from shinestacker.algorithms import AlignFrames, BalanceFrames, PyramidStack
except Exception as e:
    print(f"RUNNER_ERROR:import failed: {e}", flush=True)
    traceback.print_exc()
    sys.exit(1)

jobs_file = sys.argv[1]
with open(jobs_file) as f:
    jobs = json.load(f)

print(f"BATCH_START:{len(jobs)}", flush=True)

for i, job_info in enumerate(jobs, 1):
    src_path = Path(job_info["src"])
    dest     = Path(job_info["dest"])

    print(f"STACK_START:{i}:{len(jobs)}:{src_path.parent.name}/{src_path.name}", flush=True)

    raw_files = sorted([
        f for f in src_path.iterdir()
        if f.suffix.lower() in RAW_EXTS
        and not f.name.startswith("._")
        and "_preview" not in f.name
        and "best_focus" not in f.name
    ])

    if not raw_files:
        print(f"STACK_FAILED:{src_path}:no raw files found", flush=True)
        continue

    print(f"[runner] {len(raw_files)} raw frame(s)", flush=True)

    # Temp dir scoped to THIS job — aligned TIFFs are deleted when the
    # with-block exits, before the next job even starts.
    with tempfile.TemporaryDirectory(prefix="stackbatch_") as tmpdir:
        frames_dir = Path(tmpdir) / "frames"
        frames_dir.mkdir()
        for f in raw_files:
            os.symlink(f, frames_dir / f.name)

        try:
            job = StackJob(name=f"stack_{i:03d}",
                           working_path=tmpdir,
                           input_path="frames")
            job.add_action(CombinedActions("aligned",
                                           actions=[AlignFrames(), BalanceFrames()]))
            job.add_action(FocusStack("stacked", PyramidStack()))
            print(f"PHASE:aligning:{len(raw_files)}", flush=True)
            job.run()
            print(f"PHASE:stacking:{len(raw_files)}", flush=True)
        except Exception as e:
            print(f"STACK_FAILED:{src_path}:{e}", flush=True)
            traceback.print_exc()
            continue  # with-block exits → temp dir cleaned up

        stacked_dir = Path(tmpdir) / "stacked"
        outputs = []
        if stacked_dir.exists():
            outputs = sorted(stacked_dir.glob("*.*"),
                             key=lambda p: p.stat().st_mtime, reverse=True)
            outputs = [p for p in outputs
                       if p.suffix.lower() in (".jpg", ".jpeg", ".tif", ".tiff", ".png")]

        if not outputs:
            print(f"STACK_FAILED:{src_path}:no output in stacked/", flush=True)
            continue

        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(outputs[0], dest)
        print(f"STACK_DONE:{src_path}:{dest}", flush=True)
    # ← with-block exits here: aligned TIFFs gone, next job starts clean

print("[runner] all jobs complete", flush=True)
'''


# ── Helpers ───────────────────────────────────────────────────────────────────

def _find_python() -> str | None:
    """Return path to a usable Python 3 on macOS, or None."""
    import shutil as _sh
    for candidate in [
        '/opt/homebrew/bin/python3.14',
        '/opt/homebrew/bin/python3',
        '/usr/local/bin/python3.14',
        '/usr/local/bin/python3',
        '/usr/bin/python3',
    ]:
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    return _sh.which('python3') or _sh.which('python')


def _check_shinestacker() -> str | None:
    """Return None if ready, or a human-readable error string."""
    if not os.path.isdir(_SS_APP_RESOURCES):
        return (
            "shinestacker.app not found in /Applications/\n\n"
            "Please install shinestacker.app there.\n"
            "Download from: https://www.shinestacker.com"
        )
    if not _find_python():
        return (
            "No Python 3 interpreter found.\n\n"
            "Install Python 3 via Homebrew:  brew install python3"
        )
    return None


# ── Batch orchestrator ────────────────────────────────────────────────────────

def batch_process_project(parent_dir, log_fn=print) -> int:
    """
    Walk a PiSlider macro project folder and focus-stack every raw image folder.

    Structure expected:
        <project>/orbit_NNN/stack_NNN/<rot_aux>/slot_A/
            *.dng / *.ARW / *.CR3 / *.NEF / *.tif   (raw frames)
            *_preview.jpg                              (sidecars — excluded)

    Output per stack:
        best_focus.jpg  in the source folder  (skip marker for reruns)
        <project>/[orbit_NNN/]colmap/images/stack_NNN.jpg

    Returns the number of stacks successfully processed.
    Raises RuntimeError for fatal setup problems.
    """
    err = _check_shinestacker()
    if err:
        raise RuntimeError(err)

    parent_path = Path(parent_dir).resolve()
    if not parent_path.exists():
        raise RuntimeError(f"Directory does not exist:\n{parent_path}")

    log_fn(f"Scanning: {parent_path}\n")

    # Raw/DNG/TIFF only — JPEGs are preview sidecars or final outputs, never source
    extensions = (
        '*.dng', '*.DNG',
        '*.arw', '*.ARW',
        '*.cr3', '*.CR3',
        '*.nef', '*.NEF',
        '*.tif', '*.tiff', '*.TIF', '*.TIFF',
    )
    image_files: list[str] = []
    for ext in extensions:
        matches = glob.glob(os.path.join(parent_path, "**", ext), recursive=True)
        matches = [f for f in matches
                   if not f.endswith('_preview.jpg')
                   and 'best_focus' not in os.path.basename(f)
                   and os.path.join('colmap', 'images') not in f]
        image_files.extend(matches)

    # Deduplicate (macOS case-insensitive FS returns *.arw + *.ARW duplicates)
    seen: set[str] = set()
    image_files = [f for f in image_files
                   if not (f.lower() in seen or seen.add(f.lower()))]

    stack_folders = sorted(set(os.path.dirname(f) for f in image_files))

    if not stack_folders:
        raise RuntimeError(
            "No source image folders found.\n\n"
            "Check that:\n"
            "  • The volume is mounted\n"
            "  • The project contains .dng / .ARW / .CR3 files\n"
            "  • You selected the project root (not a subfolder)\n\n"
            f"Searched: {parent_path}"
        )

    log_fn(f"Found {len(stack_folders)} stack folder(s).\n{'=' * 60}")

    # ── Build job list ────────────────────────────────────────────────────────
    colmap_dirs: set[Path] = set()
    pending_jobs: list[dict] = []   # stacks that need processing
    already_done = 0
    skipped = 0

    for src_dir in stack_folders:
        src_path  = Path(src_dir)
        rel_parts = src_path.relative_to(parent_path).parts

        orbit_name = next((p for p in rel_parts if p.lower().startswith('orbit_')), None)
        stack_name = next((p for p in rel_parts if p.lower().startswith('stack_')), rel_parts[0])

        colmap_images_dir = (
            parent_path / orbit_name / "colmap" / "images"
            if orbit_name else
            parent_path / "colmap" / "images"
        )
        colmap_images_dir.mkdir(parents=True, exist_ok=True)
        colmap_dirs.add(colmap_images_dir)

        inplace_file = src_path / "best_focus.jpg"
        colmap_file  = colmap_images_dir / f"{stack_name}.jpg"

        if inplace_file.exists():
            if not colmap_file.exists():
                shutil.copy2(inplace_file, colmap_file)
                log_fn(f"  {src_path.relative_to(parent_path)} → copied to COLMAP (already stacked)")
                already_done += 1
            else:
                log_fn(f"  {src_path.relative_to(parent_path)} → skipping (complete)")
                skipped += 1
            continue

        # Destination: a temp result path next to the source folder;
        # the orchestrator moves it to both inplace_file and colmap_file.
        pending_jobs.append({
            "src":        str(src_path),
            "dest":       str(src_path.parent / f"_stackresult_{src_path.name}.tif"),
            "inplace":    str(inplace_file),
            "colmap":     str(colmap_file),
            "stack_name": stack_name,
            "rel":        str(src_path.relative_to(parent_path)),
        })

    if not pending_jobs:
        log_fn(f"\nNothing to process — {skipped} already complete.")
        return already_done

    log_fn(f"\n{len(pending_jobs)} stack(s) to process ({skipped} already done).")
    log_fn(f"Launching shinestacker...\n{'=' * 60}")

    # ── Run all pending jobs in ONE subprocess ────────────────────────────────
    python = _find_python()

    # Write runner script to temp file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.py',
                                     delete=False, prefix='ss_runner_') as f:
        f.write(_STACK_RUNNER_SCRIPT)
        runner_path = f.name

    # Write jobs list to temp JSON file
    jobs_payload = [{"src": j["src"], "dest": j["dest"]} for j in pending_jobs]
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json',
                                     delete=False, prefix='ss_jobs_') as f:
        json.dump(jobs_payload, f)
        jobs_path = f.name

    # Map src_path → job metadata for result handling
    job_by_src = {j["src"]: j for j in pending_jobs}

    processed = already_done
    errors = 0

    try:
        proc = subprocess.Popen(
            [python, runner_path, jobs_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

        for line in proc.stdout:
            line = line.rstrip('\n')

            if line.startswith('RUNNER_ERROR:'):
                log_fn(f"FATAL: {line[len('RUNNER_ERROR:'):]}")

            elif line.startswith('BATCH_START:') or line.startswith('STACK_START:') or line.startswith('PHASE:'):
                log_fn(line)   # pass structured markers straight to GUI

            elif line.startswith('STACK_DONE:'):
                _, src_str, dest_str = line.split(':', 2)
                job = job_by_src.get(src_str)
                if job:
                    result = Path(dest_str)
                    inplace = Path(job["inplace"])
                    colmap  = Path(job["colmap"])
                    try:
                        shutil.copy2(result, inplace)
                        shutil.copy2(result, colmap)
                        result.unlink(missing_ok=True)
                        log_fn(f"✓ {job['rel']}  →  colmap/images/{job['stack_name']}.jpg")
                        processed += 1
                    except Exception as e:
                        log_fn(f"✗ {job['rel']}  copy failed: {e}")
                        errors += 1

            elif line.startswith('STACK_FAILED:'):
                _, src_str, msg = line.split(':', 2)
                job = job_by_src.get(src_str, {})
                log_fn(f"✗ {job.get('rel', src_str)}  ERROR: {msg}")
                errors += 1

            else:
                log_fn(f"  {line}")

        proc.wait()

    finally:
        try:
            os.unlink(runner_path)
            os.unlink(jobs_path)
        except OSError:
            pass

    # ── Summary ───────────────────────────────────────────────────────────────
    log_fn(f"\n{'=' * 60}")
    log_fn(f"Done.  Processed: {processed}  Skipped: {skipped}  Errors: {errors}")
    if colmap_dirs:
        log_fn("\nCOLMAP input folders:")
        for d in sorted(colmap_dirs):
            imgs = list(d.glob("*.jpg"))
            log_fn(f"  {d}")
            log_fn(f"  ({len(imgs)} image(s) ready)")

    return processed


# ── GUI ───────────────────────────────────────────────────────────────────────

class StackBatchApp:
    def __init__(self, root):
        self.root = root
        self.root.title("PiSlider Focus Stack Batcher")
        self.root.geometry("720x620")
        self.root.configure(bg="#2c3e50")
        self.root.resizable(True, True)

        # ── Header ────────────────────────────────────────────────────────────
        tk.Label(root, text="PiSlider Batch Stack Processing",
                 font=("Helvetica", 16, "bold"),
                 fg="#ecf0f1", bg="#2c3e50").pack(pady=(14, 6))

        # ── Folder picker ─────────────────────────────────────────────────────
        picker = tk.Frame(root, bg="#2c3e50")
        picker.pack(fill="x", padx=20)
        self.path_var = tk.StringVar(value="No folder selected…")
        tk.Entry(picker, textvariable=self.path_var, font=("Helvetica", 11),
                 state="readonly").pack(side="left", padx=(0,6), pady=4,
                                        expand=True, fill="x")
        tk.Button(picker, text="Browse Project", command=self.browse_folder,
                  font=("Helvetica", 11, "bold"),
                  bg="#3498db", fg="white",
                  highlightbackground="#2c3e50").pack(side="right", pady=4)

        # ── Progress panel ────────────────────────────────────────────────────
        prog_frame = tk.Frame(root, bg="#1a252f", bd=0)
        prog_frame.pack(fill="x", padx=20, pady=(6, 0))

        # Row 1: stack counter + phase label
        row1 = tk.Frame(prog_frame, bg="#1a252f")
        row1.pack(fill="x", padx=10, pady=(8, 2))

        self.stack_counter_var = tk.StringVar(value="Stack —  /  —")
        tk.Label(row1, textvariable=self.stack_counter_var,
                 font=("Helvetica", 12, "bold"),
                 fg="#ecf0f1", bg="#1a252f").pack(side="left")

        self.phase_var = tk.StringVar(value="")
        self.phase_lbl = tk.Label(row1, textvariable=self.phase_var,
                                  font=("Helvetica", 11),
                                  fg="#f39c12", bg="#1a252f")
        self.phase_lbl.pack(side="right")

        # Row 2: current stack name
        self.current_var = tk.StringVar(value="Select a project folder to begin")
        tk.Label(prog_frame, textvariable=self.current_var,
                 font=("Helvetica", 10), fg="#95a5a6", bg="#1a252f",
                 anchor="w").pack(fill="x", padx=10, pady=(0, 4))

        # Row 3: progress bar
        import tkinter.ttk as ttk
        style = ttk.Style()
        style.theme_use("default")
        style.configure("Stack.Horizontal.TProgressbar",
                        troughcolor="#0d1b24",
                        background="#2ecc71",
                        thickness=14)
        self.progress_bar = ttk.Progressbar(prog_frame, style="Stack.Horizontal.TProgressbar",
                                             orient="horizontal", mode="determinate")
        self.progress_bar.pack(fill="x", padx=10, pady=(0, 8))

        # Row 4: done / error counters
        row4 = tk.Frame(prog_frame, bg="#1a252f")
        row4.pack(fill="x", padx=10, pady=(0, 8))
        self.done_var  = tk.StringVar(value="Done: 0")
        self.error_var = tk.StringVar(value="Errors: 0")
        tk.Label(row4, textvariable=self.done_var,
                 font=("Helvetica", 10), fg="#2ecc71", bg="#1a252f").pack(side="left", padx=(0,16))
        tk.Label(row4, textvariable=self.error_var,
                 font=("Helvetica", 10), fg="#e74c3c", bg="#1a252f").pack(side="left")

        # ── Log area ──────────────────────────────────────────────────────────
        self.log_area = scrolledtext.ScrolledText(
            root, font=("Courier", 10),
            bg="#1e272e", fg="#95a5a6",
            insertbackground="white", state="disabled")
        self.log_area.pack(fill="both", expand=True, padx=20, pady=(8, 6))

        # Colour tags
        self.log_area.tag_config("ok",      foreground="#2ecc71")
        self.log_area.tag_config("err",     foreground="#e74c3c")
        self.log_area.tag_config("header",  foreground="#ecf0f1", font=("Courier", 10, "bold"))
        self.log_area.tag_config("detail",  foreground="#636e72")
        self.log_area.tag_config("default", foreground="#95a5a6")

        err = _check_shinestacker()
        if err:
            self._log(f"⚠️  {err}", "err")
        else:
            python = _find_python()
            self._log(f"shinestacker.app found  •  Python: {python}", "ok")
            self._log("Select a project folder to begin.\n", "default")

        # ── Start button ──────────────────────────────────────────────────────
        self.run_btn = tk.Button(
            root, text="▶  Start Batch Stacking",
            command=self.start_stacking_thread,
            font=("Helvetica", 13, "bold"),
            bg="#2ecc71", fg="white", height=2,
            highlightbackground="#2c3e50",
            state="disabled")
        self.run_btn.pack(fill="x", padx=20, pady=(0, 14))

        # Internal counters updated from worker thread
        self._total_stacks = 0
        self._done_count   = 0
        self._error_count  = 0

    # ── Logging ───────────────────────────────────────────────────────────────

    def _log(self, text, tag="default"):
        self.root.after(0, self._log_main, text, tag)

    def _log_main(self, text, tag="default"):
        self.log_area.configure(state="normal")
        self.log_area.insert(tk.END, text + "\n", tag)
        self.log_area.see(tk.END)
        self.log_area.configure(state="disabled")

    # ── Progress helpers (called from worker thread via root.after) ───────────

    def _set_progress(self, stack_i, total, label, phase=""):
        self.stack_counter_var.set(f"Stack  {stack_i}  /  {total}")
        self.current_var.set(label)
        self.phase_var.set(phase)
        self.progress_bar["maximum"] = total
        self.progress_bar["value"]   = stack_i - 1   # fill on START of each job

    def _tick_done(self, success: bool):
        if success:
            self._done_count += 1
        else:
            self._error_count += 1
        self.done_var.set(f"Done: {self._done_count}")
        self.error_var.set(f"Errors: {self._error_count}")
        self.progress_bar["value"] = self._done_count + self._error_count

    def _set_phase(self, phase: str):
        self.phase_var.set(phase)

    def _reset_progress(self, total):
        self._total_stacks = total
        self._done_count   = 0
        self._error_count  = 0
        self.stack_counter_var.set(f"Stack  0  /  {total}")
        self.current_var.set("Starting shinestacker…")
        self.phase_var.set("")
        self.done_var.set("Done: 0")
        self.error_var.set("Errors: 0")
        self.progress_bar["maximum"] = total
        self.progress_bar["value"]   = 0

    def _finish_progress(self):
        self.progress_bar["value"] = self.progress_bar["maximum"]
        self.phase_var.set("")
        self.current_var.set("Complete")

    # ── Folder picker ─────────────────────────────────────────────────────────

    def browse_folder(self):
        d = filedialog.askdirectory(title="Select PiSlider Project Root Folder")
        if d:
            self.path_var.set(d)
            self.run_btn.configure(state="normal")
            self.current_var.set(d)

    # ── Worker ────────────────────────────────────────────────────────────────

    def start_stacking_thread(self):
        self.run_btn.configure(state="disabled", bg="#7f8c8d",
                               text="⏳  Processing…")
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        target = self.path_var.get()
        self._log(f"\n▶ Starting: {target}", "header")

        def log_fn(text):
            # Route structured markers to progress panel; rest to log
            if text.startswith("BATCH_START:"):
                total = int(text.split(":")[1])
                self.root.after(0, self._reset_progress, total)
            elif text.startswith("STACK_START:"):
                _, i, total, label = text.split(":", 3)
                self.root.after(0, self._set_progress,
                                int(i), int(total), label, "⏳ aligning…")
                self._log(f"\n[{i}/{total}] {label}", "header")
            elif text.startswith("PHASE:"):
                _, phase, n = text.split(":", 2)
                phase_text = f"⚡ stacking {n} frames…" if phase == "stacking" \
                             else f"⚙ aligning {n} frames…"
                self.root.after(0, self._set_phase, phase_text)
            elif text.startswith("✓"):
                self.root.after(0, self._tick_done, True)
                self._log(text, "ok")
            elif text.startswith("✗"):
                self.root.after(0, self._tick_done, False)
                self._log(text, "err")
            elif text.startswith("  ") or text.startswith("    "):
                self._log(text, "detail")
            else:
                self._log(text, "default")

        try:
            count = batch_process_project(target, log_fn=log_fn)
            self.root.after(0, self._finish_progress)
            if count > 0:
                self.root.after(0, lambda: messagebox.showinfo(
                    "Complete",
                    f"Focus stacking done.\n{count} stack(s) processed.\n\n"
                    f"COLMAP images are in:\n{target}/[orbit]/colmap/images/"))
            else:
                self.root.after(0, lambda: messagebox.showinfo(
                    "Nothing to do",
                    "All stacks were already processed.\n"
                    "Check the log for details."))
        except RuntimeError as e:
            msg = str(e)
            self._log(f"\nERROR: {msg}", "err")
            self.root.after(0, lambda: messagebox.showerror("Error", msg))
        except Exception as e:
            self._log(f"\nFATAL: {e}", "err")
            self.root.after(0, lambda: messagebox.showerror("Fatal Error", str(e)))
        finally:
            self.root.after(0, lambda: self.run_btn.configure(
                state="normal", bg="#2ecc71", text="▶  Start Batch Stacking"))


if __name__ == "__main__":
    root = tk.Tk()
    app = StackBatchApp(root)
    root.mainloop()
