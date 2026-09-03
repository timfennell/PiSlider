#!/usr/bin/env python3
"""
retime_server.py — PiSlider Retime standalone app server.

Usage:
    python3 retime_server.py
    open http://localhost:9077

No DaVinci connection required. Select plate and clip folders in the
browser, click Generate — retimed sequences + an importable Resolve
timeline XML are written to disk.

Import into Resolve: File → Import Timeline → PiSlider_Retime.xml

Requires: pip3 install flask flask-cors scipy numpy
"""

import io
import json
import os
import queue
import subprocess
import sys
import threading
import webbrowser
from pathlib import Path

from flask import Flask, Response, jsonify, request, send_file, stream_with_context
from flask_cors import CORS

sys.path.insert(0, str(Path(__file__).parent))
import retime as rt

app         = Flask(__name__)
CORS(app)
PORT        = 9077
_HERE       = Path(__file__).parent
_PANEL_HTML = _HERE / "retime_plugin" / "index.html"


# ── Panel UI ──────────────────────────────────────────────────────────────────

@app.route("/")
def panel():
    return send_file(_PANEL_HTML)


# ── Status ────────────────────────────────────────────────────────────────────

@app.route("/status")
def status():
    return jsonify({"ok": True})


# ── Validate folders (quick check before full run) ────────────────────────────

@app.route("/validate", methods=["POST"])
def validate():
    data   = request.json or {}
    plate  = data.get("plate", "").strip()
    clips  = [c.strip() for c in data.get("clips", []) if c.strip()]
    errors = []

    if not plate:
        errors.append("Plate folder is required.")
    elif not Path(plate).is_dir():
        errors.append(f"Plate folder not found: {plate}")
    else:
        try:
            rt.find_sidecar(plate)
        except FileNotFoundError:
            errors.append(f"No motion sidecar (.json) found in plate folder: {plate}")

    for c in clips:
        p = Path(c)
        if not p.is_dir():
            errors.append(f"Clip folder not found: {c}")
        else:
            try:
                rt.find_sidecar(c)
            except FileNotFoundError:
                errors.append(f"No motion sidecar (.json) found in clip folder: {c}")

    return jsonify({"ok": len(errors) == 0, "errors": errors})


# ── Generate (retime + XML) — SSE stream ─────────────────────────────────────

@app.route("/generate", methods=["POST"])
def generate_timeline():
    data         = request.json or {}
    plate        = data.get("plate", "").strip()
    clips        = [c.strip() for c in data.get("clips", []) if c.strip()]
    fps          = float(data.get("fps", 24))
    correct_tilt = bool(data.get("correct_tilt", False))
    tilt_factor  = data.get("tilt_factor")
    output_root  = (data.get("output_root") or "").strip() or None
    seq_name     = data.get("sequence_name", "PiSlider Retime").strip()

    if tilt_factor is not None:
        try:
            tilt_factor = float(tilt_factor)
        except (TypeError, ValueError):
            tilt_factor = None

    def stream():
        log_q = queue.Queue()

        class _Writer(io.TextIOBase):
            def write(self, s):
                if s and s.strip():
                    log_q.put(s.rstrip())
                return len(s)

        def _run():
            old_out = sys.stdout
            sys.stdout = _Writer()
            xml_path = None
            try:
                xml_path = rt.retime_and_generate_xml(
                    plate_folder=plate,
                    clip_folders=clips,
                    output_root=output_root,
                    fps=fps,
                    correct_tilt=correct_tilt,
                    tilt_factor=tilt_factor,
                    sequence_name=seq_name,
                )
            except Exception as exc:
                import traceback
                log_q.put(f"ERROR: {exc}")
                log_q.put(traceback.format_exc())
            finally:
                sys.stdout = old_out
                log_q.put({"__done__": True, "xml": str(xml_path) if xml_path else None})

        threading.Thread(target=_run, daemon=True).start()

        while True:
            msg = log_q.get()
            if isinstance(msg, dict) and "__done__" in msg:
                yield f"data: {json.dumps(msg)}\n\n"
                break
            yield f"data: {json.dumps(str(msg))}\n\n"

    return Response(
        stream_with_context(stream()),
        content_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


# ── Native folder picker ─────────────────────────────────────────────────────

@app.route("/browse", methods=["POST"])
def browse():
    """Show a native folder-picker dialog using osascript."""
    data   = request.json or {}
    prompt = data.get("prompt", "Choose a folder")
    try:
        result = subprocess.run(
            ["osascript", "-e",
             f'POSIX path of (choose folder with prompt "{prompt}")'],
            capture_output=True, text=True, timeout=120,
        )
        path = result.stdout.strip().rstrip("/") or None
    except Exception:
        path = None
    return jsonify({"path": path})


# ── Open output folder in Finder ──────────────────────────────────────────────

@app.route("/reveal", methods=["POST"])
def reveal():
    data = request.json or {}
    p    = data.get("path", "")
    if p and Path(p).exists():
        subprocess.run(["open", "-R", p])
    return jsonify({"ok": True})


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=PORT)
    p.add_argument("--no-browser", action="store_true")
    args = p.parse_args()

    url = f"http://localhost:{args.port}"
    print(f"PiSlider Retime  →  {url}")
    print("Ctrl-C to stop.\n")

    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()

    app.run(host="127.0.0.1", port=args.port, threaded=True)
