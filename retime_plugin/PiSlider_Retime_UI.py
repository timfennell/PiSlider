"""
PiSlider Retime — DaVinci Resolve UIManager panel.
Works with DaVinci Resolve free and Studio.

Install:
  cp this file to:
  ~/Library/Application Support/Blackmagic Design/DaVinci Resolve/Fusion/Scripts/Edit/

Then: Workspace → Scripts → Edit → PiSlider_Retime_UI

Requires: Resolve Preferences → System → General →
          "External scripting using" = Local
"""

import json
import os
import subprocess
import sys
import threading
import urllib.request

# ── Config ────────────────────────────────────────────────────────────────────
SERVER_SCRIPT = os.path.expanduser(
    "~/Documents/slider claud code/retime_server.py"
)
PORT = 9077

# ── Resolve globals (injected by Resolve when running as a Script) ─────────
ui         = fusion.UIManager
dispatcher = bmd.UIDispatcher(ui)
WIN_ID     = "com.pislider.retime"

# ── Helpers ───────────────────────────────────────────────────────────────────

def server_running():
    try:
        urllib.request.urlopen(f"http://localhost:{PORT}/status", timeout=1)
        return True
    except Exception:
        return False


def start_server():
    if server_running():
        return True
    proc = subprocess.Popen(
        [sys.executable, SERVER_SCRIPT],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    for _ in range(16):
        import time; time.sleep(0.5)
        if server_running():
            return True
    return False


def get_timeline_tracks():
    """Return list of track label strings from the current Resolve timeline."""
    try:
        project  = resolve.GetProjectManager().GetCurrentProject()
        timeline = project.GetCurrentTimeline() if project else None
        if not timeline:
            return []
        n      = timeline.GetTrackCount("video")
        tracks = []
        for i in range(1, n + 1):
            items = timeline.GetItemListInTrack("video", i) or []
            clips = []
            for item in items:
                try:
                    mpi = item.GetMediaPoolItem()
                    clips.append(mpi.GetClipProperty("Clip Name") if mpi else "?")
                except Exception:
                    clips.append("?")
            clips_str = ", ".join(clips[:3]) + ("…" if len(clips) > 3 else "")
            tracks.append(f"V{i}  {clips_str}")
        return tracks
    except Exception as e:
        return [f"Error: {e}"]


# ── Build window ──────────────────────────────────────────────────────────────

# Prevent duplicate windows
existing = ui.FindWindow(WIN_ID)
if existing:
    existing.Show()
    existing.Raise()
else:
    tracks      = get_timeline_tracks()
    track_items = tracks if tracks else ["No timeline — open a project first"]

    win = dispatcher.AddWindow(
        {
            "ID":          WIN_ID,
            "WindowTitle": "PiSlider Retime",
            "Geometry":    [200, 200, 460, 540],
        },
        ui.VGroup({"Spacing": 6, "Weight": 1}, [

            ui.Label({"Text": "PiSlider Retime",
                      "Font": ui.Font({"PixelSize": 16, "Bold": True}),
                      "Alignment": {"AlignHCenter": True},
                      "Weight": 0}),

            ui.Label({"Text": "Select plate track:", "Weight": 0}),
            ui.ComboBox({"ID": "TrackCombo", "Weight": 0}),

            ui.Button({"ID": "RefreshBtn", "Text": "↻  Refresh timeline",
                       "Weight": 0}),

            ui.VGap(4),

            ui.CheckBox({"ID": "TiltCheck",
                         "Text": "Tilt keystoning correction",
                         "Checked": False, "Weight": 0}),

            ui.HGroup({"Weight": 0, "Spacing": 8}, [
                ui.Label({"Text": "Tilt factor:", "Weight": 1}),
                ui.LineEdit({"ID": "TiltFactor",
                             "PlaceholderText": "auto (from focal_mm)",
                             "Weight": 2}),
            ]),

            ui.VGap(4),

            ui.Button({"ID": "RunBtn", "Text": "▶  Retime",
                       "Weight": 0}),

            ui.Label({"Text": "Log:", "Weight": 0}),
            ui.TextEdit({
                "ID":       "Log",
                "ReadOnly": True,
                "Weight":   3,
                "Font":     ui.Font({"Family": "Menlo", "PixelSize": 11}),
            }),
        ])
    )

    # Populate track combo
    combo = win.Find("TrackCombo")
    for t in track_items:
        combo.AddItem(t)

    # ── Event handlers ────────────────────────────────────────────────────────

    def log(msg, replace=False):
        """Append a line to the log TextEdit."""
        el = win.Find("Log")
        if replace:
            el.PlainText = msg + "\n"
        else:
            el.PlainText = el.PlainText + msg + "\n"

    def OnClose(ev):
        dispatcher.ExitLoop()

    def OnRefresh(ev):
        t = get_timeline_tracks()
        t = t if t else ["No timeline — open a project first"]
        c = win.Find("TrackCombo")
        c.Clear()
        for item in t:
            c.AddItem(item)
        log("Timeline refreshed.", replace=True)

    def OnRun(ev):
        plate_track  = win.Find("TrackCombo").CurrentIndex + 1
        correct_tilt = win.Find("TiltCheck").Checked
        tilt_str     = win.Find("TiltFactor").PlaceholderText
        tilt_val     = win.Find("TiltFactor").Text.strip()
        tilt_factor  = float(tilt_val) if tilt_val else None

        win.Find("RunBtn").Enabled = False
        log("Starting server…", replace=True)

        def _run():
            if not start_server():
                win.Find("Log").PlainText = "ERROR: server did not start.\n"
                win.Find("RunBtn").Enabled = True
                return

            log("Server ready. Running retime…")
            body = json.dumps({
                "plate_track":  plate_track,
                "correct_tilt": correct_tilt,
                "tilt_factor":  tilt_factor,
            }).encode()

            try:
                req = urllib.request.Request(
                    f"http://localhost:{PORT}/retime",
                    data=body,
                    headers={"Content-Type": "application/json"},
                )
                with urllib.request.urlopen(req, timeout=300) as resp:
                    buf = ""
                    for chunk in iter(lambda: resp.read(256).decode("utf-8", errors="replace"), ""):
                        buf += chunk
                        while "\n\n" in buf:
                            event, buf = buf.split("\n\n", 1)
                            for line in event.splitlines():
                                if line.startswith("data:"):
                                    payload = line[5:].strip()
                                    if payload == "__DONE__":
                                        log("✓ Done.")
                                    else:
                                        try:
                                            log(json.loads(payload))
                                        except Exception:
                                            log(payload)
            except Exception as e:
                log(f"ERROR: {e}")

            win.Find("RunBtn").Enabled = True

        threading.Thread(target=_run, daemon=True).start()

    win.On[WIN_ID].Close    = OnClose
    win.On["RefreshBtn"].Clicked = OnRefresh
    win.On["RunBtn"].Clicked     = OnRun

    win.Show()
    dispatcher.RunLoop()
