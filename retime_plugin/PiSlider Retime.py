"""
PiSlider Retime — DaVinci Resolve script launcher.

Installs to: Workspace → Scripts → Utility → PiSlider Retime

Starts retime_server.py (if not already running) then opens the
panel UI in your default browser at http://localhost:9077
"""

import subprocess
import sys
import urllib.request
import webbrowser
from pathlib import Path

_SERVER = Path(__file__).parent.parent.parent.parent.parent / \
    "Documents" / "slider claud code" / "retime_server.py"
_PORT   = 9077
_URL    = f"http://localhost:{_PORT}"


def _server_running():
    try:
        urllib.request.urlopen(f"{_URL}/status", timeout=1)
        return True
    except Exception:
        return False


if not _server_running():
    if not _SERVER.exists():
        print(f"retime_server.py not found at:\n  {_SERVER}")
        print("Update the path in this script to match your install location.")
    else:
        print(f"Starting PiSlider Retime server…")
        subprocess.Popen(
            [sys.executable, str(_SERVER)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        # Give the server a moment to bind
        import time
        for _ in range(10):
            time.sleep(0.5)
            if _server_running():
                break

if _server_running():
    print(f"PiSlider Retime panel → {_URL}")
    webbrowser.open(_URL)
else:
    print("Server did not start. Run manually:")
    print(f"  python3 \"{_SERVER}\"")
