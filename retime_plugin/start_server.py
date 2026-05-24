"""
Resolve startup script — auto-launches retime_server.py when DaVinci opens.

Install:
  cp start_server.py \
    ~/Library/Application\ Support/Blackmagic\ Design/DaVinci\ Resolve/Fusion/Scripts/Startup/pislider_retime.py
"""

import subprocess
import sys
import urllib.request
from pathlib import Path

_SERVER = Path(__file__).parent.parent / "retime_server.py"
_PORT   = 9077


def _already_running():
    try:
        urllib.request.urlopen(f"http://localhost:{_PORT}/status", timeout=1)
        return True
    except Exception:
        return False


if _SERVER.exists() and not _already_running():
    subprocess.Popen(
        [sys.executable, str(_SERVER)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
