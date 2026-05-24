# PiSlider

A Raspberry Pi 5-powered camera slider system with 3-axis motion control, designed for professional timelapse, cinematic, and macro 3D scanning photography.

## Overview

PiSlider combines precision motor control, a web-based interface, and a sophisticated exposure automation system into one portable platform. Connect to the slider's WiFi hotspot, open a browser, and control everything from your phone or laptop — no app install required.

**Hardware:**
- Raspberry Pi 5
- 3-axis motion: rail (linear), pan (rotation), tilt (auxiliary)
- Sony IMX477 camera module (or compatible)
- Stepper motor controllers

**Shooting Modes:**
- **Timelapse** — long-duration frame capture with optional Holy Grail auto-exposure for sunrise/sunset/overnight sequences
- **Cinema** — real-time motion along programmed paths for video use
- **Macro 3D Scan** — automated focus stacking with geodesic orbital camera positioning for photogrammetry / COLMAP output

---

## Quick Start

### 1. Connect to PiSlider

Power on the PiSlider. It broadcasts a WiFi hotspot:

- **SSID:** `PiSlider`
- **Password:** *(set during setup)*

Connect your phone or laptop to `PiSlider` WiFi, then open:

```
http://pislider.local:8000/
```

The server starts automatically when you connect to the hotspot. No manual SSH or app launch required.

### 2. Set Up the Pi (First Time)

See **[docs/setup.md](docs/setup.md)** for complete Pi configuration including:
- NetworkManager hotspot setup
- nginx reverse proxy (port 80 → 8000)
- mDNS / `pislider.local` hostname
- systemd services (auto-start on hotspot connect)

---

## Repository Structure

```
pislider/
├── app.py                  # Main FastAPI server — all modes, REST API, WebSocket
├── hardware.py             # Motor controller interface, step math
├── slider.py               # SliderState, axis management
├── macro_engine.py         # Macro 3D scan sequencer
├── cinematic_engine.py     # Cinema move sequencer
├── motion_engine.py        # Generic motion primitives
├── holygrail.py            # Holy Grail auto-exposure system
├── distributions.py        # Geodesic sphere point distributions
├── gamepad.py              # Bluetooth gamepad support
├── neopixel_status.py      # LED status indicators
├── pislider_wake.py        # On-demand server wake listener
├── retime.py               # DaVinci Resolve retime export
├── retime_server.py        # Retime HTTP server
├── requirements.txt        # Python dependencies
├── run.sh                  # Start server manually
├── setup.sh                # Initial system setup
├── install-service.sh      # Install systemd services
├── setup_hostname.sh       # Configure hotspot, nginx, mDNS
├── pislider.service        # systemd unit — main server
├── pislider-wake.service   # systemd unit — on-demand wake listener
├── web/                    # Browser UI (HTML, CSS, JS)
│   ├── index.html
│   ├── style.css
│   ├── main.js
│   ├── graph.html          # Timelapse / HolyGrail graph view
│   └── macro-graph.html    # Macro scan visualizer
├── retime_plugin/          # DaVinci Resolve integration
├── batch_stacker/          # Standalone macOS focus stacking app
│   ├── gui_stacker.py      # StackBatch GUI (tkinter)
│   └── StackBatch.spec     # PyInstaller build spec
└── docs/
    ├── setup.md            # Pi hardware and software setup
    ├── timelapse.md        # Timelapse mode guide
    ├── cinema.md           # Cinema mode guide
    ├── macro.md            # Macro 3D scanning guide
    └── holygrail.md        # Holy Grail auto-exposure deep dive
```

---

## Documentation

| Guide | Description |
|-------|-------------|
| [Setup](docs/setup.md) | First-time Pi setup, hotspot, networking |
| [Timelapse](docs/timelapse.md) | Timelapse mode, intervals, motion, Holy Grail |
| [Cinema](docs/cinema.md) | Real-time cinematic moves |
| [Macro / 3D Scan](docs/macro.md) | Focus stacking and photogrammetry output |
| [Holy Grail](docs/holygrail.md) | Full technical deep-dive on the auto-exposure engine |

---

## StackBatch — macOS Focus Stacking App

`batch_stacker/` contains a standalone macOS app for batch processing focus stacks produced by a Macro scan session.

**Requirements:**
- macOS with [shinestacker.app](https://www.shinestacker.com) installed in `/Applications`
- Homebrew Python 3.x (`/opt/homebrew/bin/python3`)

**Build:**
```bash
cd batch_stacker
pip install pyinstaller
pyinstaller StackBatch.spec
```

The app appears in `dist/StackBatch.app` (~27MB, no bundled dependencies).

See [batch_stacker/gui_stacker.py](batch_stacker/gui_stacker.py) for the source.

---

## License

MIT License — see LICENSE file.
