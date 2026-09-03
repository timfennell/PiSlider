# PiSlider Retime — DaVinci Resolve Plugin (DOES NOT WORK YET)

A DaVinci Resolve Workflow Integration panel that reads PiSlider motion metadata and generates a retime curve for your timelapse or cinema clip — letting you speed up, slow down, or ramp playback speed in sync with the slider's physical movement data.

---

## What It Does

When the PiSlider captures a timelapse or cinema move, it logs the motion data for every frame (position, speed, timestamp). The Retime plugin reads this log and creates a speed curve in DaVinci Resolve's retime editor — so you can non-linearly adjust playback while keeping it synchronized with the slider's movement.

Typical uses:
- Speed ramp a push-in shot to peak speed at the climax
- Slow down a section where the slider was moving fastest
- Match playback speed to music beats using the motion curve as a guide

---

## Requirements

- **DaVinci Resolve 18.0+** (free or Studio)
- **macOS** (Windows support untested)
- **Python 3** — for the local retime server
- **Node.js** — for the Electron-based plugin panel

---

## Installation

1. Copy the entire `retime_plugin/` folder to your DaVinci Resolve Workflow Integrations directory:
   ```
   ~/Library/Application Support/Blackmagic Design/DaVinci Resolve/Workflow Integrations/
   ```

2. Install Python dependencies:
   ```bash
   pip install fastapi uvicorn
   ```

3. Update the server script path in `main.js` if you've moved the project folder:
   ```js
   const SERVER_SCRIPT = path.join(
       require('os').homedir(),
       'Documents', 'pislider', 'retime_server.py'
   );
   ```

4. Restart DaVinci Resolve — the **PiSlider Retime** panel will appear under Workspace → Workflow Integrations.

---

## Usage

1. Open DaVinci Resolve and load your timelapse or cinema clip
2. Open the **PiSlider Retime** panel from Workspace → Workflow Integrations
3. In the panel, select the motion log file from your PiSlider session
4. Click **Apply Retime Curve** — the plugin generates speed keyframes on the selected clip
5. Fine-tune in the Retime editor as needed

---

## How It Works

The plugin runs as two components:

**`retime_server.py`** — A local FastAPI server (port 9077) that reads PiSlider motion logs and exposes the data as a REST API.

**`main.js`** — An Electron-based DaVinci Resolve Workflow Integration that starts the server on launch, displays the panel UI (`index.html`), and communicates with Resolve via the `WorkflowIntegration.node` native module.

When DaVinci Resolve loads the plugin, `main.js` automatically spawns `retime_server.py` in the background if it isn't already running, then loads the panel UI which talks to the server to fetch motion data and apply it to the timeline.

---

## Files

| File | Description |
|------|-------------|
| `main.js` | Plugin entry point — starts server, opens panel window |
| `index.html` | Panel UI |
| `retime_server.py` | Local FastAPI server — reads PiSlider motion logs |
| `manifest.json` | DaVinci Resolve plugin manifest |
| `manifest.xml` | Workflow Integration XML descriptor |
| `package.json` | Node.js package definition |
| `preload.js` | Electron preload script |
| `WorkflowIntegration.node` | Native DaVinci Resolve integration module |
| `PiSlider Retime.py` | Python UI companion |
| `PiSlider_Retime_UI.py` | UI helper module |
