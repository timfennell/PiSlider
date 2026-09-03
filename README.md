# PiSlider

A three-axis motion-control camera rig that runs on a Raspberry Pi 5 and is driven
entirely from a browser over its own WiFi.

It is built for the kind of shot you cannot babysit: a sunset timelapse that has to
ramp ten stops on its own, or a thousand-frame macro scan that runs for six hours and
has to land every frame in the same place. The Pi hosts its own access point, so you
connect a laptop or phone to it in a field with no signal and drive everything from a
web page.

**[Connect and shoot →](docs/setup.md)** · no terminal required

---

## Contents

- [What it does](#what-it-does)
- [Timelapse](#timelapse) · [Cinematic](#cinematic) · [Macro 3D scanning](#macro-3d-scanning)
- [Across every mode](#across-every-mode)
- [Post-capture tools](#post-capture-tools)
- [Hardware](#hardware)
- [How it fits together](#how-it-fits-together)
- [Install](#install)
- [Documentation](#documentation)
- [Project status](#project-status)

---

## What it does

Three motorised axes — **slide** (linear rail, in mm), **pan** and **tilt** (in
degrees) — coordinated by a FastAPI backend with a browser front end. Motion,
exposure and capture are sequenced together, so a move and a shutter release are
planned as one operation rather than two things that happen to overlap.

Three capture modes share that foundation.

---

## Timelapse

Interval capture with motion between frames, aimed at sequences that run for hours.

- **Motion during capture** — the rig moves between frames rather than during them, so
  every exposure is taken from a dead-stop. No motion blur from the rig itself.
- **Eased travel** across the whole sequence, so the move accelerates and settles
  instead of starting and stopping abruptly.
- **Adaptive interval** — the frame interval stretches automatically when the exposure
  grows past it, which is what stops a night sequence from colliding with its own
  shutter.

### Holy Grail exposure ramping

The hard part of a sunrise or sunset timelapse is that the light changes by more than
ten stops while the camera's meter runs out of range. PiSlider ramps exposure using a
three-layer controller rather than metering alone:

1. **An astronomical model.** Sun and moon position are computed from your latitude,
   longitude and clock — including a moonlight brightness contribution weighted by
   phase and altitude, detection of the sun or moon actually entering the frame, and a
   look-ahead ramp that begins compensating *before* the disc arrives.
2. **A measurement tracker.** A recency-weighted linear regression over recent frames
   estimates both the current scene brightness and its rate of change, with anomaly
   rejection so a passing headlight or a bird doesn't move the exposure.
3. **A confidence blend.** How much the system trusts pixels versus the model shifts
   continuously with sun altitude and the regression's own R², so it leans on
   measurement in daylight and on the model once the meter goes blind.

Output is rate-limited per frame, so the exposure ramps smoothly instead of stepping,
and both shutter and ISO are chosen against your camera's real limits — including
dual-gain sensors, where the controller avoids the ISO dead zone that would otherwise
cause a visible brightness jump.

---

## Cinematic

Real-time moves for video, rather than frame-by-frame capture.

- **Multi-axis moves** — slide, pan and tilt run together as one coordinated
  trajectory.
- **Easing and speed control** with a physically-modelled inertia engine, so moves
  start and stop the way a weighted head does instead of snapping to velocity.
- **Motion scripts** — build a move, save it, replay it exactly. The same move can be
  run repeatedly for takes that need to match.
- **Live gamepad control** — drive the rig by hand with an 8BitDo Pro 2, with the
  inertia model applied to stick input so handheld moves come out smooth.

---

## Macro 3D scanning

The most involved mode: automated capture of a small subject from many angles, with a
focus stack at every angle, intended to feed photogrammetry and Gaussian splatting.

- **Orbits** — the rig walks a path around the subject, capturing a full stack at each
  node.
- **Focus stacks** — at each node the camera steps along the rail between a near and a
  far limit, set in absolute step counts rather than millimetres so positions repeat
  exactly across sessions.
- **Exposure slots** — each position can be captured several times under different
  lighting, named per slot (diffuse, side light, and so on), so one automated run
  produces every lighting variant of the same geometry. This is what makes
  triangulation matting possible: shoot the same frame against black and against white
  and the subject can be cut out cleanly afterward.
- **Node distribution** — nodes can be placed evenly over the reachable surface rather
  than clustering at the poles the way a naive spiral does, with a serpentine path
  connecting them so consecutive frames always overlap.
- **Two temperaments** — *SCAN* mode prioritises coverage and repeatability for
  reconstruction; *ART* mode prioritises how the move looks on screen.

### Stereo 3D

Any scan position can capture a stereo pair instead of a single frame, with a
configurable angular offset, for VR video, side-by-side 3D, anaglyph, or light-field
reconstruction.

---

## Across every mode

- **Browser UI over the rig's own hotspot.** No app to install, no internet, multiple
  people can connect at once.
- **Live preview** with a zoomable loupe for checking critical focus.
- **Progress graph** with capture thumbnails, so you can see what has been shot
  without interrupting the run.
- **Gamepad support** for physical control.
- **NeoPixel status LED** for reading rig state from across a room.
- **Wake-on-demand** — a lightweight listener starts the main server only when someone
  connects, so the rig isn't burning battery while it waits.
- **Runs as a systemd service** and comes back on its own after a reboot or a crash.

---

## Post-capture tools

These live in the repo but run on a desktop, not the Pi:

| | |
|---|---|
| **`retime.py` / `retime_plugin/`** | Time-remaps a shot sequence using the motion sidecar recorded during capture, with a DaVinci Resolve integration |
| **`batch_stacker/`** | Batch focus stacking across a whole scan |
| **`tilt_corrector/`** | Perspective correction for captured plates |
| **`stitching.py`** | Panorama and mask stitching via OpenCV |

---

## Hardware

- Raspberry Pi 5 (4GB or 8GB)
- Three stepper axes driven by **TMC2209** controllers — supported both in standalone
  step/dir mode and over UART, which is what enables true velocity control for
  cinematic moves and joystick input
- **Cameras:** Raspberry Pi camera modules via `libcamera`/Picamera2, Sony bodies over
  WiFi, and USB tethered cameras via `gphoto2`
- **Two WiFi radios** — one hosts the hotspot you connect to, the second is reserved
  for the camera link so pairing a camera never drops your connection
- Optional NeoPixel strip and 8BitDo Pro 2 gamepad

Full parts list and wiring are still being written up.

---

## How it fits together

```
app.py                FastAPI application, command table, capture loop
hardware.py           motor / GPIO interface, TMC2209 control
motion_engine.py      trajectory generation, easing, limits
slider.py             high-level motion helpers
cinematic_engine.py   real-time move playback and inertia model
macro_engine.py       focus-stack and 3D scan sequencing
holygrail.py          astronomical + measured exposure ramping
distributions.py      motion distribution curves
gamepad.py            8BitDo Pro 2 input
neopixel_status.py    status LED
pislider_wake.py      on-demand wake trigger
web/                  browser UI
docs/                 mode guides and setup
```

`simulate_hg.py` replays a full day through the exposure controller without hardware,
which is how holy-grail changes are checked before they reach a rig.

---

## Install

Full instructions, including creating the WiFi hotspot, are in
**[docs/setup.md](docs/setup.md)**. In short, on the Pi:

```bash
git clone https://github.com/timfennell/PiSlider.git ~/Projects/pislider
cd ~/Projects/pislider
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
sudo bash setup_hostname.sh     # hostname, mDNS, nginx proxy
sudo bash install-service.sh    # systemd services
```

The virtualenv must be named `.venv` — `run.sh` sources it by that name.

Creating the access point is a separate manual step; it is not done by any script
here. See [docs/setup.md § Create the WiFi hotspot](docs/setup.md).

---

## Documentation

| | |
|---|---|
| [docs/setup.md](docs/setup.md) | Connecting to the rig, and building one from scratch |
| [docs/timelapse.md](docs/timelapse.md) | Timelapse mode |
| [docs/cinema.md](docs/cinema.md) | Cinematic moves, scripts, gamepad |
| [docs/macro.md](docs/macro.md) | Macro scanning, stacking, COLMAP reconstruction |
| [docs/holygrail.md](docs/holygrail.md) | Exposure controller internals |

---

## Project status

Actively developed, and used on a working rig.

- **Holy Grail** — the exposure controller was substantially reworked and is verified
  in simulation across a full sunset into astronomical night, in both anchor and
  no-anchor metering modes. **Not yet field-tested since that rework.**
- **Macro node distribution** — even node placement with a connected serpentine path
  is implemented but has not yet been run on hardware.
- **Hardware documentation** — parts list and wiring guide still to come.

---

## License

Not yet specified.
