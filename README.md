# PiSlider

Motion-control camera slider running on a Raspberry Pi 5, driven from a phone
or laptop over WiFi.

Three-axis motion (slide, pan, tilt) with a FastAPI backend and a browser UI,
built for long timelapse and macro capture sessions where the rig has to be
left alone for hours.

## Modes

- **Timelapse** — including a holy-grail mode that ramps exposure through
  sunset/sunrise using computed sun position rather than metering, so the
  transition stays smooth as the scene falls out of the meter's range.
- **Cinematic** — eased real-time moves for video.
- **Macro** — focus-stack sequencing for specimen work, shooting black/white
  backdrop pairs per focus step for downstream triangulation matting.
- **Stereo 3D** — paired offset captures.

## Layout

    app.py               FastAPI app and command table
    motion_engine.py     axis motion, easing, limits
    macro_engine.py      focus-stack sequencing
    cinematic_engine.py  real-time move playback
    holygrail.py         exposure ramping
    hardware.py          motor / GPIO interface
    gamepad.py           physical controller input
    retime.py            post-capture retiming
    retime_plugin/       DaVinci Resolve integration
    web/                 browser UI

## Install (on the Pi)

    pip3 install -r requirements.txt
    ./setup.sh

`pislider.service` and `pislider-wake.service` run it as a systemd service so
the rig comes up on boot.

## Related

- [MattePro](https://github.com/timfennell/mattepro) — matting and focus
  stacking for the macro captures
- [TimelapsePro](https://github.com/timfennell/timelapse-pro) — RAW post
  processing for the timelapse sequences
