# PiSlider

Motion-control camera slider running on a Raspberry Pi 5, driven from a phone
or laptop over WiFi.

Three-axis motion (slide, pan, tilt) with a FastAPI backend and a browser UI,
built for long timelapse and macro capture sessions where the rig has to be
left alone for hours.

This app is designed to be used with the pi setup as a hotspot. then the app interface can be accessed by a webpage: http://pislider.local:8000/ the pi needs configuration to be correctly setup. I'm planning to create an script to do this setup, but it is in the works.

#Known issues:
Holygrail mode isn't working in the version. Hopefully will be updated soon.
Macro scan mode multi stack node distribution isn't finished yet. Coming soon
#

Hardware lists and DIY setup instructions coming soon!

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
