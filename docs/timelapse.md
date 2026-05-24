# Timelapse Mode

PiSlider's timelapse mode captures a sequence of still frames at a defined interval, optionally moving the slider between frames to create motion in the final video. It supports both simple fixed-exposure and advanced **Holy Grail** auto-exposure for day-to-night or night-to-day transitions.

---

## Overview

A timelapse is a sequence of photos taken over a long period, then played back at video speed. A 10-second, 24fps clip requires 240 frames. If you shoot every 5 seconds, that's 20 minutes of real time. If you shoot every 30 seconds, that's 2 hours.

The PiSlider handles:
- Triggering the camera at the correct interval
- Moving the slider rail, pan, or tilt between exposures (motion timelapse)
- Dynamically adjusting exposure settings for changing light conditions (Holy Grail)
- Logging all frames with metadata (timestamp, EV, shutter, ISO, Kelvin)

---

## Basic Setup

In the PiSlider web interface:

1. **Set interval** — how many seconds between frames
2. **Set frame count** — total number of frames to capture
3. **Configure camera settings** — or enable Holy Grail for auto
4. **Set motion** (optional) — start/end positions for rail, pan, tilt
5. **Press Start**

The system then runs autonomously until all frames are captured or you stop it.

---

## Motion During Timelapse

You can program the slider to move smoothly across its range over the course of the timelapse. This creates a slow, cinematic tracking shot combined with the timelapse compression effect.

**Supported axes:**
- **Rail** — linear slide position (mm)
- **Pan** — horizontal rotation (degrees)
- **Tilt** — vertical tilt (degrees)

Motion is distributed evenly across frames: if you have 200 frames and 100mm of travel, each frame the rail moves 0.5mm.

For subtle drift motion, a few mm of travel over the whole sequence is often more effective than dramatic sweeps.

---

## Exposure Modes

### Manual / Fixed Exposure

Set shutter speed, ISO, and aperture directly. Best for stable lighting conditions — daytime blue-sky shoots, night shoots after dark has fully settled.

### Holy Grail Mode

Automatic exposure control that smoothly tracks changing light through sunrise, sunset, and full day-to-night transitions. The system continuously adjusts shutter, ISO, and white balance without jarring step changes.

See **[Holy Grail Mode](holygrail.md)** for a full technical deep dive.

---

## Frame Interval Guidelines

| Condition | Recommended Interval |
|-----------|---------------------|
| Daytime clouds / movement | 2–5 seconds |
| Golden hour / sunset | 5–8 seconds |
| Twilight transition | 8–12 seconds |
| Deep night / stars | 20–30 seconds |
| All-day (12+ hour) | 10–20 seconds |

During Holy Grail sessions, the system can automatically adjust the interval per phase — shooting faster during golden hour and slower at night.

---

## Output and Post-Processing

Frames are saved as DNGs (raw) when raw mode is enabled, or JPEGs for lighter sessions. DNGs preserve full dynamic range for grading in Lightroom or Capture One.

**Recommended post-processing workflow:**
1. **Grade one frame** in Lightroom to your desired look
2. **Sync** settings across all frames
3. For Holy Grail sessions: use **LRTimelapse** or **GBDeflicker** to remove any remaining exposure steps
4. **Export as image sequence**, then use DaVinci Resolve, After Effects, or Final Cut to assemble the video

### Retime Plugin

The PiSlider includes a DaVinci Resolve retime plugin (`retime_plugin/`) that reads the motion metadata log and generates a retime curve. This lets you speed up or slow down the timelapse playback non-linearly, synchronized with the slider's movement data.

---

## Tips

- **Vibration delay** — set a short pause (0.5–2s) after each move before capturing. Motor vibration causes blur. Longer delay = sharper frames, longer total sequence time.
- **Intervalometer discipline** — keep your shutter speed shorter than the interval minus the vibration delay. A 5-second interval with a 1-second vibration delay means shutter can't exceed ~3.8 seconds.
- **Battery / power** — long sessions need a reliable power supply. A UPS or large battery bank is recommended for overnight shoots.
- **SD card speed** — RAW files are large. Use a fast A2-rated SD card or USB SSD for the output drive.
- **Test run** — do a short 20-frame test at your desired settings before committing to a 3-hour shoot. Verify exposure, focus, and motion look correct.
