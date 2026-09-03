# Cinema Mode

Cinema mode executes real-time programmed camera moves for video production — smooth, repeatable motion paths at controlled speeds. Unlike timelapse (which captures stills over minutes or hours), cinema moves happen in real time as the camera rolls video.

---

## Overview

A cinema move is defined by:
- **Start position** — where the slider, pan, and tilt begin
- **End position** — where they end up
- **Duration** — how long the move takes (in seconds)
- **Easing** — how the acceleration and deceleration behave

Once programmed, you can trigger the move repeatedly for consistent, repeatable takes — useful for B-roll, product shots, and motion control video.

---

## Setting Up a Move

1. Jog the slider to the **start position** using the manual controls
2. Click **Set Start** to save the position
3. Jog to the **end position**
4. Click **Set End** to save
5. Set the **duration** (e.g., 8 seconds)
6. Press **Run** — the slider returns to start, then executes the move

You can reverse the move (end → start) with the **Reverse** button, which is useful for in-camera reveal shots.

---

## Multi-Axis Moves

All three axes (rail, pan, tilt) can move simultaneously. The system synchronizes them so all axes start and finish together, regardless of the distance each needs to travel.

**Example moves:**
- **Track and pan** — rail slides left as the camera pans right to follow a subject
- **Pedestal** — rail moves forward as tilt adjusts to maintain frame
- **Arc** — pan rotates smoothly while rail stays stationary
- **Push-in with reveal** — rail moves forward while tilt tilts up to reveal a location

---

## Speed and Easing

**Speed** is determined by the duration — shorter duration = faster move.

**Easing** (acceleration curves) can be adjusted:
- **Linear** — constant speed throughout. Feels mechanical but useful for product rotation.
- **Ease In/Out** — starts and ends slowly, peaks in the middle. Most natural for organic camera work.
- **Custom** — adjust ease-in and ease-out independently.

Smooth easing is critical for professional-looking moves. An abrupt start or stop will look amateurish even with precise positioning.

---

## Motion Scripts

You can save and recall named motion presets as **Motion Scripts** (`web/motion_scripts.json`). A motion script stores:
- All axis start and end positions
- Duration
- Easing parameters
- A descriptive name

This lets you recall a shot exactly for reshoots, or share a script between sessions.

---

## Gamepad Control

The PiSlider supports Bluetooth gamepads for real-time manual control during cinema moves. Connect a compatible gamepad and use:
- Left stick — rail + pan
- Right stick — tilt + speed modifier
- Buttons — trigger move, return to start, adjust speed

This enables live "operator" control for unpredictable subjects while still using the slider's motor-controlled smooth movement.

---

## Tips for Clean Cinema Moves

- **Balance the camera** before programming moves. An unbalanced head will cause the motors to strain and may produce subtle jitter.
- **Test at speed** — a move that looks smooth at 2x preview speed may reveal motor stepping artifacts at real speed. Always do a dry run.
- **Vibration isolation** — rubber feet under the slider and a solid tripod reduce vibration coupling.
- **Cable management** — camera cables should have enough slack to not tension at the end of travel. A taught cable will stop a move or pull the camera.
- **Frame rate and motion blur** — for video, use the 180° shutter rule: shutter = 1/(2 × frame rate). At 24fps, use 1/48s shutter. This gives natural motion blur that matches the move.
