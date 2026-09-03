#!/usr/bin/env python3
"""
slider.py — High-level motion control for PiSlider.

Contains:
1. LinearAxis & RotationAxis (High-level trackers for discrete timelapse moves)
2. TrajectoryPlayer (Executes motion_engine splines in real-time for cinematic mode)
3. TimelapseTrajectoryPlayer (Executes the same splines frame-by-frame for timelapse)

Time-scaling design
───────────────────
Both players take the same MotionEngine trajectory (positions in mm / degrees).
The spatial PATH is identical — only the timing changes.

  Cinematic: frames play at real-time FPS (60 fps default).
             Total duration = n_frames / fps.

  Timelapse: each frame corresponds to one timelapse exposure.
             Total duration = n_frames × interval_s.
             The camera moves between exposures; move_duration per step is
             configurable (default: interval × 0.4 leaving 60% for exposure
             + settle).

To time-scale a move: keep the same keyframes, change n_frames or total_duration.
  e.g. 200 frames at 5s interval = 1000s sequence with identical camera arc
       as 200 frames at 10s interval — just twice as slow.

Usage
─────
  # Build trajectory (same for both modes)
  engine = MotionEngine()
  engine.add_keyframe(0, 0, 0)
  engine.add_keyframe(200, 45, -10)
  traj = engine.generate_trajectory(total_frames=200)

  # Cinematic playback (real-time, 60 fps)
  player = TrajectoryPlayer(hw)
  player.play(*traj)

  # Timelapse step-by-step
  tl_player = TimelapseTrajectoryPlayer(hw, steps_per_mm=50.0,
                                         pan_steps_per_deg=66.667,
                                         tilt_steps_per_deg=133.333)
  for frame_idx, done in tl_player.iter_frames(*traj, move_fraction=0.4):
      trigger_camera()       # your timelapse shutter logic here
      if done: break
"""

import time
import threading
import logging

class TrajectoryPlayer:
    """
    Executes a high-speed cinematic motion trajectory by streaming
    velocity commands to the TMC2209 drivers via UART.

    UART address map (from hardware.py wiring rev 2026-02-17):
        Addr 0 → Tilt   (All Low)
        Addr 1 → Pan    (MS1 High)
        Addr 2 → Slider (MS2 High)
    """
    ADDR_TILT   = 0
    ADDR_PAN    = 1
    ADDR_SLIDER = 2

    # VACTUAL conversion factors (calibrated from test_uart_optimized.py)
    VACTUAL_PER_MM_S       = 12.500
    VACTUAL_PER_DEG_S_PAN  = 15.625
    VACTUAL_PER_DEG_S_TILT = 23.125

    def __init__(self, hardware, steps_per_mm: float = 6.25, pan_steps_per_deg: float = 8.333, tilt_steps_per_deg: float = 16.667):
        self.hardware = hardware
        self.steps_per_mm = steps_per_mm
        self.pan_steps_per_deg = pan_steps_per_deg
        self.tilt_steps_per_deg = tilt_steps_per_deg

    def play(self, traj_slider, traj_pan, traj_tilt, fps: int = 60):
        """
        Plays back spatial arrays smoothly.
        """
        total_frames = len(traj_slider)
        if total_frames < 2:
            logging.error("Trajectory too short to play.")
            return

        dt = 1.0 / fps
        logging.info(f"Playing Trajectory: {total_frames} frames at {fps} FPS.")

        self.hardware.enable_motors(True)
        start_time = time.perf_counter()

        for i in range(total_frames - 1):
            # 1. Calculate physical delta for this frame
            delta_slider = traj_slider[i+1] - traj_slider[i]
            delta_pan = traj_pan[i+1] - traj_pan[i]
            delta_tilt = traj_tilt[i+1] - traj_tilt[i]

            # 2. Convert to velocity (Units per second)
            v_slider_units_s = delta_slider / dt
            v_pan_units_s = delta_pan / dt
            v_tilt_units_s = delta_tilt / dt

            # 3. Convert physical velocity to VACTUAL register values
            v_slider_vactual = int(v_slider_units_s * self.VACTUAL_PER_MM_S)
            v_pan_vactual    = int(v_pan_units_s    * self.VACTUAL_PER_DEG_S_PAN)
            v_tilt_vactual   = int(v_tilt_units_s   * self.VACTUAL_PER_DEG_S_TILT)

            # 4. Stream to TMC2209 VACTUAL (hardware-timed, jitter-free)
            self.hardware.set_tmc_velocity(self.ADDR_SLIDER, v_slider_vactual)
            self.hardware.set_tmc_velocity(self.ADDR_PAN,    v_pan_vactual)
            self.hardware.set_tmc_velocity(self.ADDR_TILT,   v_tilt_vactual)

            # 5. Precision Timing Engine (Avoids Linux OS Jitter)
            next_frame_time = start_time + ((i + 1) * dt)

            # Sleep to free CPU, but wake up 2ms early
            sleep_time = next_frame_time - time.perf_counter() - 0.002
            if sleep_time > 0:
                time.sleep(sleep_time)

            # Microsecond-perfect lock for the end of the frame
            while time.perf_counter() < next_frame_time:
                pass

        # 6. Stop all axes
        self.hardware.stop_all_axes()
        logging.info("Playback complete.")


class LinearAxis:
    """
    High-level wrapper for the slider motor (used for discrete timelapse moves).

    NOTE: For focus rail macro mode, position is tracked in actual STEPS, not mm.
    The steps_per_mm parameter is kept for backward compatibility with timelapse mode,
    but for macro scanning we use raw step counts to avoid conversion errors.
    """
    def __init__(self, hardware, addr=0, steps_per_mm=100.0, max_mm=300.0, settle_time=0.5):
        self.hw = hardware
        self.addr = addr
        self.steps_per_mm = steps_per_mm
        self.max_mm = max_mm
        self.settle_time = settle_time
        self.current_mm = 0.0
        self.current_steps = 0  # ← Track actual step position for macro mode
        self.soft_min = 0.0           # soft limit for macro mode (mm, for UI reference)
        self.soft_max = max_mm        # soft limit for macro mode (mm, for UI reference)
        self.soft_min_steps = 0       # soft limit in steps (set explicitly in app.py for macro mode)
        self.soft_max_steps = 0       # soft limit in steps (set explicitly in app.py for macro mode)

    def move_to_mm(self, target_mm: float, duration_s: float = 1.0) -> bool:
        # Respect soft limits (not hardcoded 0-max_mm, which breaks negative positions)
        target_mm = max(self.soft_min, min(target_mm, self.soft_max))
        delta_mm = target_mm - self.current_mm
        steps = int(delta_mm * self.steps_per_mm)

        # Note: move_axes_simultaneous uses physical STEP/DIR pins, not UART.
        # But for TrajectoryPlayer and UART Odometer, we track the address.
        self.hw.move_axes_simultaneous(slider_steps=steps, pan_steps=0, tilt_steps=0, duration_s=duration_s)
        self.current_mm = target_mm
        self.current_steps += steps  # Track actual step count for macro mode
        time.sleep(self.settle_time)
        return True

    def get_position_mm(self) -> float:
        return self.current_mm


class RotationAxis:
    """
    High-level wrapper for rotation (pan/tilt) motors.
    """
    def __init__(self, hardware, addr=1, steps_per_deg=133.333, settle_time=0.5):
        self.hw = hardware
        self.addr = addr
        self.steps_per_deg = steps_per_deg
        self.settle_time = settle_time
        self.current_deg = 0.0
        self.soft_min = -90.0         # soft limit (degrees) — can be expanded to ±180
        self.soft_max = 90.0          # soft limit (degrees) — for full 360°, set to [-180, 180]

    def move_to_deg(self, target_deg: float, duration_s: float = 1.0) -> bool:
        delta_deg = target_deg - self.current_deg
        steps = int(delta_deg * self.steps_per_deg)
        if self.addr == 1: # Pan
            self.hw.move_axes_simultaneous(0, steps, 0, duration_s)
        elif self.addr == 2: # Tilt
            self.hw.move_axes_simultaneous(0, 0, steps, duration_s)
            
        self.current_deg = target_deg
        time.sleep(self.settle_time)
        return True

    def get_position_deg(self) -> float:
        return self.current_deg

    def sweep_degrees(self, start_deg: float, end_deg: float, frames: int):
        if frames < 2:
            frames = 2
        positions = []
        step_deg = (end_deg - start_deg) / (frames - 1)
        for i in range(frames):
            positions.append(start_deg + i * step_deg)
        return positions


class TimelapseTrajectoryPlayer:
    """
    Execute a MotionEngine trajectory one frame at a time for timelapse sequences.

    The spatial path is IDENTICAL to what TrajectoryPlayer uses for cinematic
    playback — only the timing changes. This means you can design a move once
    and use it in both modes.

    Time-scaling
    ────────────
    n_frames  controls how many positions are sampled along the spline.
    interval_s is the timelapse interval (shutter-to-shutter time).
    move_fraction controls what share of each interval is used for movement
    (default 0.4 = 40%, leaving 60% for exposure + vibration settle).

    Changing n_frames or interval_s scales the DURATION of the sequence while
    keeping the exact same camera arc:
      200 frames × 5s  → 1000s sequence
      200 frames × 10s → 2000s sequence   (same arc, twice as slow)
      400 frames × 5s  → 2000s sequence   (same arc, each step half as large)

    Usage
    ─────
    Generate trajectory with n_frames matching your timelapse frame count:
      traj = engine.generate_trajectory(total_frames=n_frames)

    Then call step_to_frame() inside your timelapse loop:
      tl = TimelapseTrajectoryPlayer(hw, ...)
      tl.load(*traj)
      for frame in range(n_frames):
          tl.step_to_frame(frame, move_duration_s)
          trigger_camera()
          wait_for_interval()
    """

    def __init__(self, hardware,
                 steps_per_mm: float = 50.0,
                 pan_steps_per_deg: float = 66.667,
                 tilt_steps_per_deg: float = 133.333):
        self.hardware = hardware
        self.steps_per_mm = steps_per_mm
        self.pan_steps_per_deg = pan_steps_per_deg
        self.tilt_steps_per_deg = tilt_steps_per_deg

        self._traj_slider = None
        self._traj_pan    = None
        self._traj_tilt   = None
        self._stop_flag   = threading.Event()

        # Software position tracking (mirrors InertiaEngine position state)
        self.current_mm  = 0.0
        self.current_pan = 0.0
        self.current_tilt = 0.0

    def load(self, traj_slider, traj_pan, traj_tilt):
        """Load a trajectory generated by MotionEngine.generate_trajectory()."""
        self._traj_slider = traj_slider
        self._traj_pan    = traj_pan
        self._traj_tilt   = traj_tilt

    @property
    def total_frames(self) -> int:
        return len(self._traj_slider) if self._traj_slider is not None else 0

    def stop(self):
        self._stop_flag.set()

    def step_to_frame(self, frame_idx: int, move_duration_s: float = 1.0) -> bool:
        """
        Move all axes to the position for frame_idx using Bresenham stepping.

        move_duration_s: how long the move should take (use interval × move_fraction).
        Returns True if the move completed, False if stop() was called mid-move.
        """
        if self._traj_slider is None:
            raise RuntimeError("Call load() before step_to_frame()")
        frame_idx = max(0, min(frame_idx, self.total_frames - 1))

        target_mm  = float(self._traj_slider[frame_idx])
        target_pan = float(self._traj_pan[frame_idx])
        target_tilt= float(self._traj_tilt[frame_idx])

        delta_slider = int((target_mm   - self.current_mm)   * self.steps_per_mm)
        delta_pan    = int((target_pan  - self.current_pan)   * self.pan_steps_per_deg)
        delta_tilt   = int((target_tilt - self.current_tilt)  * self.tilt_steps_per_deg)

        self._stop_flag.clear()
        self.hardware.move_axes_simultaneous(
            slider_steps = delta_slider,
            pan_steps    = delta_pan,
            tilt_steps   = delta_tilt,
            duration_s   = move_duration_s,
        )

        # Update tracked position
        self.current_mm   = target_mm
        self.current_pan  = target_pan
        self.current_tilt = target_tilt

        return not self._stop_flag.is_set()

    def return_to_start(self, duration_s: float = 3.0):
        """Move back to frame 0 (start position)."""
        self.step_to_frame(0, duration_s)