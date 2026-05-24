#!/usr/bin/env python3
"""
hardware.py — Unified Hardware Controller for PiSlider (Raspberry Pi 5)

SOURCE OF TRUTH: Matches wiring diagram rev 2026-02-17
─────────────────────────────────────────────────────
Pi Header (BCM) → Function
─────────────────────────────────────────────────────
GP2  (Pin  3) → Slider STEP
GP3  (Pin  5) → Slider DIR
GP17 (Pin 11) → Pan DIR
GP16 (Pin 36) → Pan STEP   [moved from GP18 — PWM0 conflict on Pi 5]
GP19 (Pin 35) → Tilt STEP  [HW PWM ch3 — moved from GP27; Hall sensor relocated]
GP22 (Pin 15) → Tilt DIR
GP10 (Pin 19) → ENABLE    (Active LOW — shared across all three drivers)
GP4  (Pin  7) → Flash Sync Trigger (PC-sync pulse to external flash)
GP8  (Pin 24) → Camera Trigger S2 (via optocoupler adaptor board)
GP5  (Pin 29) → Relay 2   (Laser Projector)
GP6  (Pin 31) → Relay 1   (Ring Light)
GP13 (Pin 33) → Aux Trigger
GP27 (Pin 13) → Hall Sensor
GP9  (Pin 21) → Endstop
─────────────────────────────────────────────────────
Standalone TMC2209 mode (no UART):
  MS1/MS2 jumpers removed → both float LOW → 1/8 microstepping
  Module TX pins connected to 3.3V → PDN_UART held HIGH
  Current controlled by VREF trim pot on each module
  Step counts at 1/8 step:
    Slider  50   steps/mm   (200 × 8 / 32)
    Pan     66.667 steps/°  (200 × 8 × 15 / 360)
    Tilt   133.333 steps/°  (200 × 8 × 30 / 360)
─────────────────────────────────────────────────────
"""

import time
import logging
import lgpio

logger = logging.getLogger("PiSlider.HW")

# =============================================================================
# GPIO PINOUT — BCM numbers
# =============================================================================
PIN_SLIDER_STEP = 2
PIN_SLIDER_DIR  = 3
PIN_PAN_STEP    = 16   # GP16 (Pin 36) — no PWM conflicts
PIN_PAN_DIR     = 17
PIN_TILT_STEP   = 19   # MULTIPLEXED TO HARDWARE PWM CHANNEL 3
PIN_TILT_DIR    = 22

PIN_ENABLE      = 10   # Active LOW — pulls all TMC2209 EN pins
PIN_ENDSTOP     = 9    # Endstop input
PIN_CAMERA      = 8    # S2 shutter trigger via optocoupler
PIN_RELAY1      = 6    # Relay 1 → Ring Light (macro mode)
PIN_RELAY2      = 5    # Relay 2 → Laser Projector (macro mode)
PIN_FAN         = None # Disabled to prevent PWM0 conflict with Pan
PIN_AUX         = 13   # Aux Trigger
PIN_HALL        = 27   # Hall Sensor
PIN_FLASH       = 4    # Flash sync trigger — brief pulse to PC-sync port of external flash

# =============================================================================
# 1/8 MICROSTEP CONVERSION FACTORS
# =============================================================================
# Focus rail: 2mm pitch lead screw → 1 revolution = 2mm movement
# 200 steps/rev × 8 microsteps = 1600 microsteps/rev → 800 microsteps/mm
STEPS_PER_MM         = 800.000   # Focus rail: 200 × 8 / 2mm pitch
SLIDER_STEPS_PER_MM  =  50.000   # [LEGACY — do not use, use STEPS_PER_MM instead]
PAN_STEPS_PER_DEG    =  66.667   # 200 × 8 × 15 / 360
TILT_STEPS_PER_DEG   = 133.333   # 200 × 8 × 30 / 360

# VACTUAL-to-steps/sec bridge (for cinematic engine compatibility)
# vactual was calibrated as:  VACTUAL = physical_velocity × VACTUAL_PER_UNIT
#   VACTUAL_PER_MM_S      = 12.500   (slider)
#   VACTUAL_PER_DEG_S_PAN = 15.625   (pan)
#   VACTUAL_PER_DEG_S_TLT = 23.125   (tilt)
# Bridge multiplier = STEPS_PER_UNIT / VACTUAL_PER_UNIT
_VACTUAL_TO_SPS = {
    0: ('tilt',   TILT_STEPS_PER_DEG  / 23.125),   # ≈ 5.765 sps/vactual
    1: ('pan',    PAN_STEPS_PER_DEG   / 15.625),   # ≈ 4.267 sps/vactual
    2: ('slider', SLIDER_STEPS_PER_MM / 12.500),   # = 4.000 sps/vactual
}


class HardwareController:
    """
    Unified hardware interface for:
      • TMC2209 UART velocity/position control (cinematic + joystick)
      • Coordinated Bresenham pulse stepping (timelapse MSM)
      • Camera shutter trigger (S2 optocoupler)
      • Relay control (ring light, laser projector)
      • Fan PWM
      • Endstop reading
    """

    def __init__(self, gpio_chip_index: int = 4):  # Pi 5: main GPIO is gpiochip4
        self.gpio_chip_index = gpio_chip_index
        try:
            self.gpio_chip = lgpio.gpiochip_open(gpio_chip_index)
            self._init_gpio()
            self.enable_motors(False)   # Safety: start disabled
            self.inversions = {0: False, 1: False, 2: False}
            logger.info("HardwareController initialised (standalone STEP/DIR mode, 1/8 step).")
        except lgpio.error as e:
            if 'busy' in str(e).lower():
                # GPIO lines still claimed by a previous process — force-free and retry
                logger.warning(f"GPIO busy on first attempt — freeing chip and retrying in 1s…")
                try:
                    lgpio.gpiochip_close(self.gpio_chip)
                except Exception:
                    pass
                import time as _t; _t.sleep(1.0)
                try:
                    self.gpio_chip = lgpio.gpiochip_open(gpio_chip_index)
                    self._init_gpio()
                    self.enable_motors(False)
                    logger.info("HardwareController initialised (after GPIO retry).")
                except Exception as e2:
                    logger.error(f"Hardware init failed after retry: {e2}")
                    raise
            else:
                logger.error(f"Hardware init failed: {e}")
                raise
        except Exception as e:
            logger.error(f"Hardware init failed: {e}")
            raise

    # -------------------------------------------------------------------------
    # GPIO INITIALISATION
    # -------------------------------------------------------------------------
    def _init_gpio(self):
        outputs = [
            PIN_SLIDER_STEP, PIN_SLIDER_DIR,
            PIN_PAN_STEP,    PIN_PAN_DIR,
            PIN_TILT_STEP,   PIN_TILT_DIR,
            PIN_ENABLE,
            PIN_CAMERA,
            PIN_RELAY1,
            PIN_RELAY2,
            PIN_AUX,
            PIN_FLASH,
        ]
        for pin in outputs:
            lgpio.gpio_claim_output(self.gpio_chip, pin)
            lgpio.gpio_write(self.gpio_chip, pin, 0)

        lgpio.gpio_claim_input(self.gpio_chip, PIN_ENDSTOP, lgpio.SET_PULL_UP)
        lgpio.gpio_claim_input(self.gpio_chip, PIN_HALL,    lgpio.SET_PULL_UP)

    # -------------------------------------------------------------------------
    # TMC2209 UART — STUBS (drivers run in standalone mode, no UART needed)
    # -------------------------------------------------------------------------
    def _send_tmc_uart(self, addr: int, reg: int, data: int):
        """No-op: drivers are in standalone mode (UART not connected)."""
        pass

    def _read_tmc_uart(self, addr: int, reg: int) -> int:
        """No-op: returns 0 (standalone mode, no UART)."""
        return 0

    # -------------------------------------------------------------------------
    # TMC2209 — HIGH LEVEL
    # -------------------------------------------------------------------------
    def set_tmc_velocity(self, addr: int, velocity: int):
        """
        Cinematic velocity command — bridged to STEP/DIR PWM.

        Converts signed VACTUAL units (as used by InertiaEngine / TrajectoryPlayer)
        to steps/sec and calls set_axis_speed().

        addr: 0=Tilt, 1=Pan, 2=Slider
        velocity: signed VACTUAL units
        """
        mapping = _VACTUAL_TO_SPS.get(addr)
        if mapping is None:
            return
        axis_name, multiplier = mapping
        # Interpret as signed 24-bit value
        v = int(velocity)
        if v >= (1 << 23):
            v -= (1 << 24)
        self.set_axis_speed(axis_name, v * multiplier)

    def get_tmc_position(self, addr: int) -> int:
        """Returns 0 — position tracking is software-only in standalone mode."""
        return 0

    def get_tmc_driver_status(self, addr: int) -> dict:
        """Returns dummy status — driver status unavailable in standalone mode."""
        return {
            "ot": False, "otpw": False, "s2ga": False, "s2gb": False,
            "ola": False, "olb": False, "stst": False, "raw": 0,
        }

    def set_tmc_current(self, addr: int, run_current: int = 16, hold_current: int = 8):
        """No-op: current is set by VREF trim pot on each module."""
        pass

    def init_tmc_drivers(self):
        """No-op: drivers run in standalone mode, no UART configuration needed."""
        logger.info("Standalone mode — TMC2209 drivers need no UART init (VREF pots set current).")

    # -------------------------------------------------------------------------
    # COORDINATED BRESENHAM STEPPING (Move-Shoot-Move timelapse)
    # -------------------------------------------------------------------------
    def move_axes_simultaneous(
        self,
        slider_steps: int,
        pan_steps: int,
        tilt_steps: int,
        duration_s: float,
    ):
        """
        Pulse all three motors concurrently using Bresenham interpolation
        so they arrive at their targets at exactly the same time.

        Steps are signed (positive = forward direction per DIR pin logic).
        Duration controls overall speed — longer = slower.
        """
        s_dir = 1 if slider_steps >= 0 else 0
        p_dir = 1 if pan_steps    >= 0 else 0
        t_dir = 1 if tilt_steps   >= 0 else 0

        # Stop any active PWM waveforms and reclaim STEP pins as plain GPIO outputs.
        # InertiaEngine drives STEP pins via lgpio.tx_pwm(), which keeps the pin
        # in "waveform-owned" state even after tx_pwm(duty=0) stops the pulses.
        # gpio_write() on a waveform-owned pin throws an lgpio error.  Calling
        # gpio_claim_output() transfers ownership back to the GPIO layer so the
        # Bresenham bit-bang below can use gpio_write() safely.
        for p in (PIN_SLIDER_STEP, PIN_PAN_STEP, PIN_TILT_STEP):
            lgpio.tx_pwm(self.gpio_chip, p, 100, 0)       # ensure waveform stopped
            lgpio.gpio_claim_output(self.gpio_chip, p)    # reclaim for gpio_write()
            lgpio.gpio_write(self.gpio_chip, p, 0)        # ensure pin is LOW before stepping

        # Apply inversions
        if self.inversions.get(0): s_dir = 1 - s_dir
        if self.inversions.get(1): p_dir = 1 - p_dir
        if self.inversions.get(2): t_dir = 1 - t_dir

        lgpio.gpio_write(self.gpio_chip, PIN_SLIDER_DIR, s_dir)
        lgpio.gpio_write(self.gpio_chip, PIN_PAN_DIR,    p_dir)
        lgpio.gpio_write(self.gpio_chip, PIN_TILT_DIR,   t_dir)

        s_tgt = abs(slider_steps)
        p_tgt = abs(pan_steps)
        t_tgt = abs(tilt_steps)
        max_steps = max(s_tgt, p_tgt, t_tgt)

        if max_steps == 0:
            return

        # Detailed motor control logging
        if slider_steps != 0:
            logger.info(f"🔵 SLIDER: {slider_steps:+d} steps, {abs(slider_steps)/STEPS_PER_MM:.2f}mm @ {duration_s:.2f}s")
        if pan_steps != 0:
            logger.info(f"🟢 PAN: {pan_steps:+d} steps, {abs(pan_steps)/PAN_STEPS_PER_DEG:.2f}° @ {duration_s:.2f}s")
        if tilt_steps != 0:
            logger.info(f"🔴 TILT: {tilt_steps:+d} steps, {abs(tilt_steps)/TILT_STEPS_PER_DEG:.2f}° @ {duration_s:.2f}s")

        # Trapezoidal velocity profile — ramp up, hold, ramp down.
        # Prevents jarring instantaneous starts/stops that stress hardware and
        # increase vibration settling time needed before capture.
        #
        # Profile: 25% ramp-up | 50% constant peak speed | 25% ramp-down
        # The peak (fastest) delay is chosen so total step time = duration_s.
        #
        # Average speed analysis:
        #   Ramp portion: average speed = 0.75× peak (linear from 0.5× to 1×)
        #   Constant portion: speed = 1× peak
        #   Total average = 0.25×0.75 + 0.50×1.0 + 0.25×0.75 = 0.875× peak
        #   → peak_delay = (duration_s / max_steps) × 0.875
        #   → start_delay = peak_delay × 2.0 (half peak speed at endpoints)
        #
        # For very short moves (< 8 steps), ramp collapses to constant speed.
        avg_delay   = duration_s / max_steps
        peak_delay  = avg_delay * 0.875          # fastest inter-step delay (constant phase)
        start_delay = peak_delay * 2.0           # slowest inter-step delay (at start/end)
        ramp_steps  = max(1, max_steps // 4)     # 25% of total steps for each ramp

        def _step_delay(step_n: int) -> float:
            """Return inter-step delay for step_n using trapezoidal profile."""
            if max_steps < 8:
                return avg_delay   # too short to ramp meaningfully
            if step_n < ramp_steps:
                # Ramp up: delay decreases linearly from start_delay to peak_delay
                t = step_n / ramp_steps
                return start_delay + (peak_delay - start_delay) * t
            elif step_n >= max_steps - ramp_steps:
                # Ramp down: delay increases linearly from peak_delay to start_delay
                t = (max_steps - 1 - step_n) / ramp_steps
                return start_delay + (peak_delay - start_delay) * t
            else:
                return peak_delay

        logger.debug(f"Bresenham trapezoidal: max_steps={max_steps}, "
                    f"peak_delay={peak_delay*1000:.2f}ms, start_delay={start_delay*1000:.2f}ms, "
                    f"ramp_steps={ramp_steps}, total≈{duration_s:.2f}s")

        s_err = p_err = t_err = 0
        endstop_check_interval = max(1, max_steps // 50)   # check ~50 times per move

        # Safety timeout: if loop runs longer than 2× expected duration, abort
        start_time = time.time()
        timeout_s = duration_s * 2.0 + 5.0

        for step_n in range(max_steps):
            # Safety timeout check
            elapsed = time.time() - start_time
            if elapsed > timeout_s:
                logger.error(f"move_axes_simultaneous timeout! Elapsed {elapsed:.1f}s > {timeout_s:.1f}s "
                           f"(expected {duration_s:.1f}s). Aborting to prevent runaway.")
                break

            # Endstop check every N steps to protect slider from over-travel.
            if step_n % endstop_check_interval == 0:
                if s_tgt > 0 and lgpio.gpio_read(self.gpio_chip, PIN_ENDSTOP) == 0:
                    logger.warning("Endstop triggered — halting slider steps.")
                    s_tgt = 0
                if lgpio.gpio_read(self.gpio_chip, PIN_HALL) == 0:
                    logger.warning("Hall sensor triggered — halting slider steps.")
                    s_tgt = 0

            s_err += s_tgt
            if s_err >= max_steps:
                lgpio.gpio_write(self.gpio_chip, PIN_SLIDER_STEP, 1)
                s_err -= max_steps

            p_err += p_tgt
            if p_err >= max_steps:
                lgpio.gpio_write(self.gpio_chip, PIN_PAN_STEP, 1)
                p_err -= max_steps

            t_err += t_tgt
            if t_err >= max_steps:
                lgpio.gpio_write(self.gpio_chip, PIN_TILT_STEP, 1)
                t_err -= max_steps

            time.sleep(0.0001)   # ~100µs pulse width
            lgpio.gpio_write(self.gpio_chip, PIN_SLIDER_STEP, 0)
            lgpio.gpio_write(self.gpio_chip, PIN_PAN_STEP,    0)
            lgpio.gpio_write(self.gpio_chip, PIN_TILT_STEP,   0)

            time.sleep(max(0, _step_delay(step_n) - 0.0001))

    # -------------------------------------------------------------------------
    # PERIPHERALS
    # -------------------------------------------------------------------------
    def enable_motors(self, enable: bool):
        """Enable or disable all TMC2209 drivers (EN pin, active LOW)."""
        lgpio.gpio_write(self.gpio_chip, PIN_ENABLE, 0 if enable else 1)

    def set_inversions(self, slider: bool, pan: bool, tilt: bool):
        """Set direction inversion for each axis."""
        self.inversions[0] = bool(slider)
        self.inversions[1] = bool(pan)
        self.inversions[2] = bool(tilt)
        logger.info(f"Motor inversions updated: slider={slider}, pan={pan}, tilt={tilt}")

    def trigger_camera(self, duration_s: float = 0.2):
        """Fire the S2 shutter trigger via optocoupler on GP8."""
        lgpio.gpio_write(self.gpio_chip, PIN_CAMERA, 1)
        time.sleep(duration_s)
        lgpio.gpio_write(self.gpio_chip, PIN_CAMERA, 0)

    def trigger_flash(self, duration_s: float = 0.010):
        """Pulse the flash sync output (GP4) to fire an external flash.

        A 10 ms pulse is enough to trigger any standard PC-sync flash.
        The caller should record time.time() immediately before this call
        to get the most accurate flash_sync_wall timestamp.
        """
        lgpio.gpio_write(self.gpio_chip, PIN_FLASH, 1)
        time.sleep(duration_s)
        lgpio.gpio_write(self.gpio_chip, PIN_FLASH, 0)

    def set_fan(self, state: bool):
        pass # Fan disabled

    def set_relay1(self, on: bool):
        """
        Relay 1 (GP6) — Ring Light for macro/focus-stack mode.
        True = energise relay = light ON.
        """
        lgpio.gpio_write(self.gpio_chip, PIN_RELAY1, 1 if on else 0)

    def set_relay2(self, on: bool):
        """
        Relay 2 (GP5) — Laser Structured-Light Projector.
        True = energise relay = laser ON.
        """
        lgpio.gpio_write(self.gpio_chip, PIN_RELAY2, 1 if on else 0)

    def read_endstop(self) -> bool:
        """
        Read the endstop input (GP9).
        Returns True when triggered (active LOW — pulled high normally).
        Note: only valid when no UART transaction is in progress.
        """
        return lgpio.gpio_read(self.gpio_chip, PIN_ENDSTOP) == 0

    def read_hall_sensor(self) -> bool:
        """
        Read the Hall sensor (GP19).
        Returns True when triggered (typically active LOW when magnet is near).
        """
        return lgpio.gpio_read(self.gpio_chip, PIN_HALL) == 0

    # -------------------------------------------------------------------------
    # STEP/DIR SPEED CONTROL (Cinema mode — PWM on STEP pins)
    # -------------------------------------------------------------------------
    _AXIS_PINS = {
        'slider': (PIN_SLIDER_STEP, PIN_SLIDER_DIR, 0),  # (step, dir, inversion_key)
        'pan':    (PIN_PAN_STEP,    PIN_PAN_DIR,    1),
        'tilt':   (PIN_TILT_STEP,   PIN_TILT_DIR,   2),
    }

    def set_axis_speed(self, axis: str, steps_per_sec: float):
        """
        Drive an axis via STEP/DIR pins using PWM.

        axis:          'slider', 'pan', or 'tilt'
        steps_per_sec: signed step rate (negative = reverse direction)
        """
        step_pin, dir_pin, inv_key = self._AXIS_PINS[axis]
        freq = abs(steps_per_sec)
        forward = steps_per_sec >= 0

        if self.inversions.get(inv_key):
            forward = not forward

        if freq < 0.5:
            # Stop the waveform by calling tx_pwm with duty_cycle=0.
            # lgpio docs: "If pwmDutyCycle is 0 the GPIO is set to LOW and
            # no more pulses are created."
            #
            # IMPORTANT: do NOT call gpio_write() here.  tx_pwm() claims the
            # STEP pin for its waveform thread; gpio_write() on a waveform-owned
            # pin (a) throws an lgpio error that crashes _tick() and stops the
            # InertiaEngine, and (b) is immediately overridden by the waveform
            # thread anyway — so it cannot stop the motor.  tx_pwm(duty=0) is
            # the only reliable way to stop an active waveform.
            lgpio.tx_pwm(self.gpio_chip, step_pin, 100, 0)
            return

        lgpio.gpio_write(self.gpio_chip, dir_pin, 1 if forward else 0)
        lgpio.tx_pwm(self.gpio_chip, step_pin, freq, 50)

    def stop_all_axes(self):
        """Stop PWM on all STEP pins (holds pin low)."""
        for pin in (PIN_SLIDER_STEP, PIN_PAN_STEP, PIN_TILT_STEP):
            lgpio.tx_pwm(self.gpio_chip, pin, 100, 0)

    # -------------------------------------------------------------------------
    # MICROSTEPPING
    # -------------------------------------------------------------------------
    def set_microstepping(self, addr: int, mstep: int):
        """No-op: microstepping fixed at 1/8 by MS1=MS2=0 hardware jumpers."""
        logger.debug(f"set_microstepping ignored (standalone 1/8 step mode)")

    def set_mode_microstepping(self, mode: str):
        """No-op: microstepping fixed at 1/8 by MS1=MS2=0 hardware jumpers."""
        pass

    # -------------------------------------------------------------------------
    # SAFE SHUTDOWN
    # -------------------------------------------------------------------------
    def cleanup(self):
        """Gracefully stop all motion and release GPIO."""
        try:
            self.stop_all_axes()
            self.enable_motors(False)
            self.set_relay1(False)
            self.set_relay2(False)
            self.set_fan(0)
        except Exception:
            pass
        try:
            lgpio.gpiochip_close(self.gpio_chip)
        except Exception:
            pass
        logger.info("HardwareController shutdown complete.")
