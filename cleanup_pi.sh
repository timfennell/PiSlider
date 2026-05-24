#!/bin/bash
# Deletes unused test/diagnostic files from the Pi.
# Usage: ./cleanup_pi.sh [user@host]
# Default host: tim@pislider.local

HOST=${1:-tim@pislider.local}
DIR=/home/tim/Projects/pislider

FILES=(
  test_hall.py test_interval.py test_motor_health.py test_speeds.py
  test_uart.py test_uart_crawl.py test_uart_hold.py test_uart_max.py
  test_uart_motion.py test_uart_optimized.py test_uart_ramp.py test_uart_read.py
  test_uart_robust.py test_uart_safe.py test_uart_slow_start.py test_uart_super.py
  test_uart_vactual.py
  uart_diag.py uart_pin_scan.py uart_port_scan.py uart_raw_debug.py uart_verify.py
  bb_vactual_test.py bitbang_uart_test.py vactual_test.py
  calibrate_180.py motor_180_repeat.py motor_continuous.py motor_test.py
  check_shorts.py check_uart_health.py
  diag_hardware.py diag_slow.py
  gpio_test.py pin_toggle.py pwm_test.py
  measure_steps.py pan_scan.py pan_step_test.py pan_uart_test.py
  single_driver_test.py step_test.py tmc_test.py
  tilt_with_pan_motor.py nuclear_clear.py
  sidecar.py discover.py diagnose.py
)

echo "Connecting to $HOST..."
ssh "$HOST" "
  cd $DIR
  for f in ${FILES[*]}; do
    if [ -f \"\$f\" ]; then
      rm \"\$f\" && echo \"  deleted \$f\"
    fi
  done
  rm -rf __pycache__ venv_test
  echo 'Done. Remaining files:'
  ls
"
