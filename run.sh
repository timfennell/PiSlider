#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# run.sh — PiSlider launch script with auto-restart on crash
# ─────────────────────────────────────────────────────────────────────────────

cd "$(dirname "$0")"
source /home/tim/Projects/pislider/.venv/bin/activate

# Kill any previous instance holding GPIO handles
STALE=$(pgrep -f "python.*app\.py" 2>/dev/null)
if [ -n "$STALE" ]; then
    echo "Cleaning up stale PiSlider process(es): $STALE"
    kill -TERM $STALE 2>/dev/null
    sleep 2
    kill -KILL $STALE 2>/dev/null
    sleep 1
fi

# Auto-restart loop — if app crashes it comes back automatically.
# A clean exit (code 0, e.g. from os.execv restart) also loops back.
# Press Ctrl+C twice to actually stop.
RESTART_DELAY=3

while true; do
    echo "[$(date '+%H:%M:%S')] Starting PiSlider..."
    python3 app.py
    EXIT_CODE=$?
    echo "[$(date '+%H:%M:%S')] PiSlider exited (code $EXIT_CODE)."

    # Exit code 42 = intentional shutdown (future use for clean stop)
    if [ $EXIT_CODE -eq 42 ]; then
        echo "Clean shutdown requested — not restarting."
        break
    fi

    echo "Restarting in ${RESTART_DELAY}s... (Ctrl+C to abort)"
    sleep $RESTART_DELAY
done
