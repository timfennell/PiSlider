#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# install-service.sh — Install PiSlider on-demand wake system
#
# After running this:
#   • pislider-wake.service  starts on boot (tiny, ~8 MB, 0% CPU)
#   • pislider.service       starts only when you visit the URL
#   • Stopping pislider re-arms the wake trigger automatically
#
# Run once on the Pi with:  sudo bash install-service.sh
# ─────────────────────────────────────────────────────────────────────────────

set -e

PROJECT_DIR="/home/tim/Projects/pislider"
SYSTEMD_DIR="/etc/systemd/system"

echo "=== PiSlider Service Installer ==="
echo ""

# Must run as root
if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: Run this with sudo: sudo bash install-service.sh"
    exit 1
fi

# Check required files exist
for f in pislider.service pislider-wake.service pislider_wake.py run.sh; do
    if [ ! -f "$PROJECT_DIR/$f" ]; then
        echo "ERROR: $PROJECT_DIR/$f not found."
        exit 1
    fi
done

# ── Stop and clean up any currently running instances ─────────────────────
echo "→ Stopping any running PiSlider processes..."
systemctl stop pislider      2>/dev/null || true
systemctl stop pislider-wake 2>/dev/null || true
pkill -f "python.*app\.py"   2>/dev/null || true
pkill -f "pislider_wake\.py" 2>/dev/null || true
sleep 2

# ── Install service files ──────────────────────────────────────────────────
echo "→ Installing service files..."
cp "$PROJECT_DIR/pislider.service"      "$SYSTEMD_DIR/pislider.service"
cp "$PROJECT_DIR/pislider-wake.service" "$SYSTEMD_DIR/pislider-wake.service"
chmod 644 "$SYSTEMD_DIR/pislider.service"
chmod 644 "$SYSTEMD_DIR/pislider-wake.service"

# ── Reload systemd ─────────────────────────────────────────────────────────
echo "→ Reloading systemd daemon..."
systemctl daemon-reload

# ── Enable only the wake trigger on boot ──────────────────────────────────
# pislider.service is NOT enabled — it starts on demand via the wake trigger.
echo "→ Enabling wake trigger on boot (pislider itself starts on demand)..."
systemctl disable pislider      2>/dev/null || true   # ensure full app does NOT auto-start
systemctl enable  pislider-wake

# ── Start the wake trigger now ────────────────────────────────────────────
echo "→ Starting wake trigger..."
systemctl start pislider-wake

sleep 2
echo ""
echo "=== Service Status ==="
systemctl status pislider-wake --no-pager -l

echo ""
echo "=== Done! ==="
echo ""
echo "How it works:"
echo "  • On boot, a tiny listener (pislider-wake) watches port 8000 — ~8 MB RAM"
echo "  • Visit http://pislider.local:8000/ — you'll see a 'Starting...' splash"
echo "  • The full app starts automatically (~10 seconds)"
echo "  • Stop the app → wake trigger re-arms for next time"
echo ""
echo "Useful commands:"
echo "  sudo systemctl status pislider          — check if full app is running"
echo "  sudo systemctl status pislider-wake     — check if wake trigger is armed"
echo "  sudo systemctl stop pislider            — stop the app (re-arms wake)"
echo "  sudo systemctl restart pislider         — restart after code changes"
echo "  sudo journalctl -u pislider -f          — live app log"
echo "  sudo journalctl -u pislider-wake -f     — live wake trigger log"
