#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# setup_hostname.sh — PiSlider network hostname configuration
#
# After running this script you can reach the interface at:
#   http://pislider          — when connected to the Pi's own WiFi hotspot
#   http://pislider.local    — when Pi and computer are on the same LAN
#   http://localhost:8000    — always works from the Pi itself
#
# Run once as root:  sudo bash setup_hostname.sh
# ─────────────────────────────────────────────────────────────────────────────

set -e

HOSTNAME="pislider"
APP_PORT=8000

echo "═══════════════════════════════════════════"
echo "  PiSlider Hostname Setup"
echo "═══════════════════════════════════════════"

# ── 1. Set system hostname ────────────────────────────────────────────────────
echo "→ Setting hostname to '$HOSTNAME'..."
hostnamectl set-hostname "$HOSTNAME"
# Update /etc/hosts so localhost resolution still works
if ! grep -q "$HOSTNAME" /etc/hosts; then
    sed -i "s/127.0.1.1.*/127.0.1.1\t$HOSTNAME/" /etc/hosts
    # If there was no 127.0.1.1 line, add one
    if ! grep -q "127.0.1.1" /etc/hosts; then
        echo "127.0.1.1    $HOSTNAME" >> /etc/hosts
    fi
fi
echo "   ✓ Hostname set."

# ── 2. Enable mDNS (avahi) for http://pislider.local on LAN ──────────────────
echo "→ Configuring mDNS (avahi-daemon)..."
if ! dpkg -l avahi-daemon &>/dev/null; then
    apt-get install -y avahi-daemon
fi
# Ensure avahi is enabled and running
systemctl enable avahi-daemon
systemctl restart avahi-daemon
echo "   ✓ mDNS active — 'pislider.local' will resolve on your LAN."

# ── 3. Configure dnsmasq for hotspot DNS (http://pislider with no port) ───────
# When the Pi runs its own WiFi hotspot, dnsmasq handles DHCP and DNS for
# connected clients. We add a static A record so 'pislider' resolves to the
# Pi's hotspot IP (typically 10.42.0.1 for NetworkManager hotspot).
echo "→ Configuring dnsmasq for hotspot DNS..."

if ! dpkg -l dnsmasq &>/dev/null; then
    apt-get install -y dnsmasq
fi

DNSMASQ_CONF="/etc/dnsmasq.d/pislider.conf"
cat > "$DNSMASQ_CONF" <<EOF
# PiSlider — resolve 'pislider' to this Pi for hotspot clients
# Both http://pislider and http://pislider.local will work.

# Hotspot interface (NetworkManager default is 10.42.0.1)
# If your hotspot uses a different subnet, update this IP.
address=/pislider/10.42.0.1
address=/pislider.local/10.42.0.1

# Also respond to plain 'pislider' without .local
domain=local
expand-hosts
EOF

echo "   ✓ dnsmasq config written to $DNSMASQ_CONF"

# ── 4. Nginx reverse proxy on port 80 → app port 8000 ────────────────────────
# This lets http://pislider (port 80, the browser default) work without
# typing the port number. Without this you'd need http://pislider:8000.
echo "→ Setting up nginx reverse proxy (port 80 → $APP_PORT)..."

if ! dpkg -l nginx &>/dev/null; then
    apt-get install -y nginx
fi

NGINX_CONF="/etc/nginx/sites-available/pislider"
cat > "$NGINX_CONF" <<EOF
# PiSlider nginx reverse proxy
# Forwards http://pislider (port 80) to the FastAPI app on port $APP_PORT
# Also handles WebSocket upgrade for /ws

server {
    listen 80 default_server;
    listen [::]:80 default_server;
    server_name pislider pislider.local _;

    # Increase buffer for MJPEG streaming
    proxy_buffering off;

    location / {
        proxy_pass         http://127.0.0.1:$APP_PORT;
        proxy_http_version 1.1;
        proxy_set_header   Host \$host;
        proxy_set_header   X-Real-IP \$remote_addr;
        proxy_read_timeout 3600s;
    }

    # WebSocket support (/ws endpoint)
    location /ws {
        proxy_pass         http://127.0.0.1:$APP_PORT/ws;
        proxy_http_version 1.1;
        proxy_set_header   Upgrade \$http_upgrade;
        proxy_set_header   Connection "upgrade";
        proxy_set_header   Host \$host;
        proxy_read_timeout 3600s;
    }

    # MJPEG video stream (disable buffering for live preview)
    location /video_feed {
        proxy_pass         http://127.0.0.1:$APP_PORT/video_feed;
        proxy_http_version 1.1;
        proxy_set_header   Host \$host;
        proxy_buffering    off;
        proxy_cache        off;
        proxy_read_timeout 3600s;
        add_header         Cache-Control "no-cache, no-store";
    }
}
EOF

# Enable site
ln -sf "$NGINX_CONF" /etc/nginx/sites-enabled/pislider
# Disable default nginx page if present
rm -f /etc/nginx/sites-enabled/default

nginx -t && systemctl enable nginx && systemctl restart nginx
echo "   ✓ nginx running — port 80 → $APP_PORT"

# ── 5. Restart dnsmasq (if it's the system dnsmasq, not NetworkManager's) ────
# NetworkManager runs its own internal dnsmasq; we don't want to fight it.
# Our config in dnsmasq.d/ will be picked up by NM's dnsmasq automatically
# when the hotspot starts.
if systemctl is-active --quiet dnsmasq 2>/dev/null; then
    systemctl restart dnsmasq
    echo "   ✓ dnsmasq restarted."
else
    echo "   ℹ dnsmasq not running standalone — config will be picked up"
    echo "     by NetworkManager's dnsmasq when the hotspot activates."
fi

# ── 6. Summary ────────────────────────────────────────────────────────────────
echo ""
echo "═══════════════════════════════════════════"
echo "  Setup complete!"
echo ""
echo "  Access PiSlider at:"
echo "    http://pislider        (hotspot clients, no port needed)"
echo "    http://pislider.local  (same LAN via mDNS)"
echo "    http://$(hostname -I | awk '{print $1}'):$APP_PORT  (direct IP, always works)"
echo ""
echo "  NOTE: Browser GPS (for Holy Grail mode) requires either:"
echo "    - HTTPS, OR"
echo "    - localhost / 127.0.0.1 (treated as secure by browsers)"
echo "    The 📍 button works fine when accessed via http://pislider"
echo "    on modern browsers (Chrome/Firefox allow geolocation on"
echo "    local network addresses without HTTPS)."
echo "═══════════════════════════════════════════"
