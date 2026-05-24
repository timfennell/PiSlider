# PiSlider Setup Guide

This guide covers everything needed to set up a Raspberry Pi 5 as a PiSlider controller — from a fresh Raspberry Pi OS install through full WiFi hotspot, web server, and automatic startup configuration.

---

## Hardware Requirements

- Raspberry Pi 5 (4GB or 8GB RAM recommended)
- MicroSD card (32GB+ Class 10 / A2)
- Sony IMX477 camera module or compatible
- Stepper motor controllers (connected via GPIO / UART)
- 12V power supply (sufficient for motors + Pi)
- Optional: NeoPixel LED strip for status indication

---

## 1. Raspberry Pi OS Install

Flash **Raspberry Pi OS Lite (64-bit)** (bookworm) using Raspberry Pi Imager.

In the Imager's advanced settings, set:
- Hostname: `pislider`
- Enable SSH with a password or key
- Configure your home WiFi for initial setup

Boot the Pi and SSH in.

---

## 2. System Dependencies

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
    python3-pip python3-venv \
    libcamera-apps \
    nginx \
    avahi-daemon \
    dnsmasq \
    network-manager \
    git
```

---

## 3. Install PiSlider

```bash
cd ~
git clone https://github.com/youruser/pislider.git
cd pislider
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

**requirements.txt includes:**
- `fastapi`, `uvicorn` — web server
- `pydantic` — data models
- `numpy`, `scipy` — math
- `astral` — sun/moon ephemeris for Holy Grail
- `opencv-python-headless` — image analysis
- `rawpy` — DNG raw file reading
- `lgpio` — GPIO for Raspberry Pi 5
- `crcmod` — motor controller communication

---

## 4. WiFi Hotspot Setup

The PiSlider creates its own WiFi access point so you can connect to it in the field without an internet router.

Run the setup script (once, as root):

```bash
sudo bash setup_hostname.sh
```

This script:
1. Sets the hostname to `pislider`
2. Configures NetworkManager to create a WiFi hotspot (`PiSlider` SSID)
3. Installs nginx as a reverse proxy (port 80 → 8000)
4. Configures avahi-daemon for mDNS (`pislider.local` hostname)
5. Sets up dnsmasq so devices connected to the hotspot can resolve `pislider.local`

### Manual Hotspot Configuration

If you prefer to configure the hotspot manually:

```bash
# Create hotspot via NetworkManager
sudo nmcli device wifi hotspot \
    ifname wlan0 \
    ssid "PiSlider" \
    password "yourpassword" \
    band bg

# Make it persistent (auto-start on boot)
sudo nmcli connection modify Hotspot \
    connection.autoconnect yes \
    connection.autoconnect-priority 10
```

---

## 5. nginx Reverse Proxy

nginx listens on port 80 and forwards to the PiSlider FastAPI server on port 8000. This means you access the app at `http://pislider.local/` (no port number required) from your browser.

`/etc/nginx/sites-available/pislider`:
```nginx
server {
    listen 80;
    server_name pislider.local pislider;

    location / {
        proxy_pass http://127.0.0.1:8000;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
        proxy_read_timeout 3600;
    }
}
```

Enable and restart:
```bash
sudo ln -s /etc/nginx/sites-available/pislider /etc/nginx/sites-enabled/
sudo nginx -t
sudo systemctl restart nginx
```

---

## 6. mDNS — `pislider.local`

Avahi provides mDNS so that `pislider.local` resolves to the Pi's IP address on the local network.

```bash
# Verify avahi is running
sudo systemctl status avahi-daemon

# Test resolution from another device on the same network
ping pislider.local
```

When connected to the PiSlider hotspot, dnsmasq also resolves `pislider.local` locally, so the address works even without a router.

---

## 7. Systemd Services — Automatic Server Start

Two systemd services handle automatic startup:

### `pislider.service` — Main Server

The FastAPI/uvicorn server that runs the web interface.

```bash
sudo bash install-service.sh
sudo systemctl enable pislider
sudo systemctl start pislider
```

### `pislider-wake.service` — On-Demand Wake

A lightweight listener that wakes the main server when a client connects to the hotspot. This means the heavy server process doesn't run unless someone is actively connected — useful for battery life in the field.

```bash
sudo systemctl enable pislider-wake
sudo systemctl start pislider-wake
```

### How On-Demand Wake Works

1. `pislider-wake.service` runs `pislider_wake.py` — a minimal socket listener
2. When your phone/laptop connects to the `PiSlider` hotspot and tries to reach `pislider.local`, the wake listener detects the connection
3. It starts `pislider.service` via `systemctl start`
4. Your browser gets the PiSlider UI within a second or two

When you disconnect from the hotspot (or after a timeout), the main server shuts back down.

---

## 8. Connecting and Using PiSlider

1. Power on the PiSlider
2. On your device, connect to WiFi network **`PiSlider`**
3. Open a browser and go to: **`http://pislider.local:8000/`**
   - Or just: **`http://pislider.local/`** (via nginx on port 80)
4. The PiSlider web interface loads

The interface works on iPhone, Android, Mac, and Windows — any modern browser.

---

## 9. Camera Setup

The PiSlider uses `libcamera` for camera control. Verify your camera is detected:

```bash
libcamera-still --list-cameras
```

The app configures camera parameters (shutter, ISO, aperture) via `libcamera-still` subprocess calls. DNGs are captured when raw mode is enabled.

For focus stacking (macro mode), use a manual lens or set the camera to manual focus. Autofocus changes between frames will ruin the stack.

---

## 10. File Storage

Captured images are saved to the path configured in the UI. By default this is a connected USB drive or SD card. For long macro sessions, an external SSD via USB 3 is strongly recommended.

The PiSlider web UI lets you browse and configure the output directory.

