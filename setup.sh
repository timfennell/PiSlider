#!/usr/bin/env bash
#
# setup.sh — one-time installer for PiSlider dependencies
#
# Installs:
#   - System-level packages (libcamera, gphoto2, lgpio, opencv, etc.)
#   - Python virtual environment (.venv)
#   - Python requirements via pip WITHOUT sudo
#
# Usage:
#   cd /home/tim/Projects/pislider
#   sudo ./setup.sh
#

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$PROJECT_DIR/.venv"

echo "PiSlider setup starting..."
echo "Project directory: $PROJECT_DIR"
echo

# ---------------------------------------------------------------------------
# 1. Update system package lists
# ---------------------------------------------------------------------------
echo ">>> Updating apt package lists..."
sudo apt update

# ---------------------------------------------------------------------------
# 2. Install essential build tools
# ---------------------------------------------------------------------------
echo ">>> Installing core development tools..."
sudo apt install -y \
    python3-dev \
    python3-venv \
    python3-pip \
    build-essential \
    pkg-config

# ---------------------------------------------------------------------------
# 3. Install camera, GPIO, imaging, and library dependencies
# ---------------------------------------------------------------------------
echo ">>> Installing camera, gphoto2, GPIO, and imaging dependencies..."

sudo apt install -y \
    python3-picamera2 \
    python3-libcamera \
    libcamera-apps \
    gphoto2 \
    libgphoto2-dev \
    libgpiod2 \
    python3-lgpio \
    python3-opencv \
    libopencv-dev \
    libatlas-base-dev \
    libxslt1-dev \
    libxml2-dev \
    dcraw \
    exiftool

# ---------------------------------------------------------------------------
# 4. Create virtual environment (if missing)
# ---------------------------------------------------------------------------
if [ ! -d "$VENV_DIR" ]; then
    echo ">>> Creating Python virtual environment at $VENV_DIR ..."
    python3 -m venv "$VENV_DIR"
else
    echo ">>> Virtual environment already exists at $VENV_DIR"
fi

# ---------------------------------------------------------------------------
# 5. Activate virtual environment (WITHOUT sudo)
# ---------------------------------------------------------------------------
echo ">>> Activating virtual environment..."
# shellcheck disable=SC1090
source "$VENV_DIR/bin/activate"

# Upgrade pip + wheel
echo ">>> Upgrading pip, setuptools, and wheel..."
pip install --upgrade pip setuptools wheel

# ---------------------------------------------------------------------------
# 6. Install Python dependencies (WITHOUT sudo!)
# ---------------------------------------------------------------------------
REQ_FILE="$PROJECT_DIR/requirements.txt"
if [ ! -f "$REQ_FILE" ]; then
    echo "ERROR: requirements.txt not found at $REQ_FILE"
    exit 1
fi

echo ">>> Installing Python packages from requirements.txt..."
pip install -r "$REQ_FILE"

echo
echo "========================================================"
echo "PiSlider setup complete!"
echo
echo "To run PiSlider now, use:"
echo "  cd \"$PROJECT_DIR\""
echo "  ./run.sh"
echo
echo "Or activate the virtual environment manually with:"
echo "  source \"$VENV_DIR/bin/activate\""
echo
echo "========================================================"
