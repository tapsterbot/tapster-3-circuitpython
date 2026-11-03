#!/bin/bash
# Usage: ./deploy.sh [serial port | CIRCUITPY mount]
#   ESP32: uploads over serial with ampy (/dev/cu.usbserial-*)
#   Pico:  copies to the CIRCUITPY USB drive (/Volumes/CIRCUITPY)
set -e

cd "$(dirname "$0")"

# code.py last, so auto-reload sees the other files already in place
FILES="config.json dynamixel.py kinematics.py motion.py hardware.py pbm.py code.py"

TARGET="${1:-}"
if [ -z "$TARGET" ]; then
    if [ -d /Volumes/CIRCUITPY ]; then
        TARGET=/Volumes/CIRCUITPY
    else
        TARGET="$(ls /dev/cu.usbserial-* 2>/dev/null | head -1)"
    fi
fi

if [ -z "$TARGET" ]; then
    echo "No board found. Usage: ./deploy.sh [port | CIRCUITPY mount]"
    exit 1
fi

echo "Deploying to $TARGET"

for f in $FILES; do
    echo "  $f"
    if [ -d "$TARGET" ]; then
        cp "firmware/$f" "$TARGET/$f"
    else
        ampy --port "$TARGET" put "firmware/$f"
    fi
done

[ -d "$TARGET" ] && sync

echo "Done."
