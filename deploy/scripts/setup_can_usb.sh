#!/bin/bash
# Setup CAN interface via USB-CAN adapter (RobStride / CANable / slcand)
# The adapter appears as /dev/ttyACM0 (or ttyACM1)
#
# Usage: ./setup_can_usb.sh [device]
#   device: /dev/ttyACM0 (default)

set -e

DEVICE="${1:-/dev/ttyUSB0}"

if [ ! -e "$DEVICE" ]; then
    echo "ERROR: $DEVICE not found. Is the USB-CAN adapter plugged in?"
    echo "Try: ls /dev/ttyACM*"
    exit 1
fi

echo "Setting up CAN via USB adapter: $DEVICE"

# Load kernel modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
sudo modprobe slcan

# Kill any existing slcand on this device
sudo pkill -f "slcand.*$DEVICE" 2>/dev/null || true
sudo ip link set can0 down 2>/dev/null || true
sudo pkill -f "slcand.*can0" 2>/dev/null || true
sleep 0.5

# Start slcand: -o (open), -c (close on exit), -s8 (1Mbps)
# Speed flags: s0=10k, s1=20k, s2=50k, s3=100k, s4=125k, s5=250k, s6=500k, s7=800k, s8=1M
sudo slcand -o -c -s8 "$DEVICE" can0

# Bring up interface
sudo ip link set up can0
sudo ifconfig can0 txqueuelen 1000

echo "CAN0: $(ip -details link show can0 | grep can | head -1)"
echo "Device: $DEVICE → can0 (1Mbps, all 12 motors)"
echo "Done."
