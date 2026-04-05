#!/bin/bash
# Setup native CAN interface for Jetson Orin Nano
# Single CAN bus (can0) — all 12 motors on one channel
# Run once after each boot

set -e

BITRATE=1000000
TXQLEN=1000

echo "Setting up Jetson native CAN interface..."

iface=can0

if ! ip link show "$iface" &>/dev/null; then
    echo "  ❌ $iface not found"
    exit 1
fi

# Bring down first (reset any error state)
sudo ip link set "$iface" down 2>/dev/null || true
sleep 0.2

# Configure and bring up at 1Mbps
sudo ip link set "$iface" type can bitrate "$BITRATE"
sudo ip link set "$iface" up
sudo ip link set "$iface" txqueuelen "$TXQLEN"

state=$(ip -details link show "$iface" | grep "can state" | awk '{print $3}')
echo "  ✅ $iface: bitrate=${BITRATE}, txqlen=${TXQLEN}, state=${state}"

echo ""
echo "can0: All 12 motors (IDs 1-12)"
echo "Done."
