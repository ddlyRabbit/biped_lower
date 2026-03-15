#!/bin/bash
# Setup dual CAN interfaces for Waveshare 2-CH CAN HAT
# MCP2515 × 2 on SPI0 (CE0 + CE1), SocketCAN can0 + can1
# Run once after each boot, or add to /etc/rc.local

set -e

BITRATE=1000000
TXQLEN=1000

echo "Setting up dual CAN interfaces..."

for iface in can0 can1; do
    if ! ip link show "$iface" &>/dev/null; then
        echo "  ❌ $iface not found — check device tree overlays"
        continue
    fi

    # Bring down first (reset any error state)
    sudo ip link set "$iface" down 2>/dev/null || true
    sleep 0.2

    # Bring up at 1Mbps
    sudo ip link set "$iface" up type can bitrate "$BITRATE"

    # Increase TX queue length for burst commands (6 motors × write+read per bus)
    sudo ifconfig "$iface" txqueuelen "$TXQLEN"

    state=$(ip -details link show "$iface" | grep "can state" | awk '{print $3}')
    echo "  ✅ $iface: bitrate=${BITRATE}, txqlen=${TXQLEN}, state=${state}"
done

echo ""
echo "can0: Right leg (R_hip_pitch:1 .. R_foot_roll:6)"
echo "can1: Left leg  (L_hip_pitch:7 .. L_foot_roll:12)"
echo "Done."
