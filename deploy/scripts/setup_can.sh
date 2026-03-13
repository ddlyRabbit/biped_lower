#!/bin/bash
# Setup CAN interface for Waveshare RS485 CAN HAT (B)
# MCP2515 on SPI0, single SocketCAN interface (can0)
# Run once after each boot, or add to /etc/rc.local

set -e

echo "Setting up CAN interface..."

# Bring up CAN0 at 1Mbps
sudo ip link set can0 up type can bitrate 1000000

# Increase TX queue length for burst commands (12 motors × write+read)
sudo ifconfig can0 txqueuelen 1000

echo "CAN0: $(ip -details link show can0 | grep can | head -1)"
echo "Done. All 12 motors on can0."
