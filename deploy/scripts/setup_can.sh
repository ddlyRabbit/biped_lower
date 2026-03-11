#!/bin/bash
# Setup CAN interfaces for 2-CH CAN HAT (MCP2515)
# Run once after each boot, or add to /etc/rc.local

set -e

echo "Setting up CAN interfaces..."

# Bring up CAN0 and CAN1 at 1Mbps
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000

# Increase TX queue length for burst commands
sudo ifconfig can0 txqueuelen 1000
sudo ifconfig can1 txqueuelen 1000

echo "CAN0: $(ip -details link show can0 | grep can | head -1)"
echo "CAN1: $(ip -details link show can1 | grep can | head -1)"
echo "Done."
