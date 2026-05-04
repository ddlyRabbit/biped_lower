#!/bin/bash

# Default CAN interface
IFACE=${1:-can0}
DBC_FILE="$(dirname "$0")/robstride.dbc"

if [ ! -f "$DBC_FILE" ]; then
    echo "Error: Cannot find $DBC_FILE"
    exit 1
fi

if ! command -v cantools &> /dev/null; then
    echo "Error: cantools not installed. Run: pip3 install cantools"
    exit 1
fi

echo "Listening on $IFACE using $DBC_FILE..."
echo "Press Ctrl+C to stop."
echo "--------------------------------------------------------"

# candump format: 
#   can0  123#00112233
# We need to make sure the output matches what cantools expects.
# cantools decode expects standard candump log format by default, but it can read stdin.

candump -t a $IFACE | cantools decode --single-line $DBC_FILE
