#!/bin/bash
# Start RoboCap UDP Sender
# Usage: ./start_udp_sender.sh [udp_host] [udp_port] [rebocap_port]
# Example: ./start_udp_sender.sh 127.0.0.1 5678 7690

cd "$(dirname "$0")"

UDP_HOST=${1:-127.0.0.1}
UDP_PORT=${2:-5678}
REBOCAP_PORT=${3:-7690}

echo "Starting RoboCap UDP Sender..."
echo "UDP Target: $UDP_HOST:$UDP_PORT"
echo "RoboCap Port: $REBOCAP_PORT"
echo ""

python3 robocap_udp_sender.py "$UDP_HOST" "$UDP_PORT" "$REBOCAP_PORT"
