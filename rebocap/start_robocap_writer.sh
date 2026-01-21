#!/bin/bash
# Start RoboCap shared memory writer

echo "=========================================="
echo "  RoboCap Shared Memory Writer"
echo "=========================================="
echo ""

# Check if rebocap_ws_sdk is available
if [ ! -d "rebocap_ws_sdk" ]; then
    echo "Error: rebocap_ws_sdk directory not found"
    echo "Please make sure you're in the rebocap directory"
    exit 1
fi

# Check Python version
PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}' | cut -d. -f1,2)
echo "Python version: $PYTHON_VERSION"

# Run the writer
echo "Starting RoboCap writer..."
echo "Make sure RoboCap software is running on port 7690"
echo ""

python3 robocap_shm_writer.py

echo ""
echo "RoboCap writer stopped"
