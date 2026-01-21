#!/bin/bash
# Launch script for RoboCap teleoperation

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=========================================="
echo "  RoboCap → OpenArm Teleoperation"
echo "=========================================="
echo ""

# Check if URDF path is provided
if [ $# -lt 1 ]; then
    echo -e "${RED}Error: URDF path not provided${NC}"
    echo "Usage: $0 <urdf_path> [arm_side] [can_interface]"
    echo "Example: $0 /path/to/openarm.urdf right_arm can0"
    exit 1
fi

URDF_PATH=$1
ARM_SIDE=${2:-right_arm}
CAN_INTERFACE=${3:-can0}

# Check if URDF file exists
if [ ! -f "$URDF_PATH" ]; then
    echo -e "${RED}Error: URDF file not found: $URDF_PATH${NC}"
    exit 1
fi

# Check if Python RoboCap writer is running
if ! pgrep -f "robocap_shm_writer.py" > /dev/null; then
    echo -e "${YELLOW}Warning: Python RoboCap writer is not running${NC}"
    echo "Please start it first with:"
    echo "  cd rebocap && python3 robocap_shm_writer.py"
    echo ""
    read -p "Do you want to continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo -e "${GREEN}Configuration:${NC}"
echo "  URDF path     : $URDF_PATH"
echo "  Arm side      : $ARM_SIDE"
echo "  CAN interface : $CAN_INTERFACE"
echo ""

# Run the control program
echo -e "${GREEN}Starting RoboCap teleoperation...${NC}"
./build/robocap_control "$URDF_PATH" "$ARM_SIDE" "$CAN_INTERFACE"

echo ""
echo -e "${GREEN}Teleoperation stopped${NC}"
