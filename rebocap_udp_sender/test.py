#!/usr/bin/env python3
"""
Test script for GripperController class
Continuously prints torque values to monitor GUI functionality
"""

import time
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from gripper_controller import GripperController

def main():
    print("=" * 50)
    print("GripperController Test Script")
    print("=" * 50)
    print("This script will:")
    print("- Start the GUI with horizontal progress bars")
    print("- Show real-time torque values for both grippers")
    print("- Continuously print current torque values")
    print("- Allow testing of keyboard controls")
    print()
    print("Keyboard Controls:")
    print("↑↓: Select Left/Right gripper (highlighted in blue)")
    print("←→: Adjust selected gripper torque (±0.1)")
    print("Space: Open selected gripper (temporary -1.0)")
    print()
    print("Instructions:")
    print("- Use keyboard controls to adjust torque values")
    print("- Watch GUI progress bars and terminal output")
    print("- Selected gripper is highlighted in blue")
    print("- Press Ctrl+C to exit")
    print("=" * 50)
    print()

    # Create gripper controller with default values
    controller = GripperController(
        left_torque=0.0,
        right_torque=0.0,
        min_torque=0.0,
        max_torque=10.0
    )

    # Start the GUI
    print("Starting GUI...")
    controller.start_gui()

    # Give GUI time to initialize
    time.sleep(1)

    try:
        print("Monitoring torque values (updates every 0.1 seconds)...")
        print("Format: Left Torque | Right Torque")
        print("-" * 40)

        while True:
            left_torque, right_torque = controller.get_torques()
            print(f"\rLeft: {left_torque:6.2f} | Right: {right_torque:6.2f}", end='', flush=True)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nStopping test...")
    finally:
        print("Closing GUI...")
        controller.stop_gui()
        print("Test completed.")

if __name__ == "__main__":
    main()