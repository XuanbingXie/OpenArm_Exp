#!/usr/bin/env python3
"""
Test UDP Sender - Sends dummy joint angles for testing
No RoboCap required
"""

import socket
import json
import time
import math
import sys


def main():
    # Parse arguments
    udp_host = sys.argv[1] if len(sys.argv) > 1 else '127.0.0.1'
    udp_port = int(sys.argv[2]) if len(sys.argv) > 2 else 5678
    
    print("=" * 60)
    print("  Test UDP Sender (No RoboCap Required)")
    print("=" * 60)
    print(f"Sending to: {udp_host}:{udp_port}")
    print("Press Ctrl+C to stop\n")
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    frame = 0
    start_time = time.time()
    
    try:
        while True:
            # Generate sinusoidal joint angles for testing
            t = time.time() - start_time
            joints = [
                # 0.3 * math.sin(t * 0.5),      # Joint 0: slow oscillation
                # 0.2 * math.sin(t * 0.7),      # Joint 1
                # 0.1 * math.sin(t * 0.9),      # Joint 2
                # 0.4 * math.sin(t * 1.1),      # Joint 3: elbow
                # 0.2 * math.sin(t * 1.3),      # Joint 4
                # 0.1 * math.sin(t * 1.5),      # Joint 5
                # 0.3 * math.sin(t * 1.7),      # Joint 6
                0,0,0,0,0,0,0
            ]
            
            # Oscillating gripper
            gripper = 0.5 + 0.5 * math.sin(t * 0.3)
            
            # Dummy pelvis position
            pelvis = [0.0, 1.0, 0.0]
            
            # Create JSON packet
            data = {
                'timestamp': time.time(),
                'joints': joints,
                'gripper': gripper,
                'pelvis': pelvis
            }
            
            # Send via UDP
            json_data = json.dumps(data)
            bytes_sent = sock.sendto(json_data.encode('utf-8'), (udp_host, udp_port))
            
            frame += 1
            
            # Print status every 60 frames
            if frame % 60 == 0:
                print(f"[{frame:6d}] Sent {bytes_sent} bytes | Gripper: {gripper:.3f}")
            
            # Send at ~60 Hz
            time.sleep(1.0 / 60.0)
    
    except KeyboardInterrupt:
        print("\n🛑 Stopped")
    finally:
        sock.close()
        print("✅ Socket closed")


if __name__ == "__main__":
    main()
