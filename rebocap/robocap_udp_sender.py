#!/usr/bin/env python3
"""
RoboCap UDP Sender
Reads data from RoboCap SDK and sends joint angles via UDP
"""

import time
import numpy as np
import socket
import json
import sys
import signal

# Add rebocap SDK to path
sys.path.insert(0, '.')
import rebocap_ws_sdk


class RoboCapUdpSender:
    """Sends RoboCap joint angles via UDP"""
    
    def __init__(self, udp_host='127.0.0.1', udp_port=5678, rebocap_port=7690):
        self.udp_host = udp_host
        self.udp_port = udp_port
        self.rebocap_port = rebocap_port
        self.total_updates = 0
        self.running = True
        
        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"✅ UDP socket created, sending to {udp_host}:{udp_port}")
        
        # Initialize RoboCap SDK
        print(f"[INFO] Connecting to RoboCap on port {rebocap_port}...")
        self.sdk = rebocap_ws_sdk.RebocapWsSdk(
            coordinate_type=rebocap_ws_sdk.CoordinateType.UnityCoordinate,
            use_global_rotation=True
        )
        self.sdk.set_pose_msg_callback(self.on_pose_data)
        self.sdk.set_exception_close_callback(self.on_exception_close)
        
        ret = self.sdk.open(rebocap_port)
        if ret != 0:
            self.cleanup()
            raise RuntimeError(f"Failed to connect to RoboCap (error code: {ret})")
        
        print("✅ RoboCap connected successfully!")
        print(f"📡 Sending joint angles via UDP to {udp_host}:{udp_port}...")
        print("   Press Ctrl+C to stop\n")
        
        # Setup signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def signal_handler(self, sig, frame):
        print("\n🛑 Received interrupt signal, shutting down...")
        self.running = False
    
    def on_exception_close(self):
        print("⚠️  RoboCap connection closed unexpectedly")
        self.running = False
    
    def on_pose_data(self, sdk, tran, pose24, static_index, ts):
        """Callback when new pose data is received from RoboCap"""
        try:
            # Extract right arm joint quaternions (indices from SMPL skeleton)
            r_shoulder = pose24[17]  # R_Shoulder [w, x, y, z]
            r_elbow = pose24[19]     # R_Elbow
            r_wrist = pose24[21]     # R_Wrist
            r_hand = pose24[23]      # R_Hand
            
            # Convert quaternions to OpenArm joint angles
            joint_angles = self.map_to_openarm_joints(
                r_shoulder, r_elbow, r_wrist, r_hand
            )
            
            # Estimate gripper position from hand orientation
            gripper_pos = self.estimate_gripper_position(r_hand)
            
            # Send via UDP
            self.send_udp(ts, joint_angles, gripper_pos, tran)
            
            self.total_updates += 1
            
            # Print status every 60 frames (~1 second at 60Hz)
            if self.total_updates % 60 == 0:
                print(f"[{self.total_updates:6d}] Joints: {[f'{j:6.3f}' for j in joint_angles]} | "
                      f"Gripper: {gripper_pos:.3f}")
        
        except Exception as e:
            print(f"❌ Error in pose callback: {e}")
    
    def send_udp(self, timestamp, joint_angles, gripper_pos, pelvis_pos):
        """Send data via UDP as JSON"""
        data = {
            'timestamp': timestamp,
            'joints': joint_angles,
            'gripper': gripper_pos,
            'pelvis': pelvis_pos
        }
        
        # Convert to JSON and send
        json_data = json.dumps(data)
        self.sock.sendto(json_data.encode('utf-8'), (self.udp_host, self.udp_port))
    
    def map_to_openarm_joints(self, shoulder, elbow, wrist, hand):
        """
        Map RoboCap quaternions to OpenArm 7-DOF joint angles
        
        OpenArm joint order (typical 7-DOF arm):
        0: Shoulder yaw (rotation around vertical axis)
        1: Shoulder pitch (up/down)
        2: Shoulder roll (arm rotation)
        3: Elbow pitch (bend)
        4: Wrist yaw (rotation)
        5: Wrist pitch (up/down)
        6: Wrist roll (hand rotation)
        """
        # Convert quaternions to Euler angles
        shoulder_euler = self.quat_to_euler(shoulder)
        elbow_euler = self.quat_to_euler(elbow)
        wrist_euler = self.quat_to_euler(wrist)
        
        # Map to OpenArm joints
        # NOTE: This mapping needs to be calibrated based on your specific setup!
        # You may need to adjust signs, offsets, and scaling factors
        joints = [
            shoulder_euler[2],      # Joint 0: Shoulder yaw
            shoulder_euler[1],      # Joint 1: Shoulder pitch
            shoulder_euler[0],      # Joint 2: Shoulder roll
            elbow_euler[1],         # Joint 3: Elbow pitch
            wrist_euler[2],         # Joint 4: Wrist yaw
            wrist_euler[1],         # Joint 5: Wrist pitch
            wrist_euler[0],         # Joint 6: Wrist roll
        ]
        
        return joints
    
    def quat_to_euler(self, quat):
        """
        Convert quaternion [w, x, y, z] to Euler angles [roll, pitch, yaw]
        Using ZYX convention (yaw-pitch-roll)
        """
        w, x, y, z = quat
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        sinp = np.clip(sinp, -1.0, 1.0)
        pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return [roll, pitch, yaw]
    
    def estimate_gripper_position(self, hand_quat):
        """
        Estimate gripper open/close from hand orientation
        This is a simple heuristic - you may want to use a different method
        """
        # Simple example: use hand roll angle
        roll, pitch, yaw = self.quat_to_euler(hand_quat)
        
        # Map roll angle to gripper position (0 = closed, 1 = open)
        # Adjust these values based on your hand gestures
        gripper = np.clip((roll + np.pi/4) / (np.pi/2), 0.0, 1.0)
        
        return gripper
    
    def run(self):
        """Main loop"""
        try:
            print("🚀 RoboCap UDP teleoperation active!")
            while self.running:
                time.sleep(0.1)  # Data is updated in callback
        except KeyboardInterrupt:
            print("\n🛑 Keyboard interrupt received")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("\n🧹 Cleaning up...")
        try:
            self.sdk.close()
            print("✅ RoboCap SDK closed")
        except:
            pass
        
        try:
            self.sock.close()
            print("✅ UDP socket closed")
        except:
            pass
        
        print("👋 Goodbye!")


def main():
    print("=" * 60)
    print("  RoboCap → OpenArm Teleoperation (UDP Sender)")
    print("=" * 60)
    print()
    
    # Parse command line arguments
    udp_host = '127.0.0.1'
    udp_port = 5678
    rebocap_port = 7690
    
    if len(sys.argv) >= 2:
        udp_host = sys.argv[1]
    if len(sys.argv) >= 3:
        udp_port = int(sys.argv[2])
    if len(sys.argv) >= 4:
        rebocap_port = int(sys.argv[3])
    
    try:
        sender = RoboCapUdpSender(
            udp_host=udp_host,
            udp_port=udp_port,
            rebocap_port=rebocap_port
        )
        sender.run()
    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
