#!/usr/bin/env python3
"""
RoboCap Shared Memory Writer
Reads data from RoboCap SDK and writes to shared memory for C++ OpenArm control
"""

import time
import numpy as np
from multiprocessing import shared_memory
import struct
import sys
import signal

# Add rebocap SDK to path
sys.path.insert(0, '.')
import rebocap_ws_sdk


class RoboCapShmWriter:
    """Writes RoboCap data to shared memory for C++ consumption"""
    
    # Shared memory layout offsets (must match C++ struct)
    OFFSET_SEQUENCE = 0
    OFFSET_DATA_READY = 8
    OFFSET_TIMESTAMP = 16
    OFFSET_ARM_JOINTS = 24
    OFFSET_GRIPPER = 80
    OFFSET_SHOULDER_QUAT = 88
    OFFSET_ELBOW_QUAT = 120
    OFFSET_WRIST_QUAT = 152
    OFFSET_HAND_QUAT = 184
    OFFSET_PELVIS = 216
    OFFSET_TOTAL_UPDATES = 240
    OFFSET_LAST_UPDATE = 248
    
    def __init__(self, shm_name='robocap_openarm_data', port=7690):
        self.shm_name = shm_name
        self.port = port
        self.sequence = 0
        self.total_updates = 0
        self.running = True
        
        # Create shared memory (512 bytes is enough)
        try:
            # Try to unlink existing shared memory first
            try:
                existing_shm = shared_memory.SharedMemory(name=shm_name)
                existing_shm.close()
                existing_shm.unlink()
                print(f"[INFO] Cleaned up existing shared memory: {shm_name}")
            except FileNotFoundError:
                pass
            
            self.shm = shared_memory.SharedMemory(
                name=shm_name,
                create=True,
                size=512
            )
            print(f"✅ Created shared memory: {shm_name} (size: 512 bytes)")
        except Exception as e:
            print(f"❌ Failed to create shared memory: {e}")
            raise
        
        # Initialize shared memory to zeros
        self.shm.buf[:] = bytes(512)
        
        # Initialize RoboCap SDK
        print(f"[INFO] Connecting to RoboCap on port {port}...")
        self.sdk = rebocap_ws_sdk.RebocapWsSdk(
            coordinate_type=rebocap_ws_sdk.CoordinateType.UnityCoordinate,
            use_global_rotation=True
        )
        self.sdk.set_pose_msg_callback(self.on_pose_data)
        self.sdk.set_exception_close_callback(self.on_exception_close)
        
        ret = self.sdk.open(port)
        if ret != 0:
            self.cleanup()
            raise RuntimeError(f"Failed to connect to RoboCap (error code: {ret})")
        
        print("✅ RoboCap connected successfully!")
        print("📡 Writing motion capture data to shared memory...")
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
            
            # Write to shared memory
            self.write_to_shm(ts, joint_angles, gripper_pos, 
                            r_shoulder, r_elbow, r_wrist, r_hand, tran)
            
            self.total_updates += 1
            
            # Print status every 60 frames (~1 second at 60Hz)
            if self.total_updates % 60 == 0:
                print(f"[{self.total_updates:6d}] Joints: {[f'{j:6.3f}' for j in joint_angles]} | "
                      f"Gripper: {gripper_pos:.3f} | Seq: {self.sequence}")
        
        except Exception as e:
            print(f"❌ Error in pose callback: {e}")
    
    def write_to_shm(self, timestamp, joint_angles, gripper_pos,
                     r_shoulder, r_elbow, r_wrist, r_hand, pelvis_pos):
        """Write data to shared memory"""
        buf = self.shm.buf
        
        # Sequence number
        self.sequence += 1
        struct.pack_into('Q', buf, self.OFFSET_SEQUENCE, self.sequence)
        
        # Data ready flag
        struct.pack_into('B', buf, self.OFFSET_DATA_READY, 1)
        
        # Timestamp
        struct.pack_into('d', buf, self.OFFSET_TIMESTAMP, timestamp)
        
        # Right arm joints (7 doubles)
        for i, angle in enumerate(joint_angles):
            struct.pack_into('d', buf, self.OFFSET_ARM_JOINTS + i * 8, angle)
        
        # Gripper position
        struct.pack_into('d', buf, self.OFFSET_GRIPPER, gripper_pos)
        
        # Raw quaternions (for debugging)
        offset = self.OFFSET_SHOULDER_QUAT
        for quat in [r_shoulder, r_elbow, r_wrist, r_hand]:
            for val in quat:
                struct.pack_into('d', buf, offset, val)
                offset += 8
        
        # Pelvis position
        for i, pos in enumerate(pelvis_pos):
            struct.pack_into('d', buf, self.OFFSET_PELVIS + i * 8, pos)
        
        # Statistics
        struct.pack_into('Q', buf, self.OFFSET_TOTAL_UPDATES, self.total_updates)
        struct.pack_into('d', buf, self.OFFSET_LAST_UPDATE, time.time())
    
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
        
        # Apply scaling and offset if needed
        # joints = [self.apply_joint_limits(j, i) for i, j in enumerate(joints)]
        
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
    
    def apply_joint_limits(self, angle, joint_idx):
        """Apply joint limits and scaling (optional)"""
        # Define joint limits for each joint (example values)
        limits = [
            (-np.pi, np.pi),      # Joint 0
            (-np.pi/2, np.pi/2),  # Joint 1
            (-np.pi, np.pi),      # Joint 2
            (0, np.pi),           # Joint 3 (elbow)
            (-np.pi, np.pi),      # Joint 4
            (-np.pi/2, np.pi/2),  # Joint 5
            (-np.pi, np.pi),      # Joint 6
        ]
        
        min_angle, max_angle = limits[joint_idx]
        return np.clip(angle, min_angle, max_angle)
    
    def run(self):
        """Main loop"""
        try:
            print("🚀 RoboCap teleoperation active!")
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
            self.shm.close()
            self.shm.unlink()
            print(f"✅ Shared memory '{self.shm_name}' cleaned up")
        except:
            pass
        
        print("👋 Goodbye!")


def main():
    print("=" * 60)
    print("  RoboCap → OpenArm Teleoperation (Shared Memory Writer)")
    print("=" * 60)
    print()
    
    try:
        writer = RoboCapShmWriter(
            shm_name='robocap_openarm_data',
            port=7690
        )
        writer.run()
    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
