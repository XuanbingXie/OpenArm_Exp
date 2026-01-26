#!/usr/bin/env python3
"""
RoboCap UDP Sender
Reads data from RoboCap SDK and sends joint angles via UDP
"""

from pyexpat import model
import time
import numpy as np
import socket
import json
import sys
import signal
from scipy.spatial.transform import Rotation as R
from contants import JOINTS_MIN_VALUE, JOINTS_MAX_VALUE
from shoulder_solver import IncrementalShoulderSolver
import rebocap_ws_sdk

class RoboCapUdpSender:
    """Sends RoboCap joint angles via UDP"""
    
    def __init__(self, udp_host='255.255.255.255', udp_port=5678, rebocap_port=7690):
        self.udp_host = udp_host
        self.udp_port = udp_port
        self.rebocap_port = rebocap_port
        self.total_updates = 0
        self.running = True
        self.debug = True
        self.print_interval = 60

        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # Initialize RoboCap SDK(Blender coordinate system, local rotation)
        self.sdk = rebocap_ws_sdk.RebocapWsSdk(
            coordinate_type=rebocap_ws_sdk.CoordinateType.BlenderCoordinate,
            use_global_rotation=False
        )
        self.sdk.set_pose_msg_callback(self.on_pose_data)
        self.sdk.set_exception_close_callback(self.on_exception_close)
        ret = self.sdk.open(rebocap_port)
        if ret != 0:
            self.cleanup()
            raise RuntimeError(f"Failed to connect to RoboCap (error code: {ret})")

        # Solver for shoulder joints
        self.shoulder_solver = IncrementalShoulderSolver()

        # Setup signal handler
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        self.running = False
    
    def on_exception_close(self):
        self.running = False
    
    def on_pose_data(self, sdk, tran, pose24, static_index, ts):
        """Callback when new pose data is received from RoboCap"""
        try:
            r_shoulder = pose24[-8]  # R_Shoulder [x, y, z, w]
            r_elbow = pose24[-6]     # R_Elbow
            r_wrist = pose24[-4]     # R_Wrist
            r_hand = pose24[-2]      # R_Hand
            
            # Convert quaternions to OpenArm joint angles
            # ----------------Developing-----------------
            joint_angles = self.map_to_openarm_joints(
                r_shoulder, r_elbow, r_wrist, r_hand
            )
            
            # Estimate gripper position from hand orientation
            # ----------------Developing-----------------
            gripper_pos = self.estimate_gripper_position(r_hand)
            
            # Send via UDP
            self.send_udp(ts, joint_angles, gripper_pos, tran)
                
            self.total_updates += 1
            
            # Print status every 60 frames (~1 second at 60Hz)
            if self.debug and self.total_updates % self.print_interval == 0:
                print(f"Joints: {[f'{j:6.3f}' for j in joint_angles]} | "
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
    
    #--------------------Developing-----------------
    def map_to_openarm_joints(self, shoulder, elbow, wrist, hand):
        """
        Map RoboCap quaternions to OpenArm 7-DOF joint angles        
        """
        if not self.shoulder_solver.init_flag:
            ## Euler angle decomposition
            r_shoulder = R.from_quat(shoulder, scalar_first=False)
            j3, j2, j1 = r_shoulder.as_euler('xyx')
            j2 -= np.pi / 2.0  # Adjust for OpenArm's zero position

            # ## Twist Swing decomposition(Not good)     
            # # Get twist quaternion with the X axis as the twist axis
            # q_twist = np.array([shoulder[0], 0, 0, shoulder[3]])
            
            # # Normalize q_twist
            # norm = np.linalg.norm(q_twist)
            # if norm < 1e-6:
            #     q_twist = np.array([0, 0, 0, 1.0])
            # else:
            #     q_twist /= norm
            # # Get swing quaternion       
            # r_total = R.from_quat(shoulder, scalar_first=False)
            # r_twist = R.from_quat(q_twist)
            # r_swing = r_total * r_twist.inv()
            
            # # Twist angle(j3)
            # twist_angle = 2 * np.arctan2(q_twist[0], q_twist[3])
            
            # # Swing angle (j1, j2)        
            # # Do not use euler angle
            # v = r_swing.apply([1, 0, 0])  # [vx, vy, vz]
            # r_frame_change = R.from_euler('x', 90, degrees=True)
            # v = r_frame_change.apply(v)
            # j1 = np.arctan2(v[2], v[1]) 
            # j2 = -np.arctan2(v[0], np.sqrt(v[1]**2 + v[2]**2))
            # j3 = twist_angle
            
            self.shoulder_solver.init_joints([j1, j2, j3])
        else:
            j1, j2, j3 = self.shoulder_solver.solve(shoulder)

        ## Just use Euler angles for elbow and wrist
        r_elbow = R.from_quat(elbow, scalar_first=False)
        z, _, x = r_elbow.as_euler('zyx')
        j4 = -z
        j5 = (x + np.pi/2)
        r_wrist = R.from_quat(wrist, scalar_first=False)
        z, y, _ = r_wrist.as_euler('zyx')
        j6 = z
        j7 = -y

        # ## Restrict j1(may not useful)
        # if j1 > np.pi * 5.0 / 6.0 and j1 < np.pi:
        #     j1 -= np.pi * 2 

        joints = [j1, j2, j3, j4, j5, j6, j7]
        joints = np.clip(joints, JOINTS_MIN_VALUE, JOINTS_MAX_VALUE).tolist()

        if self.debug and self.total_updates % self.print_interval == 0:
            print(f"Debug Joints(Not clip): {j1:.3f}, {j2:.3f}, {j3:.3f}, {j4:.3f}, {j5:.3f}, {j6:.3f}, {j7:.3f}")
            print(f"Debug Joints: {joints[0]:.3f}, {joints[1]:.3f}, {joints[2]:.3f}, {joints[3]:.3f}, {joints[4]:.3f}, {joints[5]:.3f}, {joints[6]:.3f}")

        return joints
    
    def estimate_gripper_position(self, hand_quat):
        gripper = 0

        return gripper
    
    def run(self):
        try:
            while self.running:
                time.sleep(0.1)  
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()
    def cleanup(self):
        try:
            self.sdk.close()
            print("RoboCap SDK closed")
        except:
            pass
        
        try:
            self.sock.close()
            print("UDP socket closed")
        except:
            pass


def main():
    print("=" * 60)
    print("  RoboCap → OpenArm Teleoperation (UDP Sender)")
    print("=" * 60)
    print()
    
    # Parse command line arguments
    udp_host = '255.255.255.255' ## Default to broadcast
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
            rebocap_port=rebocap_port,
        )
        sender.run()
    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
