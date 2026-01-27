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
from contants import LEFT_JOINTS_MIN_VALUE, LEFT_JOINTS_MAX_VALUE, RIGHT_JOINTS_MIN_VALUE, RIGHT_JOINTS_MAX_VALUE
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

        # Solver for shoulder joints
        # self.shoulder_solver = IncrementalShoulderSolver()
        self.l_shoulder_initialized = False
        self.l_shoulder_solver = IncrementalShoulderSolver('left', LEFT_JOINTS_MIN_VALUE[:3], LEFT_JOINTS_MAX_VALUE[:3])
        self.r_shoulder_initialized = False
        self.r_shoulder_solver = IncrementalShoulderSolver('right', RIGHT_JOINTS_MIN_VALUE[:3], RIGHT_JOINTS_MAX_VALUE[:3])

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


        # Setup signal handler
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        self.running = False
    
    def on_exception_close(self):
        self.running = False
    
    def on_pose_data(self, sdk, tran, pose24, static_index, ts):
        """Callback when new pose data is received from RoboCap"""
        try:
            # Convert quaternions to OpenArm joint angles
            joint_angles = self.map_to_openarm_joints(pose24)
            
            # Send via UDP
            self.send_udp(ts, joint_angles)
               
            # Print status every 60 frames (~1 second at 60Hz)
            self.total_updates += 1
            # if self.debug and self.total_updates % self.print_interval == 0:
            #     print(f"Joints: {[f'{j:6.3f}' for j in joint_angles]}")
        
        except Exception as e:
            print(f"❌ Error in pose callback: {e}")
    
    def send_udp(self, timestamp, joint_angles):
        """Send data via UDP as JSON"""
        data = {
            'timestamp': timestamp,
            'joints': joint_angles,
        }
        
        # Convert to JSON and send
        json_data = json.dumps(data)
        self.sock.sendto(json_data.encode('utf-8'), (self.udp_host, self.udp_port))
    
    #--------------------Developing-----------------
    def map_to_openarm_joints(self, pose24):
        """
        Map RoboCap quaternions to OpenArm 7-DOF joint angles        
        """
        ## left        
        left_shoulder = pose24[-8]
        left_elbow = pose24[-6]
        left_wrist = pose24[-4]

        ## right
        right_shoulder = pose24[-7]
        right_elbow = pose24[-5]
        right_wrist = pose24[-3]

        ## ----------------For left arm--------------------
        if (not self.l_shoulder_initialized):
            # Euer angle decomposition
            l_shoulder = R.from_quat(left_shoulder, scalar_first=False)
            l_j1, l_j2, l_j3 = l_shoulder.as_euler('XYX')
            l_j2 -= np.pi / 2.0  # Adjust for OpenArm's
            self.l_shoulder_solver.init_joints([l_j1, l_j2, l_j3])
            self.l_shoulder_initialized = True
        else:
            l_j1, l_j2, l_j3 = self.l_shoulder_solver.solve(left_shoulder)
        # Just use Euler angles for elbow and wrist
        l_elbow = R.from_quat(left_elbow, scalar_first=False)
        z, _, x = l_elbow.as_euler('ZYX')
        l_j4 = -z
        l_j5 = (x - np.pi/2)
        l_wrist = R.from_quat(left_wrist, scalar_first=False)
        z, y, _ = l_wrist.as_euler('ZYX')
        l_j6 = z
        l_j7 = y


        ## ----------------For right arm--------------------
        if (not self.r_shoulder_initialized):
            # Euler angle decomposition
            r_shoulder = R.from_quat(right_shoulder, scalar_first=False)
            r_j1, r_j2, r_j3 = r_shoulder.as_euler('XYX')
            if r_j1 > 0:
                r_j1 -= np.pi
            else:
                r_j1 += np.pi
            r_j1 = -r_j1
            r_j2 -= np.pi / 2.0
            r_j2 = -r_j2
            if r_j3 > 0:
                r_j3 -= np.pi
            else:
                r_j3 += np.pi
            r_j3 = -r_j3
            self.r_shoulder_solver.init_joints([r_j1, r_j2, r_j3])
            self.r_shoulder_initialized = True
        else:
            r_j1, r_j2, r_j3 = self.r_shoulder_solver.solve(right_shoulder)
                
        ## Elbow and wrist (for right arm)
        r_elbow = R.from_quat(right_elbow, scalar_first=False)
        z, _, x = r_elbow.as_euler('ZYX')
        r_j4 = z
        r_j5 = -x + (np.pi/2)
        r_wrist = R.from_quat(right_wrist, scalar_first=False)
        z, y, _ = r_wrist.as_euler('ZYX')
        r_j6 = z
        r_j7 = y

        joints = [l_j1, l_j2, l_j3, l_j4, l_j5, l_j6, l_j7,
                  r_j1, r_j2, r_j3, r_j4, r_j5, r_j6, r_j7]
        joints = np.clip(joints, LEFT_JOINTS_MIN_VALUE+RIGHT_JOINTS_MIN_VALUE, LEFT_JOINTS_MAX_VALUE+RIGHT_JOINTS_MAX_VALUE).tolist()

        if self.debug and self.total_updates % self.print_interval == 0:
            print(f"Debug Left Joints(Not clip): {l_j1:.3f}, {l_j2:.3f}, {l_j3:.3f}, {l_j4:.3f}, {l_j5:.3f}, {l_j6:.3f}, {l_j7:.3f}")
            print(f"Debug Left Joints(Clipped): {joints[0]:.3f}, {joints[1]:.3f}, {joints[2]:.3f}, {joints[3]:.3f}, {joints[4]:.3f}, {joints[5]:.3f}, {joints[6]:.3f}")
            print(f"Debug Right Joints(Not clip): {r_j1:.3f}, {r_j2:.3f}, {r_j3:.3f}, {r_j4:.3f}, {r_j5:.3f}, {r_j6:.3f}, {r_j7:.3f}")
            print(f"Debug Right Joints(Clipped): {joints[7]:.3f}, {joints[8]:.3f}, {joints[9]:.3f}, {joints[10]:.3f}, {joints[11]:.3f}, {joints[12]:.3f}, {joints[13]:.3f}")

        return joints

    
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
