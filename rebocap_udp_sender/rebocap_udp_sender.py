#!/usr/bin/env python3
"""
RoboCap UDP Sender
Reads data from RoboCap SDK and sends joint angles via UDP
"""

import time
import threading
import os
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"
import psutil
import ctypes
import sys
import signal
import numpy as np
import socket
from scipy.spatial.transform import Rotation as R

from shoulder_solver import IncrementalShoulderSolver
from gripper_controller import GripperController
from writer import Writer
from contants import LEFT_JOINTS_MIN_VALUE, LEFT_JOINTS_MAX_VALUE, RIGHT_JOINTS_MIN_VALUE, RIGHT_JOINTS_MAX_VALUE
import rebocap_ws_sdk

class ReboCapUdpSender:
    """Sends RoboCap joint angles via UDP"""
    
    def __init__(self, udp_host='255.255.255.255', udp_port=5678, rebocap_port=7690):
        self.udp_host = udp_host
        self.udp_port = udp_port
        self.rebocap_port = rebocap_port
        self.total_updates = 0
        self.running = True
        self.debug = False
        self.print_interval = 60
        
        # Frequency tracking
        self.on_pose_data_count = 0
        self.send_udp_count = 0
        self.min_send_udp_interval = 1.0
        self.max_send_udp_interval = 0.0
        self.frequency_udp_start_time = time.time()
        self.frequency_send_udp_start_time = time.time()

        self.frequency_pose_start_time = time.time()
        self.last_pose_data_time = -1
        self.min_pose_data_interval = 1.0
        self.max_pose_data_interval = 0.0
        self.frequency_print_interval = 1  # seconds

        # Send frequency
        self.send_freq = 200  
        self.interval = 1.0 / self.send_freq
        # Set process priority 
        p = psutil.Process(os.getpid())
        p.nice(psutil.HIGH_PRIORITY_CLASS) # for wins
        # Set timer resolution to 1ms
        self.winmm = ctypes.WinDLL('winmm')
        self.winmm.timeBeginPeriod(1)

        # Pose storage for interpolation(debug)
        self.last_pose = None
        self.last_timestamp = None
        self.cur_pose = None
        self.cur_timestamp = None
        self.pose_lock = threading.Lock()
        self.key_times = [0, 1.2]

        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # Solver for shoulder joints
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

        # Initialize gripper controller
        self.gripper_controller = GripperController()
        # self.gripper_controller.start_gui()

        # Initialize writer for debug
        self.writer = Writer(debug=self.debug)

        # Setup signal handler
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        self.running = False
    
    def on_exception_close(self):
        self.running = False
    
    def on_pose_data(self, sdk, tran, pose24, static_index, ts):
        """Callback when new pose data is received from RoboCap"""
        with self.pose_lock:
            self.last_pose = self.cur_pose
            self.last_timestamp = self.cur_timestamp
            self.cur_pose = pose24  
            self.cur_timestamp = time.perf_counter()

        # # Frequency tracking for on_pose_data
        # self.on_pose_data_count += 1
        # current_time = time.time()
        # if self.last_pose_data_time > 0:
        #     interval = current_time - self.last_pose_data_time
        #     self.min_pose_data_interval = min(self.min_pose_data_interval, interval)
        #     self.max_pose_data_interval = max(self.max_pose_data_interval, interval)

        # if current_time - self.frequency_pose_start_time >= self.frequency_print_interval:
        #     on_pose_freq = self.on_pose_data_count / (current_time - self.frequency_pose_start_time)
        #     print(f"on_pose_data frequency: {on_pose_freq:.2f} Hz, min_interval: {self.min_pose_data_interval:.4f}s, max_interval: {self.max_pose_data_interval:.4f}s")
        #     self.on_pose_data_count = 0
        #     self.frequency_pose_start_time = current_time
        #     self.min_pose_data_interval = 1.0
        #     self.max_pose_data_interval = 0.0
        # self.last_pose_data_time = current_time
        # if self.debug:
        #     pre_joints = self.map_to_openarm_joints_no_incre_solver(pose24)
        #     self.writer.record(now, pre_joints, pre_joints)
    
    def send_udp(self, timestamp, joint_angles):
        # Pack and send raw float32 data (timestamp, 14 joints, left_grip, right_grip)
        left_torque, right_torque = self.gripper_controller.get_torques()

        # Ensure joint_angles length is 14
        ja = np.asarray(joint_angles, dtype=np.float32)
        if ja.size < 14:
            # pad with zeros if unexpectedly short
            padded = np.zeros(14, dtype=np.float32)
            padded[:ja.size] = ja
            ja = padded
        elif ja.size > 14:
            ja = ja[:14]

        pkt = np.empty(1 + 14 + 2, dtype=np.float32)
        pkt[0] = np.float32(timestamp)
        pkt[1:15] = ja
        pkt[15] = np.float32(left_torque)
        pkt[16] = np.float32(right_torque)

        try:
            self.sock.sendto(pkt.tobytes(), (self.udp_host, self.udp_port))
        except Exception as e:
            if self.debug:
                print(f"UDP send failed: {e}")
        
        # Frequency tracking for send_udp
        self.send_udp_count += 1
        current_time = time.time()
        if current_time - self.frequency_udp_start_time >= self.frequency_print_interval:
            send_freq = self.send_udp_count / (current_time - self.frequency_udp_start_time)
            print(f"send_udp frequency: {send_freq:.2f} Hz")
            self.send_udp_count = 0
            self.frequency_udp_start_time = current_time

    def send_loop(self):
        """Send loop running at high frequency with interpolation"""
        next_time = time.perf_counter() + self.interval
        while self.running:
            if (time.perf_counter() > next_time + self.interval):
                print(f"Warning: send loop is lagging behind, lagging time:{time.perf_counter() - next_time:.4f} seconds")
                next_time = time.perf_counter() + self.interval

            remain = next_time - time.perf_counter()
            if remain > 0.01:
                time.sleep(remain - 0.01)

            while time.perf_counter() < next_time:
                pass
            
            with self.pose_lock:
                cur_pose = self.cur_pose
                cur_ts = self.cur_timestamp
                last_pose = self.last_pose
                last_ts = self.last_timestamp

            if cur_pose is not None and last_pose is not None and cur_ts is not None and last_ts is not None:
                duration = cur_ts - last_ts
                if duration > 0:

                    start_time = time.time()
                    now = time.perf_counter()
                    alpha = (now - duration - last_ts) / duration
                    alpha = np.clip(alpha, self.key_times[0], self.key_times[1])
                    interp_pose = self.interpolate_pose(last_pose, cur_pose, alpha)
                    joint_angles = self.map_to_openarm_joints_interp(interp_pose)
                    self.send_udp(now, joint_angles)
                    
                    cur_time = time.time()
                    cur_interval = cur_time - start_time
                    self.min_send_udp_interval = min(self.min_send_udp_interval, cur_interval)
                    self.max_send_udp_interval = max(self.max_send_udp_interval, cur_interval)
                    if (cur_time - self.frequency_send_udp_start_time) >= self.frequency_print_interval:
                        print(f"Sender: min_interval: {self.min_send_udp_interval:.4f}s, max_interval: {self.max_send_udp_interval:.4f}s")
                        self.min_send_udp_interval = 1.0
                        self.max_send_udp_interval = 0.0
                        self.frequency_send_udp_start_time = cur_time


                    # Record pre and post interpolation joints for debug
                    if self.debug:
                        pre_joints = self.map_to_openarm_joints_no_incre_solver(cur_pose)
                        self.writer.record(now, pre_joints, joint_angles)
                else:
                    print(f"Warning: Non-positive duration between poses, duration is {duration}")
            next_time += self.interval

    def interpolate_pose(self, pose1, pose2, alpha):
        """Interpolate between two poses using SLERP for quaternions"""
        # interp_pose = []
        # Indices of interest: -11 collar, -8 shoulder, -6 elbow, -4 wrist for left; -10, -7, -5, -3 for right
        indices = [-11, -8, -6, -4, -10, -7, -5, -3]
        ## Slerp(Slow)
        # for i in range(-24, 0, 1):
        #     if i in indices:
        #         key_rots = R.from_quat([pose1[i], pose2[i]], scalar_first=False)
        #         slerp = Slerp(self.key_times, key_rots)
        #         interp_r = slerp(alpha)
        #         interp_q = interp_r.as_quat(scalar_first=False)
        #         interp_pose.append(interp_q.tolist())
        #     else:
        #         # For other joints, linear interpolation or copy
        #         interp_pose.append(pose2[i][:])  # Copy pose2 for now
        
        ## NLERP
        q1 = np.array([pose1[i] for i in indices])
        q2 = np.array([pose2[i] for i in indices])
        dot = np.sum(q1 * q2, axis=1, keepdims=True)
        q2 = np.where(dot < 0, -q2, q2)
        interp_q = (1.0 - alpha) * q1 + alpha * q2
        norms = np.linalg.norm(interp_q, axis=1, keepdims=True)
        interp_q /= norms
        new_pose = list(pose2) 
        for idx, i in enumerate(indices):
            new_pose[i] = interp_q[idx].tolist()
        
        return new_pose

    def map_to_openarm_joints_interp(self, pose24):
        """
        Map interpolated RoboCap quaternions to OpenArm 7-DOF joint angles
        Same as map_to_openarm_joints but for interpolated pose
        """
        ## left
        left_collar = pose24[-11]
        left_shoulder = pose24[-8]
        left_elbow = pose24[-6]
        left_wrist = pose24[-4]

        ## right
        right_collar = pose24[-10]
        right_shoulder = pose24[-7]
        right_elbow = pose24[-5]
        right_wrist = pose24[-3]

        ## Fuse collar rotation and shoulder rotation
        left_shoulder = (R.from_quat(left_collar) * R.from_quat(left_shoulder)).as_quat()
        right_shoulder = (R.from_quat(right_collar) * R.from_quat(right_shoulder)).as_quat()


        ## ----------------For left arm--------------------
        if (not self.l_shoulder_initialized):
            # Euler angle decomposition
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
        l_j5 = x
        l_wrist = R.from_quat(left_wrist, scalar_first=False)
        z, y, _ = l_wrist.as_euler('ZYX')
        l_j6 = -y
        l_j7 = z


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
        r_j5 = -x - (np.pi/4)
        r_wrist = R.from_quat(right_wrist, scalar_first=False)
        z, y, _ = r_wrist.as_euler('ZYX')
        r_j6 = -z
        r_j7 = -y

        joints = [l_j1, l_j2, l_j3, l_j4, l_j5, l_j6, l_j7,
                  r_j1, r_j2, r_j3, r_j4, r_j5, r_j6, r_j7]
        joints = np.clip(joints, LEFT_JOINTS_MIN_VALUE+RIGHT_JOINTS_MIN_VALUE, LEFT_JOINTS_MAX_VALUE+RIGHT_JOINTS_MAX_VALUE).tolist()

        # self.total_updates += 1
        # if self.debug and self.total_updates % self.print_interval == 0:
        #     print(f"Debug Left Joints(Not clip): {l_j1:.3f}, {l_j2:.3f}, {l_j3:.3f}, {l_j4:.3f}, {l_j5:.3f}, {l_j6:.3f}, {l_j7:.3f}")
        #     print(f"Debug Left Joints(Clipped): {joints[0]:.3f}, {joints[1]:.3f}, {joints[2]:.3f}, {joints[3]:.3f}, {joints[4]:.3f}, {joints[5]:.3f}, {joints[6]:.3f}")
        #     print(f"Debug Right Joints(Not clip): {r_j1:.3f}, {r_j2:.3f}, {r_j3:.3f}, {r_j4:.3f}, {r_j5:.3f}, {r_j6:.3f}, {r_j7:.3f}")
        #     print(f"Debug Right Joints(Clipped): {joints[7]:.3f}, {joints[8]:.3f}, {joints[9]:.3f}, {joints[10]:.3f}, {joints[11]:.3f}, {joints[12]:.3f}, {joints[13]:.3f}")
        
        return joints

    def map_to_openarm_joints_no_incre_solver(self, pose24):
        """
        Map interpolated RoboCap quaternions to OpenArm 7-DOF joint angles
        Same as map_to_openarm_joints but for interpolated pose
        """
        ## left
        left_collar = pose24[-11]
        left_shoulder = pose24[-8]
        left_elbow = pose24[-6]
        left_wrist = pose24[-4]

        ## right
        right_collar = pose24[-10]
        right_shoulder = pose24[-7]
        right_elbow = pose24[-5]
        right_wrist = pose24[-3]

        ## Fuse collar rotation and shoulder rotation
        left_shoulder = (R.from_quat(left_collar) * R.from_quat(left_shoulder)).as_quat()
        right_shoulder = (R.from_quat(right_collar) * R.from_quat(right_shoulder)).as_quat()


        ## ----------------For left arm--------------------
        # Euler angle decomposition
        l_shoulder = R.from_quat(left_shoulder, scalar_first=False)
        l_j1, l_j2, l_j3 = l_shoulder.as_euler('XYX')
        l_j2 -= np.pi / 2.0  # Adjust for OpenArm's

        # Just use Euler angles for elbow and wrist
        l_elbow = R.from_quat(left_elbow, scalar_first=False)
        z, _, x = l_elbow.as_euler('ZYX')
        l_j4 = -z
        l_j5 = x
        l_wrist = R.from_quat(left_wrist, scalar_first=False)
        z, y, _ = l_wrist.as_euler('ZYX')
        l_j6 = -y
        l_j7 = z

        ## ----------------For right arm--------------------
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

        ## Elbow and wrist (for right arm)
        r_elbow = R.from_quat(right_elbow, scalar_first=False)
        z, _, x = r_elbow.as_euler('ZYX')
        r_j4 = z
        r_j5 = -x - (np.pi/4)
        r_wrist = R.from_quat(right_wrist, scalar_first=False)
        z, y, _ = r_wrist.as_euler('ZYX')
        r_j6 = -z
        r_j7 = -y

        joints = [l_j1, l_j2, l_j3, l_j4, l_j5, l_j6, l_j7,
                  r_j1, r_j2, r_j3, r_j4, r_j5, r_j6, r_j7]
        joints = np.clip(joints, LEFT_JOINTS_MIN_VALUE+RIGHT_JOINTS_MIN_VALUE, LEFT_JOINTS_MAX_VALUE+RIGHT_JOINTS_MAX_VALUE).tolist()

        return joints
    
    def run(self):
        try:
            while self.running:
                self.send_loop()
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

    def cleanup(self):
        self.running = False
        try:
            self.gripper_controller.stop_gui()
            print("Gripper controller GUI stopped")
        except:
            pass

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

        # 恢复计时器精度
        self.winmm.timeEndPeriod(1)

def main():
    print("=" * 60)
    print("  RoboCap → OpenArm Teleoperation (UDP Sender)")
    print("=" * 60)
    print()
    
    # Parse command line arguments
    udp_host = '192.168.0.13' ## Default to broadcast
    udp_port = 5678
    rebocap_port = 7690
    
    if len(sys.argv) >= 2:
        udp_host = sys.argv[1]
    if len(sys.argv) >= 3:
        udp_port = int(sys.argv[2])
    if len(sys.argv) >= 4:
        rebocap_port = int(sys.argv[3])
    
    try:
        sender = ReboCapUdpSender(
            udp_host=udp_host,
            udp_port=udp_port,
            rebocap_port=rebocap_port,
        )
        sender.run()
    except Exception as e:
        print(f"\n Fatal error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
