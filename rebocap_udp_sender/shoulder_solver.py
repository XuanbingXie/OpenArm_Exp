import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import time

class IncrementalShoulderSolver:
    def __init__(self, arm_type='left', 
                 joints_min_value=[-np.pi, -np.pi, -np.pi], 
                 joints_max_value=[np.pi, np.pi, np.pi], 
                 damping=0.1, step_size=0.5):
        if arm_type not in ['left', 'right']:
            raise ValueError("arm_type must be 'left' or 'right'")
        
        self.arm_type = arm_type  
        self.current_joints = np.array([0, 0, 0], dtype=float)
        self.joints_min_value = np.array(joints_min_value, dtype=float)
        self.joints_max_value = np.array(joints_max_value, dtype=float)
        self.lam = damping
        self.damp = self.lam**2 * np.eye(3)
        self.alpha = step_size

        self.min_send_udp_interval = 1.0
        self.max_send_udp_interval = 0.0
        self.frequency_udp_start_time = time.perf_counter()
        self.frequency_send_udp_start_time = time.perf_counter()
        self.frequency_print_interval = 1  # seconds

        if self.arm_type == 'left':
            self.js = [1, 1, 1] 
        else:  
            self.js = [-1, -1, -1]
                                            
    def physics_euler_to_euler_with_bias(self, euler_angles):
        if self.arm_type == 'left':
            euler_angles[1] = euler_angles[1] + np.pi/2
        else:
            euler_angles[0] = -euler_angles[0]
            if( euler_angles[0] < 0):
                euler_angles[0] += np.pi
            else:   
                euler_angles[0] -= np.pi
            euler_angles[1] = -euler_angles[1]
            euler_angles[1] += np.pi / 2.0

            euler_angles[2] = -euler_angles[2]
            if( euler_angles[2] < 0):
                euler_angles[2] += np.pi
            else:   
                euler_angles[2] -= np.pi 
        return euler_angles


    def solve(self, target_quat):
        """
        target_quat: [x, y, z, w]
        """
        j1, j2, j3 = self.current_joints
        
        # Transform to Rebocap System
        j1_phys, j2_phys, j3_phys = self.physics_euler_to_euler_with_bias([j1, j2, j3])
        
        r_curr = R.from_euler('XYX', [j1_phys, j2_phys, j3_phys])
        r_target = R.from_quat(target_quat)

        # Compute Rotation Error
        error_rot = r_target * r_curr.inv()
        omega = error_rot.as_rotvec() # 得到 [wx, wy, wz]
        
        # Construct jocabian
        s1, c1 = math.sin(j1_phys), math.cos(j1_phys)
        s2, c2 = math.sin(j2_phys), math.cos(j2_phys)
        
        J = np.array([
            [1*self.js[0], 0,                 c2*self.js[2]],
            [0,            c1 * self.js[1],  s1*s2*self.js[2]],
            [0,            s1 * self.js[1], -c1*s2*self.js[2]]
        ])
        

        start_time = time.perf_counter()

        # 阻尼最小二乘法
        # Delta_Theta = J^T * inv(J*J^T + lambda^2 * I) * omega
        jj_t = J @ J.T + self.damp
        delta_theta = J.T @ np.linalg.solve(jj_t, omega)

        cur_time = time.perf_counter()
        cur_interval = cur_time - start_time
        self.min_send_udp_interval = min(self.min_send_udp_interval, cur_interval)
        self.max_send_udp_interval = max(self.max_send_udp_interval, cur_interval)
        if (cur_time - self.frequency_send_udp_start_time) >= self.frequency_print_interval:
            print(f"Sender: min_interval: {self.min_send_udp_interval:.4f}s, max_interval: {self.max_send_udp_interval:.4f}s")
            self.min_send_udp_interval = 1.0
            self.max_send_udp_interval = 0.0
            self.frequency_send_udp_start_time = cur_time

        # Update
        delta_theta = np.clip(delta_theta, -0.1, 0.1) 
        self.current_joints += self.alpha * delta_theta
        
        # Clip
        self.current_joints = np.clip(
            self.current_joints, 
            self.joints_min_value, 
            self.joints_max_value
        )
        
        return self.current_joints.tolist()
    
    def init_joints(self, joints):
        self.current_joints = np.array(joints, dtype=float)