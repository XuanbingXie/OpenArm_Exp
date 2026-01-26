import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from contants import JOINTS_MIN_VALUE, JOINTS_MAX_VALUE

class IncrementalShoulderSolver:
    def __init__(self, initial_joints=[0.0, 0.0, 0.0], damping=0.1, step_size=0.5):
        self.init_flag = False
        self.current_joints = np.array(initial_joints, dtype=float)
        self.lam = damping  # 阻尼系数，越大越平滑但响应变慢
        self.alpha = step_size # 步长限制，防止单帧移动过快

    def solve(self, target_quat):
        j1, j2, j3 = self.current_joints
        
        r_curr = R.from_euler('XYX', [j1, j2 + np.pi/2, j3])
        r_target = R.from_quat(target_quat)

        error_rot = r_target * r_curr.inv()
        omega = error_rot.as_rotvec() 
        
        s1, c1 = np.sin(j1), np.cos(j1)
        j2_phys = j2 + np.pi/2
        s2, c2 = np.sin(j2_phys), np.cos(j2_phys)

        J = np.array([
            [1, 0,   c2],
            [0, c1,  s1*s2],
            [0, s1, -c1*s2]
        ])

        jj_t = J @ J.T
        damp = self.lam**2 * np.eye(3)
        delta_theta = J.T @ np.linalg.solve(jj_t + damp, omega)

        delta_theta = np.clip(delta_theta, -0.1, 0.1) 
        self.current_joints += self.alpha * delta_theta
        self.current_joints = np.clip(self.current_joints, JOINTS_MIN_VALUE[:3], JOINTS_MAX_VALUE[:3])
        return self.current_joints.tolist()
    
    def init_joints(self, joints):
        self.current_joints = np.array(joints, dtype=float)
        self.init_flag = True