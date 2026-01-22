import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
import numpy as np
import time
import os
from ament_index_python.packages import get_package_share_directory

class TrajectoryClient(Node):
    def __init__(self):
        super().__init__('trajectory_client')
        # 声明参数
        self.declare_parameter('npz_file_path', 'data/replay_arm_full.npz')
        # 创建 Action 客户端，连接到你的控制器
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/left_joint_trajectory_controller/follow_joint_trajectory'
        )
        # 订阅关节状态
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        self.current_positions = None

    def _joint_state_callback(self, msg):
        # 提取手臂关节的位置
        joint_names = [
            "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
            "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
            "openarm_left_joint7"
        ]
        positions = []
        for name in joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
            else:
                self.get_logger().warn(f'Joint {name} not found in joint_states')
                return
        if len(positions) == len(joint_names):
            self.current_positions = positions

    def get_current_positions(self):
        # 等待获取当前位置
        for _ in range(50):  # 最多等待 5 秒
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_positions is not None:
                break
        if self.current_positions is None:
            self.get_logger().error('无法获取当前关节位置')
            return None
        return self.current_positions

    def send_goal(self, waypoints, total_time=10.0, transition_points=10):
        # 获取当前位置
        current_pos = self.get_current_positions()
        if current_pos is None:
            return None

        # 轨迹起始点
        start_pos = waypoints[0]

        # 生成过渡段：从当前位置到轨迹起始点，采样 transition_points 个点
        transition_waypoints = []
        for i in range(transition_points):
            alpha = i / (transition_points - 1)  # 从 0 到 1
            pos = [(1 - alpha) * c + alpha * s for c, s in zip(current_pos, start_pos)]
            transition_waypoints.append(pos)

        # 完整航点：过渡段 + 轨迹点
        full_waypoints = transition_waypoints + waypoints

        goal_msg = FollowJointTrajectory.Goal()

        # 1. 关节名称必须与硬件接口一致
        goal_msg.trajectory.joint_names = [
            "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
            "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
            "openarm_left_joint7"
        ]

        # 2. 填充航点
        num_points = len(full_waypoints)
        dt = total_time / (num_points - 1) if num_points > 1 else 0.1  # 时间间隔
        for i, pos in enumerate(full_waypoints):
            point = JointTrajectoryPoint()
            point.positions = pos
            # 可选：设置速度和加速度限制以平滑运动
            # point.velocities = [0.1] * len(pos)  # 示例速度限制
            # point.accelerations = [0.05] * len(pos)  # 示例加速度限制
            # 时间分配：均匀分布，总时间 total_time
            time_sec = i * dt
            point.time_from_start = Duration(sec=int(time_sec), nanosec=int((time_sec % 1) * 1e9))
            goal_msg.trajectory.points.append(point)

        # 3. 发送指令
        self._action_client.wait_for_server()
        self.get_logger().info('正在发送轨迹目标...')
        return self._action_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    client = TrajectoryClient()

    # 等待一段时间让订阅器获取关节状态
    time.sleep(1.0)  # 等待 1 秒

    # 获取包共享目录和参数
    package_share_directory = get_package_share_directory('openarm_test')
    relative_npz_path = client.get_parameter('npz_file_path').get_parameter_value().string_value
    npz_file_path = os.path.join(package_share_directory, relative_npz_path)

    # 从 NPZ 文件加载轨迹
    try:
        data = np.load(npz_file_path)
        trajectory = data['arr_0']
        # 扩展轨迹到7个关节，如果当前只有3个，后4个设为0
        extended_trajectory = []
        for point in trajectory:
            if len(point) == 3:
                extended_point = [point[0], point[1], point[2], 0.0, 0.0, 0.0, 0.0]
            else:
                extended_point = point.tolist() if hasattr(point, 'tolist') else list(point)
            extended_trajectory.append(extended_point)
        my_waypoints = extended_trajectory
        print(f"Loaded trajectory with {len(my_waypoints)} points, each with {len(my_waypoints[0])} joints.")
    except Exception as e:
        print(f"Error loading trajectory: {e}")
        rclpy.shutdown()
        return

    # 配置参数
    total_time =20.0  # 总执行时间 (秒)
    transition_points = 100  # 过渡段采样点数

    future = client.send_goal(my_waypoints, total_time=total_time, transition_points=transition_points)
    if future is not None:
        rclpy.spin_until_future_complete(client, future)

        client.get_logger().info('轨迹指令已下达')
    else:
        client.get_logger().error('发送轨迹失败')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
