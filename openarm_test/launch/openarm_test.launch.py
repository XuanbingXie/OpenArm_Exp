from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包共享目录
    package_share_directory = get_package_share_directory('openarm_test')
    config_file = os.path.join(package_share_directory, 'config', 'openarm_test.yaml')

    return LaunchDescription([
        # 运行 trajectory_client 节点，并加载参数文件
        Node(
            package='openarm_test',
            executable='trajectory_executor',
            name='trajectory_client',
            parameters=[config_file],
            output='screen',
        ),
    ])
