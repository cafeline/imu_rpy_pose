from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('imu_rpy_pose')
    param_file = os.path.join(pkg_share, 'config', 'imu_rpy_pose.param.yaml')

    imu_node = Node(
        package='imu_rpy_pose',
        executable='imu_rpy_pose_node',
        name='imu_rpy_pose',
        output='screen',
        parameters=[param_file]
    )

    return LaunchDescription([imu_node])
