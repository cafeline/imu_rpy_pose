from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('livox_imu_test')
    param_file = os.path.join(pkg_share, 'config', 'livox_imu_test.param.yaml')

    imu_node = Node(
        package='livox_imu_test',
        executable='livox_imu_test_node',
        name='livox_imu_test',
        output='screen',
        parameters=[param_file]
    )

    return LaunchDescription([imu_node])
