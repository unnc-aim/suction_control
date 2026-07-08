from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('suction_control'),
        'config',
        'suction_control_r2.yaml'
    )

    suction_control_node = Node(
        package='suction_control',
        executable='suction_control_node',
        name='suction_control',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        suction_control_node
    ])
