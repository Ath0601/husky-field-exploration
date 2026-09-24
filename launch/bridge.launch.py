from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    fieldrobo_dir = get_package_share_directory('fieldrobo')

    bridge_config = os.path.join(
        fieldrobo_dir,
        'config',
        'bridge_config.yaml'
    )

    lidar_bridge_config = os.path.join(
        fieldrobo_dir,
        'config',
        'lidar_bridge_config.yaml'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='fieldrobo_bridge',
        output='screen',
        parameters=[
            {
                'config_file': bridge_config
            }
        ]
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        parameters=[
            {
                'config_file': lidar_bridge_config,
                'override_frame_id': 'scan'
            }
        ]
    )

    return LaunchDescription([
        bridge,
        lidar_bridge
    ])