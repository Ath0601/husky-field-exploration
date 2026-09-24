from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fieldrobo',
            executable='traj_generate',
            name='trajectory_generator',
            parameters=[{
                'use_sim_time': True,
                'trajectory_type': 'circle',
                'publish_frequency': 50.0,
                'radius': 2.0,
                'period': 30.0,
                'center_x': 0.0,
                'center_y': 0.0,
            }],
            output='screen'
        )
    ])