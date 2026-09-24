from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    fieldrobo_dir = get_package_share_directory('fieldrobo')

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    husky_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'husky_velocity_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    pose_controller = Node(
        package='fieldrobo_controller',
        executable='pose_controller',
        parameters=[
            {
                'use_sim_time': True,
                'k_position': 1.2,
                'k_heading': 2.5,
                'k_final_heading': 1.5,
                'max_linear_velocity': 1.0,
                'max_angular_velocity': 2.0,
                'position_tolerance': 0.05,
                'heading_tolerance': 0.05
            }
        ],
        output='screen'
    )

    rviz_config = os.path.join(
        fieldrobo_dir,
        'config',
        'rviz',
        'control.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            rviz_config
        ],
        parameters=[
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([

        # Wait for Gazebo and ros2_control
        TimerAction(
            period=3.0,
            actions=[
                joint_state_broadcaster_spawner
            ]
        ),

        TimerAction(
            period=5.0,
            actions=[
                husky_controller_spawner
            ]
        ),

        TimerAction(
            period=7.0,
            actions=[
                pose_controller
            ]
        ),

        TimerAction(
            period=3.0,
            actions=[
                rviz
            ]
        )
    ])