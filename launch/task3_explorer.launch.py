from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package paths
    pkg_share = FindPackageShare('fieldrobo')
    fieldrobo_share = get_package_share_directory('fieldrobo')

    # Task 3 world
    task3_world = PathJoinSubstitution([
        pkg_share,
        'world',
        'task3_explorer.sdf'
    ])

    # Task 1 simulation
    task1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share,
                'launch',
                'task1_sim.launch.py',
            ])
        ),
        launch_arguments={
            'world': task3_world,
            'rviz_config': 'navigation.rviz',
            'spawn_x': '0.0',
            'spawn_y': '-3.5',
            'spawn_z': '0.3',
        }.items()
    )

    # Nav2 parameters
    nav2_params = os.path.join(
        fieldrobo_share,
        'config',
        'nav2params.yaml'
    )

    # RTAB-Map
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rtabmap_launch'),
                'launch',
                'rtabmap.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'odom_topic': '/husky_velocity_controller/odom',
            'odom_frame_id': '',
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'map_topic': '/map',
            'visual_odometry': 'false',
            'icp_odometry': 'false',
            'subscribe_scan': 'false',
            'subscribe_scan_cloud': 'true',
            'scan_cloud_topic': '/velodyne_points/points',
            'depth': 'false',
            'subscribe_rgb': 'false',
            'subscribe_rgbd': 'false',
            'approx_sync': 'true',
            'odom_sensor_sync': 'true',
            'topic_queue_size': '30',
            'sync_queue_size': '30',
            'qos': '0',
            'qos_odom': '0',
            'wait_for_transform': '0.5',
            'database_path': '~/.ros/fieldrobo_task3.db',
            'rtabmap_args': '--delete_db_on_start',
            'rtabmap_viz': 'false',
            'rviz': 'false',
        }.items(),
    )

    # Nav2 lifecycle nodes
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    # TF remappings
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]


    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': True}
        ],
        remappings=remappings,
    )

    # Smoother Server
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': True}
        ],
        remappings=remappings,
    )

    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': True}
        ],
        remappings=remappings,
    )

    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': True}
        ],
        remappings=remappings,
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': True}
        ],
        remappings=remappings,
    )

    # Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': True}
        ],
        remappings=remappings,
    )

    # Velocity Smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': True}
        ],
        remappings=remappings + [
            (
                'cmd_vel_smoothed',
                '/husky_velocity_controller/cmd_vel_unstamped'
            )
        ],
    )

    # Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': lifecycle_nodes,
        }],
    )

    # Nav2 node list
    nav2_nodes = [
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
    ]

    # Launch
    return LaunchDescription([
        task1_launch,
        TimerAction(
            period=10.0,
            actions=[rtabmap]
        ),
        TimerAction(
            period=12.0,
            actions=nav2_nodes
        )
    ])