from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    FindExecutable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ============================================================
    # Package paths
    # ============================================================

    pkg_share = FindPackageShare('fieldrobo')

    world_path = LaunchConfiguration('world')

    xacro_file = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'clearpathHusky.urdf.xacro'
    ])

    controller_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'controller.yaml'
    ])

    rviz_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'rviz',
        LaunchConfiguration('rviz_config')
    ])


    # ============================================================
    # Launch arguments
    # ============================================================

    use_sim_time = LaunchConfiguration('use_sim_time')


    # ============================================================
    # Robot description
    # ============================================================

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_file
    ])

    # ============================================================
    # Robot State Publisher
    # ============================================================

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',

        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(
                    robot_description_content,
                    value_type=str
                ),
            }
        ],

        output='screen'
    )


    # ============================================================
    # Gazebo Fortress
    # ============================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),

        launch_arguments={
            'gz_args': ['-r ', world_path],
            'on_exit_shutdown': 'true'
        }.items()
    )


    # ============================================================
    # Spawn Husky into Gazebo
    # ============================================================

    create_husky = Node(
        package='ros_gz_sim',
        executable='create',

        arguments=[
            '-name',
            'husky',
            '-topic',
            'robot_description',
            '-world', 'fieldrobo_world',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
        ],

        output='screen'
    )


    # ============================================================
    # Joint State Broadcaster
    # ============================================================

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


    # ============================================================
    # Husky velocity controller
    # ============================================================

    husky_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',

        arguments=[
            'husky_velocity_controller',
            '--controller-manager',
            '/controller_manager'
        ],

        output='screen'
    )


    # ============================================================
    # ROS-Gazebo bridge
    # ============================================================

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_share,
                'launch',
                'bridge.launch.py'
            ])
        ])
    )


    # ============================================================
    # RViz2
    # ============================================================

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',

        arguments=[
            '-d',
            rviz_config
        ],

        parameters=[
            {
                'use_sim_time': use_sim_time
            }
        ],

        output='screen'
    )


    # ============================================================
    # Launch sequence
    # ============================================================

    return LaunchDescription([

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock'
        ),

        DeclareLaunchArgument(
            name='rviz_config',
            default_value='control.rviz',
            description='RViz configuration file'
        ),

        DeclareLaunchArgument(
            name='world',
            default_value=PathJoinSubstitution([
                pkg_share,
                'world',
                'world.sdf'
            ]),
            description='Gazebo world SDF file'
        ),

        DeclareLaunchArgument(
            'spawn_x',
            default_value='0.0',
            description='Husky spawn X'
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='0.0',
            description='Husky spawn Y'
        ),
        DeclareLaunchArgument(
            'spawn_z',
            default_value='0.3',
            description='Husky spawn Z'
        ),

        # Start Gazebo
        gazebo,

        # Publish robot_description + TF
        robot_state_publisher_node,

        # Spawn robot after Gazebo and robot_state_publisher start
        TimerAction(
            period=3.0,
            actions=[
                create_husky
            ]
        ),

        # Start controllers after the robot has been spawned
        TimerAction(
            period=6.0,
            actions=[
                joint_state_broadcaster_spawner,
                husky_velocity_controller_spawner,
            ]
        ),

        # Start bridge and RViz
        TimerAction(
            period=6.0,
            actions=[
                bridge,
                rviz
            ]
        ),
    ])