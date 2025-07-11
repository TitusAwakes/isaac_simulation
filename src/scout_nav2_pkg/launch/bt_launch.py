import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Caminho para o launch do MoveIt
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('piper_camera_moveit_config'),
                'launch',
                'demo.launch.py'
            )
        ])
    )

    # Caminho para os parâmetros do Nav2
    nav2_params_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'params',
        'nav2_params.yaml'
    )

    # Launch do Nav2 Bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ]),
        launch_arguments={
            'params_file': './params/nav2_params.yaml',
            'use_sim_time': 'true',
            'autostart': 'true',
            'map': './params/map.yaml'
        }.items()
    )

    # Nó executor do Behavior Tree
    bt_runner_node = Node(
        package='scout_nav2_pkg',
        executable='bt_runner',
        name='bt_runner',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Initial pose publisher command (one shot)
    initial_pose_pub_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '/initialpose',
            'geometry_msgs/PoseWithCovarianceStamped',
            "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}",
            '--once'
        ],
        output='screen'
    )

    # Delay initial pose pub by 5 seconds to ensure AMCL is up
    delayed_initial_pose_pub = TimerAction(
        period=5.0,
        actions=[initial_pose_pub_cmd]
    )
    
    cmd_vel_pub_cmd = ExecuteProcess(
    cmd=[
        'ros2', 'topic', 'pub', '/cmd_vel',
        'geometry_msgs/msg/Twist',
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}",
        '--once'
    ],
    output='screen'
)

    return LaunchDescription([
        nav2_launch,
        moveit_launch,
        delayed_initial_pose_pub,
        cmd_vel_pub_cmd
    ])

