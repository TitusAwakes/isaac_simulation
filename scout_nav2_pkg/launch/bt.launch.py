import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    scout_nav2_pkg_dir = get_package_share_directory('scout_nav2_pkg')  # Your package name

    # Paths to parameter and map files
    nav2_params_file = os.path.join(scout_nav2_pkg_dir, 'params', 'nav2_params.yaml')
    map_file = os.path.join(scout_nav2_pkg_dir, 'params', 'map.yaml')

    print(f"Nav2 Params file: {nav2_params_file}")

    # Nav2 Launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_file,
            'map': map_file,
            'use_sim_time': True,
            'autostart': True,
            'log_level': 'controller_server:=debug'
        }.items()
    )


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

