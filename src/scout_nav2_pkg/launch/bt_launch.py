import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
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
            'use_sim_time': 'false',
            'autostart': 'true',
            'map': './params/map_params.yaml' # Coloque se usar mapa estático
        }.items()
    )

    # Nó executor do Behavior Tree
    bt_runner_node = Node(
        package='scout_nav2_pkg',
        executable='bt_runner',
        name='bt_runner',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        moveit_launch,
        nav2_launch,
        bt_runner_node,
    ])

